#include "sensors_tof.h"
#include "shared_state.h"
#include "pins.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

namespace {
constexpr uint8_t kDefaultAddress = 0x29;
constexpr uint32_t kSideBudgetUs = 20000, kSidePeriodMs = 25;
constexpr uint32_t kParkingBudgetUs = 33000, kParkingPeriodMs = 40;
constexpr uint32_t kResetMs = 2;
constexpr uint32_t kBootMs = 10;
constexpr uint32_t kInitTimeoutMs = 100;
constexpr uint32_t kPollSlotMs = 2; // One sensor per slot: 8 ms sweep for four.
constexpr uint32_t kFreshMs = 150;
constexpr uint32_t kFrameTimeoutMs = 250;
constexpr uint32_t kRetryMs = 5000;
static_assert(kSidePeriodMs * 1000 >= kSideBudgetUs + 4000, "Side timing margin");
static_assert(kParkingPeriodMs * 1000 >= kParkingBudgetUs + 4000, "Parking timing margin");
static_assert((TOF_ENABLED_MASK & ~0x0F) == 0, "Only four ToF slots exist");

struct Config {
    uint8_t pin, address;
    uint16_t maxMm;
    uint32_t budgetUs, periodMs;
    const char* name;
};
const Config config[TOF_COUNT] = {
    {PIN_XSHUT_4, 0x30, 150, kParkingBudgetUs, kParkingPeriodMs, "Rear"},
    {PIN_XSHUT_2, 0x31, 600, kSideBudgetUs, kSidePeriodMs, "Left"},
    {PIN_XSHUT_3, 0x32, 600, kSideBudgetUs, kSidePeriodMs, "Right"},
    {PIN_XSHUT_1, 0x33, 150, kParkingBudgetUs, kParkingPeriodMs, "Front"}
};

// Pololu's last_status only describes the LAST write, and does not report
// short requestFrom() reads. Latch every failure across an entire operation.
// This forwarding adapter uses Arduino-Pico's virtual TwoWire methods. It
// never begins/ends a second controller; all traffic uses the existing Wire.
class CheckedWire : public TwoWire {
public:
    CheckedWire() : TwoWire(i2c0, PIN_I2C_SDA, PIN_I2C_SCL) {}
    void clear() { failed = false; }
    bool ok() const { return !failed; }
    void beginTransmission(uint8_t address) override {
        if (!failed) Wire.beginTransmission(address);
    }
    size_t write(uint8_t value) override {
        if (failed) return 0;
        size_t n = Wire.write(value);
        if (n != 1) failed = true;
        return n;
    }
    size_t write(const uint8_t* data, size_t size) override {
        if (failed) return 0;
        size_t n = Wire.write(data, size);
        if (n != size) failed = true;
        return n;
    }
    uint8_t endTransmission(bool stop) override {
        if (failed) return 4;
        uint8_t status = Wire.endTransmission(stop);
        if (status != 0) failed = true;
        return status;
    }
    uint8_t endTransmission() override { return endTransmission(true); }
    size_t requestFrom(uint8_t address, size_t size, bool stop) override {
        if (failed) return 0;
        size_t n = Wire.requestFrom(address, size, stop);
        if (n != size) failed = true;
        return n;
    }
    size_t requestFrom(uint8_t address, size_t size) override {
        return requestFrom(address, size, true);
    }
    int read() override {
        int value = failed ? -1 : Wire.read();
        if (value >= 0) return value;
        failed = true;
        // Library configuration math cannot safely consume 0 or 0xFF for
        // oscillator/VCSEL registers. A failed operation is ALWAYS discarded;
        // this filler only lets the library unwind without divide-by-zero.
        return 1;
    }
    int available() override { return failed ? 0 : Wire.available(); }
private:
    bool failed = false;
};
CheckedWire bus;
VL53L1X sensors[TOF_COUNT];

struct State {
    bool running = false;
    bool sampled = false;
    uint16_t samples[3] = {};
    uint8_t next = 0;
    uint16_t distance = TOF_INVALID_MM;
    uint32_t lastValid = 0, lastFrame = 0, downSince = 0;
    uint32_t retryDelay = kResetMs;
};
State state[TOF_COUNT];
int8_t booting = -1; // Only this sensor may be at 0x29.
uint32_t releasedAt = 0, lastPoll = 0;
uint8_t pollIndex = 0, recoveryIndex = 0;

bool elapsed(uint32_t now, uint32_t then, uint32_t interval) {
    return uint32_t(now - then) >= interval;
}
void holdReset(uint8_t i) {
    digitalWrite(config[i].pin, LOW); // Latch LOW before enabling output.
    pinMode(config[i].pin, OUTPUT);
}
void invalidate(uint8_t i) {
    state[i].sampled = false;
    state[i].distance = TOF_INVALID_MM;
    state[i].next = 0;
}
void resetBank() {
    for (uint8_t i = 0; i < TOF_COUNT; ++i) {
        holdReset(i);
        state[i] = State{};
        state[i].downSince = millis();
    }
    booting = -1;
}
void retire(uint8_t i) {
    holdReset(i); // Never leave a failed/unaddressed device awake.
    state[i].running = false;
    invalidate(i);
    state[i].downSince = millis();
    state[i].retryDelay = kRetryMs;
    if (booting == i) booting = -1;
}

// Caller holds WIRE_LOCK. Probe status is independent of CheckedWire's latch.
bool responds(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}
bool configure(uint8_t i) {
    // XSHUT restores the hardware address AND requires fresh library state
    // (address, calibration flags, saved VHV settings, oscillator cache).
    sensors[i] = VL53L1X();
    VL53L1X& sensor = sensors[i];
    sensor.setBus(&bus);
    sensor.setTimeout(kInitTimeoutMs);
    bus.clear();
    if (sensor.readReg16Bit(VL53L1X::IDENTIFICATION__MODEL_ID) != 0xEACC || !bus.ok()) return false;
    if ((sensor.readReg(VL53L1X::FIRMWARE__SYSTEM_STATUS) & 1) == 0 || !bus.ok()) return false;
    if (sensor.readReg16Bit(VL53L1X::OSC_MEASURED__FAST_OSC__FREQUENCY) == 0 || !bus.ok()) return false;
    if (sensor.readReg16Bit(VL53L1X::RESULT__OSC_CALIBRATE_VAL) == 0 || !bus.ok()) return false;
    if (!sensor.init() || !bus.ok() || sensor.timeoutOccurred()) return false;
    sensor.setAddress(config[i].address);
    if (!bus.ok()) return false;
    if (sensor.readReg(VL53L1X::I2C_SLAVE__DEVICE_ADDRESS) != config[i].address || !bus.ok()) return false;
    if (sensor.readReg16Bit(VL53L1X::IDENTIFICATION__MODEL_ID) != 0xEACC || !bus.ok()) return false;
    // The released sensor must have left the default address.
    if (responds(kDefaultAddress)) return false;
    if (!sensor.setDistanceMode(VL53L1X::Short) || !bus.ok()) return false;
    if (!sensor.setMeasurementTimingBudget(config[i].budgetUs) || !bus.ok()) return false;
    uint32_t actualBudget = sensor.getMeasurementTimingBudget();
    if (!bus.ok() || actualBudget < config[i].budgetUs - 1000 || actualBudget > config[i].budgetUs + 1000) return false;
    sensor.startContinuous(config[i].periodMs);
    return bus.ok();
}

void recover(uint32_t now) {
    if (booting >= 0) {
        if (!elapsed(now, releasedAt, kBootMs)) return;
        uint8_t i = uint8_t(booting);
        WIRE_LOCK();
        bool ok = configure(i);
        WIRE_UNLOCK();
        booting = -1;
        if (ok) {
            state[i].running = true;
            state[i].lastFrame = millis();
            Serial.printf("[ToF] %s ready @ 0x%02X\n", config[i].name, config[i].address);
        } else {
            retire(i);
            Serial.printf("[ToF] %s unavailable; retry in 5 s\n", config[i].name);
        }
        return;
    }
    for (uint8_t n = 0; n < TOF_COUNT; ++n) {
        uint8_t i = recoveryIndex;
        recoveryIndex = (recoveryIndex + 1) % TOF_COUNT;
        if (!(TOF_ENABLED_MASK & (1u << i)) || state[i].running ||
            !elapsed(now, state[i].downSince, state[i].retryDelay)) continue;
        WIRE_LOCK();
        bool conflict = responds(kDefaultAddress) || responds(config[i].address);
        WIRE_UNLOCK();
        if (conflict) {
            // A brownout can put an already-running sensor back at 0x29.
            // Re-isolate the entire ToF bank before assigning addresses again.
            resetBank();
            for (auto& s : state) s.retryDelay = kRetryMs;
            Serial.println("[ToF] Address conflict; bank held in reset for 5 s");
            return;
        }
        pinMode(config[i].pin, INPUT); // Release XSHUT; never drive it HIGH.
        booting = i;
        releasedAt = millis();
        return;
    }
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}
void poll(uint8_t i) {
    State& s = state[i];
    if (!s.running) return;
    WIRE_LOCK();
    bus.clear();
    bool ready = sensors[i].dataReady();
    uint16_t raw = 0;
    bool valid = false;
    if (bus.ok() && ready) {
        raw = sensors[i].read(false);
        valid = sensors[i].ranging_data.range_status == VL53L1X::RangeValid;
    }
    bool ioOk = bus.ok() && !sensors[i].timeoutOccurred();
    WIRE_UNLOCK();
    if (!ioOk) { retire(i); return; }
    uint32_t now = millis();
    if (!ready) {
        if (elapsed(now, s.lastFrame, kFrameTimeoutMs)) retire(i);
        return;
    }
    s.lastFrame = now; // Invalid optical returns still prove the sensor is alive.
    if (!valid) return;
    if (elapsed(now, s.lastValid, kFreshMs)) invalidate(i);
    uint16_t value = raw < config[i].maxMm ? raw : config[i].maxMm;
    if (!s.sampled) {
        // Seed from the first real sample; never publish artificial zero ranges.
        for (auto& sample : s.samples) sample = value;
        s.sampled = true;
    }
    s.samples[s.next] = value;
    s.next = (s.next + 1) % 3;
    s.distance = median3(s.samples[0], s.samples[1], s.samples[2]);
    s.lastValid = now;
}
} // namespace

void tof_init() {
    resetBank();
    pollIndex = recoveryIndex = 0;
    lastPoll = millis();
    // Startup and retries are advanced by tof_update(), with no boot delays
    // blocking UART/LED service. Wire and g_wire_mutex must already be ready.
}
void tof_update() {
    uint32_t now = millis();
    if (elapsed(now, lastPoll, kPollSlotMs)) {
        lastPoll = now;
        poll(pollIndex);
        pollIndex = (pollIndex + 1) % TOF_COUNT;
    }
    recover(millis());
}
uint16_t tof_get(uint8_t i) {
    if (i >= TOF_COUNT || !state[i].running || !state[i].sampled ||
        elapsed(millis(), state[i].lastValid, kFreshMs)) return TOF_INVALID_MM;
    return state[i].distance;
}
bool tof_connected(uint8_t i) {
    return i < TOF_COUNT && state[i].running &&
           !elapsed(millis(), state[i].lastFrame, kFrameTimeoutMs);
}
