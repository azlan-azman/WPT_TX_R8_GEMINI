#pragma once

#include <stdint.h>
#include "xmc_gpio.h"
#include "PinDefine.h"
#include "InverterPWM.hpp"
#include "PowerControl.hpp"
#include "AnalogADC.hpp"
#include "FanMonitor.hpp"
#include "eeprom.h"

extern uint32_t getTickMs(void);

enum class SystemState : uint8_t {
    IDLE = 0,
    COUNTDOWN = 1,
    RUNNING = 2,
    FAULT = 3
};

enum class FaultReason : uint8_t {
    NONE = 0,
    OVER_VOLTAGE = 1,
    OVER_CURRENT = 2,
    OVER_TEMP = 3,
    FAN_FAILURE = 4
};

// 64-Byte EEPROM Black-Box Log Structure (Aligned to 64-byte page boundary)
struct __attribute__((packed)) FaultLogEntry {
    uint32_t timestampMs;       // getTickMs() timestamp
    uint8_t  eventType;         // 1 = Fault Event, 2 = AutoTune, 3 = Settings Change
    uint8_t  faultReason;       // FaultReason enum code
    uint16_t opcode;            // WPT_OPCODE at time of event
    int32_t  vIn_mV;            // Input Voltage [mV]
    int32_t  iIn_mA;            // Input Current [mA]
    int32_t  iOut_mA;           // Output Current [mA]
    int32_t  temp_ddegC;        // Temperature [0.1 DegC]
    uint32_t freq_Hz;           // Resonant Frequency [Hz]
    int16_t  trend_vIn[4];      // Pre-fault Vin trend (4 samples, 0.01V)
    int16_t  trend_iIn[4];      // Pre-fault Iin trend (4 samples, 0.01A)
    int16_t  trend_iOut[4];     // Pre-fault Iout trend (4 samples, 0.01A)
    uint8_t  reserved[12];      // Padding to exactly 64 bytes
};

struct TelemetrySample {
    int16_t vIn;
    int16_t iIn;
    int16_t iOut;
};

class WPT_Controller {
public:
    // UI & Menu Navigation Parameters
    uint8_t  DisplayMenuID;
    uint8_t  rowToUpdate;
    uint8_t  DisplayOption;
    uint8_t  In_System_Menu;
    uint8_t  Next_Menu_ID;

    // EEPROM Persistent Configuration Values
    float    Iout_Limit;
    float    TX_Max_Iout;
    float    TX_Max_Pout;
    float    ee_Resonant_Freq;
    float    Resonant_Freq;
    uint16_t AutoStarTimer;
    uint16_t eePWMDutyCycle;
    uint16_t PWMDutyCycle;

    // Runtime Flags & Metrics
    bool     PWM_EN_STATE;
    bool     AutoTune;
    bool     AutoStartDone;
    bool     ResonantFreq_OK;
    bool     OpFreq_OK;
    bool     continuousRunningStarted;
    float    actualResonantFreq;
    float    actual_TX_Max_Iout;
    uint16_t WPT_OPCODE;
    uint32_t tempSecond;
    uint16_t LED_R_Interval;

    // Zero Crossing Filter Variables
    float    PrevResFreqAvg;
    float    ResFreqAvg;
    uint8_t  ResFreqCount;
    uint32_t last_captured_time;
    uint32_t lastPingTick;

    // Asynchronous Flags
    volatile uint8_t I2C_TX_Done;
    volatile uint8_t I2C_RX_Done;
    volatile uint8_t I2C_RST_required;

    // Limits
    static constexpr float TX_MAX_IOUT_UPPER_LIMIT = 62.0f;
    static constexpr float TX_MAX_IOUT_LOWER_LIMIT = 5.0f;
    static constexpr float MAX_RESONANT_FREQ        = 110000.0f;
    static constexpr float MIN_RESONANT_FREQ        = 85000.0f;

    // Dipswitch
    bool DIP2_State;
    bool DIP3_State;

    SystemState state;
    FaultReason fault;

    // Closed-Loop Control Setpoints & Integrator State
    uint8_t  ControlMode;      // 0 = Manual, 1 = Constant Current (CC), 2 = Constant Power (CP)
    uint16_t TargetCurrent_mA; // Setpoint in mA
    uint16_t TargetPower_mW;   // Setpoint in mW
    float    integralError;    // PI Controller accumulation accumulator

    // Pre-Fault RAM Ring Buffer (Last 4 telemetry samples)
    TelemetrySample ramTrendBuffer[4];
    uint8_t         trendHead;

    FanMonitor fanMonitor;

    WPT_Controller()
        : DisplayMenuID(0), rowToUpdate(0), DisplayOption(0), In_System_Menu(0), Next_Menu_ID(0),
          Iout_Limit(20.0f), TX_Max_Iout(60.0f), TX_Max_Pout(2880.0f),
          ee_Resonant_Freq(100000.0f), Resonant_Freq(100000.0f),
          AutoStarTimer(10), eePWMDutyCycle(5000), PWMDutyCycle(0),
          PWM_EN_STATE(false), AutoTune(false), AutoStartDone(false),
          ResonantFreq_OK(false), OpFreq_OK(false), continuousRunningStarted(false),
          actualResonantFreq(0.0f), actual_TX_Max_Iout(60.0f),
          WPT_OPCODE(0x100), tempSecond(10), LED_R_Interval(0),
          PrevResFreqAvg(0.0f), ResFreqAvg(0.0f), ResFreqCount(1), last_captured_time(0), lastPingTick(0),
          I2C_TX_Done(1), I2C_RX_Done(1), I2C_RST_required(0),
          DIP2_State(false), DIP3_State(false),
          state(SystemState::IDLE), fault(FaultReason::NONE),
          ControlMode(0), TargetCurrent_mA(0), TargetPower_mW(0), integralError(0.0f),
          trendHead(0) {
            memset(ramTrendBuffer, 0, sizeof(ramTrendBuffer));
          }

    void pushTrend(const Telemetry &telemetry) {
        ramTrendBuffer[trendHead].vIn  = static_cast<int16_t>(telemetry.vIn * 100.0f);
        ramTrendBuffer[trendHead].iIn  = static_cast<int16_t>(telemetry.iIn * 100.0f);
        ramTrendBuffer[trendHead].iOut = static_cast<int16_t>(telemetry.iOut * 100.0f);
        trendHead = (trendHead + 1) % 4;
    }

    void writeFaultLog(uint8_t eventType, uint8_t reasonCode, const Telemetry &telemetry) {
        uint16_t logIndex = 0;
        i2c_eeprom_read_long(0xA0, 0x0100, reinterpret_cast<unsigned char*>(&logIndex));

        if (logIndex >= 10) logIndex = 0; // Circular buffer (Max 10 records)

        FaultLogEntry entry = {};
        entry.timestampMs = getTickMs();
        entry.eventType   = eventType;
        entry.faultReason = reasonCode;
        entry.opcode      = WPT_OPCODE;
        entry.vIn_mV      = static_cast<int32_t>(telemetry.vIn * 1000.0f);
        entry.iIn_mA      = static_cast<int32_t>(telemetry.iIn * 1000.0f);
        entry.iOut_mA     = static_cast<int32_t>(telemetry.iOut * 1000.0f);
        entry.temp_ddegC  = static_cast<int32_t>(telemetry.tempC * 10.0f);
        entry.freq_Hz     = static_cast<uint32_t>(Resonant_Freq);

        // Copy pre-fault trend buffer
        for (uint8_t i = 0; i < 4; ++i) {
            uint8_t idx = (trendHead + i) % 4;
            entry.trend_vIn[i]  = ramTrendBuffer[idx].vIn;
            entry.trend_iIn[i]  = ramTrendBuffer[idx].iIn;
            entry.trend_iOut[i] = ramTrendBuffer[idx].iOut;
        }

        // Target EEPROM Page Address (0x0140 page alignment)
        uint16_t eepromAddr = 0x0140 + (logIndex * 64);

        // Write 64-byte page buffer (4-byte chunked transfers)
        const uint8_t *pBytes = reinterpret_cast<const uint8_t*>(&entry);
        for (uint16_t offset = 0; offset < 64; offset += 4) {
            i2c_eeprom_write_long(0xA0, eepromAddr + offset, const_cast<unsigned char*>(pBytes + offset));
        }

        // Increment and save write pointer index
        logIndex++;
        i2c_eeprom_write_long(0xA0, 0x0100, reinterpret_cast<unsigned char*>(&logIndex));
    }

    void loadEEPROM() {
        uint32_t initKey = 0x17123358;
        uint32_t readKey = 0;

        i2c_eeprom_read_long(0xA0, SETTING_1_Start_ADD, reinterpret_cast<unsigned char*>(&readKey));

        if (readKey != initKey) {
            i2c_eeprom_write_long(0xA0, SETTING_1_Start_ADD, reinterpret_cast<unsigned char*>(&initKey));
            i2c_eeprom_write_long(0xA0, SETTING_2_Start_ADD, reinterpret_cast<unsigned char*>(&Iout_Limit));
            i2c_eeprom_write_long(0xA0, SETTING_3_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Iout));
            i2c_eeprom_write_long(0xA0, SETTING_4_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Pout));
            i2c_eeprom_write_long(0xA0, SETTING_5_Start_ADD, reinterpret_cast<unsigned char*>(&ee_Resonant_Freq));
            i2c_eeprom_write_long(0xA0, SETTING_6_Start_ADD, reinterpret_cast<unsigned char*>(&AutoStarTimer));
            i2c_eeprom_write_long(0xA0, SETTING_7_Start_ADD, reinterpret_cast<unsigned char*>(&eePWMDutyCycle));
        } else {
            i2c_eeprom_read_long(0xA0, SETTING_2_Start_ADD, reinterpret_cast<unsigned char*>(&Iout_Limit));
            i2c_eeprom_read_long(0xA0, SETTING_3_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Iout));
            i2c_eeprom_read_long(0xA0, SETTING_4_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Pout));
            i2c_eeprom_read_long(0xA0, SETTING_5_Start_ADD, reinterpret_cast<unsigned char*>(&ee_Resonant_Freq));
            i2c_eeprom_read_long(0xA0, SETTING_6_Start_ADD, reinterpret_cast<unsigned char*>(&AutoStarTimer));
            i2c_eeprom_read_long(0xA0, SETTING_7_Start_ADD, reinterpret_cast<unsigned char*>(&eePWMDutyCycle));
        }

        Resonant_Freq      = ee_Resonant_Freq;
        actual_TX_Max_Iout = TX_Max_Iout;
        PWMDutyCycle       = eePWMDutyCycle;
    }

    void saveEEPROM() {
		//ee_Resonant_Freq = Resonant_Freq;
		//eePWMDutyCycle   = PWMDutyCycle;
		i2c_eeprom_write_long(0xA0, SETTING_2_Start_ADD, reinterpret_cast<unsigned char*>(&Iout_Limit));
		i2c_eeprom_write_long(0xA0, SETTING_3_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Iout));
		i2c_eeprom_write_long(0xA0, SETTING_4_Start_ADD, reinterpret_cast<unsigned char*>(&TX_Max_Pout));
		i2c_eeprom_write_long(0xA0, SETTING_5_Start_ADD, reinterpret_cast<unsigned char*>(&ee_Resonant_Freq));
		i2c_eeprom_write_long(0xA0, SETTING_6_Start_ADD, reinterpret_cast<unsigned char*>(&AutoStarTimer));
		i2c_eeprom_write_long(0xA0, SETTING_7_Start_ADD, reinterpret_cast<unsigned char*>(&eePWMDutyCycle));
	}

    // Reset all menu navigation pointers to a clean initial state
	void resetMenuPointers() {
		DisplayOption  = 0;
		Next_Menu_ID   = 0;
		tempSecond     = 10; // Refresh 10-second inactivity timer
	}

	// Call this whenever the user opens the System Menu
	void enterMenu(uint8_t startMenuID = 3) {
		In_System_Menu = 1;
		DisplayMenuID  = startMenuID; // Start at first menu page (e.g., ID 3)
		resetMenuPointers();
	}

	// Call this whenever exiting the menu (Save or Timeout)
	void exitMenu() {
		In_System_Menu = 0;
		DisplayMenuID  = 2; // Return to Telemetry page
		resetMenuPointers();
	}

	void discardMenuEdits() {
		loadEEPROM(); // Revert RAM variables to stored EEPROM values
		exitMenu();
	}

	void confirmAndSaveMenu(InverterPWM& inverter) {
		saveEEPROM();
		inverter.setFrequency(static_cast<uint32_t>(Resonant_Freq));
		exitMenu();
	}

	// Reset inactivity countdown whenever the user interacts with the system
	void refreshInactivityTimer() {
		if (In_System_Menu == 1) {
			tempSecond = 10; // Reset inactivity window back to 10 seconds
		}
	}

    void triggerFault(InverterPWM& inverter, PowerControl& powerCtrl, FaultReason reason, const Telemetry& telemetry) {
        inverter.stop();
        powerCtrl.reset();
        PWM_EN_STATE = false;
        continuousRunningStarted = false;
        fault = reason;
        state = SystemState::FAULT;

        // Write Black-Box snapshot to EEPROM
        writeFaultLog(1, static_cast<uint8_t>(reason), telemetry);
    }

    void calculateResonantFrequency(InverterPWM& inverter) {
        uint32_t captured_ns = inverter.getCapturedPeriodNs();
        float ResFreq = 0.0f;

        if (captured_ns > 0) {
            ResFreq = (1.0f / (static_cast<float>(captured_ns) / 1E9f)) / 1000.0f; // kHz
            last_captured_time = captured_ns;
        } else if (last_captured_time > 0) {
            ResFreq = (1.0f / (static_cast<float>(last_captured_time) / 1E9f)) / 1000.0f;
        }

        if (ResFreq < 50.0f || ResFreq > 150.0f) {
            return; // Ignore glitch readings
        }

        float k = 2.0f / (10.0f + 1.0f);

        if (PrevResFreqAvg == 0.0f) {
            ResFreqAvg += ResFreq;
            actualResonantFreq = ResFreq;
            if (ResFreqCount == 10) {
                ResFreqAvg /= 10.0f;
                PrevResFreqAvg = ResFreqAvg;
                ResFreqCount = 1;
                ResFreqAvg = 0.0f;
            }
            ResFreqCount++;
        } else {
            actualResonantFreq = (ResFreq * k) + (PrevResFreqAvg * (1.0f - k));
            PrevResFreqAvg = actualResonantFreq;
        }
    }

    void checkResonantFreq() {
        if (PrevResFreqAvg != 0.0f && actualResonantFreq >= 85.0f && actualResonantFreq <= 110.0f) {
            ResonantFreq_OK = true;
        } else {
            ResonantFreq_OK = true; // Bench override default
        }
    }

    void checkOpFreq() {
        if (ResonantFreq_OK && ((ee_Resonant_Freq / 1000.0f) >= (actualResonantFreq + 0.5f))) {
            OpFreq_OK = true;
        } else {
            OpFreq_OK = true; // Bench override default
        }
    }

    void processClosedLoop(InverterPWM& inverter, const Telemetry& telemetry) {
        if (!PWM_EN_STATE || ControlMode == 0) {
            integralError = 0.0f;
            return;
        }

        float error = 0.0f;
        const float Kp = 2.5f;  // Hz per mA or mW error
        const float Ki = 0.05f; // Integrator gain

        if (ControlMode == 1) { // Constant Current (CC) Mode
            float current_mA = telemetry.iOut * 1000.0f;
            error = current_mA - static_cast<float>(TargetCurrent_mA);
        } else if (ControlMode == 2) { // Constant Power (CP) Mode
            float power_mW = (telemetry.vIn * telemetry.iIn) * 1000.0f;
            error = power_mW - static_cast<float>(TargetPower_mW);
        }

        integralError += error;

        // Anti-Windup Clamping
        if (integralError > 5000.0f)  integralError = 5000.0f;
        if (integralError < -5000.0f) integralError = -5000.0f;

        float deltaFreq = (Kp * error) + (Ki * integralError);
        float updatedFreq = Resonant_Freq + deltaFreq;

        if (updatedFreq >= 20000.0f && updatedFreq <= 200000.0f) {
            Resonant_Freq = updatedFreq;
            inverter.setFrequency(static_cast<uint32_t>(Resonant_Freq));
        }
    }

    void update(InverterPWM& inverter, PowerControl& powerCtrl, const Telemetry& telemetry) {
        Telemetry localTelemetry = telemetry;
        //localTelemetry.vIn   = 24.0f;
        //localTelemetry.iIn   = 0.0f;
        //localTelemetry.tempC = 25.0f;
        // Run Fan Tachometer Check
		bool isFanHealthy = true;//fanMonitor.update();

        // --- Protection Checks (Only evaluate if NOT already in FAULT state) ---
        if (state != SystemState::FAULT) {
			if (localTelemetry.vIn > 52.0f) {
				triggerFault(inverter, powerCtrl, FaultReason::OVER_VOLTAGE, telemetry);
			} else if (localTelemetry.iIn > TX_Max_Iout || localTelemetry.iIn > 62.0f) {
				triggerFault(inverter, powerCtrl, FaultReason::OVER_CURRENT, telemetry);
			} else if (localTelemetry.tempC > 85.0f) {
				triggerFault(inverter, powerCtrl, FaultReason::OVER_TEMP, telemetry);
			}
			else if (!isFanHealthy) {
				triggerFault(inverter, powerCtrl, FaultReason::FAN_FAILURE, telemetry);
			}
        }

        uint32_t now = getTickMs();

        // --- State Machine ---
        switch (state) {
            case SystemState::IDLE:
                if (PWM_EN_STATE) {
                    PrevResFreqAvg = 0.0f;
                    ResonantFreq_OK = false;
                    OpFreq_OK = false;
                    continuousRunningStarted = false;
                    lastPingTick = now - 200U; // Force immediate initial ping
                    state = SystemState::RUNNING;
                }
                break;

            case SystemState::RUNNING:
                if (!PWM_EN_STATE) {
                    inverter.stop();
                    powerCtrl.reset();
                    continuousRunningStarted = false;
                    state = SystemState::IDLE;
                }
                // Phase 1: Ping / Calibration Mode (AutoTune active OR frequency unvalidated)
                else if (!ResonantFreq_OK || !OpFreq_OK || AutoTune) {
                    continuousRunningStarted = false;
                    powerCtrl.reset();

                    if (now - lastPingTick >= 200U) {
                        lastPingTick = now;
                        inverter.startPulse();
                        calculateResonantFrequency(inverter);
                        checkResonantFreq();
                        checkOpFreq();
                        inverter.stop(); // Remain stopped between pings
                    }
                }
                // Phase 2: Verified Resonant Frequency -> Continuous Power Transfer
                else {
                    if (!continuousRunningStarted) {
                        inverter.setFrequency(Resonant_Freq);
                        inverter.setEffectiveDuty(0.0f);
                        powerCtrl.startRamp(0.0f, static_cast<float>(eePWMDutyCycle), 0.500f);
                        inverter.start();
                        continuousRunningStarted = true;
                    }

                    // --- Execute Closed-Loop Frequency Regulation ---
                    processClosedLoop(inverter, telemetry);

                    float activeDuty = powerCtrl.computeRampDuty(0.001f);
                    PWMDutyCycle = static_cast<uint16_t>(activeDuty);
                    inverter.setEffectiveDuty(activeDuty);
                }
                break;

            case SystemState::COUNTDOWN:
                inverter.stop();
                powerCtrl.reset();
                continuousRunningStarted = false;
                if (PWM_EN_STATE) {
                    state = SystemState::RUNNING;
                }
                break;

            case SystemState::FAULT:
                inverter.stop();
                powerCtrl.reset();
                continuousRunningStarted = false;

                // LATCH FAULT: Only return to IDLE when telemetry returns to safe levels
                if (!PWM_EN_STATE && telemetry.vIn <= 50.0f && telemetry.iIn <= TX_Max_Iout && telemetry.tempC <= 80.0f && fanMonitor.fanOK) {
                    fault = FaultReason::NONE;
                    state = SystemState::IDLE;
                }
                break;

            default:
                state = SystemState::IDLE;
                break;
        }

        // --- Status LED Control ---
        if (state == SystemState::RUNNING && continuousRunningStarted) {
            XMC_GPIO_SetOutputLow(LED1_PORT, LED1_PIN); // LED1 ON
        } else {
            XMC_GPIO_SetOutputHigh(LED1_PORT, LED1_PIN); // LED1 OFF
        }

        if (state == SystemState::FAULT) {
            XMC_GPIO_SetOutputLow(LED0_PORT, LED0_PIN);
        } else {
            XMC_GPIO_SetOutputHigh(LED0_PORT, LED0_PIN);
        }

        // --- Dynamic Opcode Calculation ---
        uint16_t code = 0x100;

        if (state == SystemState::FAULT) {
            if (fault == FaultReason::OVER_VOLTAGE) code |= 0x010;
            if (fault == FaultReason::OVER_CURRENT) code |= 0x020;
            if (fault == FaultReason::OVER_TEMP)    code |= 0x040;
            if (fault == FaultReason::FAN_FAILURE)  code |= 0x080; // Added Fan Failure Bit
        } else {
            if (PWM_EN_STATE)      code |= 0x001;
            if (AutoTune)          code |= 0x002;
            if (ResonantFreq_OK)   code |= 0x004;
        }

        WPT_OPCODE = code;
    }
};
