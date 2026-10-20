#pragma once

#include <stdint.h>
#include <math.h>
#include "xmc_vadc.h"
#include "xmc_scu.h"

struct Telemetry {
    float vIn   = 0.0f;
    float iIn   = 0.0f;
    float iOut  = 0.0f;
    float vRef  = 0.0f;
    float tempC = 0.0f;
};

class AnalogADC {
private:
    float avgXFactor  = 0.0f;
    float avgVin      = 0.0f;
    uint8_t tempCount = 0;
    float lastTempC   = 25.0f;

    inline float calculateAverage(float newVal, float currentAvg, int count) {
        if (currentAvg == 0.0f) return newVal;
        return (currentAvg * (static_cast<float>(count) - 1.0f) + newVal) / static_cast<float>(count);
    }

public:
    void init() {
        XMC_SCU_CLOCK_UngatePeripheralClock(XMC_SCU_PERIPHERAL_CLOCK_VADC);
        XMC_VADC_GLOBAL_EnableModule();

        // 1. Global Module Config
        XMC_VADC_GLOBAL_CONFIG_t gConfig = {};
        gConfig.clock_config.globcfg = (3U << VADC_GLOBCFG_DIVA_Pos);
        XMC_VADC_GLOBAL_Init(VADC, &gConfig);

        // 2. Global ICLASS 0 Configuration (12-bit, Sample Time 2)
        XMC_VADC_GLOBAL_CLASS_t g_iclass = {};
        g_iclass.sample_time_std_conv     = 2U;
        g_iclass.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        XMC_VADC_GLOBAL_InputClassInit(VADC, g_iclass, XMC_VADC_GROUP_CONV_STD, 0U);

        // 3. Power on Groups 0 & 1
        XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
        XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

        // 4. Startup Calibration
        XMC_VADC_GLOBAL_StartupCalibration(VADC);
        uint32_t timeout = 100000U;
        while ((VADC->GLOBCFG & VADC_GLOBCFG_SUCAL_Msk) && --timeout);

        // 5. Initialize Group ICLASS 0 for Groups 0 & 1 (ADV Apps)
        XMC_VADC_GROUP_CLASS_t grp_iclass = {};
        grp_iclass.sample_time_std_conv     = 2U;
        grp_iclass.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        XMC_VADC_GROUP_InputClassInit(VADC_G0, grp_iclass, XMC_VADC_GROUP_CONV_STD, 0U);
        XMC_VADC_GROUP_InputClassInit(VADC_G1, grp_iclass, XMC_VADC_GROUP_CONV_STD, 0U);

        // 6. Base Channel & Result Configurations
        XMC_VADC_CHANNEL_CONFIG_t ch_cfg = {};
        ch_cfg.lower_boundary_select      = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0;
        ch_cfg.upper_boundary_select      = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0;
        ch_cfg.event_gen_criteria         = XMC_VADC_CHANNEL_EVGEN_NEVER;
        ch_cfg.alternate_reference        = XMC_VADC_CHANNEL_REF_INTREF;
        ch_cfg.result_alignment           = XMC_VADC_RESULT_ALIGN_RIGHT;
        ch_cfg.broken_wire_detect_channel = XMC_VADC_CHANNEL_BWDCH_VAGND;

        XMC_VADC_RESULT_CONFIG_t res_cfg = {};
        res_cfg.post_processing_mode      = XMC_VADC_DMM_REDUCTION_MODE;

        // --- Group 0 Channels ---
        // ADC_VREF (P2.6): Group 0 CH 0 -> G0_RES[7]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 7U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        ch_cfg.channel_priority  = 0U;
        XMC_VADC_GROUP_ChannelInit(VADC_G0, 0U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G0, 7U, &res_cfg);

        // ADC_ITRACK (P2.2): Group 0 CH 1 -> G0_RES[9], Aliased to CH7
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GROUP_CLASS0;
        ch_cfg.result_reg_number = 9U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_CH7;
        ch_cfg.channel_priority  = 1U;
        XMC_VADC_GROUP_ChannelInit(VADC_G0, 1U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G0, 9U, &res_cfg);

        // --- Group 1 Channels ---
        // ADC_IOUT (P2.7): Group 1 CH 0 -> G1_RES[10], Aliased to CH1
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GROUP_CLASS0;
        ch_cfg.result_reg_number = 10U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_CH1;
        ch_cfg.channel_priority  = 1U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 0U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 10U, &res_cfg);

        // ADC_IIN (P2.3): Group 1 CH 5 -> G1_RES[12]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 12U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        ch_cfg.channel_priority  = 0U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 5U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 12U, &res_cfg);

        // ADC_TEMP (P2.4): Group 1 CH 6 -> G1_RES[4]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 4U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        ch_cfg.channel_priority  = 0U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 6U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 4U, &res_cfg);

        // ADC_VIN (P2.5): Group 1 CH 7 -> G1_RES[11]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 11U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        ch_cfg.channel_priority  = 0U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 7U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 11U, &res_cfg);

        // =====================================================================
        // 7. Group 1 Local Scan (Hardware Triggered for ADC_IOUT)
        // =====================================================================
        XMC_VADC_SCAN_CONFIG_t g1_scan_config = {};
        g1_scan_config.conv_start_mode   = XMC_VADC_STARTMODE_CIR;
        g1_scan_config.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_3;
        g1_scan_config.trigger_signal    = XMC_VADC_REQ_TR_I;
        g1_scan_config.trigger_edge      = XMC_VADC_TRIGGER_EDGE_RISING;
        g1_scan_config.gate_signal       = XMC_VADC_REQ_GT_A;
        g1_scan_config.timer_mode        = 0U;
        g1_scan_config.external_trigger  = true;
        g1_scan_config.req_src_interrupt = false;
        g1_scan_config.enable_auto_scan  = false;
        g1_scan_config.load_mode         = XMC_VADC_SCAN_LOAD_COMBINE;

        XMC_VADC_GROUP_ScanInit(VADC_G1, &g1_scan_config);
        XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);
        XMC_VADC_GROUP_ScanAddMultipleChannels(VADC_G1, (1U << 0U)); // Channel 0 (Aliased to CH1)
        XMC_VADC_GROUP_ScanEnableArbitrationSlot(VADC_G1);

        // =====================================================================
        // 8. Background Scan Sequence (Continuous for ADC_VREF, IIN, TEMP, VIN)
        // =====================================================================
        XMC_VADC_BACKGROUND_CONFIG_t bg_config = {};
        bg_config.conv_start_mode   = XMC_VADC_STARTMODE_CIR;
        bg_config.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_1;
        bg_config.trigger_signal    = XMC_VADC_REQ_TR_A;
        bg_config.trigger_edge      = XMC_VADC_TRIGGER_EDGE_NONE;
        bg_config.gate_signal       = XMC_VADC_REQ_GT_A;
        bg_config.timer_mode        = 0U;
        bg_config.external_trigger  = false;
        bg_config.req_src_interrupt = 0U;
        bg_config.enable_auto_scan  = true;
        bg_config.load_mode         = XMC_VADC_SCAN_LOAD_OVERWRITE;

        XMC_VADC_GLOBAL_BackgroundInit(VADC, &bg_config);

        // Add telemetry channels to background sequence
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 0U, 0U); // ADC_VREF: G0 CH0
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 5U); // ADC_IIN:  G1 CH5
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 6U); // ADC_TEMP: G1 CH6
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 7U); // ADC_VIN:  G1 CH7

        XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
    }

    inline Telemetry process(uint16_t rawVin, uint16_t rawIin, uint16_t rawIout, uint16_t rawVref, uint16_t rawTemp, bool dip3State = false) {
        Telemetry data;

        // 1. xFactor = 2.5 / TempVref
        float xFactor = (rawVref > 0) ? (2.5f / static_cast<float>(rawVref)) : 0.001221f;
        avgXFactor = calculateAverage(xFactor, avgXFactor, 5);
        data.vRef = static_cast<float>(rawVref);

        // 2. Vin = TempVin * AvgxFactor / 0.0033
        float vin = static_cast<float>(rawVin) * avgXFactor / 0.0033f;
        if (vin == 0.0f) vin = 1.0f;
        avgVin = calculateAverage(vin, avgVin, 5);
        data.vIn = avgVin;

        // 3. Iin Calculation (DIP3 = 50 mV/A, Default = 40 mV/A)
        float iinSens = dip3State ? 0.05f : 0.04f;
        data.iIn = ((static_cast<float>(rawIin) * avgXFactor) - 0.5f) / iinSens;

        // 4. Iout Calculation (40 mV/A)
        data.iOut = ((static_cast<float>(rawIout) * avgXFactor) - 0.5f) / 0.04f;

        // 5. NTC Temperature Calculation
        if (tempCount < 10) {
            tempCount++;
            data.tempC = lastTempC;
        } else {
            tempCount = 0;
            if (rawTemp > 0 && rawTemp < 4095) {
                float rawF = static_cast<float>(rawTemp);
                float x = logf(rawF / (4095.0f - rawF));
                float tempData = (1.0f / 298.15f) + (1.0f / 3977.0f) * x;
                lastTempC = (1.0f / tempData) - 273.15f;
            } else {
                lastTempC = 25.0f;
            }
            data.tempC = lastTempC;
        }

        return data;
    }
};