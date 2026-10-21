#pragma once

#include <stdint.h>
#include <math.h>
#include "xmc_vadc.h"
#include "xmc_scu.h"
#include "xmc_gpio.h"

struct Telemetry {
    float vIn    = 0.0f;
    float iIn    = 0.0f;
    float iOut   = 0.0f;
    float iTrack = 0.0f;
    float vRef   = 0.0f;
    float tempC  = 0.0f;
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

        // 1. Disconnect Port 2 Digital Input Buffers (P2.2 through P2.7)
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 2U);
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 3U);
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 4U);
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 5U);
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 6U);
        XMC_GPIO_DisableDigitalInput(XMC_GPIO_PORT2, 7U);

        // 2. Global Module Config
        XMC_VADC_GLOBAL_CONFIG_t gConfig = {};
        XMC_VADC_GLOBAL_Init(VADC, &gConfig);

        XMC_VADC_GLOBAL_CLASS_t g_iclass = {};
        g_iclass.sample_time_std_conv     = 2U;
        g_iclass.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        XMC_VADC_GLOBAL_InputClassInit(VADC, g_iclass, XMC_VADC_GROUP_CONV_STD, 0U);

        // 3. Group 0 & Group 1 Initialization
        XMC_VADC_GROUP_CONFIG_t grp_cfg = {};
        grp_cfg.class0.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        grp_cfg.class0.sample_time_std_conv     = 0U;
        grp_cfg.class1.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        grp_cfg.class1.sample_time_std_conv     = 0U;
        grp_cfg.arbitration_round_length        = 0U;
        grp_cfg.arbiter_mode                    = XMC_VADC_GROUP_ARBMODE_ALWAYS;

        XMC_VADC_GROUP_Init(VADC_G0, &grp_cfg);
        XMC_VADC_GROUP_Init(VADC_G1, &grp_cfg);

        XMC_VADC_GROUP_CLASS_t grp_iclass = {};
        grp_iclass.sample_time_std_conv     = 2U;
        grp_iclass.conversion_mode_standard = XMC_VADC_CONVMODE_12BIT;
        XMC_VADC_GROUP_InputClassInit(VADC_G0, grp_iclass, XMC_VADC_GROUP_CONV_STD, 0U);
        XMC_VADC_GROUP_InputClassInit(VADC_G1, grp_iclass, XMC_VADC_GROUP_CONV_STD, 0U);

        // Power on Groups
        XMC_VADC_GROUP_SetPowerMode(VADC_G0, XMC_VADC_GROUP_POWERMODE_NORMAL);
        XMC_VADC_GROUP_SetPowerMode(VADC_G1, XMC_VADC_GROUP_POWERMODE_NORMAL);

        // Enable Post-Calibration and Run Startup Calibration
        XMC_VADC_GLOBAL_EnablePostCalibration(VADC, 0U);
        XMC_VADC_GLOBAL_EnablePostCalibration(VADC, 1U);
        XMC_VADC_GLOBAL_StartupCalibration(VADC);
        uint32_t timeout = 100000U;
        while ((VADC->GLOBCFG & VADC_GLOBCFG_SUCAL_Msk) && --timeout);

        // 4. Channel & Result Configurations
        XMC_VADC_CHANNEL_CONFIG_t ch_cfg = {};
        ch_cfg.lower_boundary_select      = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0;
        ch_cfg.upper_boundary_select      = XMC_VADC_CHANNEL_BOUNDARY_GROUP_BOUND0;
        ch_cfg.event_gen_criteria         = XMC_VADC_CHANNEL_EVGEN_NEVER;
        ch_cfg.alternate_reference        = XMC_VADC_CHANNEL_REF_INTREF;
        ch_cfg.result_alignment           = XMC_VADC_RESULT_ALIGN_RIGHT;
        ch_cfg.broken_wire_detect_channel = XMC_VADC_CHANNEL_BWDCH_VAGND;

        XMC_VADC_RESULT_CONFIG_t res_cfg = {};
        res_cfg.post_processing_mode      = XMC_VADC_DMM_REDUCTION_MODE;

        // ADC_VREF (P2.6): G0 CH0 -> G0_RES[7]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 7U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        ch_cfg.channel_priority  = 0U;
        XMC_VADC_GROUP_ChannelInit(VADC_G0, 0U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G0, 7U, &res_cfg);

        // ADC_ITRACK (P2.2): G0 CH1 -> G0_RES[9], Aliased to CH7
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GROUP_CLASS0;
        ch_cfg.result_reg_number = 9U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_CH7;
        ch_cfg.channel_priority  = 1U;
        XMC_VADC_GROUP_ChannelInit(VADC_G0, 1U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G0, 9U, &res_cfg);

        // ADC_IOUT (P2.7): G1 CH0 -> G1_RES[10], Aliased to CH1
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GROUP_CLASS0;
        ch_cfg.result_reg_number = 10U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_CH1;
        ch_cfg.channel_priority  = 1U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 0U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 10U, &res_cfg);

        // Reset priority for Background Scan Channels
        ch_cfg.channel_priority  = 0U;

        // ADC_IIN (P2.3): G1 CH5 -> G1_RES[12]
        ch_cfg.input_class       = XMC_VADC_CHANNEL_CONV_GLOBAL_CLASS0;
        ch_cfg.result_reg_number = 12U;
        ch_cfg.alias_channel     = XMC_VADC_CHANNEL_ALIAS_DISABLED;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 5U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 12U, &res_cfg);

        // ADC_TEMP (P2.4): G1 CH6 -> G1_RES[4]
        ch_cfg.result_reg_number = 4U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 6U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 4U, &res_cfg);

        // ADC_VIN (P2.5): G1 CH7 -> G1_RES[11]
        ch_cfg.result_reg_number = 11U;
        XMC_VADC_GROUP_ChannelInit(VADC_G1, 7U, &ch_cfg);
        XMC_VADC_GROUP_ResultInit(VADC_G1, 11U, &res_cfg);

        // 5. Group 0 Local Scan for ADC_ITRACK
        XMC_VADC_SCAN_CONFIG_t g0_scan_config = {};
        g0_scan_config.conv_start_mode   = XMC_VADC_STARTMODE_WFS;
        g0_scan_config.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_0;
        g0_scan_config.trigger_signal    = XMC_VADC_REQ_TR_A;
        g0_scan_config.gate_signal       = XMC_VADC_REQ_GT_A;
        g0_scan_config.load_mode         = XMC_VADC_SCAN_LOAD_COMBINE;

        XMC_VADC_GROUP_ScanInit(VADC_G0, &g0_scan_config);
        XMC_VADC_GROUP_ScanSetGatingMode(VADC_G0, XMC_VADC_GATEMODE_IGNORE);
        XMC_VADC_GROUP_ScanAddMultipleChannels(VADC_G0, (1U << 1U));
        XMC_VADC_GROUP_ScanEnableArbitrationSlot(VADC_G0);

        // 6. Group 1 Local Scan for ADC_IOUT
        XMC_VADC_SCAN_CONFIG_t g1_scan_config = {};
        g1_scan_config.conv_start_mode   = XMC_VADC_STARTMODE_CIR;
        g1_scan_config.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_3;
        g1_scan_config.trigger_signal    = XMC_VADC_REQ_TR_I;
        g1_scan_config.trigger_edge      = XMC_VADC_TRIGGER_EDGE_RISING;
        g1_scan_config.gate_signal       = XMC_VADC_REQ_GT_A;
        g1_scan_config.external_trigger  = true;
        g1_scan_config.load_mode         = XMC_VADC_SCAN_LOAD_COMBINE;

        XMC_VADC_GROUP_ScanInit(VADC_G1, &g1_scan_config);
        XMC_VADC_GROUP_ScanSetGatingMode(VADC_G1, XMC_VADC_GATEMODE_IGNORE);
        XMC_VADC_GROUP_ScanAddMultipleChannels(VADC_G1, (1U << 0U));
        XMC_VADC_GROUP_ScanEnableArbitrationSlot(VADC_G1);

        // Initial trigger for both scans
        XMC_VADC_GROUP_ScanTriggerConversion(VADC_G0);
        XMC_VADC_GROUP_ScanTriggerConversion(VADC_G1);

        // 7. Continuous Background Auto-Scan (ADC_VREF, IIN, TEMP, VIN)
        XMC_VADC_BACKGROUND_CONFIG_t bg_config = {};
        bg_config.conv_start_mode   = XMC_VADC_STARTMODE_CIR;
        bg_config.req_src_priority  = XMC_VADC_GROUP_RS_PRIORITY_1;
        bg_config.trigger_signal    = XMC_VADC_REQ_TR_A;
        bg_config.gate_signal       = XMC_VADC_REQ_GT_A;
        bg_config.enable_auto_scan  = true;
        bg_config.load_mode         = XMC_VADC_SCAN_LOAD_OVERWRITE;

        XMC_VADC_GLOBAL_BackgroundInit(VADC, &bg_config);
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 0U, 0U); // VREF: G0 CH0
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 5U); // IIN:  G1 CH5
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 6U); // TEMP: G1 CH6
        XMC_VADC_GLOBAL_BackgroundAddChannelToSequence(VADC, 1U, 7U); // VIN:  G1 CH7

        // Correct XMClib Group-level arbitration slot enables
        XMC_VADC_GROUP_BackgroundEnableArbitrationSlot(VADC_G0);
        XMC_VADC_GROUP_BackgroundEnableArbitrationSlot(VADC_G1);

        XMC_VADC_GLOBAL_BackgroundTriggerConversion(VADC);
    }

    inline Telemetry readAndProcess(bool dip3State = false) {
        uint16_t rawVref   = static_cast<uint16_t>(VADC_G0->RES[7]  & 0x0FFF);
        uint16_t rawItrack = static_cast<uint16_t>(VADC_G0->RES[9]  & 0x0FFF);
        uint16_t rawIin    = static_cast<uint16_t>(VADC_G1->RES[12] & 0x0FFF);
        uint16_t rawIout   = static_cast<uint16_t>(VADC_G1->RES[10] & 0x0FFF);
        uint16_t rawVin    = static_cast<uint16_t>(VADC_G1->RES[11] & 0x0FFF);
        uint16_t rawTemp   = static_cast<uint16_t>(VADC_G1->RES[4]  & 0x0FFF);

        // Keep local scans active when running in software polling
        XMC_VADC_GROUP_ScanTriggerConversion(VADC_G0);

        Telemetry data;

        float xFactor = (rawVref > 0) ? (2.5f / static_cast<float>(rawVref)) : 0.000806f;
        avgXFactor = calculateAverage(xFactor, avgXFactor, 5);
        data.vRef = static_cast<float>(rawVref);

        float vin = static_cast<float>(rawVin) * avgXFactor / 0.0033f;
        if (vin == 0.0f) vin = 1.0f;
        avgVin = calculateAverage(vin, avgVin, 5);
        data.vIn = avgVin;

        float iinSens = dip3State ? 0.05f : 0.04f;
        data.iIn = ((static_cast<float>(rawIin) * avgXFactor) - 0.5f) / iinSens;

        data.iOut   = ((static_cast<float>(rawIout) * avgXFactor) - 0.5f) / 0.04f;
        data.iTrack = static_cast<float>(rawItrack) * avgXFactor;

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