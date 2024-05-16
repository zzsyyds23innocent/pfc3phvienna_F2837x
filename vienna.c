//#############################################################################
//
// FILE:  vienna.c
//
// TITLE: This is the solution file.
//
//#############################################################################
// $TI Release: TIDM_1000 v4.01.05.00 $
// $Release Date: Thu Dec 14 13:12:56 CST 2023 $
// $Copyright:
// Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
//
// ALL RIGHTS RESERVED
// $
//#############################################################################

//
//********************* the includes ******************************************
//

#include "vienna.h"

//
//*********************** C2000 SYSTEM SW FRAMEWORK  **************************
//

//
//*********************** globals *********************************************
//

//
// Control Variables
//

#pragma SET_DATA_SECTION("controlVariables")

//
//--- System Related ---
// DCL Library, voltage compensator
// DC Bus set point
//
volatile float32_t VIENNA_vBusRef_pu;
volatile float32_t VIENNA_vBusRefSlewed_pu;

VIENNA_GV VIENNA_gv;

volatile float32_t VIENNA_gv_out, VIENNA_voltage_error_pu;
volatile uint16_t VIENNA_nonLinearVoltageLoopFlag;

//
// Current compensator
// Peak value of the Ii, Io current set point
//
volatile float32_t VIENNA_iLRef_pu, VIENNA_iL1Ref_pu, VIENNA_iL2Ref_pu, VIENNA_iL3Ref_pu;
volatile float32_t VIENNA_gi_out1, VIENNA_gi_out2, VIENNA_gi_out3;
volatile float32_t VIENNA_gi_gainKp;

//
// Balance controller
//
volatile float32_t VIENNA_gs_gainKp;
volatile float32_t VIENNA_gs_out;

volatile float32_t VIENNA_vBusDiff_pu, VIENNA_vBusZero_pu;

//
// Sine analyzer block for RMS Volt, Curr and Power measurements
//
POWER_MEAS_SINE_ANALYZER VIENNA_sine_mains1, VIENNA_sine_mains2,
                         VIENNA_sine_mains3;

//
// Measurement Variables
// Inductor Current Measurement
//
volatile float32_t VIENNA_iL1MeasADC_pu, VIENNA_iL2MeasADC_pu, VIENNA_iL3MeasADC_pu;
volatile float32_t VIENNA_iL1MeasSD_pu, VIENNA_iL2MeasSD_pu, VIENNA_iL3MeasSD_pu;
volatile float32_t VIENNA_iL1Meas_pu, VIENNA_iL2Meas_pu, VIENNA_iL3Meas_pu;

//
// Inductor Current Measurement Offset
//
volatile float32_t VIENNA_iL1MeasOffset_pu, VIENNA_iL2MeasOffset_pu,
                   VIENNA_iL3MeasOffset_pu;

//
// Output Voltage Bus measurement
//
volatile float32_t VIENNA_vBusMNMeas_pu, VIENNA_vBusPMMeas_pu, VIENNA_vBusMeas_pu,
                   VIENNA_vBusHalfMeas_pu;
volatile float32_t VIENNA_vBusMNMeasAvg_pu, VIENNA_vBusPMMeasAvg_pu,
                   VIENNA_vBusMeasAvg_pu;

//
// variables used for calibration of output voltage measurements
//
volatile float32_t VIENNA_m_VBusMNMeas_pu, VIENNA_b_VBusMNMeas_pu;
volatile float32_t VIENNA_m_VBusPMMeas_pu, VIENNA_b_VBusPMMeas_pu;

//
// Input Grid Voltage Measurement
//
volatile float32_t VIENNA_v1Meas_pu, VIENNA_v2Meas_pu, VIENNA_v3Meas_pu;
volatile float32_t VIENNA_v1MeasOffset_pu, VIENNA_v2MeasOffset_pu,
                   VIENNA_v3MeasOffset_pu;

volatile float32_t VIENNA_vRmsMeasAvg_pu;

//
// Display Values
//
volatile float32_t  VIENNA_guiVbusMN_Volts, VIENNA_guiVbusPM_Volts, VIENNA_guiVbus_Volts,
                VIENNA_guiV1_Volts, VIENNA_guiV2_Volts, VIENNA_guiV3_Volts,
                VIENNA_guiIL1_Amps, VIENNA_guiIL2_Amps, VIENNA_guiIL3_Amps,
                VIENNA_guiIL1sd_Amps, VIENNA_guiIL2sd_Amps, VIENNA_guiIL3sd_Amps;

volatile float32_t VIENNA_guiACFreq_Hz;
volatile float32_t VIENNA_guiPrms1_W, VIENNA_guiPrms2_W, VIENNA_guiPrms3_W,
                   VIENNA_guiPrmsTotal_W;
volatile float32_t VIENNA_guiIrms1_Amps, VIENNA_guiIrms2_Amps, VIENNA_guiIrms3_Amps;
volatile float32_t VIENNA_guiVrms1_Volts, VIENNA_guiVrms2_Volts, VIENNA_guiVrms3_Volts;
volatile float32_t VIENNA_guiPF1, VIENNA_guiPF2, VIENNA_guiPF3;
volatile float32_t VIENNA_guiVA1_VA, VIENNA_guiVA2_VA, VIENNA_guiVA3_VA;

float32_t VIENNA_guiVbusTripLimit_Volts;

uint16_t VIENNA_guiPowerStageStart;
uint16_t VIENNA_guiPowerStageStop;

//
// PFC Filtered DC bus measurement
//
volatile float32_t VIENNA_vBusAvg_pu;

volatile float32_t VIENNA_iL1_CalibrationGain = 1.0; //0.95999979;
volatile float32_t VIENNA_iL2_CalibrationGain = 1.0;
volatile float32_t VIENNA_iL3_CalibrationGain = 1.0; // 0.9850000;

//
// variables for third harmonic injection
//
volatile float32_t VIENNA_vMin_pu, VIENNA_vMax_pu;
volatile float32_t VIENNA_thirdHarmonicInjection;

//
// individual duty cycles for each phase
//
volatile float32_t VIENNA_duty1PU, VIENNA_duty2PU, VIENNA_duty3PU;
volatile float32_t VIENNA_dutyPU_DC;

//
// Flags for clearing trips and closing the loops
//
int16_t VIENNA_closeGiLoop, VIENNA_closeGvLoop, VIENNA_closeGsLoop,
        VIENNA_clearTrip, VIENNA_firstTimeGvLoop;

volatile int VIENNA_updateCoeff = 0;

volatile int16_t VIENNA_thirdHarmonicInjectionEnable = 1;

volatile float32_t VIENNA_iL1Ref_prev_pu, VIENNA_iL2Ref_prev_pu, VIENNA_iL3Ref_prev_pu;
volatile float32_t VIENNA_inductor_voltage_drop_feedforward1,
        VIENNA_inductor_voltage_drop_feedforward2,
        VIENNA_inductor_voltage_drop_feedforward3;

volatile uint16_t VIENNA_busVoltageSlew_pu = 0;

//
//-----------------------------------------------------------------------------
// Enum for build level of software and board status
//
enum VIENNA_BuildLevel_enum VIENNA_buildInfo = BuildLevel_1_OpenLoop ;

enum VIENNA_boardState_enum VIENNA_boardState = PowerStageOFF;

enum VIENNA_boardStatus_enum VIENNA_boardStatus = boardStatus_Idle;


#pragma SET_DATA_SECTION()

//
// datalogger
//
DLOG_4CH VIENNA_dLog1;
float32_t VIENNA_dBuff1[100], VIENNA_dBuff2[100], VIENNA_dBuff3[100],
          VIENNA_dBuff4[100];
float32_t VIENNA_dVal1, VIENNA_dVal2, VIENNA_dVal3, VIENNA_dVal4;

//
//--- SFRA Related Variables ----
//
float32_t VIENNA_plantMagVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_plantPhaseVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_olMagVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_olPhaseVect[VIENNA_SFRA_FREQ_LENGTH];
float32_t VIENNA_freqVect[VIENNA_SFRA_FREQ_LENGTH];

SFRA_F32 VIENNA_sfra1;

//
// Variables used to calibrate measurement offsets
//Offset filter coefficient K1: 0.05/(T+0.05);
//
float32_t VIENNA_k1 = 0.998;

//
//Offset filter coefficient K2: T/(T+0.05)
//
float32_t VIENNA_k2 = 0.001999;
int16_t VIENNA_offsetCalCounter;
float32_t VIENNA_offset165;

//
// Stand Alone Flash Image Instrumentation
//
int16_t VIENNA_i;
int16_t VIENNA_timer1;

//
// updateBoardStatus()
//
void VIENNA_updateBoardStatus(void)
{
#if VIENNA_INCR_BUILD == 1
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_1_OpenLoop;
    #else
        VIENNA_buildInfo = BuildLevel_1_OpenLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 2
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_2_CurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_2_CurrentLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 3
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_3_VoltageAndCurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_3_VoltageAndCurrentLoop_CLA;
    #endif
#elif VIENNA_INCR_BUILD == 4
    #if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
        VIENNA_buildInfo = BuildLevel_4_BalanceVoltageAndCurrentLoop;
    #else
        VIENNA_buildInfo = BuildLevel_4_BalanceVoltageAndCurrentLoop_CLA;
    #endif
#endif
}

void VIENNA_runSFRABackGroundTasks(void)
{

    SFRA_F32_runBackgroundTask(&VIENNA_sfra1);
    SFRA_GUI_runSerialHostComms(&VIENNA_sfra1);

}

//
// setupSFRA
//
void VIENNA_setupSFRA(void)
{
    SFRA_F32_reset(&VIENNA_sfra1);
    SFRA_F32_config(&VIENNA_sfra1,
                    VIENNA_SFRA_ISR_FREQ_HZ,
                    VIENNA_SFRA_AMPLITUDE,
                    VIENNA_SFRA_FREQ_LENGTH,
                    VIENNA_SFRA_FREQ_START,
                    VIENNA_SFRA_FREQ_STEP_MULTIPLY,
                    VIENNA_plantMagVect,
                    VIENNA_plantPhaseVect,
                    VIENNA_olMagVect,
                    VIENNA_olPhaseVect,
                    NULL,
                    NULL,
                    VIENNA_freqVect,
                    1);

    SFRA_F32_resetFreqRespArray(&VIENNA_sfra1);

    SFRA_F32_initFreqArrayWithLogSteps(&VIENNA_sfra1,
                                       VIENNA_SFRA_FREQ_START,
                                       VIENNA_SFRA_FREQ_STEP_MULTIPLY);

    //
    //configures the SCI channel for communication with SFRA host GUI
    //to change SCI channel change #defines in the settings.h file
    //the GUI also changes a LED status, this can also be changed with #define
    //in the file pointed to above
    //
    SFRA_GUI_config(VIENNA_SFRA_GUI_SCI_BASE,
                    VIENNA_SCI_VBUS_CLK,
                    VIENNA_SFRA_GUI_SCI_BAUDRATE,
                    VIENNA_SFRA_GUI_SCIRX_GPIO,
                    VIENNA_SFRA_GUI_SCIRX_GPIO_PIN_CONFIG,
                    VIENNA_SFRA_GUI_SCITX_GPIO,
                    VIENNA_SFRA_GUI_SCITX_GPIO_PIN_CONFIG,
                    VIENNA_SFRA_GUI_LED_INDICATOR,
                    VIENNA_SFRA_GUI_LED_GPIO,
                    VIENNA_SFRA_GUI_LED_GPIO_PIN_CONFIG,
                    &VIENNA_sfra1,
                    1);

}
//
//===========================================================================
// No more.
//===========================================================================
//

//
// globalVariablesInit()
//
void VIENNA_globalVariablesInit(void)
{

    VIENNA_vBusRef_pu = 0;
    VIENNA_vBusRefSlewed_pu = 0;

    VIENNA_gv.Ki = VIENNA_GV_PI_KI;
    VIENNA_gv.Kp = VIENNA_GV_PI_KP;
    VIENNA_gv.Umax = VIENNA_GV_PI_MAX;
    VIENNA_gv.Umin = VIENNA_GV_PI_MIN;
    VIENNA_gv.i10 = 0;
    VIENNA_gv.i6 = 0;

    VIENNA_gv_out = 0;
    VIENNA_voltage_error_pu = 0;
    #if VIENNA_NON_LINEAR_VOLTAGE_LOOP == 1
        VIENNA_nonLinearVoltageLoopFlag = 1;
    #else
        VIENNA_nonLinearVoltageLoopFlag = 0;
    #endif

    VIENNA_gi_out1 = 0;
    VIENNA_gi_out2 = 0;
    VIENNA_gi_out3 = 0;
    VIENNA_gi_gainKp = VIENNA_GI_GAINKP;

    VIENNA_gs_gainKp = VIENNA_GS_GAINKP;

    //
    //sine analyzer initialization
    //
    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains1);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains1,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);

    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains2);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains2,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);

    POWER_MEAS_SINE_ANALYZER_reset(&VIENNA_sine_mains3);
    POWER_MEAS_SINE_ANALYZER_config(&VIENNA_sine_mains3,
                                    VIENNA_ISR_10KHZ_FREQUENCY_HZ,
                                    (float32_t)0.08,
                                    (float32_t)VIENNA_GRID_MAX_FREQ_HZ,
                                    (float32_t)VIENNA_GRID_MIN_FREQ_HZ);


    DLOG_4CH_reset(&VIENNA_dLog1);
    DLOG_4CH_config(&VIENNA_dLog1,
                    &VIENNA_dVal1,&VIENNA_dVal2,&VIENNA_dVal3,&VIENNA_dVal4,
                    VIENNA_dBuff1, VIENNA_dBuff2, VIENNA_dBuff3, VIENNA_dBuff4,
                    100, 0.05, 5);

    VIENNA_iL1MeasOffset_pu = 0.0f;
    VIENNA_iL2MeasOffset_pu = 0.0f;
    VIENNA_iL3MeasOffset_pu = 0.0f;

    VIENNA_vBusPMMeasAvg_pu = 0;
    VIENNA_vBusMNMeasAvg_pu = 0;
    VIENNA_vBusMeasAvg_pu = 0;

    VIENNA_m_VBusPMMeas_pu = 0.97495237f;
    VIENNA_b_VBusPMMeas_pu = 0.0002906f;

    VIENNA_m_VBusMNMeas_pu = 0.977087472f;
    VIENNA_b_VBusMNMeas_pu = 0.003299208f;

    VIENNA_offsetCalCounter = 0;

    VIENNA_guiPowerStageStart = 0;
    VIENNA_guiPowerStageStop = 0;

    VIENNA_vRmsMeasAvg_pu = 0.0f;
    VIENNA_vBusAvg_pu = 0;

    VIENNA_iL1Ref_pu = 0;
    VIENNA_iL2Ref_pu = 0;
    VIENNA_iL3Ref_pu = 0;
    VIENNA_iLRef_pu = 0.05f;

    VIENNA_closeGiLoop = 0;
    VIENNA_closeGsLoop = 0;
    VIENNA_closeGvLoop = 0;
    VIENNA_clearTrip = 0;

    VIENNA_duty1PU = 0.0f;
    VIENNA_duty2PU = 0.0f;
    VIENNA_duty3PU = 0.0f;

    VIENNA_dutyPU_DC = 0.5f;

    VIENNA_iL1Ref_prev_pu = 0;
    VIENNA_iL2Ref_prev_pu = 0;
    VIENNA_iL3Ref_prev_pu = 0;

    VIENNA_firstTimeGvLoop = 1;
    VIENNA_closeGvLoop = 0;
    VIENNA_closeGiLoop = 0;
    VIENNA_closeGsLoop = 0;
    VIENNA_vBusRef_pu = ((float32_t)VIENNA_VBUS_REF_SET_VOLTS /
                   (float32_t)VIENNA_V_MAX_SENSE_VOLTS );

    VIENNA_vBusZero_pu = 0.0f;

    VIENNA_guiVbusTripLimit_Volts = VIENNA_VBUS_TRIP_LIMIT_VOLTS;

}

//
// calibrateOffset()
//
void VIENNA_calibrateOffset()
{
    int16_t VIENNA_offsetCalCounter = 0;

    VIENNA_offsetCalCounter = 0;
    VIENNA_iL1MeasOffset_pu = 0;
    VIENNA_iL2MeasOffset_pu = 0;
    VIENNA_iL3MeasOffset_pu = 0;

    VIENNA_v1MeasOffset_pu = 0;
    VIENNA_v2MeasOffset_pu = 0;
    VIENNA_v3MeasOffset_pu = 0;

    VIENNA_gi_out1 = 0;

    while(VIENNA_offsetCalCounter < 25000)
    {
        if(VIENNA_HAL_getPWMInterruptFlag(
                           VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE) == 1)
        {
            if(VIENNA_offsetCalCounter > 1000)
            {
                //
                // offset of the inductor current sense
                //
                VIENNA_iL1MeasOffset_pu = VIENNA_k1 * (VIENNA_iL1MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IL1_FB_1 + VIENNA_IL1_FB_2 +
                                VIENNA_IL1_FB_3 + VIENNA_IL1_FB_4 )
                                     * 0.25 * VIENNA_ADC_PU_SCALE_FACTOR;
                VIENNA_iL2MeasOffset_pu = VIENNA_k1 * (VIENNA_iL2MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IL2_FB_1 + VIENNA_IL2_FB_2 +
                                VIENNA_IL2_FB_3 + VIENNA_IL2_FB_4)
                                   * 0.25 * VIENNA_ADC_PU_SCALE_FACTOR;
                VIENNA_iL3MeasOffset_pu = VIENNA_k1 * (VIENNA_iL3MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_IL3_FB_1 + VIENNA_IL3_FB_2 +
                                VIENNA_IL3_FB_3 + VIENNA_IL3_FB_4)
                                  * 0.25 * VIENNA_ADC_PU_SCALE_FACTOR;

                //
                // offset of the inductor current sense
                //
                VIENNA_v1MeasOffset_pu = VIENNA_k1 * (VIENNA_v1MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_V1_FB_1 + VIENNA_V1_FB_2 +
                                VIENNA_V1_FB_3 + VIENNA_V1_FB_4)
                                  * 0.25f * VIENNA_ADC_PU_SCALE_FACTOR;
                VIENNA_v2MeasOffset_pu = VIENNA_k1 * (VIENNA_v2MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_V2_FB_1 + VIENNA_V2_FB_2 +
                                VIENNA_V2_FB_3 + VIENNA_V2_FB_4)
                                  * 0.25f * VIENNA_ADC_PU_SCALE_FACTOR;
                VIENNA_v3MeasOffset_pu = VIENNA_k1 * (VIENNA_v3MeasOffset_pu) +
                        VIENNA_k2 * (VIENNA_V3_FB_1 + VIENNA_V3_FB_2 +
                                VIENNA_V3_FB_3 + VIENNA_V3_FB_4)
                                  * 0.25f * VIENNA_ADC_PU_SCALE_FACTOR;
            }
            VIENNA_HAL_clearPWMInterruptFlag(
                                   VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE);
            VIENNA_offsetCalCounter++;
        }
    }

    //
    // NO use using the PPB because of the over sampling
    //

}
