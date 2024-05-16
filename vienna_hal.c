//#############################################################################
//
// FILE:     vienna_hal.c
//
// TITLE:    solution hardware abstraction layer
//           This file consists of board related initialization
//           this file is used to make the
//           main file more readable
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

#include "vienna_hal.h"

//
// Device setup
//

//
// This routine sets up the basic device ocnfiguration such as initializing PLL
// copying code from FLASH to RAM
//

//
// device_setup()
//
void VIENNA_HAL_setupDevice(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // initialize CPU timers
    //

    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE,
                       DEVICE_SYSCLK_FREQ / VIENNA_CPU_TIMER0_FREQ_HZ );
    CPUTimer_setPeriod(CPUTIMER1_BASE,
                       DEVICE_SYSCLK_FREQ / VIENNA_CPU_TIMER1_FREQ_HZ );
    CPUTimer_setPeriod(CPUTIMER2_BASE,
                       DEVICE_SYSCLK_FREQ / VIENNA_CPU_TIMER2_FREQ_HZ );

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER2_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_stopTimer(CPUTIMER2_BASE);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_setEmulationMode(CPUTIMER2_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER2_BASE);

    CPUTimer_resumeTimer(CPUTIMER0_BASE);
    CPUTimer_resumeTimer(CPUTIMER1_BASE);
    CPUTimer_resumeTimer(CPUTIMER2_BASE);

}

//
// PWM Related
// configure3phViennaPWM()
//
void VIENNA_HAL_setupPWM(uint32_t base1, uint32_t base2, uint32_t base3,
                           uint16_t pwm_period_ticks)
{
    VIENNA_HAL_configurePWMUpDwnCnt(base1, pwm_period_ticks);
    VIENNA_HAL_configurePWMUpDwnCnt(base2, pwm_period_ticks);
    VIENNA_HAL_configurePWMUpDwnCnt(base3, pwm_period_ticks);

    //
    // configure PWM 1 as master and Phase 2, 3 as slaves and
    // let it pass the sync in pulse from PWM1
    //
    EPWM_disablePhaseShiftLoad(base1);

    EPWM_setSyncOutPulseMode(base1, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);


    EPWM_enablePhaseShiftLoad(base2);

    EPWM_setSyncOutPulseMode(base2, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);


    EPWM_setPhaseShift(base2, 2);
    EPWM_setCountModeAfterSync(base2, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    EPWM_enablePhaseShiftLoad(base3);


    EPWM_setSyncOutPulseMode(base3, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);


    EPWM_setPhaseShift(base3, 2);
    EPWM_setCountModeAfterSync(base3, EPWM_COUNT_MODE_UP_AFTER_SYNC);

}

//
// configurePWM1chUpDwnCnt()
//
void VIENNA_HAL_configurePWMUpDwnCnt(uint32_t base1, uint16_t pwm_period_ticks)
{
    //
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBasePeriod(base1, pwm_period_ticks >> 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);

    //
    // Counter Compare Submodule Registers
    // set duty 0% initially
    //
    EPWM_setCounterCompareValue(base1, EPWM_COUNTER_COMPARE_A,
                                pwm_period_ticks >> 1);
    EPWM_setCounterCompareShadowLoadMode(base1, EPWM_COUNTER_COMPARE_A,
                                       EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Action Qualifier SubModule Registers
    // CTR = CMPA@UP , set to 1
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // CTR = CMPA@Down , set to 0
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // CTR = 0 , set to 0
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW,
                                        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    //
    // CTR = CMPA@UP , set to 1
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH,
                                    EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // CTR = CMPA@Down , set to 0
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW,
                                        EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // CTR = 0 , set to 0
    //
    EPWM_setActionQualifierAction(base1, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW,
                                        EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

}

//
// setPinAsPWM()
//
void VIENNA_HAL_setPinsAsPWM()
{

    GPIO_writePin(VIENNA_HIGH_FREQ_PWM1_H_GPIO, 0);
    GPIO_writePin(VIENNA_HIGH_FREQ_PWM1_L_GPIO, 0);
    GPIO_writePin(VIENNA_HIGH_FREQ_PWM2_H_GPIO, 0);
    GPIO_writePin(VIENNA_HIGH_FREQ_PWM2_L_GPIO, 0);
    GPIO_writePin(VIENNA_HIGH_FREQ_PWM3_H_GPIO, 0);
    GPIO_writePin(VIENNA_HIGH_FREQ_PWM3_L_GPIO, 0);

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM1_H_GPIO , GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM1_H_GPIO , GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM1_H_GPIO_PIN_CONFIG );

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM1_L_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM1_L_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM1_L_GPIO_PIN_CONFIG );

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM2_H_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM2_H_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM2_H_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM2_L_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM2_L_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM2_L_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM3_H_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM3_H_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM3_H_GPIO_PIN_CONFIG);

    GPIO_setDirectionMode(VIENNA_HIGH_FREQ_PWM3_L_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(VIENNA_HIGH_FREQ_PWM3_L_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(VIENNA_HIGH_FREQ_PWM3_L_GPIO_PIN_CONFIG);
}

//
// disablePWMCLKCounting
//
void VIENNA_HAL_disablePWMClkCounting(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//
// enablePWMCLKCounting
//
void VIENNA_HAL_enablePWMClkCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//
// setupEPWMtoTriggerADCSOC(uint32_t base)
//
void VIENNA_HAL_setupTriggerForADC(uint32_t base)
{
    //
    // Select SOC from counter at ctr = CMPBD
    //
    EPWM_setADCTriggerSource(base, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPB );

    //
    // write value to CMPB
    //
    EPWM_setCounterCompareValue(base, EPWM_COUNTER_COMPARE_B,
           ((uint16_t)((float32_t)VIENNA_PFC3PH_PWM_PERIOD * (float32_t)0.5)
           - (uint16_t)( (float32_t)VIENNA_IL1_ACQPS_SYS_CLKS * (float32_t)6.0
     * (float32_t)(VIENNA_PWMSYSCLOCK_FREQ_HZ / (float32_t)VIENNA_CPU_SYS_CLOCK_FREQ_HZ) )
           - (uint16_t)( (float32_t) VIENNA_ADC_CONV_TIME
               * (float32_t) VIENNA_PWMSYSCLOCK_FREQ_HZ * (float32_t)6.0 ) ) );

    //
    // Generate pulse on 1st even
    //
    EPWM_setADCTriggerEventPrescale(base, EPWM_SOC_A, 1);

    //
    // Enable SOC on A group
    //
    EPWM_enableADCTrigger(base, EPWM_SOC_A);

}

//
// ADC Related
// setupADC()
//
void VIENNA_HAL_setupADC(void)
{


    //
    // set ADCCLK divider to /4
    //

    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);


    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // power up the ADC
    //
    ADC_enableConverter(ADCA_BASE);

    //
    //set ADCCLK divider to /4
    //

    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCB_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);


    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);

    //
    // power up the ADC
    //
    ADC_enableConverter(ADCB_BASE);

    //
    //set ADCCLK divider to /4
    //

    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCC_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);


    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    //
    // power up the ADC
    //
    ADC_enableConverter(ADCC_BASE);

    ADC_setPrescaler(ADCD_BASE, ADC_CLK_DIV_4_0); //set ADCCLK divider to /4
    ADC_setMode(ADCD_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCD_BASE, ADC_PULSE_END_OF_CONV);

    //
    //power up the ADC
    //
    ADC_enableConverter(ADCD_BASE);


    DEVICE_DELAY_US(1000);

    //
    // setup conversion on ADCA for inverter current and voltages
    // iL1Meas
    //
    ADC_setupSOC(VIENNA_IL1_ADC_MODULE, VIENNA_IL1_ADC_SOC_NO_1,
                 VIENNA_IL1_ADC_TRIG_SOURCE,
                 VIENNA_IL1_ADC_PIN, VIENNA_IL1_ACQPS_SYS_CLKS);

    //
    // iL2Meas
    //
    ADC_setupSOC(VIENNA_IL2_ADC_MODULE, VIENNA_IL2_ADC_SOC_NO_1,
                 VIENNA_IL2_ADC_TRIG_SOURCE,
                 VIENNA_IL2_ADC_PIN,  VIENNA_IL2_ACQPS_SYS_CLKS);

    //
    // iL3Meas
    //
    ADC_setupSOC(VIENNA_IL3_ADC_MODULE, VIENNA_IL3_ADC_SOC_NO_1,
                 VIENNA_IL3_ADC_TRIG_SOURCE,
                 VIENNA_IL3_ADC_PIN,  VIENNA_IL3_ACQPS_SYS_CLKS);

    //
    // iL1Meas2
    //
    ADC_setupSOC(VIENNA_IL1_ADC_MODULE, VIENNA_IL1_ADC_SOC_NO_2,
                 VIENNA_IL1_ADC_TRIG_SOURCE,
                 VIENNA_IL1_ADC_PIN,  VIENNA_IL1_ACQPS_SYS_CLKS);

    //
    // iL2Meas2
    //
    ADC_setupSOC(VIENNA_IL2_ADC_MODULE, VIENNA_IL2_ADC_SOC_NO_2,
                 VIENNA_IL2_ADC_TRIG_SOURCE,
                 VIENNA_IL2_ADC_PIN,  VIENNA_IL2_ACQPS_SYS_CLKS);

    //
    // iL3Meas2
    //
    ADC_setupSOC(VIENNA_IL3_ADC_MODULE, VIENNA_IL3_ADC_SOC_NO_2,
                 VIENNA_IL3_ADC_TRIG_SOURCE,
                 VIENNA_IL3_ADC_PIN,  VIENNA_IL3_ACQPS_SYS_CLKS);

    //
    // iL1Meas3
    //
    ADC_setupSOC(VIENNA_IL1_ADC_MODULE, VIENNA_IL1_ADC_SOC_NO_3,
                 VIENNA_IL1_ADC_TRIG_SOURCE,
                 VIENNA_IL1_ADC_PIN, VIENNA_IL1_ACQPS_SYS_CLKS);

    //
    // iL2Meas3
    //
    ADC_setupSOC(VIENNA_IL2_ADC_MODULE, VIENNA_IL2_ADC_SOC_NO_3,
                 VIENNA_IL2_ADC_TRIG_SOURCE,
                 VIENNA_IL2_ADC_PIN, VIENNA_IL2_ACQPS_SYS_CLKS);

    //
    // iL3Meas3
    //
    ADC_setupSOC(VIENNA_IL3_ADC_MODULE, VIENNA_IL3_ADC_SOC_NO_3,
                 VIENNA_IL3_ADC_TRIG_SOURCE,
                 VIENNA_IL3_ADC_PIN,  VIENNA_IL3_ACQPS_SYS_CLKS);

    //
    // iL1Meas4
    //
    ADC_setupSOC(VIENNA_IL1_ADC_MODULE, VIENNA_IL1_ADC_SOC_NO_4,
                 VIENNA_IL1_ADC_TRIG_SOURCE,
                 VIENNA_IL1_ADC_PIN, VIENNA_IL1_ACQPS_SYS_CLKS);

    //
    // iL2Meas4
    //
    ADC_setupSOC(VIENNA_IL2_ADC_MODULE, VIENNA_IL2_ADC_SOC_NO_4,
                 VIENNA_IL2_ADC_TRIG_SOURCE,
                 VIENNA_IL2_ADC_PIN, VIENNA_IL2_ACQPS_SYS_CLKS);

    //
    // iL3Meas4
    //
    ADC_setupSOC(VIENNA_IL3_ADC_MODULE, VIENNA_IL3_ADC_SOC_NO_4,
                 VIENNA_IL3_ADC_TRIG_SOURCE,
                 VIENNA_IL3_ADC_PIN, VIENNA_IL3_ACQPS_SYS_CLKS);

    //
    // v1Meas
    //
    ADC_setupSOC(VIENNA_V1_ADC_MODULE, VIENNA_V1_ADC_SOC_NO_1,
                 VIENNA_V1_ADC_TRIG_SOURCE,
                 VIENNA_V1_ADC_PIN, VIENNA_V1_ACQPS_SYS_CLKS);

    //
    // v2Meas
    //
    ADC_setupSOC(VIENNA_V2_ADC_MODULE, VIENNA_V2_ADC_SOC_NO_1,
                 VIENNA_V2_ADC_TRIG_SOURCE,
                 VIENNA_V2_ADC_PIN, VIENNA_V2_ACQPS_SYS_CLKS);

    //
    // v3Meas
    //
    ADC_setupSOC(VIENNA_V3_ADC_MODULE, VIENNA_V3_ADC_SOC_NO_1,
                 VIENNA_V3_ADC_TRIG_SOURCE,
                 VIENNA_V3_ADC_PIN, VIENNA_V3_ACQPS_SYS_CLKS);

    //
    // v1Meas2
    //
    ADC_setupSOC(VIENNA_V1_ADC_MODULE, VIENNA_V1_ADC_SOC_NO_2,
                 VIENNA_V1_ADC_TRIG_SOURCE,
                 VIENNA_V1_ADC_PIN, VIENNA_V1_ACQPS_SYS_CLKS);

    //
    // v2Meas2
    //
    ADC_setupSOC(VIENNA_V2_ADC_MODULE, VIENNA_V2_ADC_SOC_NO_2,
                 VIENNA_V2_ADC_TRIG_SOURCE,
                 VIENNA_V2_ADC_PIN, VIENNA_V2_ACQPS_SYS_CLKS);

    //
    // v3Meas2
    //
    ADC_setupSOC(VIENNA_V3_ADC_MODULE, VIENNA_V3_ADC_SOC_NO_2,
                 VIENNA_V3_ADC_TRIG_SOURCE,
                 VIENNA_V3_ADC_PIN, VIENNA_V3_ACQPS_SYS_CLKS);

    //
    // v1Meas3
    //
    ADC_setupSOC(VIENNA_V1_ADC_MODULE, VIENNA_V1_ADC_SOC_NO_3,
                 VIENNA_V1_ADC_TRIG_SOURCE,
                 VIENNA_V1_ADC_PIN, VIENNA_V1_ACQPS_SYS_CLKS);

    //
    // v2Meas3
    //
    ADC_setupSOC(VIENNA_V2_ADC_MODULE, VIENNA_V2_ADC_SOC_NO_3,
                 VIENNA_V2_ADC_TRIG_SOURCE,
                 VIENNA_V2_ADC_PIN, VIENNA_V2_ACQPS_SYS_CLKS);

    //
    // v3Meas3
    //
    ADC_setupSOC(VIENNA_V3_ADC_MODULE, VIENNA_V3_ADC_SOC_NO_3,
                 VIENNA_V3_ADC_TRIG_SOURCE,
                 VIENNA_V3_ADC_PIN, VIENNA_V3_ACQPS_SYS_CLKS);

    //
    // v1Meas4
    //
    ADC_setupSOC(VIENNA_V1_ADC_MODULE, VIENNA_V1_ADC_SOC_NO_4,
                 VIENNA_V1_ADC_TRIG_SOURCE,
                 VIENNA_V1_ADC_PIN, VIENNA_V1_ACQPS_SYS_CLKS);

    //
    // v2Meas4
    //
    ADC_setupSOC(VIENNA_V2_ADC_MODULE, VIENNA_V2_ADC_SOC_NO_4,
                 VIENNA_V2_ADC_TRIG_SOURCE,
                 VIENNA_V2_ADC_PIN, VIENNA_V2_ACQPS_SYS_CLKS);

    //
    // v3Meas4
    //
    ADC_setupSOC(VIENNA_V3_ADC_MODULE, VIENNA_V3_ADC_SOC_NO_4,
                 VIENNA_V3_ADC_TRIG_SOURCE,
                 VIENNA_V3_ADC_PIN, VIENNA_V3_ACQPS_SYS_CLKS);

    //
    // vBusMeas
    //
    ADC_setupSOC(VIENNA_VBUSPM_ADC_MODULE, VIENNA_VBUSPM_ADC_SOC_NO_1,
                 VIENNA_VBUSPM_ADC_TRIG_SOURCE, VIENNA_VBUSPM_ADC_PIN,
                 VIENNA_VBUSPM_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSPM_ADC_MODULE, VIENNA_VBUSPM_ADC_SOC_NO_2,
                 VIENNA_VBUSPM_ADC_TRIG_SOURCE, VIENNA_VBUSPM_ADC_PIN,
                 VIENNA_VBUSPM_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSPM_ADC_MODULE, VIENNA_VBUSPM_ADC_SOC_NO_3,
                 VIENNA_VBUSPM_ADC_TRIG_SOURCE, VIENNA_VBUSPM_ADC_PIN,
                 VIENNA_VBUSPM_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSPM_ADC_MODULE, VIENNA_VBUSPM_ADC_SOC_NO_4,
                 VIENNA_VBUSPM_ADC_TRIG_SOURCE, VIENNA_VBUSPM_ADC_PIN,
                 VIENNA_VBUSPM_ACQPS_SYS_CLKS);

    //
    // vBusMidMeas
    //
    ADC_setupSOC(VIENNA_VBUSMN_ADC_MODULE, VIENNA_VBUSMN_ADC_SOC_NO_1,
                 VIENNA_VBUSMN_ADC_TRIG_SOURCE, VIENNA_VBUSMN_ADC_PIN,
                 VIENNA_VBUSMN_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSMN_ADC_MODULE, VIENNA_VBUSMN_ADC_SOC_NO_2,
                 VIENNA_VBUSMN_ADC_TRIG_SOURCE, VIENNA_VBUSMN_ADC_PIN,
                 VIENNA_VBUSMN_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSMN_ADC_MODULE, VIENNA_VBUSMN_ADC_SOC_NO_3,
                 VIENNA_VBUSMN_ADC_TRIG_SOURCE, VIENNA_VBUSMN_ADC_PIN,
                 VIENNA_VBUSMN_ACQPS_SYS_CLKS);
    ADC_setupSOC(VIENNA_VBUSMN_ADC_MODULE, VIENNA_VBUSMN_ADC_SOC_NO_4,
                 VIENNA_VBUSMN_ADC_TRIG_SOURCE, VIENNA_VBUSMN_ADC_PIN,
                 VIENNA_VBUSMN_ACQPS_SYS_CLKS);
}

void VIENNA_HAL_setupCMPSS(uint32_t base1, float32_t current_limit,
                           float32_t current_max_sense )
{
    //
    //Enable CMPSS1
    //
    CMPSS_enableModule(base1);

    //
    //Use VDDA as the reference for comparator DACs
    //
    CMPSS_configDAC(base1,
                 CMPSS_DACVAL_SYSCLK | CMPSS_DACREF_VDDA | CMPSS_DACSRC_SHDW);

    CMPSS_setDACValueHigh(base1,
                          2048 +
                          (int16_t)((float32_t)current_limit
                                  * (float32_t)2048.0 /
                                  (float32_t)current_max_sense));
    CMPSS_setDACValueLow(base1,
                         2048 -
                         (int16_t)((float32_t)current_limit
                                 * (float32_t)2048.0 /
                                 (float32_t)current_max_sense));

    //
    // Make sure the asynchronous path compare high and low event
    // does not go to the OR gate with latched digital filter output
    // hence no additional parameter CMPSS_OR_ASYNC_OUT_W_FILT  is passed
    // comparator oputput is "not" inverted for high compare event
    //
    CMPSS_configHighComparator(base1, CMPSS_INSRC_DAC );

    //
    // Comparator output is inverted for for low compare event
    //
    CMPSS_configLowComparator(base1, CMPSS_INSRC_DAC | CMPSS_INV_INVERTED);

    CMPSS_configFilterHigh(base1, 2, 10, 7);
    CMPSS_configFilterLow(base1, 2, 10, 7);

    //
    //Reset filter logic & start filtering
    //
    CMPSS_initFilterHigh(base1);
    CMPSS_initFilterLow(base1);

    //
    // Configure CTRIPOUT path
    //
    CMPSS_configOutputsHigh(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);
    CMPSS_configOutputsLow(base1, CMPSS_TRIP_FILTER | CMPSS_TRIP_FILTER);

    //
    //Comparator hysteresis control , set to 2x typical value
    //
    CMPSS_setHysteresis(base1, 2);

    //
    // Clear the latched comparator events
    //
    CMPSS_clearFilterLatchHigh(base1);
    CMPSS_clearFilterLatchLow(base1);
}

//
// setupBoardProtection()
//
void VIENNA_HAL_setupBoardProtection(uint32_t base1, uint32_t base2,
                                     uint32_t base3,
                          float32_t current_limit, float32_t current_max_sense)
{

    //
    // configure TRIP5 for ADC comparator based trip
    //

    //
    // Disable all the muxes first
    //
    XBAR_enableEPWMMux(XBAR_TRIP5, 0x00);

    #if VIENNA_BOARD_PROT_IL1_CUR == 1


        VIENNA_HAL_setupCMPSS(VIENNA_BOARD_PROT_IL1_CUR_CMPSS_BASE,
                              current_limit,
                              current_max_sense );
        XBAR_setEPWMMuxConfig(XBAR_TRIP5,
                              VIENNA_BOARD_PROT_IL1_CUR_XBAR_MUX_VAL);
        XBAR_enableEPWMMux(XBAR_TRIP5,
                           VIENNA_BOARD_PROT_IL1_CUR_XBAR_MUX);
    #else
        #warning VIENNA_BOARD_PROT_IL1_CUR is disabled
    #endif

    #if VIENNA_BOARD_PROT_IL2_CUR == 1


        VIENNA_HAL_setupCMPSS(VIENNA_BOARD_PROT_IL2_CUR_CMPSS_BASE,
                              current_limit,
                              current_max_sense );
        XBAR_setEPWMMuxConfig(XBAR_TRIP5,
                              VIENNA_BOARD_PROT_IL2_CUR_XBAR_MUX_VAL);
        XBAR_enableEPWMMux(XBAR_TRIP5,
                           VIENNA_BOARD_PROT_IL2_CUR_XBAR_MUX);
    #else
        #warning VIENNA_BOARD_PROT_IL2_CUR is disabled
    #endif

    #if VIENNA_BOARD_PROT_IL3_CUR == 1


        VIENNA_HAL_setupCMPSS(VIENNA_BOARD_PROT_IL3_CUR_CMPSS_BASE,
                              current_limit,
                          current_max_sense );
        XBAR_setEPWMMuxConfig(XBAR_TRIP5,
                              VIENNA_BOARD_PROT_IL3_CUR_XBAR_MUX_VAL);
        XBAR_enableEPWMMux(XBAR_TRIP5,
                           VIENNA_BOARD_PROT_IL3_CUR_XBAR_MUX);
    #else
        #warning VIENNA_BOARD_PROT_IL3_CUR is disabled
    #endif


    //
    // SDFM Comparator OSR and filter were set in the SDFM setup routine
    // set the thresholds here
    //
    SDFM_setCompFilterHighThreshold(VIENNA_IL1_SDFM_BASE,
                                    VIENNA_IL1_SDFM_FILTER,
                                    VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 +
               ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                             (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );

    SDFM_setCompFilterLowThreshold(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                                   VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 -
               ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                             (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );


    SDFM_clearInterruptFlag(VIENNA_IL1_SDFM_BASE, 0x8000FFFF);

    SDFM_setCompFilterHighThreshold(VIENNA_IL2_SDFM_BASE,
                                    VIENNA_IL2_SDFM_FILTER,
                                    VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 +
               ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                             (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );

    SDFM_setCompFilterLowThreshold(VIENNA_IL2_SDFM_BASE,
                                   VIENNA_IL2_SDFM_FILTER,
                                   VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 -
              ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                            (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );


    SDFM_clearInterruptFlag(VIENNA_IL2_SDFM_BASE, 0x8000FFFF);

    SDFM_setCompFilterHighThreshold(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                                    VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 +
               ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                             (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );

    SDFM_setCompFilterLowThreshold(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                                   VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 -
              ( (uint16_t) ((float32_t)(current_limit / current_max_sense) *
                            (float32_t) VIENNA_SDFM_CMPSS_MAX_VAL_DIV_2 ) ) );


    SDFM_clearInterruptFlag(VIENNA_IL3_SDFM_BASE, 0x8000FFFF);

    XBAR_setEPWMMuxConfig(XBAR_TRIP4, VIENNA_IL1_SDFM_SENSE_XBAR_MUX_VAL);
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, VIENNA_IL2_SDFM_SENSE_XBAR_MUX_VAL);
    XBAR_setEPWMMuxConfig(XBAR_TRIP4, VIENNA_IL3_SDFM_SENSE_XBAR_MUX_VAL);

    XBAR_enableEPWMMux(XBAR_TRIP4, 0x00);
    XBAR_enableEPWMMux(XBAR_TRIP4, VIENNA_IL1_SDFM_SENSE_XBAR_MUX);
    XBAR_enableEPWMMux(XBAR_TRIP4, VIENNA_IL2_SDFM_SENSE_XBAR_MUX);
    XBAR_enableEPWMMux(XBAR_TRIP4, VIENNA_IL3_SDFM_SENSE_XBAR_MUX);

    VIENNA_HAL_setupPWMTrip(base1);
    VIENNA_HAL_setupPWMTrip(base2);
    VIENNA_HAL_setupPWMTrip(base3);

    VIENNA_HAL_clearCMPSSLatchFlags(VIENNA_BOARD_PROT_IL1_CUR_CMPSS_BASE);
    VIENNA_HAL_clearCMPSSLatchFlags(VIENNA_BOARD_PROT_IL2_CUR_CMPSS_BASE);
    VIENNA_HAL_clearCMPSSLatchFlags(VIENNA_BOARD_PROT_IL3_CUR_CMPSS_BASE);

}

//
// setupPWMTrip()
//
void VIENNA_HAL_setupPWMTrip(uint32_t base)
{
    //
    //Trip 5 is the input to the DCAHCOMPSEL
    //
    EPWM_selectDigitalCompareTripInput(base,
                                       EPWM_DC_TRIP_TRIPIN5,
                                       EPWM_DC_TYPE_DCAH);
    EPWM_setTripZoneDigitalCompareEventCondition(base,
                                                 EPWM_TZ_DC_OUTPUT_A1,
                                                 EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setDigitalCompareEventSource(base,
                                      EPWM_DC_MODULE_A,
                                      EPWM_DC_EVENT_1,
                                      EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSyncMode(base,
                                        EPWM_DC_MODULE_A ,
                                        EPWM_DC_EVENT_1,
                                        EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    //
    // Enable the following trips: DCAEVT1
    //
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCAEVT1);

    //
    //Trip 4 is the input to the DCBHCOMPSEL
    //
    EPWM_selectDigitalCompareTripInput(base,
                                       EPWM_DC_TRIP_TRIPIN4,
                                       EPWM_DC_TYPE_DCBH);
    EPWM_setTripZoneDigitalCompareEventCondition(base,
                                                 EPWM_TZ_DC_OUTPUT_B1,
                                                 EPWM_TZ_EVENT_DCXH_HIGH);
    EPWM_setDigitalCompareEventSource(base,
                                      EPWM_DC_MODULE_B,
                                      EPWM_DC_EVENT_1,
                                      EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);
    EPWM_setDigitalCompareEventSyncMode(base,
                                        EPWM_DC_MODULE_B ,
                                        EPWM_DC_EVENT_1,
                                        EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    //
    // Enable the following trips: DCBEVT1
    //
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_DCBEVT1);

    //
    // Enable the following trips Emulator Stop
    //
    EPWM_enableTripZoneSignals(base, EPWM_TZ_SIGNAL_CBC6);

    //
    // What do we want the OST / CBC events to do?
    // TZA events can force EPWMxA
    // TZB events can force EPWMxB
    //
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(base, EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);

    EPWM_clearTripZoneFlag(base,
                     (EPWM_TZ_INTERRUPT_OST | EPWM_TZ_INTERRUPT_DCAEVT1 |
                             EPWM_TZ_INTERRUPT_DCBEVT1));

    EPWM_forceTripZoneEvent(base, EPWM_TZ_FORCE_EVENT_OST);

}

//
// GPIO related
// setupProfilingGPIO()
//
void VIENNA_HAL_setupProfilingGPIO(void)
{
    GPIO_setDirectionMode(VIENNA_GPIO_PROFILING1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(VIENNA_GPIO_PROFILING2, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(VIENNA_GPIO_PROFILING1, GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(VIENNA_GPIO_PROFILING2, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(VIENNA_GPIO_PROFILING1_PIN_CONFIG);
    GPIO_setPinConfig(VIENNA_GPIO_PROFILING2_PIN_CONFIG);

#if VIENNA_CONTROL_RUNNING_ON == CLA_CORE
    GPIO_setMasterCore(VIENNA_GPIO_PROFILING1, GPIO_CORE_CPU1_CLA1);


#endif
}

//
// setupLEDGPIO()
//
void VIENNA_HAL_setupLEDGPIO(void)
{
    GPIO_setDirectionMode(VIENNA_GPIO_LED1, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(VIENNA_GPIO_LED2, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(VIENNA_GPIO_LED1, GPIO_QUAL_SYNC);
    GPIO_setQualificationMode(VIENNA_GPIO_LED2, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(VIENNA_GPIO_LED1_PIN_CONFIG);
    GPIO_setPinConfig(VIENNA_GPIO_LED2_PIN_CONFIG);
}

//
// toggleLED()
//
void VIENNA_HAL_toggleLED(void)
{
    static uint16_t ledCnt1 = 0;

    if(ledCnt1 == 0)
    {
        GPIO_togglePin(VIENNA_GPIO_LED1);
        ledCnt1 = 5;
    }
    else
    {
        ledCnt1--;
    }
}

//
// SDFM related
// setupSDFM()
//
void VIENNA_HAL_setupSDFM(uint16_t PWM_ticks, uint16_t PWM_ticks_in_sdfm_osr,
               uint16_t SD_clk_ecap_sys_ticks, uint16_t sdfmosr)
{

    //
    // setup CAP to generate CLK for SDFM
    //
    VIENNA_HAL_configureECAPforSDFMClk(SD_clk_ecap_sys_ticks);

    //
    // setup PWM 11 for syncing up the SD filter windows with motor control PWMs
    //
    VIENNA_HAL_configurePWMsyncforSDFM(PWM_ticks, PWM_ticks_in_sdfm_osr);


    //
    // Setup GPIO for SD
    //SD Filter iL1, Data
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL1_SDFM_DATA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL1_SDFM_DATA, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL1_SDFM_DATA, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL1_SDFM_DATA_PIN_CONFIG);

    //
    //SD Filter iL1, Clock
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL1_SDFM_CLOCK, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL1_SDFM_CLOCK, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL1_SDFM_CLOCK, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL1_SDFM_CLOCK_PIN_CONFIG);

    //
    //SD Filter iL2, Data
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL2_SDFM_DATA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL2_SDFM_DATA, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL2_SDFM_DATA, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL2_SDFM_DATA_PIN_CONFIG);

    //
    //SD Filter iL2, Clock
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL2_SDFM_CLOCK, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL2_SDFM_CLOCK, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL2_SDFM_CLOCK, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL2_SDFM_CLOCK_PIN_CONFIG);

    //
    //SD Filter iL3, Data
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL3_SDFM_DATA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL3_SDFM_DATA, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL3_SDFM_DATA, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL3_SDFM_DATA_PIN_CONFIG);

    //
    //SD Filter iL3, Clock
    //
    GPIO_setPadConfig(VIENNA_GPIO_IL3_SDFM_CLOCK, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(VIENNA_GPIO_IL3_SDFM_CLOCK, GPIO_QUAL_3SAMPLE);
    GPIO_setDirectionMode(VIENNA_GPIO_IL3_SDFM_CLOCK, GPIO_DIR_MODE_IN);
    GPIO_setPinConfig(VIENNA_GPIO_IL3_SDFM_CLOCK_PIN_CONFIG);

    //
    //******************* Input Control Module *******************************
    //

    //
    // Configure Input Control Mode:
    // Modulator Clock rate = Modulator data rate
    //
    SDFM_setupModulatorClock(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                             SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                             SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);
    SDFM_setupModulatorClock(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                             SDFM_MODULATOR_CLK_EQUAL_DATA_RATE);


    SDFM_enableFilter(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER);
    SDFM_enableFilter(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER);
    SDFM_enableFilter(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER);

    //
    //******************** Sinc filter Module ********************************
    //
    SDFM_setFilterType(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                       SDFM_FILTER_SINC_3);
    SDFM_setFilterType(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                       SDFM_FILTER_SINC_3);
    SDFM_setFilterType(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                       SDFM_FILTER_SINC_3);

    SDFM_setOutputDataFormat(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                             SDFM_DATA_FORMAT_32_BIT);
    SDFM_setOutputDataFormat(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                             SDFM_DATA_FORMAT_32_BIT);
    SDFM_setOutputDataFormat(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                             SDFM_DATA_FORMAT_32_BIT);

    SDFM_setFilterOverSamplingRatio(VIENNA_IL1_SDFM_BASE,
                                    VIENNA_IL1_SDFM_FILTER, sdfmosr - 1);
    SDFM_setFilterOverSamplingRatio(VIENNA_IL2_SDFM_BASE,
                                    VIENNA_IL2_SDFM_FILTER, sdfmosr - 1);
    SDFM_setFilterOverSamplingRatio(VIENNA_IL3_SDFM_BASE,
                                    VIENNA_IL3_SDFM_FILTER, sdfmosr - 1);

    //
    //******************* Comparator Module ********************************
    //
    SDFM_setComparatorFilterType(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                                 SDFM_FILTER_SINC_3);
    SDFM_setCompFilterOverSamplingRatio(VIENNA_IL1_SDFM_BASE,
                                        VIENNA_IL1_SDFM_FILTER,
                                        VIENNA_SDFM_CMPSS_OSR - 1);

    SDFM_setComparatorFilterType(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                                 SDFM_FILTER_SINC_3);
    SDFM_setCompFilterOverSamplingRatio(VIENNA_IL2_SDFM_BASE,
                                        VIENNA_IL2_SDFM_FILTER,
                                        VIENNA_SDFM_CMPSS_OSR - 1);

    SDFM_setComparatorFilterType(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                                 SDFM_FILTER_SINC_3);
    SDFM_setCompFilterOverSamplingRatio(VIENNA_IL3_SDFM_BASE,
                                        VIENNA_IL3_SDFM_FILTER,
                                        VIENNA_SDFM_CMPSS_OSR - 1);

    //
    //******************** EXTERNAL RESET *********************************
    //
    SDFM_enableExternalReset(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER);
    SDFM_enableExternalReset(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER);
    SDFM_enableExternalReset(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER);

    //
    //********************** Enable interrupts *****************************
    //
    SDFM_enableInterrupt(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                         SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT);
    SDFM_enableInterrupt(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                         SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT);
    SDFM_enableInterrupt(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                         SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT);

    SDFM_enableInterrupt(VIENNA_IL1_SDFM_BASE, VIENNA_IL1_SDFM_FILTER,
                         SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT);
    SDFM_enableInterrupt(VIENNA_IL2_SDFM_BASE, VIENNA_IL2_SDFM_FILTER,
                         SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT);
    SDFM_enableInterrupt(VIENNA_IL3_SDFM_BASE, VIENNA_IL3_SDFM_FILTER,
                         SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT);

    //
    // Enable master filter bit of the SDFM module 1
    //
    SDFM_enableMasterFilter(VIENNA_IL1_SDFM_BASE);
    SDFM_enableMasterInterrupt(VIENNA_IL1_SDFM_BASE);
    SDFM_enableMasterFilter(VIENNA_IL2_SDFM_BASE);
    SDFM_enableMasterInterrupt(VIENNA_IL2_SDFM_BASE);
    SDFM_enableMasterFilter(VIENNA_IL3_SDFM_BASE);
    SDFM_enableMasterInterrupt(VIENNA_IL3_SDFM_BASE);

}

//
// configureECAPforSDFMClk()
//
void VIENNA_HAL_configureECAPforSDFMClk(uint16_t SD_clk_ecap_sys_ticks)
{
    //
    // Use CAP as source for the SD Clock
    // OUTPUTXBAR1 -> GPIO24
    //

    //
    // Select ECAP O on Mux0
    //


    XBAR_setOutputMuxConfig(VIENNA_SDFM_CLK_OUTPUTXBAR,
                            VIENNA_SDFM_CLK_OUTPUTXBAR_MUX_CONFIG);

    //
    // Enable MUX0 for ECAP1.OUT
    //


    XBAR_enableOutputMux(VIENNA_SDFM_CLK_OUTPUTXBAR,
                         VIENNA_SDFM_CLK_OUTPUTXBAR_MUX);

    GPIO_setDirectionMode(VIENNA_SDFM_CLK_GPIO, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(VIENNA_SDFM_CLK_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setPinConfig(VIENNA_SDFM_CLK_PIN_CONFIG);

    //
    // Setup APWM mode on CAP peripheral, set period and compare registers
    //
    ECAP_enableAPWMMode(VIENNA_SDFM_CLK_SOURCE_BASE);

    //
    // Enable APWM mode
    //
    ECAP_setAPWMPeriod(VIENNA_SDFM_CLK_SOURCE_BASE,
                       SD_clk_ecap_sys_ticks - 1);
    ECAP_setAPWMCompare(VIENNA_SDFM_CLK_SOURCE_BASE,
                        SD_clk_ecap_sys_ticks >> 1);

    //
    // set next duty cycle to 50%
    //
    ECAP_setAPWMShadowPeriod(VIENNA_SDFM_CLK_SOURCE_BASE,
                             SD_clk_ecap_sys_ticks - 1);
    ECAP_setAPWMShadowCompare(VIENNA_SDFM_CLK_SOURCE_BASE,
                              SD_clk_ecap_sys_ticks >> 1);

    ECAP_clearInterrupt(VIENNA_SDFM_CLK_SOURCE_BASE, 0xFF);
    ECAP_clearGlobalInterrupt(VIENNA_SDFM_CLK_SOURCE_BASE);

    //
    // Start counters
    //
    ECAP_startCounter(VIENNA_SDFM_CLK_SOURCE_BASE);
}

//
// configurePWMsyncforSDFM()
//
void VIENNA_HAL_configurePWMsyncforSDFM(uint16_t PWM_ticks,
                                        uint16_t PWM_ticks_in_sdfm_osr)
{
    VIENNA_HAL_configurePWM1chUpCnt(EPWM11_BASE, PWM_ticks);

    //
    // EPWM1 SYNCOUT on ZRO is passed through EPWM2 and 3
    //
    EPWM_setSyncOutPulseMode(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
    EPWM_setSyncOutPulseMode(EPWM10_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);

    EPWM_setPhaseShift(EPWM11_BASE, 2);
    EPWM_enablePhaseShiftLoad(EPWM11_BASE);
    EPWM_setCountModeAfterSync(EPWM11_BASE, EPWM_COUNT_MODE_UP_AFTER_SYNC);

    //
    // CMPC and CMPD are used to send the SDFM Sync pulse to the filters
    //
    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_C,
                                ((EPWM_getTimeBasePeriod(EPWM11_BASE)) >> 1)
                                  - (PWM_ticks_in_sdfm_osr*1.5 ));

    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_D,
                                ((EPWM_getTimeBasePeriod(EPWM11_BASE)) >> 1)
                                - (PWM_ticks_in_sdfm_osr*1.5 ));

    //
    // CMPA is used to read the SDFM registers
    //
    EPWM_setCounterCompareValue(EPWM11_BASE, EPWM_COUNTER_COMPARE_A,
                                 ((EPWM_getTimeBasePeriod(EPWM11_BASE)) >> 1)
                                 + (PWM_ticks_in_sdfm_osr*1.5 +10));
}


//
// configurePWM1chUpCnt()
//
void VIENNA_HAL_configurePWM1chUpCnt(uint32_t base1, uint16_t period)
{
    //
    // Time Base SubModule Registers
    //
    EPWM_setPeriodLoadMode(base1, EPWM_PERIOD_DIRECT_LOAD);
    EPWM_setTimeBasePeriod(base1, period - 1);
    EPWM_setTimeBaseCounter(base1, 0);
    EPWM_setPhaseShift(base1, 0);
    EPWM_setTimeBaseCounterMode(base1, EPWM_COUNTER_MODE_UP);
    EPWM_setClockPrescaler(base1, EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

}




void VIENNA_HAL_setupCLA(void)
{
    //
    // setup CLA to register an interrupt
    //
#if VIENNA_CONTROL_RUNNING_ON == CLA_CORE

    memcpy((uint32_t *)&Cla1ProgRunStart, (uint32_t *)&Cla1ProgLoadStart,
            (uint32_t)&Cla1ProgLoadSize );

    //
    // first assign memory to CLA
    //
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS3, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMMASTER_CPU_CLA1);
    MemCfg_setLSRAMMasterSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMMASTER_CPU_CLA1);

    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS2, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS3, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4, MEMCFG_CLA_MEM_PROGRAM);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);

    //
    // Suppressing #770-D conversion from pointer to smaller integer
    // The CLA address range is 16 bits so the addresses passed to the MVECT
    // registers will be in the lower 64KW address space. Turn the warning
    // back on after the MVECTs are assigned addresses
    //
    #pragma diag_suppress = 770

    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_1, (uint16_t)&Cla1Task1);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_2, (uint16_t)&Cla1Task2);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_3, (uint16_t)&Cla1Task3);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_4, (uint16_t)&Cla1Task4);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_5, (uint16_t)&Cla1Task5);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_6, (uint16_t)&Cla1Task6);
    CLA_mapTaskVector(CLA1_BASE , CLA_MVECT_7, (uint16_t)&Cla1Task7);


    #pragma diag_warning = 770

    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_ALL);



    VIENNA_HAL_clearPWMInterruptFlag(VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE);
    CLA_forceTasks(CLA1_BASE, CLA_TASKFLAG_1);

    CLA_setTriggerSource(CLA_TASK_1, VIENNA_CLA_ISR1_TRIG);

#endif

}
