//#############################################################################
//
// FILE:   vienna_main.c
//
// TITLE: This is the main file for the solution, following is the
//         <solution>.c -> solution source file
//         <solution>.h -> solution header file
//         <solution>_settings.h -> powerSUITE generated settings
//         <solution>_hal.c -> solution hardware abstraction layer
//         <solution>_hal.h -> solution hardware abstraction layer header file
//         <solution>_clatask.cla -> cla task file
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
// the includes
//
#include "vienna.h"

//
//---  State Machine Related ---
//
int16_t vTimer0[4];         // Virtual Timers slaved off CPU Timers (A events)
int16_t vTimer1[4];         // Virtual Timers slaved off CPU Timers (B events)

//
// Variable declarations for state machine
//
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

//
// State Machine function prototypes
//------------------------------------
// Alpha states
//
void A0(void);  //state A0
void B0(void);  //state B0

//
// A branch states
//
void A1(void);  //state A1

//
// B branch states
//
void B1(void);  //state B1
void B2(void);  //state B2


void main(void)
{
    //
    // This routine sets up the basic device configuration such as
    // initializing PLL, copying code from FLASH to RAM,
    // this routine will also initialize the CPU timers that are used in
    // the background 1, 2 & 3) task for this system (CPU time)
    //

    VIENNA_HAL_setupDevice();

    //
    // Tasks State-machine init
    //
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    B_Task_Ptr = &B1;

    //
    // Stop all PWM mode clock
    //
    VIENNA_HAL_disablePWMClkCounting();

    //
    //sets up the PWMs for the VIENNA PFC
    //
    VIENNA_HAL_setupPWM(VIENNA_HIGH_FREQ_PWM1_BASE, VIENNA_HIGH_FREQ_PWM2_BASE,
                        VIENNA_HIGH_FREQ_PWM3_BASE, VIENNA_PFC3PH_PWM_PERIOD);

    //
    // power up ADC on the device
    //
    VIENNA_HAL_setupADC();

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_setupSDFM(VIENNA_PFC3PH_PWM_PERIOD,
                             VIENNA_PWM_CLK_IN_SDFM_OSR,
                             VIENNA_SD_CLK_COUNT, VIENNA_SDFM_OSR);
    #endif

    //
    //Profiling GPIO
    //
    VIENNA_HAL_setupProfilingGPIO();

    //
    //configure LED GPIO
    //
    VIENNA_HAL_setupLEDGPIO();

    //
    //initialize global variables
    //
    VIENNA_globalVariablesInit();

    //
    // Enable PWM Clocks
    //
    VIENNA_HAL_enablePWMClkCounting();

    //
    //setup PMW trigger for the ADC conversions
    //
    VIENNA_HAL_setupTriggerForADC(VIENNA_HIGH_FREQ_PWM1_BASE);

    #if VIENNA_SDFM_SENSING == 1
        VIENNA_HAL_enablePWMInterruptGeneration(
                                 VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE,
                                 (float32_t)VIENNA_PWM_CLK_IN_SDFM_OSR *
                                 (float32_t)1.5 );
    #else
        VIENNA_HAL_enablePWMInterruptGeneration(
                     VIENNA_C28x_ISR1_INTERRUPT_TRIG_PWM_BASE,
                     EPWM_getCounterCompareValue(VIENNA_HIGH_FREQ_PWM1_BASE,
                     EPWM_COUNTER_COMPARE_B) );
    #endif

    //
    // Offset Calibration Routine
    // #if VIENNA_SDFM_SENSING == 0
    //  VIENNA_calibrateOffset();
    // #endif
    //
    #if SENSING_OPTION == ADC_BASED_SENSING
        VIENNA_calibrateOffset();
    #endif

    //
    // setup protection and trips for the board
    //
    VIENNA_HAL_setupBoardProtection(VIENNA_HIGH_FREQ_PWM1_BASE,
                                    VIENNA_HIGH_FREQ_PWM2_BASE,
                                    VIENNA_HIGH_FREQ_PWM3_BASE,
                                    VIENNA_I_TRIP_LIMIT_AMPS, VIENNA_I_MAX_SENSE_AMPS);

    //
    // PWM were tripped low in the previous routine
    // safe to setup PWM pins
    //
    VIENNA_HAL_setPinsAsPWM();

    //
    // ISR Mapping
    //
    VIENNA_HAL_setupInterrupt();

    //
    // Setup SFRA
    //
    VIENNA_setupSFRA();

    //
    // IDLE loop. Just sit and loop forever, periodically will branch into
    // A0-A3, B0-B3, C0-C3 tasks
    // Frequency of this branching is set in setupDevice routine:
    //
    for(;;)
    {
        //
        // State machine entry & exit point
        //
        (*Alpha_State_Ptr)();   // jump to an Alpha state (A0,B0,...)

    }
} //END MAIN CODE

//
// ISRs are named by the priority
// ISR1 is the highest priority
// ISR2 has the next highest and so forth
//

//
// control ISR Code
//
#if VIENNA_CONTROL_RUNNING_ON == C28x_CORE
    interrupt void ISR1(void)
    {
        VIENNA_pfcControlCode();
        VIENNA_HAL_clearInterrupt(VIENNA_C28x_ISR1_INTERRUPT_PIE_GROUP_NO);
    }// control ISR Ends Here
#endif

//
// 10Khz ISR Code
//
#if VIENNA_INSTRUMENTATION_ISR_RUNNING_ON == C28x_CORE
    interrupt void ISR2(void)
    {
        VIENNA_instrumentationCode();
     }// 10kHz ISR Ends Here
#endif

//
//=============================================================================
//  STATE-MACHINE SEQUENCING AND SYNCRONIZATION FOR SLOW BACKGROUND TASKS
//=============================================================================
//
//
//--------------------------------- FRAME WORK --------------------------------
//
void A0(void)
{
    //
    // loop rate synchronizer for A-tasks
    //
    if(VIENNA_TASKA_CTR_OVFLW_STATUS == 1)
    {
        VIENNA_CLEAR_TASKA_CTR_OVFLW_FLAG;    // clear flag

        //
        // jump to an A Task (A1,A2,A3,...)
        //
        (*A_Task_Ptr)();

        vTimer0[0]++;           // virtual timer 0, instance 0 (spare)
    }
    Alpha_State_Ptr = &B0;      // Comment out to allow only A tasks
}

void B0(void)
{
    //
    // loop rate synchronizer for B-tasks
    //
    if(VIENNA_TASKB_CTR_OVFLW_STATUS  == 1)
    {
        VIENNA_CLEAR_TASKB_CTR_OVFLW_FLAG;                  // clear flag

        //
        // jump to a B Task (B1,B2,B3,...)
        //
        (*B_Task_Ptr)();

        vTimer1[0]++;           // virtual timer 1, instance 0 (spare)
    }

    //
    // Allow A state tasks
    //
    Alpha_State_Ptr = &A0;
}

//
//=============================================================================
//  A - TASKS (executed in every 1 msec)
//=============================================================================
//

void A1(void)
{
    VIENNA_runSFRABackGroundTasks();

    //
    //the next time CpuTimer0 'counter' reaches Period value go to A1
    //
    A_Task_Ptr = &A1;

}

//
//=============================================================================
//  B - TASKS (executed in every 5 msec)
//=============================================================================
//

void B1(void)
{

    VIENNA_updateBoardStatus();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B2
    //
    B_Task_Ptr = &B2;
}

void B2(void)
{
    VIENNA_HAL_toggleLED();

    //
    //the next time CpuTimer1 'counter' reaches Period value go to B1
    //
    B_Task_Ptr = &B1;

}

//
// No more.
//
