let $solution = {
    kit                  : {name: "TIDM_1000", type: "vienna", device: "F2837x"},
    cpuSysClock          : 200,
    PWMSysClkFreq		 : 100,
    eCAPSysClkFreq		 : 200,
	adcMaxRange			 : 3.3,
    pwmSwitchingFreq     : {min : 20, max : 200, default : 100},
    vInSenseMaxDefault  : 13.3,
    vOutSenseMaxDefault : 9.76,
    iLMaxSensedDefault   : 5.5,
    iLTripLevelDefault   : 4.0,
    ADCSense_Fltr_Cuttoff : 23.417,

     /* No GUI configs, init here? */
    DC_AC                : 0,
    ACFreq               : 60.0,
};

exports = {
    $solution: $solution,
};