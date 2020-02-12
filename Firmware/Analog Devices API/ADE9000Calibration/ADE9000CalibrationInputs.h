/*Calibration Procedure*/
/*Setup ADE9000. Set the desired configuration registers in ADE9000API.h file.*/
/*Fill the user inputs, transfer function and conversion constants*/
/*Follow directions shown on serial monitor */

/*User Inputs*/
#define NOMINAL_INPUT_VOLTAGE 230       //Nominal Calibrating RMS Voltage on all phases (Vrms)
#define NOMINAL_INPUT_CURRENT 5         //Nominal Calibrating RMS Current on all phases (Irms)
#define INPUT_FREQUENCY 50              //Nominal Calibrating frequency. Used to calculate Omega for phase calibration.Ensure ACCMODE register is set to correct frequency
#define CALIBRATION_ANGLE_DEGREES 50	//Calibration Angle in Degrees.Used for phase calibration

/*Transfer function*/
/****************************************************************************************************************
									Current Transfer Function
*****************************************************************************************************************
It is the voltage at the ADC input pins per input current(A) (Volts/Amp)
E.g: For a current output current transformer with Turns Ratio of 2500:1. Burden resistor on board = 5.1*2=10.2 Ohm.
Current transfer function= 1/2500*Total Burden = 1/2500*10.2= 0.00408
****************************************************************************************************************/
/****************************************************************************************************************
									Voltage Transfer Function
*****************************************************************************************************************
It is the voltage at the ADC input pins per input Voltage(V)(Volts/Volts)
E.g. The defaul atteunation factor on board is 801. 
Voltage transfer function = 1/801= 0.001248 ~=0.00125
****************************************************************************************************************/                      						 
// #define CURRENT_TRANSFER_FUNCTION 0.00408   //The RMS voltage at the ADC input pins per input RMS current  (V/A).(2500:1-->0.00408 with default burden resistors)
#define CURRENT_TRANSFER_FUNCTION 0.007869987801519  // 0,007869987801519  
#define VOLTAGE_TRANSFER_FUNCTION 0.00100532823967 // 0,00100532823967
// R_Sens1 / (R_HSR + R_PTC + R_1 + R_2 + R_3 + R_Sens1) = 1/994,7
// #define VOLTAGE_TRANSFER_FUNCTION  0.00125 //The RMS voltage at the ADC input pins per input RMS voltage (V/V) 
/****************************************************************************************************************
									Constants: Do not change 
*****************************************************************************************************************/
#define F_DSP 8000                         //Signal update Rate
#define CALIBRATION_EGY_CFG 0xF011         //Latch after EGYRDY. Sample based accumulation. Read with reset disabled. Accumulation enabled
#define EGYACCTIME 0x1F3F                  //Accumulate for a total of 8000 (EGY_TIME+1) samples.
#define CALIBRATION_ACC_TIME 1             //if EGYACCTIME= 0x1F3F, Accumulation time is 1sec. Change this if EGYACCTIME is changed.

#define CAL_ANGLE_RADIANS(x) (x*3.14159/180)


/****************************************************************************************************************
									Conversion Constants
*****************************************************************************************************************
If calibration is done with the the calibration sketch, the following conversion constants apply:
If CURRENT_TRANSFER_FUNCTION or VOLTAGE_TRANSFER_FUNCTION are changed, update the conversion constants

CAL_IRMS_CC in uA/code 				= 10^6/(CURRENT_TRANSFER_FUNCTION*ADE9000_RMS_FULL_SCALE_CODES*SQRT(2)) 
10^6/(0.007869987801519*52702092*SQRT(2))
CAL_VRMS_CC in uV/code 				= 10^6/(VOLTAGE_TRANSFER_FUNCTION*ADE9000_RMS_FULL_SCALE_CODES*SQRT(2))
10^6/(0.00100532823967*52702092*SQRT(2))
CAL_POWER_CC in mW/code 			= 10^3/(CURRENT_TRANSFER_FUNCTION*VOLTAGE_TRANSFER_FUNCTION*ADE9000_WATT_FULL_SCALE_CODES*2)
10^3/(0.007869987801519*0.00100532823967*20694066*2)
CAL_ENERGY_CC in uWhr/xTHR_HI code 	= 10^6/(CURRENT_TRANSFER_FUNCTION*VOLTAGE_TRANSFER_FUNCTION*ADE9000_WATT_FULL_SCALE_CODES*2*8000*3600*2^-13)
10^6/(0.007869987801519*0.00100532823967*20694066*2*8000*3600*2^-13)

If conversion constants are set correctly, the register codes are converted to physical parameters as:
e.g Channel A Vrms = (AVRMS(register)*CAL_VRMS_CC/10^6)
	Channel A Active Power = (AWATT(register)*CAL_POWER_CC/10^3)	
*****************************************************************************************************************/
// #define CAL_IRMS_CC		3.28849					// (uA/code)
#define CAL_IRMS_CC		1.704837886728815					// (uA/code)
// #define CAL_VRMS_CC		10.73364				// (uV/code)	
#define CAL_VRMS_CC	  13.345942989254304				// (uV/code)	

// #define CAL_POWER_CC 	4.73755					// (mW/code) Applicable for Active, reactive and apparent power
#define CAL_POWER_CC 3.053811549166564
// #define CAL_ENERGY_CC	1.34757					// (uWhr/xTHR_HI code)Applicable for Active, reactive and apparent energy
#define CAL_ENERGY_CC 0.868639729540711539544579243892812716037727882453808590377



