// HW equivalence: ATmega328P
//
// code based on 
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
//
// uses Martinez gimbal controller driver boards
// see: https://github.com/owhite/brushless_gimbal_motor/blob/master/docs/BGC.pdf
//
// these defines are relevant to those boards.

#include "PinChangeInt.h"


///////////////
// #define StudyFcCtrlNeutral
const int FcCtrlNeutral = 3000;
///////////////

/////////////////////////////////////////////////////////////////////////////////////
// used in calibration process for control control step vs. angle sensor PWM read-back
// You need to manually align the flywheel to its neutral position before power on, otherwise, control input will be 
// #define TEST_CTRL_VS_PWMREADING
// when we have no idea what range we are aiming to rotate, say a new HW limitation is added, we disable the BLDC control and 
// #define DISABLE_BLDC_CTRL
/////////////////////////////////////////////////////////////////////////////////////
#define CtrlMoveRange 830
#define CtrlHalfMoveRange (CtrlMoveRange/2)
#define RUN_2MOTOR // comment out if only one BLDC is controlled

#ifdef RUN_2MOTOR
	#define MOTOR_CNT 2
	// Since vehicle leans towards one side at rest, we want the flywheel to start with non-level position, 
	// so as to increase the total moveable range 
	const int iFlywheeStandbyPosPWM[MOTOR_CNT] = {550,-250};
	const float PWMreading2CtrlDeg[MOTOR_CNT][2] = {{-2.77095614,1786.03890},{-2.76817318,3257.36262}};
	const bool bIncreaseCtrl2ReachTerminalPos[MOTOR_CNT] = {false,true}; // false: decrease ctrl value to reach destination
#else
	#define MOTOR_CHOICE 1
	#define MOTOR_CNT 1
	const int iFlywheeStandbyPosPWM[MOTOR_CNT] = {300};
	const float PWMreading2CtrlDeg[MOTOR_CNT][2] = {{-2.73417398,1537.12381}};
	const bool bIncreaseCtrl2ReachTerminalPos[MOTOR_CNT] = {false};
#endif

#ifdef TEST_CTRL_VS_PWMREADING
    // when calibrating ctrl vs. PWMreading, we use open loop to ensure smooth rotation.
    // In the torque control mode operation, we need to use PWMreading to predict the required ctrl.
    // use this switch to chooose sensor calibration mode or running mode
    /////////////////////////////////////////////////////////////////////////////////////
    // #define CLOSE_LOOP_TORQUE_TEST
    /////////////////////////////////////////////////////////////////////////////////////
    // if a small lead deg produces consistent torque, it indicates a good estimate of ang position from PWM sensor
    #define CLOSE_LOOP_TORQUE_TEST_DEGREE 90
    
    bool SensorCalibInProgress = false;
    #define CalibrationStepInterval (uint32_t)2000
    #define StepSizeCalibration 30 // no matter which direction it rotates(defined by bIncreaseCtrl2ReachTerminalPos), put positive value here
	
	uint16_t CalibStepii = 0;
    #define DelayTicksInitialMoveOneSide 200 // we may need more time to see control value being sent to determine what iFlywheeStandbyPosPWM should be.
#else
    #define DelayTicksInitialMoveOneSide 400
#endif

//////////////

#if MOTOR_CNT != 2
#error fMotor0leadingCoeff needs updates
#endif

// [0,1]-[0,1]=>[-1,1]
// 1: motor0 reaches end-point while motor1 is at the beginning point. 
// 0: motor0 is in sync with motor1
// -1: motor0 is at the beginning point while motor1 reaches end-point
// End-point could be on left/right, depending on bIncreaseCtrl2ReachTerminalPos, and motor rotation direction when increase ctrl phase value
float fMotor0leadingCoeff = 0.0f;
float fIntegMotor0leading = 0.0f;

#define pwrOnCtrlValForCalibration 0 
//
#define PWM_A_MOTOR0 3    // ATMEGA328p DIGITAL_3, IC_PIN1
#define PWM_B_MOTOR0 5    // ATMEGA328p DIGITAL_5, IC_PIN9
#define PWM_C_MOTOR0 6    // ATMEGA328p DIGITAL_6, IC_PIN10
//
#define PWM_A_MOTOR1 9    // ATMEGA328p DIGITAL_9, IC_PIN13
#define PWM_B_MOTOR1 10   // ATMEGA328p DIGITAL_10, IC_PIN14
#define PWM_C_MOTOR1 11   // ATMEGA328p DIGITAL_11, IC_PIN15
//////////////

typedef enum{
  ANGLE_SENSOR0_PWM = 0,
  FC_CTRL_PWM = ANGLE_SENSOR0_PWM+MOTOR_CNT,
  PWM_INPUT_COUNT
} etPWMinputSrc;


// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#ifdef RUN_2MOTOR
#define ANGLE_SENSOR0_FLAG (1<<ANGLE_SENSOR0_PWM)
#define ANGLE_SENSOR1_FLAG (1<<ANGLE_SENSOR0_PWM+1)
#define ANGLE_SENSOR_FLAG (ANGLE_SENSOR0_FLAG|ANGLE_SENSOR1_FLAG)
#else
#define ANGLE_SENSOR_FLAG (1<<ANGLE_SENSOR0_PWM)
#endif
#define FC_CTRL_FLAG (1<<FC_CTRL_PWM)

int CtrlValCurrAngle[MOTOR_CNT] = {0}; // the derived ctrl value, which should move the BLDC to the angle currently measured by the angular position sensor   
int CurrCtrlVal[MOTOR_CNT] = {0}; // currently applied ctrl setting value for BLDC anglular control

volatile uint8_t bUpdateFlagsShared = 0;

volatile uint32_t prev_time[PWM_INPUT_COUNT] = {0}; volatile uint32_t TTprev_time = 0;
volatile uint16_t pwm_SharedHwR[PWM_INPUT_COUNT] = {0}; // unit:(1/2uS) HwR: Hardware Resolution: 1/8uS, due to TCCR0B = TCCR0B & 0b11111000 | 0x02. Since lowest 2 bit is constant 0, pwm_SharedHwR Resolution >> 2
volatile uint16_t pwm_value[PWM_INPUT_COUNT] = {0};     // unit:(1/2uS)

// PWM reading from timer contains quantization error and other noise, LPF the integer reading since the flywheel gimbal platform should never move too fast
volatile float fPWMangle[MOTOR_CNT] = {0.0f};

volatile bool resetReq = true;
volatile uint16_t angleSensorRdCnt = 0; // this is the count used to determine whether FC has been in the disarm status.
volatile uint16_t lastangleSensorRdCnt = 0; // if the current angleSensorRdCnt is far from lastangleSensorRdCnt, we assume user disarm the FC hence no PWM ctrl signal for a while

int16_t Fc_ctrl_cmd;
 
#define FC_CTRL_PWM_INPUT_PIN 16 // Max Dig Pin=13, A0 is 14, therefore A2 is 16 
// 14 == A0
// 15 == A1

#ifdef RUN_2MOTOR
#define ANGLE_SENSOR0_PWM_READING_PIN 14 
#define ANGLE_SENSOR1_PWM_READING_PIN 15 
uint8_t PWM_read_pin[PWM_INPUT_COUNT] = {ANGLE_SENSOR0_PWM_READING_PIN,ANGLE_SENSOR1_PWM_READING_PIN,FC_CTRL_PWM_INPUT_PIN};
#else
#define ANGLE_SENSOR_PWM_READING_PIN 14 // 14 == A0
uint8_t PWM_read_pin[PWM_INPUT_COUNT] = {ANGLE_SENSOR_PWM_READING_PIN,FC_CTRL_PWM_INPUT_PIN}; // unit is 0.5uS, since we decrease the timer0 prescale factor by 8(fiiner), and div the reading by 4. delay: 1/8ms
#endif
uint8_t interrupted_pin;

#define sineArraySize 360
const int pwmSin[sineArraySize] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166,
		      170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207,
		      211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240,
		      241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250,
		      250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255,
		      255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
		      255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250,
		      250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241,
		      240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246,
		      247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253,
		      253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255,
		      255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253,
		      253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246,
		      245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228,
		      225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189,
		      185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147,
		      143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105,
		      101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53,
		      49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15,
		      14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3,
		      2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2,
		      3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14,
		      15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8,
		      7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1,
		      1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8,
		      8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31,
		      35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82,
		      86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

const int phaseShift = sineArraySize / 3; 
int currentStepA = 0;
int currentStepB = currentStepA + phaseShift;
int currentStepC = currentStepB + phaseShift;

uint32_t loopindex = 0;

void CalcPwm(etPWMinputSrc PWMsrc)
{
    // if the pin is high, its a rising edge of the signal pulse, so lets record its value
    if(digitalRead(PWM_read_pin[PWMsrc]) == HIGH)
    { 
        prev_time[PWMsrc] = micros()/4;
    }
    else
    {
        // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
        // this gives use the time between the rising and falling edges i.e. the pulse duration.
        pwm_SharedHwR[PWMsrc] = (uint16_t)(micros()/4 - prev_time[PWMsrc]);
        // use set the throttle flag to indicate that a new throttle signal has been received
        bUpdateFlagsShared |= (1<<PWMsrc);
    }
}

void CalcFcCtrlPwm()
{
  if (angleSensorRdCnt - lastangleSensorRdCnt > 100)
  {
    resetReq = true;
  }
  lastangleSensorRdCnt = angleSensorRdCnt;
  CalcPwm(FC_CTRL_PWM);
}
 
void CalcAngleSensor0PwmReading() 
{
    angleSensorRdCnt ++;
    CalcPwm(ANGLE_SENSOR0_PWM);
}

void CalcAngleSensor1PwmReading() 
{
  CalcPwm(ANGLE_SENSOR0_PWM+1);
}

void setup() 
{
    Serial.begin(115200);
    initBGC();
    delay(1000);
    Serial.println("#Flywheel ctrl system online");
    pinMode(FC_CTRL_PWM_INPUT_PIN, INPUT); 
    digitalWrite(FC_CTRL_PWM_INPUT_PIN, HIGH);
    PCintPort::attachInterrupt(FC_CTRL_PWM_INPUT_PIN, CalcFcCtrlPwm, CHANGE);
    
    for (uint8_t ii = 0; ii < MOTOR_CNT; ii++)
    {
        pinMode(PWM_read_pin[ii], INPUT); 
        digitalWrite(PWM_read_pin[ii], HIGH);
        switch(ii)
        {
            case 0:
                PCintPort::attachInterrupt(PWM_read_pin[ii], CalcAngleSensor0PwmReading, CHANGE);
                break;
            case 1:
                PCintPort::attachInterrupt(PWM_read_pin[ii], CalcAngleSensor1PwmReading, CHANGE);
                break;
            default:
                Serial.print("skjfhgakjg");
                while(true);
                break;
        }
		#ifdef TEST_CTRL_VS_PWMREADING
			setMotor_iPosition(ii, pwrOnCtrlValForCalibration, 255);
		#endif
    }
    delay(2000);
    #ifdef CLOSE_LOOP_TORQUE_TEST
        resetReq = false;
    #else
        resetReq = true;
    #endif
}

void loop() 
{
    // create local variables to hold a local copies of the channel inputs
	// these are declared static so that thier values will be retained 
	// between calls to loop.
	//static uint16_t unThrottleIn;
	// local copy of update flags
	static uint8_t bUpdateFlags;

    resetProcedure();
    
	// check shared update flags to see if any channels have a new signal
	if(bUpdateFlagsShared)
	{
		noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

		// take a local copy of which channels were updated in case we need to use this in the rest of loop
		bUpdateFlags = bUpdateFlagsShared;
		
		// in the current code, the shared values are always populated
		// so we could copy them without testing the flags
		// however in the future this could change, so lets
		// only copy when the flags tell us we can.
		
		if(bUpdateFlags & ANGLE_SENSOR_FLAG)
		{
			for (uint8_t motorii = 0; motorii < MOTOR_CNT; motorii++)
			{
				if (bUpdateFlags & (1<<(ANGLE_SENSOR0_PWM+motorii)))
				{
					pwm_value[ANGLE_SENSOR0_PWM+motorii] = pwm_SharedHwR[ANGLE_SENSOR0_PWM+motorii];
				}
			}
		}
    
		if(bUpdateFlags & FC_CTRL_FLAG)
		{
		    pwm_value[FC_CTRL_PWM] = pwm_SharedHwR[FC_CTRL_PWM];
            //Serial.print(" Fc:");
            //Serial.println(pwm_value[FC_CTRL_PWM]);
		}
		
		// clear shared copy of updated flags as we have already taken the updates
		// we still have a local copy if we need to use it in bUpdateFlags
		bUpdateFlagsShared = 0;
		
		interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
		// as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
		// service routines own these and could update them at any time. During the update, the 
		// shared copies may contain junk. Luckily we have our local copies to work with :-)
	}
  
	// do any processing from here onwards
	// only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
	// variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by 
	// the interrupt routines and should not be used in loop

	// the following code provides simple pass through 
	// this is a good initial test, the Arduino will pass through
	// receiver input as if the Arduino is not there.
	// This should be used to confirm the circuit and power
	// before attempting any custom processing in a project.

	// we are checking to see if the channel value has changed, this is indicated  
	// by the flags. For the simple pass through we don't really need this check,
	// but for a more complex project where a new signal requires significant processing
	// this allows us to only calculate new values when we have new inputs, rather than
	// on every cycle.
	if(bUpdateFlags & ANGLE_SENSOR_FLAG)
	{
		for (uint8_t motorii = 0; motorii < MOTOR_CNT; motorii++)
		{
			if (bUpdateFlags & (1<<(ANGLE_SENSOR0_PWM+motorii)))
			{	  
                const float LPFcoeff = 0.02f;
                fPWMangle[motorii] = (1.0f-LPFcoeff)*fPWMangle[motorii] + LPFcoeff*(float)pwm_value[ANGLE_SENSOR0_PWM+motorii];
				CtrlValCurrAngle[motorii] = (int)(PWMreading2CtrlDeg[motorii][0]*fPWMangle[motorii]+PWMreading2CtrlDeg[motorii][1]);
                #if defined(TEST_CTRL_VS_PWMREADING) && !defined(CLOSE_LOOP_TORQUE_TEST)
                    if (SensorCalibInProgress)
                    {
                        Serial.print("m");
                        Serial.print(motorii);
                        Serial.print("R:");
                        Serial.print(pwm_value[ANGLE_SENSOR0_PWM+motorii]);
                        Serial.print("Ctrl:");
                        Serial.println(CurrCtrlVal[motorii]);
                    }
                #endif        
				fMotor0leadingCoeff = 
					(float)( (iFlywheeStandbyPosPWM[0] - CtrlValCurrAngle[0]) - (CtrlValCurrAngle[1] - iFlywheeStandbyPosPWM[1]) ) / (float)CtrlMoveRange;
                //Serial.print("fMotor0leadingCoeff:");
                //Serial.println(fMotor0leadingCoeff);
			}
        }
	}

    loopindex ++;
    #ifdef TEST_CTRL_VS_PWMREADING
        #ifdef CLOSE_LOOP_TORQUE_TEST
        	if(bUpdateFlags & ANGLE_SENSOR_FLAG) // update everytime we have new angle meas
        #else
        	bool allmotorReachOtherEnd = true;
        	if (loopindex%CalibrationStepInterval == 0)
        #endif
    #else
	        if(bUpdateFlags & FC_CTRL_FLAG) // when FC is not ARMED, no FC PWM ctrl pulse, therefore no BLDC torque produced when resting.
    #endif
        	{
        		for (uint8_t motorii = 0; motorii < MOTOR_CNT; motorii++)
        		{
        			#ifdef TEST_CTRL_VS_PWMREADING
        				#ifdef CLOSE_LOOP_TORQUE_TEST
        					CurrCtrlVal[motorii] = CtrlValCurrAngle[motorii] - CLOSE_LOOP_TORQUE_TEST_DEGREE; // rotate to the right
        				#else
        					int cval = iFlywheeStandbyPosPWM[motorii];
        					if (bIncreaseCtrl2ReachTerminalPos[motorii])
        					{
        						cval += CalibStepii*StepSizeCalibration;
        						if (cval > iFlywheeStandbyPosPWM[motorii]+CtrlMoveRange)
        							cval = iFlywheeStandbyPosPWM[motorii]+CtrlMoveRange;
        						else
        							allmotorReachOtherEnd = false;
        					}
        					else
        					{
        						cval -= CalibStepii*StepSizeCalibration;
        						if (cval < iFlywheeStandbyPosPWM[motorii]-CtrlMoveRange)
        							cval = iFlywheeStandbyPosPWM[motorii]-CtrlMoveRange;
        						else
        							allmotorReachOtherEnd = false;
        					}
        
        					/****************************************
        					 * open loop control, smooth rotation
        					 ***************************************/
        					CurrCtrlVal[motorii] = cval;
        					SensorCalibInProgress = true;
        				#endif // #ifdef CLOSE_LOOP_TORQUE_TEST
        			#else // #ifdef TEST_CTRL_VS_PWMREADING
                        if (motorii == 0)
						    Fc_ctrl_cmd = (int16_t)pwm_value[FC_CTRL_PWM];
                           
                        #ifdef StudyFcCtrlNeutral
                            Serial.print("C:");
                            Serial.println(Fc_ctrl_cmd);
                        #endif
						
						// FcCmd > 0: rotate towards left(view from tail to head direction) if FcCmd is added to the current BLDC ctrl value
                        // to keep both flywheels moving in quansi-sync mode, we adjust slightly the rotation control cmd for flywheel 0
						int16_t FcCmd;
						if (bIncreaseCtrl2ReachTerminalPos[motorii])
                        {
							FcCmd = (FcCtrlNeutral-Fc_ctrl_cmd)/10;
                        }
						else
                        {
							FcCmd = (Fc_ctrl_cmd-FcCtrlNeutral)/10;
                        }

                        #define SyncPID_P 8.0f
                        #define SyncPID_I 0.4f
                        fIntegMotor0leading += fMotor0leadingCoeff;
                        const float fIntegMotor0leadingLim = 20.0f;
                        if (fIntegMotor0leading > fIntegMotor0leadingLim)
                            fIntegMotor0leading = fIntegMotor0leadingLim;
                        else if (fIntegMotor0leading < -fIntegMotor0leadingLim)
                            fIntegMotor0leading = -fIntegMotor0leadingLim;
                            
                        int16_t SyncUpdateCmd = (int16_t)(SyncPID_P*fMotor0leadingCoeff+SyncPID_I*fIntegMotor0leading);
                        const int16_t SyncUpdateCmdLim = 10;
                        if (SyncUpdateCmd > SyncUpdateCmdLim)
                            SyncUpdateCmd = SyncUpdateCmdLim;
                        else if (SyncUpdateCmd < -SyncUpdateCmdLim)
                            SyncUpdateCmd = -SyncUpdateCmdLim;
                        
                        if (loopindex % 10 == 0)
                        {
                            Serial.print("Fc:");
                            Serial.print(Fc_ctrl_cmd);
                            Serial.print("motor:");
                            Serial.print(motorii);
                            Serial.print(" M0lead:");
                            Serial.print(fMotor0leadingCoeff);
                            Serial.print(" fIntegMotor0leading:");
                            Serial.print(fIntegMotor0leading);
                            Serial.print(" Sync:");
                            Serial.print(SyncUpdateCmd);
                            Serial.print(" old:");
                            Serial.print(FcCmd);
                        }
                        FcCmd += SyncUpdateCmd;
                        if (loopindex % 10 == 0)
                        {
                            Serial.print(" New:");
                            Serial.println(FcCmd);  
                        }
					
						const int16_t ElectricalPhase = 90;
						int boundedFcCmd = ((FcCmd>ElectricalPhase)?ElectricalPhase:(FcCmd<-ElectricalPhase)?-ElectricalPhase:FcCmd);
						CurrCtrlVal[motorii] = CtrlValCurrAngle[motorii] + boundedFcCmd; // 3034*0.5uS=1517uS, joystick center location. 	Full range: 2000 to 4000
        			#endif // #ifdef TEST_CTRL_VS_PWMREADING
        			setMotor_iPosition(motorii, CurrCtrlVal[motorii], 255);
        		} // for motorii
                #if defined(TEST_CTRL_VS_PWMREADING) && !defined(CLOSE_LOOP_TORQUE_TEST)
                    if (!allmotorReachOtherEnd)
                        CalibStepii ++;
                #endif
        	}
        	else
        	{
                #if defined(TEST_CTRL_VS_PWMREADING) && !defined(CLOSE_LOOP_TORQUE_TEST)
            		allmotorReachOtherEnd = false;
                #endif
        	}
            #if defined(TEST_CTRL_VS_PWMREADING) && !defined(CLOSE_LOOP_TORQUE_TEST)
                if (allmotorReachOtherEnd)
                {
                    SensorCalibInProgress = false;
                }
            #endif   

	bUpdateFlags = 0;  
}

// float fposition: -1=>1, standby position to terminal position, NOT left to right direction.
/*void setMotor_fPosition(int motor, float fposition, int power) 
{
	int iPosition = 0;
	if (bIncreaseCtrl2ReachTerminalPos[motor])
		iPosition = iCtrlMidPoint[motor]-iCtrlHalfLeft2RightRange[motor]*fposition;
	else
		iPosition = iCtrlMidPoint[motor]+iCtrlHalfLeft2RightRange[motor]*fposition;
		
	setMotor_iPosition(motor, iPosition, power);
}*/

void setMotor_iPosition(int motor, int position, int power) 
{
#ifndef DISABLE_BLDC_CTRL
    // power: the duty cycle: between 0 (always off) and 255 (always on). https://www.arduino.cc/en/Reference/AnalogWrite
    if (position >= 0)
    {
        position = position%360;
    }
    else
    {
        position = -position;
        position %= 360;
        position = 360 - position;
    }
    
    int pin1, pin2, pin3;
    int pwm_a, pwm_b, pwm_c;
    
    power = constrain(power, 0, 255); // if only it were that easy
    
    if (motor == 0) {
        pin1 = PWM_A_MOTOR0;
        pin2 = PWM_B_MOTOR0;
        pin3 = PWM_C_MOTOR0;
    }
    if (motor == 1) {
        pin1 = PWM_A_MOTOR1;
        pin2 = PWM_B_MOTOR1;
        pin3 = PWM_C_MOTOR1;
    }
    
    // get number from the sin table, change amplitude from max
    pwm_a = (pwmSin[(position + currentStepA) % 360]) * (power / 255.0);
    pwm_b = (pwmSin[(position + currentStepB) % 360]) * (power / 255.0);
    pwm_c = (pwmSin[(position + currentStepC) % 360]) * (power / 255.0);
    
    analogWrite(pin1, pwm_a);
    analogWrite(pin2, pwm_b);
    analogWrite(pin3, pwm_c);
#endif
}

void initBGC() {
  // sets the speed of PWM signals. 
  // micros() uses Timer0, if modified, micros() return value won't be in uS.
  // ~1ms-2ms PWM, reading 8800-15500 
  // The Arduino uses Timer 0 internally for the millis() and delay() functions http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
  // https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR0B = TCCR0B & 0b11111000 | 0x02;   // pins 6 and 5 
  TCCR1B = TCCR1B & 0b11111000 | 0x01;   // pins 9 and 10
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // pins 11 and 3

  pinMode(PWM_A_MOTOR0, OUTPUT); 
  pinMode(PWM_B_MOTOR0, OUTPUT); 
  pinMode(PWM_C_MOTOR0, OUTPUT); 
  
  pinMode(PWM_A_MOTOR1, OUTPUT); 
  pinMode(PWM_B_MOTOR1, OUTPUT); 
  pinMode(PWM_C_MOTOR1, OUTPUT); 
}

void resetProcedure()
{
    if (resetReq)
    {
        resetReq = false;
        for (uint8_t motorii = 0; motorii < MOTOR_CNT; motorii++)
        {
            /****************************************
             * open loop control, smooth rotation towards standby location.
             ***************************************/
            int currAlignCtrlVal;
            int CtrlSteps2ReachStandbyPos;
            int CtrlGoal;
            #ifdef TEST_CTRL_VS_PWMREADING        
                currAlignCtrlVal = pwrOnCtrlValForCalibration;
                CtrlSteps2ReachStandbyPos = iFlywheeStandbyPosPWM[motorii]-pwrOnCtrlValForCalibration;
                CtrlGoal = iFlywheeStandbyPosPWM[motorii];
            #else
                currAlignCtrlVal = (int)pwm_SharedHwR[ANGLE_SENSOR0_PWM+motorii]*PWMreading2CtrlDeg[motorii][0]+PWMreading2CtrlDeg[motorii][1];
                CtrlSteps2ReachStandbyPos = (int)((iFlywheeStandbyPosPWM[motorii]-((int)pwm_SharedHwR[ANGLE_SENSOR0_PWM+motorii]*PWMreading2CtrlDeg[motorii][0]+PWMreading2CtrlDeg[motorii][1])));
                CtrlGoal = iFlywheeStandbyPosPWM[motorii];
            #endif
            /*Serial.print(pwm_SharedHwR[ANGLE_SENSOR0_PWM+motorii]);
            Serial.print(" ");
            Serial.print(PWMreading2CtrlDeg[motorii][0]);
            Serial.print(" ");
            Serial.print(CtrlDeg2ReachStandbyPos);*/
            while(true)
            {
                const int AlignStepCtrlVal = 10;
                if (CtrlSteps2ReachStandbyPos > 0)
                {
                    currAlignCtrlVal += AlignStepCtrlVal;
                    if (currAlignCtrlVal > CtrlGoal)
                    {
                        currAlignCtrlVal = CtrlGoal;
                    }
                }
                else
                {
                    currAlignCtrlVal -= AlignStepCtrlVal;
                    if (currAlignCtrlVal < CtrlGoal)
                    {
                        currAlignCtrlVal = CtrlGoal;
                    }
                }

                setMotor_iPosition(motorii, currAlignCtrlVal, 255);
                delay(DelayTicksInitialMoveOneSide);
                #ifdef TEST_CTRL_VS_PWMREADING
                    /*  This is used to set the appropriate value for iFlywheeStandbyPosPWM and CtrlMoveRange.
                     *  Do NOT remove!*/
                    #if false
                        Serial.print("#ctrl val for motor ");
                        Serial.print(motorii);
                        Serial.print(" is:");
                        Serial.println(currAlignCtrlVal);
                    #endif
                    
                #endif
                if (currAlignCtrlVal == CtrlGoal)
                    break;
            } // while
            fPWMangle[motorii] = (float)(pwm_SharedHwR[ANGLE_SENSOR0_PWM+motorii]);         
            Serial.print("#reset OP completes for motor ");
            Serial.println(motorii);
        }
    }    
}
