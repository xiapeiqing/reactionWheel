// code based on 
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
//
// uses Martinez gimbal controller driver boards
// see: https://github.com/owhite/brushless_gimbal_motor/blob/master/docs/BGC.pdf
//
// these defines are relevant to those boards.

#include "PinChangeInt.h"

//////////////
#define MOTOR_CHOICE 2 // motor 1 uses pin 3,5,6. Timer for pin5,6 is used by micros().
#define pwrOnCtrlVal 0 
//
#define PWM_A_MOTOR1 3    // ATMEGA328p DIGITAL_3, IC_PIN1
#define PWM_B_MOTOR1 5    // ATMEGA328p DIGITAL_5, IC_PIN9
#define PWM_C_MOTOR1 6    // ATMEGA328p DIGITAL_6, IC_PIN10
//
#define PWM_A_MOTOR2 9    // ATMEGA328p DIGITAL_9, IC_PIN13
#define PWM_B_MOTOR2 10   // ATMEGA328p DIGITAL_10, IC_PIN14
#define PWM_C_MOTOR2 11   // ATMEGA328p DIGITAL_11, IC_PIN15
//////////////

typedef enum{
  ANGLE_SENSOR_PWM = 0,
  FC_CTRL_PWM,
  PWM_INPUT_COUNT
} etPWMinputSrc;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define ANGLE_SENSOR_FLAG (1<<ANGLE_SENSOR_PWM)
#define FC_CTRL_FLAG (1<<FC_CTRL_PWM)

const int stepbegin = 210; // start
const int stepsize = -10;
const int target = -210;
int CtrlValCurrAngle = 0; // the ctrl value for the measured current angular position   
int CurrCtrlVal = 0; // currently applied ctrl setting value for BLDC anglular control
volatile uint8_t bUpdateFlagsShared = 0;

volatile uint32_t prev_time[PWM_INPUT_COUNT] = {0,0};
volatile uint16_t pwm_valueShared[PWM_INPUT_COUNT] = {0,0};
volatile uint16_t pwm_value[PWM_INPUT_COUNT] = {0,0};
volatile float fPWMangle = 0.0f;
volatile bool resetReq = true;
volatile uint16_t angleSensorRdCnt = 0; // this is the count used to determine whether FC has been in the disarm status.
volatile uint16_t lastangleSensorRdCnt = 0; // if the current angleSensorRdCnt is far from lastangleSensorRdCnt, we assume user disarm the FC hence no PWM ctrl signal for a while

#define ANGLE_SENSOR_PWM_READING_PIN 15 // 15 == A1
#define FC_CTRL_PWM_INPUT_PIN 16 // 16 == A2, Analog Pin 2, Max Dig Pin=13, A0 is 14, therefore A2 is 16 
uint8_t PWM_read_pin[PWM_INPUT_COUNT] = {ANGLE_SENSOR_PWM_READING_PIN,FC_CTRL_PWM_INPUT_PIN}; // unit is 0.5uS, since we decrease the timer0 prescale factor by 8(fiiner), and div the reading by 4. delay: 1/8ms
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

int phaseShift = sineArraySize / 3; 
int currentStepA = 0;
int currentStepB = currentStepA + phaseShift;
int currentStepC = currentStepB + phaseShift;

uint32_t loopindex = 0;

void CalcPwm(etPWMinputSrc PWMsrc)
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(PWM_read_pin[PWMsrc]) == HIGH)
  { 
    prev_time[PWMsrc] = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    pwm_valueShared[PWMsrc] = (uint16_t)(micros() - prev_time[PWMsrc]);
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
 
void CalcAngleSensorPwmReading() 
{
  angleSensorRdCnt ++;
  CalcPwm(ANGLE_SENSOR_PWM);
}

void setup() {
  Serial.begin(115200);
  initBGC();
  delay(8000);
  pinMode(FC_CTRL_PWM_INPUT_PIN, INPUT); 
  digitalWrite(FC_CTRL_PWM_INPUT_PIN, HIGH);
  PCintPort::attachInterrupt(FC_CTRL_PWM_INPUT_PIN, CalcFcCtrlPwm, CHANGE);
  
  pinMode(ANGLE_SENSOR_PWM_READING_PIN, INPUT); 
  digitalWrite(ANGLE_SENSOR_PWM_READING_PIN, HIGH);
  PCintPort::attachInterrupt(ANGLE_SENSOR_PWM_READING_PIN, CalcAngleSensorPwmReading, CHANGE);

  setMotorPosition(MOTOR_CHOICE, pwrOnCtrlVal, 255);
  delay(10000);
  resetReq = true;
}


void loop() 
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unAuxIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  if (resetReq)
  {
    resetReq = false;
    #define FLYWHEEL_STANDBY_POS 300 // 550 is vertical location. Since vehicle leans towards one side at rest, we want the flywheel to be non-level, so as to increase the moveable range 
    int AlignCtrlGoal = (int)(((int)pwm_valueShared[ANGLE_SENSOR_PWM]/4 - FLYWHEEL_STANDBY_POS)*(float)2.80419877);
    const int AlignStepCtrlVal = 10;
    int currAlignCtrlVal = pwrOnCtrlVal;
    while(true)
    {
      if (AlignCtrlGoal > 0)
      {
        currAlignCtrlVal += AlignStepCtrlVal;
        if (currAlignCtrlVal > AlignCtrlGoal)
        {
          currAlignCtrlVal = AlignCtrlGoal;
        }
      }
      else
      {
        currAlignCtrlVal -= AlignStepCtrlVal;
        if (currAlignCtrlVal < AlignCtrlGoal)
        {
          currAlignCtrlVal = AlignCtrlGoal;
        }
      }
      delay(200);
      setMotorPosition(MOTOR_CHOICE, currAlignCtrlVal, 255);
      if (currAlignCtrlVal == AlignCtrlGoal)
        break;
    }
    fPWMangle = (float)(pwm_valueShared[ANGLE_SENSOR_PWM])/4.0f;    
  }

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
      const float LPFcoeff = 0.02;
      uint16_t u16Angleval = pwm_valueShared[ANGLE_SENSOR_PWM]>>2; // 2: micro() function last 2 bit const 0
      pwm_value[ANGLE_SENSOR_PWM] = u16Angleval;
      fPWMangle = (1.0f-LPFcoeff)*fPWMangle + LPFcoeff*(float)u16Angleval;
      //Serial.print(pwm_valueShared[ANGLE_SENSOR_PWM]>>2);
      //Serial.print(" ");
      //Serial.println(pwm_value[ANGLE_SENSOR_PWM]);
    }
    
    if(bUpdateFlags & FC_CTRL_FLAG)
    {
      pwm_value[FC_CTRL_PWM] = pwm_valueShared[FC_CTRL_PWM]>>2;
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
    CtrlValCurrAngle = (int)(-2.80419877*(float)fPWMangle+1450.10954);
  }

#if false
  loopindex ++;
  if (loopindex%1000 == 0)
#else
  if(bUpdateFlags & FC_CTRL_FLAG) // when FC is not ARMED, no FC PWM ctrl pulse, therefore no BLDC torque produced when resting.
#endif
  {
    int16_t ctrl = (int16_t)pwm_value[FC_CTRL_PWM];
    const int FcCtrlNeutral = 3034;

#define ElectricalPhase 90
#if false // true false
    CurrCtrlVal = CtrlValCurrAngle - ElectricalPhase; // rotate to the right
#else
    int16_t FcCmd = (ctrl-FcCtrlNeutral)/20;
    CurrCtrlVal = CtrlValCurrAngle + ((FcCmd>ElectricalPhase)?ElectricalPhase:(FcCmd<-ElectricalPhase)?-ElectricalPhase:FcCmd); // 3034*0.5uS=1517uS, joystick center location. Full range: 2000 to 4000
#endif
    setMotorPosition(MOTOR_CHOICE, CurrCtrlVal, 255);
  }
      
  bUpdateFlags = 0;  
}

void setMotorPosition(int motor, int position, int power) {
  //Serial.println(position);
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
  
  //Serial.println(position);
  int pin1, pin2, pin3;
  int pwm_a, pwm_b, pwm_c;

  power = constrain(power, 0, 255); // if only it were that easy

  if (motor == 1) {
    pin1 = PWM_A_MOTOR1;
    pin2 = PWM_B_MOTOR1;
    pin3 = PWM_C_MOTOR1;
  }
  if (motor == 2) {
    pin1 = PWM_A_MOTOR2;
    pin2 = PWM_B_MOTOR2;
    pin3 = PWM_C_MOTOR2;
  }

  // get number from the sin table, change amplitude from max
  pwm_a = (pwmSin[(position + currentStepA) % 360]) * (power / 255.0);
  pwm_b = (pwmSin[(position + currentStepB) % 360]) * (power / 255.0);
  pwm_c = (pwmSin[(position + currentStepC) % 360]) * (power / 255.0);

  analogWrite(pin1, pwm_a);
  analogWrite(pin2, pwm_b);
  analogWrite(pin3, pwm_c);
}

void initBGC() {
  // sets the speed of PWM signals. 
  // micros() uses Timer0, if modified, micros() return value won't be in uS.
  // The Arduino uses Timer 0 internally for the millis() and delay() functions http://www.righto.com/2009/07/secrets-of-arduino-pwm.html
  // https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR0B = TCCR0B & 0b11111000 | 0x02; // pins 6 and 5 
  TCCR1B = TCCR1B & 0b11111000 | 0x01;   // pins 9 and 10
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   // pins 11 and 3

  pinMode(PWM_A_MOTOR1, OUTPUT); 
  pinMode(PWM_B_MOTOR1, OUTPUT); 
  pinMode(PWM_C_MOTOR1, OUTPUT); 
  
  pinMode(PWM_A_MOTOR2, OUTPUT); 
  pinMode(PWM_B_MOTOR2, OUTPUT); 
  pinMode(PWM_C_MOTOR2, OUTPUT); 
}

