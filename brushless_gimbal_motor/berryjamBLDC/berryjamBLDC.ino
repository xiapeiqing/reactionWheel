//
// Slow and precise BLDC motor driver using SPWM and SVPWM modulation
// Part of code used from http://elabz.com/
// (c) 2015 Ignas Gramba www.berryjam.eu
//
 

const int EN1 = 5;
const int EN2 = 6;
const int EN3 = 7;

const int IN1 = 9;
const int IN2 = 10;
const int IN3 = 11;


#if true // true/false
// SPWM (Sine Wave)
const int pwmSin[] = {127,129,131,134,136,138,140,143,145,147,149,151,154,156,158,160,162,164,166,169,171,173,175,177,179,181,183,185,187,189,191,193,195,196,198,200,202,204,205,207,209,211,212,214,216,217,219,220,222,223,225,226,227,229,230,231,233,234,235,236,237,239,240,241,242,243,243,244,245,246,247,248,248,249,250,250,251,251,252,252,253,253,253,254,254,254,254,254,254,254,255,254,254,254,254,254,254,254,253,253,253,252,252,251,251,250,250,249,248,248,247,246,245,244,243,243,242,241,240,239,237,236,235,234,233,231,230,229,227,226,225,223,222,220,219,217,216,214,212,211,209,207,205,204,202,200,198,196,195,193,191,189,187,185,183,181,179,177,175,173,171,169,166,164,162,160,158,156,154,151,149,147,145,143,140,138,136,134,131,129,127,125,123,120,118,116,114,111,109,107,105,103,100,98,96,94,92,90,88,85,83,81,79,77,75,73,71,69,67,65,63,61,59,58,56,54,52,50,49,47,45,43,42,40,38,37,35,34,32,31,29,28,27,25,24,23,21,20,19,18,17,15,14,13,12,11,11,10,9,8,7,6,6,5,4,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,-1,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,4,5,6,6,7,8,9,10,11,11,12,13,14,15,17,18,19,20,21,23,24,25,27,28,29,31,32,34,35,37,38,40,42,43,45,47,49,50,52,54,56,58,59,61,63,65,67,69,71,73,75,77,79,81,83,85,88,90,92,94,96,98,100,103,105,107,109,111,114,116,118,120,123,125};
#else
/// SVPWM (Space Vector Wave)
const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
#endif


int currentStepA;
int currentStepB;
int currentStepC;
int sineArraySize;
int increment = 0;
boolean direct = true;//false; // direction true=forward, false=backward

//////////////////////////////////////////////////////////////////////////////

void setup() {

  setPwmFrequency(IN1); // Increase PWM frequency to 32 kHz  (make unaudible)
  setPwmFrequency(IN2);
  setPwmFrequency(IN3);

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  
  pinMode(EN1, OUTPUT); 
  pinMode(EN2, OUTPUT); 
  pinMode(EN3, OUTPUT); 


  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  digitalWrite(EN3, HIGH);
  

  sineArraySize = sizeof(pwmSin)/sizeof(int); // Find lookup table size
  int phaseShift = sineArraySize / 3;         // Find phase shift and initial A, B C phase values
  currentStepA = 0;
  currentStepB = currentStepA + phaseShift;
  currentStepC = currentStepB + phaseShift;

  sineArraySize--; // Convert from array Size to last PWM array number
  Serial.begin(115200);
  
  for (int i = -10; i < 11; i++)
  {
    Serial.print(i);
    Serial.print("%5=");
    Serial.println(i%5);
  }
}

//////////////////////////////////////////////////////////////////////////////

void loop() {

  analogWrite(IN1, pwmSin[currentStepA]);
  analogWrite(IN2, pwmSin[currentStepB]);
  analogWrite(IN3, pwmSin[currentStepC]);  

  if (direct==true) increment = 1;
  else increment = -1;     

  currentStepA = currentStepA + increment;
  currentStepB = currentStepB + increment;
  currentStepC = currentStepC + increment;

  if (true)
  {
    currentStepA %= sineArraySize;
    currentStepB %= sineArraySize;
    currentStepC %= sineArraySize;
  }
  else
  {
    //Check for lookup table overflow and return to opposite end if necessary
    if(currentStepA > sineArraySize)  currentStepA = 0;
    if(currentStepA < 0)  currentStepA = sineArraySize;
   
    if(currentStepB > sineArraySize)  currentStepB = 0;
    if(currentStepB < 0)  currentStepB = sineArraySize;
   
    if(currentStepC > sineArraySize)  currentStepC = 0;
    if(currentStepC < 0) currentStepC = sineArraySize; 
  }
  /// Control speed by this delay
  delay(20);

}


void setPwmFrequency(int pin) {
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | 0x01;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | 0x01;
    }
  }
  else if(pin == 3 || pin == 11) {
    TCCR2B = TCCR2B & 0b11111000 | 0x01;
  }
}
