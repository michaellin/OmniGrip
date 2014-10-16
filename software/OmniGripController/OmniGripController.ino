
/* Motor controller
 * Encoder code from http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Author: Michael Lin
 * PinOut for OmniGrip
 * Pin 2 and 3 -> Encoder pins ChA & ChB
 * Pin 4       -> Toggle Switch
 * Pin 5       -> Green LED
 * Pin 5       -> Yellow LED
 * Pin 6       -> Red LED
 */

#include <Encoder.h>
#include <math.h>

static const int   t1cntr = 31250;     //Max count for Timer1

/*Motor constants*/
static const float maxPWM = (1 << 8);  //Max PWM compare for Timer2
static const float maxOKPWM = maxPWM*0.42;  //Max Continuous current for RE25 is 1.5A and motor resistance is 2.7Ohm. So V=I*R -> maxOKPWM = 40%
static const float safePWM  = maxPWM*0.005; //PWM range safe enough to turn on the motor switch.

/*Audio Jack constants*/
static const float maxCmdGrip = t1cntr*0.8; // Dont want to command to Audio Jack more than 80% duty cycle to keep the waveform clean.

/*General constants*/
static const float maxGripEncCount = 1750; //Max encoder value when OmniGrip goes from fully open to fully closed


//static const float PWMOffset = 10; //Offset to overcome friction in grippers. Not used due to safety

//Conversions from encoder reading to desired PWM output
const float L_OmniGrip       = 55;                                 // in mm
const float encoder_range    = maxGripEncCount;                    //encoder count in a full swing
const float swing_range      = (25/encoder_range)*(M_PI/180);     // in rad
const float max_volts        = 12;                                 // in Volts
const float motor_resistance = 2.7;                                // in Ohms
const float K_torque         = 23.2;                               // in mNm/A for RE-25
const float capstan_radius   = 4.5;                                // in mm
const float L_daV            = 41;                                 // in mm da Vinci master constant(not used)
const float K_daV            = 5.4/L_daV;                          // in mN/rad da Vinci master constant(not used)

/** Conversion factor calculation
 *  I_eff = PWM*V_max/R_motor
 *  F_eff = I_eff*K_torque/capstan_radius
 *
 *  angle_reading_radian = input_encoder_reading*(25/encoder_range)*(pi/180)    25 is max swing in degrees
 *  F_des = K_stiff*angle_reading_radian/gripper_radius
 *
 *  F_eff = F_des
 *  (PWM*K_torque*V_max)/(R_motor*capstan_radius) = (encoder_reading*(25*pi/180)*K_stiff)/(encoder_range*gripper_radius)
 *  PWM, encoder_reading and K_stiff are variables. Eveything else is constant.
 */
 
//const float conv_factor = ((swing_range/encoder_range)*motor_resistance*capstan_radius*maxGripEncCount) / (L_OmniGrip*K_torque*max_volts);  //Conversion factor from encoder reading to commanded PWM in clock counter have to multiply by stiffness K
const float conv_factor = swing_range*(motor_resistance/(K_torque*max_volts))*(capstan_radius/L_OmniGrip)*maxPWM;  //Conversion factor from encoder reading to desired PWM duty cycle (in clock counter units i.e. multiply by maxPWM).

/*Pinouts*/
int GLED = 5;
int YLED = 6;
int RLED = 7;
int SWITCH = 4;

/*LPF constant*/
float alpha = 0.8;

/*Experiment Variables*/
const int totalStates = 3; //5 when testing stiffnesses
int conditionState    = 0;
int toggleSwitch      = 0; //State for switch. 1 if switch pressed. 0 otherwise.

//-----------CHANGE GAINS HERE----------
/*Array of different stiffness conditions. Mostly just need to change these.*/
//int K1[totalStates] = {40,  40,  55,  60,  80};
//int K2[totalStates] = {200, 230, 250, 270, 290};
//int K1[totalStates] = {40,  60,  75};
//int K2[totalStates] = {200, 250, 290};
int K1[totalStates] = {210, 280, 350};   //Safer set
int K2[totalStates] = {825, 1100, 1375}; //Safer set
//int K1[totalStates] = {240,  320,  400}; //Stiffer set
//int K2[totalStates] = {1050, 1400, 1750}; //Stiffer set

int force_offset = 6;   //In units of Timer counter (up to 256 for 100% PWM)
//--------------------------------

//Temporary variables
float temp1;
float prevPWM = 0;
float currPWM = 0;
float cmdPWM  = 0;
int currSwitchRead = 1;
int prevSwitchRead = 1; //Switch is HIGH when not pressed.
int LEDState;

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
Encoder myEnc(2, 3);

void setup() {
   pinMode(2, INPUT);                //Encoder ChA
   pinMode(3, INPUT);                //Encoder ChB
   
   pinMode(GLED, OUTPUT);            //Green LED
   pinMode(YLED, OUTPUT);            //Yellow LED
   pinMode(RLED, OUTPUT);            //Red LED
   
   pinMode(SWITCH, INPUT);           //Toggle Switch
   
   pinMode(10, OUTPUT);              //Drive PWM on digital pin 10 for Omni on Timer1 16-bit
   pinMode(11, OUTPUT);              //Drive PWM on digital pin 11 for Motor on Timer2 8-bit
   
   digitalWrite(11, LOW);            //Turn off this pin for safety
   
   /* Setup for Audio Jack Clock. This uses Timer 1 for sending PWM to pin 10.
    * The way this works is that there is an internal triangle waveform and a compare value OCR1B. Changing OCR1B changes the duty cycle
    * of the PWM output on Pin10 which drives the Audio Jack.*/
   TCCR1A = _BV(COM1A1)|_BV(COM1B1); //Setup Timer1 register TCCR1A for two compares COM1A1 and COM1B1
   TCCR1B = _BV(WGM13) | _BV(CS11);  //Setup Timer1 register TCCR1B with WGM13 for mode Phase and Freq correct, and CS11 for pre-scalar 16Mhz/8  
   ICR1 = t1cntr;                    //For 16Hz based on eq f=8Mhz/(2*N*ICR1) FOR ARDUINO MINI 8MHz
   
   /* This is the same as Audio Jack Clock but for the motor and it uses Timer2 instead.*/
   TCCR2A = 3 | _BV(COM2A1);         //Setup Timer2 register TCCR2A for two compares COM2A1 and COM2B1. TCCR2A = 3 is Fast PWM mode.
   TCCR2B = 1;                       //For 32KHz based on eq f=8Mhz/(256*N*ICR1) FOR ARDUINO MINI 8MHz where N is prescalar
   OCR2A = 0*maxPWM;                 //Initial PWM duty cycle for motor
   //Serial.begin(9600);
}

void loop() {
  
   /* Code for output to Audio Jack */
   long encoder_reading = myEnc.read();                        //Obtain current encoder reading
   //Serial.println(encoder_reading);
   //This should change to scale temp1 proportionally instead of capping at the bottom
   temp1 = float(encoder_reading*maxCmdGrip)/maxGripEncCount;  //Scale the encoder reading to a range corresponding to 15% to 80% for PWM on Audio Jack.
   if (int(temp1) < t1cntr*0.15) {
     OCR1B = t1cntr*0.15;                                      //Minimum PWM will be 15% duty cycle.
   } else {
     OCR1B = int(temp1);
   }
   
   /* Read toggle switch state */
   currSwitchRead = digitalRead(SWITCH);
   toggleSwitch   = switchPressed(currSwitchRead);
   //if (cmdPWM < safePWM && toggleSwitch == 1) {  //Fail-safe cmdPWM has to be low in order to allow change of stiffness
   if(toggleSwitch == 1) {
     conditionState += 1;
     if (conditionState == totalStates) { //Loop around every x amount of states
       conditionState = 0;
     }
     //LEDState = conditionState;
   }
   LEDState = conditionState+1; //conditionState+1 since I want to toggle from 1 to 3
   
   /* Stiffness control */
   float spring_pos1 = 0;                           //Spring 1 start at position 0
   float kp1 = K1[conditionState];                  //Spring 1 stiffness coefficient in mNm/rad
   float spring_pos2 = maxGripEncCount*3/5;         //Spring 2  starts at 15 degrees (Max open an close is 25 degrees)
   float kp2 = K2[conditionState];                  //Spring 2 stiffness coefficient in mNm/rad
   
   if (encoder_reading > spring_pos1) {
       cmdPWM = (kp1 * (encoder_reading-spring_pos1) * conv_factor);
   }
   if (encoder_reading > spring_pos2) {
       cmdPWM += (kp2 * (encoder_reading-spring_pos2) * conv_factor);
   }
   cmdPWM += force_offset; //Adds the force offset
   
   //Serial.println(cmdPWM);
   if (encoder_reading > spring_pos1) {
     if (cmdPWM > maxOKPWM) {   //Cap the PWM so motors dont burn off
       LEDState = 6; //Force the motor at higher current and signal it with LED
     } else {
       OCR2A = cmdPWM;
       //Serial.println(cmdPWM);
     }
   } else {
     OCR2A = 0; //Fail-safe always turn off motor if encoder value is passed fully open.
   }
   
   if (cmdPWM < safePWM) { // For indicating safe zone in the motor.
     LEDState = 5;
   }
   updateLED(LEDState); //Update LED at the end
}

/**
 * Updates LEDs depending on the current condition
 */
void updateLED (int condition) {
  digitalWrite(GLED, LOW);
  digitalWrite(YLED, LOW);
  digitalWrite(RLED, LOW);
  
  if (condition == 1) {
    digitalWrite(GLED, HIGH);
  }
  if (condition == 2) {
    digitalWrite(YLED, HIGH);
  }
  if (condition == 3) {
    digitalWrite(RLED, HIGH);
  }
  if (condition == 4) {
      digitalWrite(GLED, HIGH);
      digitalWrite(YLED, HIGH);
  }
  if (condition == 5) {    //For indicating safe zone
      digitalWrite(GLED, HIGH);
      digitalWrite(RLED, HIGH);
  }
  if (condition == 6) {    //For indicating going over maxPWM
      digitalWrite(GLED, HIGH);
      digitalWrite(YLED, HIGH);
      digitalWrite(RLED, HIGH);
  }
}

int switchPressed (int currSwitchReading) {
  if (currSwitchReading == 1 && prevSwitchRead == 0) { //Rising edge of the switch
    prevSwitchRead = 1;
    return 1;  //Only return 1 in rising edge
  } else if (currSwitchReading == 0 && prevSwitchRead == 1) {
    prevSwitchRead = 0;
  }
  return 0;  
}
