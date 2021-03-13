#include "uStepperS.h"


#define DISPENSE_LED_PIN 3
#define FAST_DISPENSE_5ML_BUTTON_PIN 23
#define SLOW_DISPENSE_5ML_BUTTON_PIN 24
#define FAST_REWIND_BUTTON_PIN 13
#define SLOW_REWIND_BUTTON_PIN 2

#define STEPS_PER_ROTATION 200
#define MICROSTEPS_PER_STEP 256
#define CONTROL_THRESHOLD 10
#define CHECK_ORIENTATION_MICROSTEPS 30
#define STALLSENSITIVITY 2


#define MAX_RPM_SLOW 100
#define MAX_RPM_FAST 225


#define BUTTON_DELAY_LONG 500
#define BUTTON_DELAY_SHORT 200


#define SYRINGE_DIAMETER_MM 34.8
#define DISPENSE_QUANTITY_ML 5
#define LEAD_SCREW_PITCH 1.5


static const float MAX_STEPS_PER_SECOND_SLOW = (MAX_RPM_SLOW / 60) * STEPS_PER_ROTATION;
static const float MAX_STEPS_PER_SECOND_FAST = (MAX_RPM_FAST / 60) * STEPS_PER_ROTATION;
static const float SYRINGE_DIAMETER_CM = SYRINGE_DIAMETER_MM / 10;
static const float SYRINGE_RADIUS_CM = SYRINGE_DIAMETER_CM / 2;
static const float SYRINGE_CIRCLE_SURFACE_AREA_CM2 = PI * pow(SYRINGE_RADIUS_CM, 2);
static const float DISPENSE_TRAVEL_CM = DISPENSE_QUANTITY_ML / SYRINGE_CIRCLE_SURFACE_AREA_CM2;
static const float DISPENSE_TRAVEL_MM = DISPENSE_TRAVEL_CM * 10;
static const float DISPENSE_ROTATION_COUNT = DISPENSE_TRAVEL_MM / LEAD_SCREW_PITCH;
static uStepperS stepper;


static bool ready = false;
static bool dispensing = false;
static bool rewinding = false;
static float dispense_steps = -1;
static float pos = -1;
static float pos_last = -1;
static float pos_start = -1;
static float pos_target = -1;
static float diff = -1;
static float diff_max = -1;
static char f2str_buffer[64];




void setup() {
  Serial.begin(9600);

  dispense_steps = round(DISPENSE_ROTATION_COUNT * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
  
  // initialize digital pin DISPENSE_LED_PIN as an output.
  pinMode(DISPENSE_LED_PIN, OUTPUT);
  digitalWrite(DISPENSE_LED_PIN, LOW);

  // initialize the pushbutton pin as an input:
  pinMode(FAST_DISPENSE_5ML_BUTTON_PIN, INPUT);
  pinMode(SLOW_DISPENSE_5ML_BUTTON_PIN, INPUT);
  pinMode(FAST_REWIND_BUTTON_PIN, INPUT);
  pinMode(SLOW_REWIND_BUTTON_PIN, INPUT);

  stepper.setup(
    CLOSEDLOOP, // mode  Default is normal mode. Pass the constant "DROPIN" to configure the uStepper to act as dropin compatible to the stepstick. Pass the constant "PID", to enable closed loop feature for regular movement functions, such as moveSteps()
    STEPS_PER_ROTATION, // stepsPerRevolution  Number of fullsteps per revolution
    10.0, // pTerm The proportional coefficent of the DROPIN PID controller, default = 10.0
    0.0, // iTerm  The integral coefficent of the DROPIN PID controller, default = 0.0
    0.0, // dTerm  The differential coefficent of the DROPIN PID controller, default = 0.0
    16, // dropinStepSize  number of steps per fullstep, send from external dropin controller
    true, // setHome When set to true, the encoder position is Reset. When set to false, the encoder position is not reset.
    0, // invert  Inverts the motor direction for dropin feature. 0 = NOT invert, 1 = invert. this has no effect for other modes than dropin
    50, // runCurrent  Sets the current (in percent) to use while motor is running.
    30 // holdCurrent Sets the current (in percent) to use while motor is NOT running
  );
 
  stepper.checkOrientation(CHECK_ORIENTATION_MICROSTEPS);

  stepper.setControlThreshold(CONTROL_THRESHOLD);

  Serial.println("READY");

  ready = true;
}



void printFloat(char* label, float value) {
  Serial.print(label);
  dtostrf(value, 0, 0, &f2str_buffer[0]);
  Serial.print(f2str_buffer);
  Serial.print("\n");
}


void checkDispensingPosition() {

  pos = stepper.driver.getPosition();

  if(pos != pos_last) {
  
    pos_last = pos;
    
    diff = pos - pos_target;
    
    if(diff == 0) {

      printFloat("overshoot : ", diff_max > 0 ? diff_max : 0);
      
      digitalWrite(DISPENSE_LED_PIN, LOW);
      
      dispensing = false;
    
    } else if(diff > 0) {

      diff_max = max(diff, diff_max);
    
    }
  }
}


void dispense(float maxVelocity) {

      pos = -1;
      pos_start = -1;
      pos_last = -1;
      pos_target = -1;
      diff = -1;
      diff_max = -1;
      dispensing = true;
    
      pos_start = stepper.driver.getPosition();
  
      pos_target = dispense_steps + pos_start;
  
      stepper.setMaxVelocity(maxVelocity);
  
      stepper.moveSteps(dispense_steps);
      
      digitalWrite(DISPENSE_LED_PIN, HIGH);

}


void loop() {

  if(ready) {
  
    if(dispensing) {

      checkDispensingPosition();

    } else if(rewinding) {

      if (digitalRead(FAST_REWIND_BUTTON_PIN) == LOW) {
      
        stepper.setRPM(0);
        
        stepper.driver.setHome();
      
        digitalWrite(DISPENSE_LED_PIN, LOW);
        
        rewinding = false;
        
        delay(BUTTON_DELAY_SHORT);
      
      } else if (digitalRead(SLOW_REWIND_BUTTON_PIN) == LOW) {
      
        stepper.stop();
        
        stepper.driver.setHome();
      
        digitalWrite(DISPENSE_LED_PIN, LOW);
        
        rewinding = false;
        
        delay(BUTTON_DELAY_SHORT);
      }

    } else {
  
      
      if (digitalRead(FAST_DISPENSE_5ML_BUTTON_PIN) == LOW) {
    
        dispense(MAX_STEPS_PER_SECOND_FAST);

      } else if (digitalRead(SLOW_DISPENSE_5ML_BUTTON_PIN) == LOW) {
      
        dispense(MAX_STEPS_PER_SECOND_SLOW);
      
      } else if (digitalRead(FAST_REWIND_BUTTON_PIN) == LOW) {

        rewinding = true;
      
        digitalWrite(DISPENSE_LED_PIN, HIGH);
        
        stepper.setRPM(MAX_RPM_FAST * -1);
        
        delay(BUTTON_DELAY_SHORT);

      } else if (digitalRead(SLOW_REWIND_BUTTON_PIN) == LOW) {

        rewinding = true;
      
        digitalWrite(DISPENSE_LED_PIN, HIGH);
        
        stepper.setRPM(MAX_RPM_SLOW * -1);
        
        delay(BUTTON_DELAY_SHORT);
      
      }

    }

  }

}

