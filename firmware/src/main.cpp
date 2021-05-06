#include <Controller.h>

#define DISPENSE_LED_PIN 3
#define FAST_DISPENSE_5ML_BUTTON_PIN 23
#define SLOW_DISPENSE_5ML_BUTTON_PIN 24
#define FAST_REWIND_BUTTON_PIN 13
#define SLOW_REWIND_BUTTON_PIN 2

#define BUTTON_DELAY_LONG 500
#define BUTTON_DELAY_SHORT 200

#define CONTROL_THRESHOLD 10
#define CHECK_ORIENTATION_MICROSTEPS 4266
#define STEPS_PER_ROTATION 200

#define MAX_RPM_SLOW 100
#define MAX_RPM_FAST 225


/*
#define SYRINGE_DIAMETER_MM 34.8
#define DISPENSE_QUANTITY_ML 5
#define LEAD_SCREW_PITCH 15

static const float SYRINGE_DIAMETER_CM = SYRINGE_DIAMETER_MM / 10;
static const float SYRINGE_RADIUS_CM = SYRINGE_DIAMETER_CM / 2;
static const float SYRINGE_CIRCLE_SURFACE_AREA_CM2 = PI * pow(SYRINGE_RADIUS_CM, 2);
static const float DISPENSE_TRAVEL_CM = DISPENSE_QUANTITY_ML / SYRINGE_CIRCLE_SURFACE_AREA_CM2;
static const float DISPENSE_TRAVEL_MM = DISPENSE_TRAVEL_CM * 10;
static const float DISPENSE_ROTATION_COUNT = DISPENSE_TRAVEL_MM / LEAD_SCREW_PITCH;

#define MICROSTEPS_PER_STEP 256

static const uint16_t MAX_STEPS_PER_SECOND_SLOW = (MAX_RPM_SLOW / 60) * STEPS_PER_ROTATION;
static const uint16_t MAX_STEPS_PER_SECOND_FAST = (MAX_RPM_FAST / 60) * STEPS_PER_ROTATION;
static const uint32_t DISPENSE_MICROSTEP_COUNT = DISPENSE_ROTATION_COUNT * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP;
*/
#define MAX_STEPS_PER_SECOND_SLOW 200
#define MAX_STEPS_PER_SECOND_FAST 400
#define DISPENSE_MICROSTEP_COUNT 17945 // 179432 when p=1.5 

static Controller* controller = Controller::getInstance();

static bool ready = false;
static bool dispensing = false;
static bool rewinding = false;

static int32_t pos = -1;
static int32_t pos_last = -1;
static int32_t pos_start = -1;
static int32_t pos_target = -1;
static int32_t diff = -1;
static int32_t diff_max = -1;




void setup() {
  Serial.begin(9600);
  
  // initialize LED digital pin as an output
  pinMode(DISPENSE_LED_PIN, OUTPUT);
  digitalWrite(DISPENSE_LED_PIN, LOW);

  // initialize pushbutton pins as inputs
  pinMode(FAST_DISPENSE_5ML_BUTTON_PIN, INPUT);
  pinMode(SLOW_DISPENSE_5ML_BUTTON_PIN, INPUT);
  pinMode(FAST_REWIND_BUTTON_PIN, INPUT);
  pinMode(SLOW_REWIND_BUTTON_PIN, INPUT);

  controller->setup(
    STEPS_PER_ROTATION, // stepsPerRevolution  Number of fullsteps per revolution
    true, // setHome When set to true, the encoder position is Reset. When set to false, the encoder position is not reset.
    85, // runCurrent  Sets the current (in percent) to use while motor is running, default = 50.0
    10 // holdCurrent Sets the current (in percent) to use while motor is NOT running, default = 30.0
  );

  
/*
  controller->checkOrientation();

  controller->setControlThreshold(CONTROL_THRESHOLD);

  controller->setMaxDeceleration(2000000);
*/
  Serial.println("READY");

  ready = true;
}

void checkDispensingPosition() {

  pos = controller->driver.getPosition();

  if(pos != pos_last) {
  
    pos_last = pos;
    
    diff = pos - pos_target;
    
    if(diff == 0) {

      Serial.printf("overshoot : %d\n", diff_max > 0 ? diff_max : 0);
      
      digitalWrite(DISPENSE_LED_PIN, LOW);
      
      dispensing = false;
    
    } else if(diff > 0) {

      diff_max = max(diff, diff_max);
    
    }
  }
}


void dispense(uint16_t maxVelocity) {

      pos = -1;
      pos_start = -1;
      pos_last = -1;
      pos_target = -1;
      diff = -1;
      diff_max = -1;
      dispensing = true;

      pos_start = controller->driver.getPosition();

      pos_target = DISPENSE_MICROSTEP_COUNT + pos_start;

      controller->setMaxVelocity(maxVelocity);

      controller->moveSteps(DISPENSE_MICROSTEP_COUNT);
      
      digitalWrite(DISPENSE_LED_PIN, HIGH);

}

void loop() {

  if(ready) {
  
    if(dispensing) {

      checkDispensingPosition();

    } else if(rewinding) {

      if (digitalRead(FAST_REWIND_BUTTON_PIN) == LOW) {
      
        controller->setRPM(0);
        
        controller->driver.setHome();
      
        digitalWrite(DISPENSE_LED_PIN, LOW);
        
        rewinding = false;
        
        delay(BUTTON_DELAY_SHORT);
      
      } else if (digitalRead(SLOW_REWIND_BUTTON_PIN) == LOW) {
      
        controller->stop();
        
        controller->driver.setHome();
      
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
        
        controller->setRPM(MAX_RPM_FAST * -1);
        
        delay(BUTTON_DELAY_SHORT);

      } else if (digitalRead(SLOW_REWIND_BUTTON_PIN) == LOW) {

        rewinding = true;
      
        digitalWrite(DISPENSE_LED_PIN, HIGH);
        
        controller->setRPM(MAX_RPM_SLOW * -1);
        
        delay(BUTTON_DELAY_SHORT);
      
      }

    }

  }

}

