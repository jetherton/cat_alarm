// Includes go here
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/////////////////////////////////////////// Macros go here

#define SERIAL_DEBUG_ENABLED 0

#define GET_NUM_ARGS(...) GET_NUM_ARGS_ACT(__VA_ARGS__, 5,4,3,2,1)
#define GET_NUM_ARGS_ACT(_1,_2,_3,_4,_5,N,...) N

#define macro_dispatcher(func, ...) \
            macro_dispatcher_(func, GET_NUM_ARGS(__VA_ARGS__))
#define macro_dispatcher_(func, nargs) \
            macro_dispatcher__(func, nargs)
#define macro_dispatcher__(func, nargs) \
            func ## nargs
            
#if SERIAL_DEBUG_ENABLED
  #define DebugPrint(...) macro_dispatcher(DebugPrint, __VA_ARGS__)(__VA_ARGS__)
  #define DebugPrintln(...) macro_dispatcher(DebugPrintln, __VA_ARGS__)(__VA_ARGS__)  
  #define DebugPrint2(str,modifier)  \
        Serial.print(millis());     \
        Serial.print(": ");    \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' ');      \
        Serial.print(__LINE__);     \
        Serial.print(' ');      \
        Serial.print(str,modifier);
  #define DebugPrint1(str)  \
        Serial.print(millis());     \
        Serial.print(": ");    \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' = ');      \
        Serial.print(__LINE__);     \
        Serial.print(' ');      \
        Serial.print(str);       
  #define DebugPrintSimple(str)  \
        Serial.print(str);           
  #define DebugPrintln2(str,modifier)  \
        Serial.print(millis());     \
        Serial.print(": ");    \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' ');      \
        Serial.print(__LINE__);     \
        Serial.print(' ');      \
        Serial.println(str,modifier);
  #define DebugPrintln1(str)  \
        Serial.print(millis());     \
        Serial.print(": ");    \
        Serial.print(__PRETTY_FUNCTION__); \
        Serial.print(' ');      \
        Serial.print(__LINE__);     \
        Serial.print(' ');      \
        Serial.println(str);          
#else
  #define DebugPrint(...) macro_dispatcher(DebugPrint, __VA_ARGS__)(__VA_ARGS__)
  #define DebugPrintln(...) macro_dispatcher(DebugPrintln, __VA_ARGS__)(__VA_ARGS__)  
  #define DebugPrintSimple(str)
  #define DebugPrint1(str)
  #define DebugPrintln1(str)
  #define DebugPrint2(str,modifier)
  #define DebugPrintln2(str,modifier)
#endif









































////////////////////////////////////// Constants
#define RELAY_REMOTE_ALARM_PIN 8 // The pin for the relay that controls the remote alarm
#define RELAY_HORN_ALARM_PIN 7 // The pin for the relay that controls the horn
#define RELAY_LIGHTS_ALARM_PIN 4 // The pin for the relay that controls the light

#define REMOTE_CONTROL_INPUT_PIN 9 // The pin where the 433mhz remote relay sends signals
#define ONBOARD_LED_OUTPUT_PIN 13 // The pin for the onboard led, pretty straight forward

#define LEARN_COUNT 1000 // How many cycles we spend learning acceptable motion levels
#define SAFETY_FACTOR 1.695 //How much buffer to give our motion thresholds

enum states {
  JUST_STARTED,
  LEARNING,
  NORMALIZING,
  ARMED,
  NOT_ARMED,
  FIRST_TRIGGER,
  ALARMING
  };

struct SystemState {
  unsigned long timeOfLastEvent = 0;
  enum states state = JUST_STARTED;   
  int remoteControlInput = -255;
  int learnCount = LEARN_COUNT;

  float maxX = -10000000.0;
  float minX = 10000000.0;
  double sumX = 0;
  
  float maxY = -10000000.0;
  float minY = 10000000.0;
  double sumY = 0;
  
  float maxZ = -10000000.0;
  float minZ = 10000000.0;
  double sumZ = 0;
  
  double avgX = 0;
  double avgY = 0;
  double avgZ = 0;
};

////////////////////////////////////// Define global vars

Adafruit_MPU6050 mpu;
struct SystemState systemState;


void setup(void) {
  
  resetSystemState();
  if (SERIAL_DEBUG_ENABLED) {
    Serial.begin(115200);
    while (!Serial) {
      delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }
  }


  DebugPrintln("\n\n\n\n\n\n");  
  DebugPrintln("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    DebugPrintln("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }



  DebugPrintln("MPU6050 Found!");

  //setupt motion detection
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  


  DebugPrintln("Setting PINS for our digital inputs and outputs");
  pinMode(RELAY_REMOTE_ALARM_PIN, OUTPUT);
  pinMode(RELAY_HORN_ALARM_PIN, OUTPUT);
  pinMode(RELAY_LIGHTS_ALARM_PIN, OUTPUT);
  pinMode(REMOTE_CONTROL_INPUT_PIN, INPUT_PULLUP);
  pinMode(ONBOARD_LED_OUTPUT_PIN, OUTPUT);


  DebugPrintln("Initialization done");
  delay(2000);
}

/**
 * Set everything back to the way it was
 */
void resetSystemState() {
  systemState.timeOfLastEvent = 0;
  systemState.state = JUST_STARTED;   
  systemState.remoteControlInput = -255;
  systemState.learnCount = LEARN_COUNT;
  
  systemState.maxX = -10000000.0;
  systemState.minX = 10000000.0;
  systemState.sumX = 0;
  
  systemState.maxY = -10000000.0;
  systemState.minY = 10000000.0;
  systemState.sumY = 0;
  
  systemState.maxZ = -10000000.0;
  systemState.minZ = 10000000.0;
  systemState.sumZ = 0;
  
  systemState.avgX = 0;
  systemState.avgY = 0;
  systemState.avgZ = 0;
}

/**
 * The main loop that handles everything.
 */
void loop() {

  switch(systemState.state) {
    case JUST_STARTED:
      DebugPrintSimple("JUST_STARTED\n");
      systemState.state = NOT_ARMED;
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      break;
    case LEARNING:
      DebugPrintSimple("LEARNING ");
      DebugPrintSimple(systemState.learnCount);
      DebugPrintSimple("\n");
      learnNormalValues();
      break;
    case NORMALIZING:
      DebugPrintSimple("JUST_STARTED\n");
      normalizeValues();
      // four flashes means the system is ready
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(2000);
      break;
    case ARMED:
      //DebugPrintSimple("ARMED\n");
      if(isMotionDetected()) {
        recordTimeOfEvent();
        flashCarLightsAndHorn(400);
        DebugPrintSimple("FIRST_TRIGGER\n");
        systemState.state = FIRST_TRIGGER;
        delay(1000);
      }
      readRemoteControl();
      break;
    case NOT_ARMED:
      //DebugPrintSimple("NOT_ARMED\n");
      readRemoteControl();
      break;
    case FIRST_TRIGGER:
      if(isMotionDetected()) {
        recordTimeOfEvent();
        DebugPrintSimple("ALARMING\n");
        systemState.state = ALARMING;
      } else if(isTimeOfLastEventMoreThanSecondsAgo(60)) {
        systemState.state = ARMED;
      } else {
        readRemoteControl();
      }
      break;
    case ALARMING:
      
      if(isTimeOfLastEventMoreThanSecondsAgo(300)) { 
        systemState.state = ARMED;
      } else {
        flashCarLightsAndHorn(1200);
        turnOnRemoteAlarm();
        delay(400);
      }
      
      readRemoteControl();
      break;
  }
 
  delay(100);
}

void flashCarLights(int duration) {
  digitalWrite(RELAY_LIGHTS_ALARM_PIN, HIGH);
  delay(duration);
  digitalWrite(RELAY_LIGHTS_ALARM_PIN, LOW);
}

/**
 * Flashes the car lights and honks the horn for the given
 * duration
 */
void flashCarLightsAndHorn(int duration) {
  digitalWrite(RELAY_LIGHTS_ALARM_PIN, HIGH);
  digitalWrite(RELAY_HORN_ALARM_PIN, HIGH);
  delay(duration);
  digitalWrite(RELAY_LIGHTS_ALARM_PIN, LOW);
  digitalWrite(RELAY_HORN_ALARM_PIN, LOW);
}

/**
 * Checks if the last event was more than so many seconds ago.
 * Handles roll overs. 
 * 
 * Returns true if the last event occured more than the specified number
 * of seconds ago.
 */
boolean isTimeOfLastEventMoreThanSecondsAgo(unsigned long secondsAgo) {

  unsigned long MAX_U_LONG = 4294967295UL;
  unsigned long MILLIS_IN_SECOND = 1000;
  unsigned long now = millis();
  unsigned long delta = 0;

  // Handle roll over.
  double deltaAsDouble = (double)now - (double)systemState.timeOfLastEvent;
  if( deltaAsDouble < 0) {
    // Roll over has happened.
    unsigned long timeToRollOver = MAX_U_LONG - systemState.timeOfLastEvent;
    delta = timeToRollOver + now;
  } else {
    // No roll over
    delta = now - systemState.timeOfLastEvent;
  }

  return delta > (secondsAgo * MILLIS_IN_SECOND);

}


/**
 * Helper method that records the time of the last event
 * whatever it may be.
 */
void recordTimeOfEvent() {
  systemState.timeOfLastEvent = millis();
}

/**
 * Reads the values from the gyro/accelerometer to see if there is 
 * any motion
 */

bool isMotionDetected() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(a.acceleration.x > systemState.maxX || a.acceleration.x < systemState.minX ||
     a.acceleration.y > systemState.maxY || a.acceleration.y < systemState.minY ||
     a.acceleration.z > systemState.maxZ || a.acceleration.z < systemState.minZ ) {
    return true;
  } else {
    return false;
  }
}

/**
 * This method takes the learned values and normalizes them.
 */
void normalizeValues() {
  
  DebugPrintSimple("\nComputing average and safety\n");
  double avgX = systemState.sumX / (double)LEARN_COUNT;
  double avgY = systemState.sumY / (double)LEARN_COUNT;
  double avgZ = systemState.sumZ / (double)LEARN_COUNT;
  
  
  systemState.maxX = ((systemState.maxX-avgX) * SAFETY_FACTOR) + avgX;
  systemState.minX = ((systemState.minX-avgX) * SAFETY_FACTOR) + avgX;
  
  systemState.maxY = ((systemState.maxY-avgY) * SAFETY_FACTOR) + avgY;
  systemState.minY = ((systemState.minY-avgY) * SAFETY_FACTOR) + avgY;
  
  systemState.maxZ = ((systemState.maxZ-avgZ) * SAFETY_FACTOR) + avgZ;
  systemState.minZ = ((systemState.minZ-avgZ) * SAFETY_FACTOR) + avgZ;
  
  
  DebugPrintSimple("MinX: ");
  DebugPrintSimple(systemState.minX);
  DebugPrintSimple("\tavgX: ");
  DebugPrintSimple(avgX);
  DebugPrintSimple("\tMaxX: ");
  DebugPrintSimple(systemState.maxX);
  DebugPrintSimple("\tm/s^2\n");
  
  DebugPrintSimple("MinY: ");
  DebugPrintSimple(systemState.minY);
  DebugPrintSimple("\tavgY: ");
  DebugPrintSimple(avgY);
  DebugPrintSimple("\tMaxY: ");
  DebugPrintSimple(systemState.maxY);
  DebugPrintSimple("\tm/s^2\n");
  
  DebugPrintSimple("MinZ: ");
  DebugPrintSimple(systemState.minZ);
  DebugPrintSimple("\tavgZ: ");
  DebugPrintSimple(avgZ);
  DebugPrintSimple("\tMaxZ: ");
  DebugPrintSimple(systemState.maxZ);
  DebugPrintSimple("\tm/s^2\n\n\n");

  systemState.state = ARMED;

}


/**
 * This method reads the sensor and learns what the normal bounds are of the
 * accerlation values
 */
void learnNormalValues() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  bool didChange = false;
  if(systemState.learnCount > 0) {
    systemState.learnCount--;

    systemState.sumX += a.acceleration.x;
    systemState.sumY += a.acceleration.y;
    systemState.sumZ += a.acceleration.z;

    if(a.acceleration.x > systemState.maxX) {
      systemState.maxX = a.acceleration.x;
      didChange = true;
    }
    if(a.acceleration.x < systemState.minX) {
      systemState.minX = a.acceleration.x;
      didChange = true;
    }

    if(a.acceleration.y > systemState.maxY) {
      systemState.maxY = a.acceleration.y;
      didChange = true;
    }
    if(a.acceleration.y < systemState.minY) {
      systemState.minY = a.acceleration.y;
      didChange = true;
    }

    if(a.acceleration.z > systemState.maxZ) {
      systemState.maxZ = a.acceleration.z;
      didChange = true;
    }
    if(a.acceleration.z < systemState.minZ) {
      systemState.minZ = a.acceleration.z;
      didChange = true;
    }  
  } else if (systemState.learnCount == 0) {
    // All done learning
    systemState.state = NORMALIZING;
  }
}

/**
 * This method reads in the signal from the
 * 433mhz remote control that is used to
 * arm/disarm the alarm
 */
void readRemoteControl() {
  int sensorValue = digitalRead(REMOTE_CONTROL_INPUT_PIN);

  if (sensorValue != systemState.remoteControlInput) {
    systemState.remoteControlInput = sensorValue;
    if (sensorValue == HIGH) {
      DebugPrintln("Remote is now HIGH");
      systemState.state = NOT_ARMED;
      digitalWrite(ONBOARD_LED_OUTPUT_PIN, LOW);
      flashCarLights(1000);
      delay(500);
      flashCarLights(1000);
    } else {
      DebugPrintln("Remote is now LOW");
      resetSystemState();
      systemState.state = LEARNING;
      digitalWrite(ONBOARD_LED_OUTPUT_PIN, HIGH);
      flashCarLights(1000);
    }
  }
}


/**
 * Method that handles whenever motion is detected
 * Decides if we alarm or not, do we set in state
 * or not. Things like that.
 */
void handleMotionDetected() {
  printGyroAccelDebugInfo();
  turnOnRemoteAlarm();
}


/**
 * Turns on the remote alarm, most likely the one in my bedroom
 * so I know to check out what's going on and maybe even call the cops.
 */
void turnOnRemoteAlarm() {
  digitalWrite(RELAY_REMOTE_ALARM_PIN, HIGH);
  delay(300);
  // The switch for the remote alarm is momentary, so turn it back off
  digitalWrite(RELAY_REMOTE_ALARM_PIN, LOW);
}


/*
 * Helper method that prints out the stats for the gyroscope
 */
void printGyroAccelDebugInfo() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  /* Print out the values */
  DebugPrintSimple("AccelX:\t");
  DebugPrintSimple(a.acceleration.x);
  DebugPrintSimple("\t");
  DebugPrintSimple("AccelY:\t");
  DebugPrintSimple(a.acceleration.y);
  DebugPrintSimple("\t");
  DebugPrintSimple("AccelZ:\t");
  DebugPrintSimple(a.acceleration.z);
//  DebugPrintSimple("\t");
//  DebugPrintSimple("GyroX:");
//  DebugPrintSimple(g.gyro.x);
//  DebugPrintSimple(",");
//  DebugPrintSimple("GyroY:");
//  DebugPrintSimple(g.gyro.y);
//  DebugPrintSimple(",");
//  DebugPrintSimple("GyroZ:");
//  DebugPrintSimple(g.gyro.z);
  DebugPrintSimple("\n");
}


 
