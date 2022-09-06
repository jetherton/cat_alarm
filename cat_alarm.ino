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

#define LEARN_COUNT 500 // How many cycles we spend learning acceptable motion levels
#define SAFETY_FACTOR 3.50 //How much buffer to give our motion thresholds

#define INITIAL_REMOTE_CONTROL_READ_STATE -255

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
  int remoteControlInput = INITIAL_REMOTE_CONTROL_READ_STATE;
  int learnCount = LEARN_COUNT;

  double maxX = -10000000.0;
  double runningMaxX = -10000000.0;
  double minX = 10000000.0;
  double runningMinX = 10000000.0;
  double sumX = 0;
  
  double maxY = -10000000.0;
  double runningMaxY = -10000000.0;
  double minY = 10000000.0;
  double runningMinY = 10000000.0;
  double sumY = 0;
  
  double maxZ = -10000000.0;
  double runningMaxZ = -10000000.0;
  double minZ = 10000000.0;
  double runningMinZ = 10000000.0;
  double sumZ = 0;
  
  double avgX = 0;
  double avgY = 0;
  double avgZ = 0;
};

////////////////////////////////////// Define global vars

Adafruit_MPU6050 mpu;
struct SystemState systemState;


void setup(void) {
  
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
  // Allowed values: https://adafruit.github.io/Adafruit_MPU6050/html/_adafruit___m_p_u6050_8h.html#a114993cadef707ad8c6ccc9c0fbf02ad
  /*
    MPU6050_RANGE_250_DEG --most sensative   
    MPU6050_RANGE_500_DEG   
    MPU6050_RANGE_1000_DEG  
    MPU6050_RANGE_2000_DE
    
    MPU6050_RANGE_2_G --most sensative
    MPU6050_RANGE_4_G
    MPU6050_RANGE_8_G
    MPU6050_RANGE_16_G
    
    MPU6050_BAND_260_HZ   
    MPU6050_BAND_184_HZ   
    MPU6050_BAND_94_HZ  
    MPU6050_BAND_44_HZ  
    MPU6050_BAND_21_HZ  
    MPU6050_BAND_10_HZ  
    MPU6050_BAND_5_HZ   -- lower frequency filters out more noise
  */
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
  


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
 * Reset the state so we can start calibration over again.
 * Because of changes in temperature we need to continually
 * re-calibrate to keep the system accurate.
 */
void resetSystemStateToCalibrate() {
  DebugPrintSimple("Reset system state\n");
  
  systemState.learnCount = LEARN_COUNT;
  
  systemState.runningMaxX = -10000000.0;
  systemState.runningMinX = 10000000.0;
  systemState.sumX = 0;
  
  systemState.runningMaxY = -10000000.0;
  systemState.runningMinY = 10000000.0;
  systemState.sumY = 0;
  
  systemState.runningMaxZ = -10000000.0;
  systemState.runningMinZ = 10000000.0;
  systemState.sumZ = 0;
  
  systemState.avgX = 0;
  systemState.avgY = 0;
  systemState.avgZ = 0;
}

/**
 * The main loop that handles everything.
 */
void loop() {

  sensors_event_t acceleration;
  switch(systemState.state) {
    case JUST_STARTED:
      DebugPrintSimple("JUST_STARTED\n");
      systemState.state = NOT_ARMED;
      DebugPrintSimple("NOT_ARMED\n");
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      break;
    case LEARNING:
      if(systemState.learnCount % 100 == 0) {
        DebugPrintSimple("LEARNING ");
        DebugPrintSimple(systemState.learnCount);
        DebugPrintSimple("\n");
      }
      acceleration = readAccelerometer();
      learnNormalValues(acceleration);
      if(systemState.learnCount == 0) {
        systemState.state = NORMALIZING;
      }
      break;
    case NORMALIZING:
      DebugPrintSimple("NORMALIZING\n");
      normalizeValues();
      DebugPrintSimple("NORMALIZED\n");
      resetSystemStateToCalibrate();
      // four flashes means the system is ready
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(200);
      flashCarLights(200);
      delay(2000);
      DebugPrintSimple("ARMED\n");
      systemState.state = ARMED;
      break;
    case ARMED:
      acceleration = readAccelerometer();
      if(isMotionDetected(acceleration)) {
        recordTimeOfEvent();
        flashCarLightsAndHorn(400);
        DebugPrintSimple("FIRST_TRIGGER\n");
        systemState.state = FIRST_TRIGGER;
        delay(1000);
        // Read it again to clear any readings from the last second
        DebugPrintSimple("Throw away motion read after first trigger\n");
        readAccelerometer();
        delay(1000);
        // Read it again to clear any readings from the last second
        DebugPrintSimple("Throw away motion read after first trigger\n");
        readAccelerometer();
      } else {
        // No events, so keep recalibrating
        if(systemState.learnCount == 0) {
          normalizeValues();
          resetSystemStateToCalibrate();
        } else { 
          learnNormalValues(acceleration);
        }
      }
      readRemoteControl();
      break;
    case NOT_ARMED:
      readRemoteControl();
      break;
    case FIRST_TRIGGER:
      acceleration = readAccelerometer();
      if(isMotionDetected(acceleration)) {
        recordTimeOfEvent();
        DebugPrintSimple("ALARMING\n");
        systemState.state = ALARMING;
      } else if(isTimeOfLastEventMoreThanSecondsAgo(60)) {
        DebugPrintSimple("First trigger time out, going back to armed\n");
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

  if(systemState.state == NOT_ARMED) {
    // We're not armed so wait longer for inputs
    delay(1000);
  } else {
    // We need to actively look for motion
    delay(100);
  }
  
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
    DebugPrintln("Handling rollover\n");
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
 * Read the accelerometer
 */
sensors_event_t readAccelerometer() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a; 
}
 

/**
 * Check to see if there is 
 * any motion
 */

bool isMotionDetected(sensors_event_t a) {

  char floatBuffer[40];
  byte precision = 8;

  if(a.acceleration.x > systemState.maxX || a.acceleration.x < systemState.minX ||
     a.acceleration.y > systemState.maxY || a.acceleration.y < systemState.minY ||
     a.acceleration.z > systemState.maxZ || a.acceleration.z < systemState.minZ ) {
      
    DebugPrintSimple("Motion Detected \ta.x: ");
    dtostrf(a.acceleration.x, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    DebugPrintSimple("\ta.y: ");
    dtostrf(a.acceleration.y, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    DebugPrintSimple("\ta.z: ");
    dtostrf(a.acceleration.z, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    DebugPrintSimple("\tm/s^2\n");
    return true;
  } else {
    return false;
  }
}

/**
 * This method takes the learned values and normalizes them.
 */
void normalizeValues() {
      
  char floatBuffer[40];
  byte precision = 8;

  
  DebugPrintSimple("\nComputing average and safety\n");
  double avgX = systemState.sumX / (double)LEARN_COUNT;
  double avgY = systemState.sumY / (double)LEARN_COUNT;
  double avgZ = systemState.sumZ / (double)LEARN_COUNT;

    DebugPrintSimple("\n running max x ");
    dtostrf(systemState.runningMaxX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n avgX ");
    dtostrf(avgX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n (systemState.runningMaxX-avgX) ");
    dtostrf((systemState.runningMaxX-avgX), precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n ((systemState.runningMaxX-avgX) * SAFETY_FACTOR) ");
    dtostrf(((systemState.runningMaxX-avgX) * (double)SAFETY_FACTOR), precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    
    DebugPrintSimple("\n ((systemState.runningMaxX-avgX) * SAFETY_FACTOR) + avgX ");
    dtostrf(((systemState.runningMaxX-avgX) * (double)SAFETY_FACTOR) + avgX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    DebugPrintSimple("\n\n");

    DebugPrintSimple("\n (double)SAFETY_FACTOR ");
    dtostrf((double)SAFETY_FACTOR, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    
    DebugPrintSimple("\n running min x ");
    dtostrf(systemState.runningMinX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n avgX ");
    dtostrf(avgX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n (systemState.runningMinX-avgX) ");
    dtostrf((systemState.runningMinX-avgX), precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);

    DebugPrintSimple("\n ((systemState.runningMinX-avgX) * SAFETY_FACTOR) ");
    dtostrf(((systemState.runningMinX-avgX) * (double)SAFETY_FACTOR), precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    
    DebugPrintSimple("\n ((systemState.runningMinX-avgX) * SAFETY_FACTOR) + avgX ");
    dtostrf(((systemState.runningMinX-avgX) * (double)SAFETY_FACTOR) + avgX, precision+3, precision, floatBuffer);
    DebugPrintSimple(floatBuffer);
    DebugPrintSimple("\n\n");
  
  systemState.maxX = ((systemState.runningMaxX-avgX) * (double)SAFETY_FACTOR) + avgX;
  systemState.minX = ((systemState.runningMinX-avgX) * (double)SAFETY_FACTOR) + avgX;
  
  systemState.maxY = ((systemState.runningMaxY-avgY) * (double)SAFETY_FACTOR) + avgY;
  systemState.minY = ((systemState.runningMinY-avgY) * (double)SAFETY_FACTOR) + avgY;
  
  systemState.maxZ = ((systemState.runningMaxZ-avgZ) * (double)SAFETY_FACTOR) + avgZ;
  systemState.minZ = ((systemState.runningMinZ-avgZ) * (double)SAFETY_FACTOR) + avgZ;
  
  
  DebugPrintSimple("MinX: ");
  dtostrf(systemState.minX, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tavgX: ");
  dtostrf(avgX, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tMaxX: ");
  dtostrf(systemState.maxX, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tm/s^2\n");
  
  DebugPrintSimple("MinY: ");
  dtostrf(systemState.minY, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tavgY: ");
  dtostrf(avgY, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tMaxY: ");
  dtostrf(systemState.maxY, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tm/s^2\n");
  
  DebugPrintSimple("MinZ: ");
  dtostrf(systemState.minZ, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tavgZ: ");
  dtostrf(avgZ, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tMaxZ: ");
  dtostrf(systemState.maxZ, precision+3, precision, floatBuffer);
  DebugPrintSimple(floatBuffer);
  DebugPrintSimple("\tm/s^2\n\n\n");
}


/**
 * This method reads the sensor and learns what the normal bounds are of the
 * accerlation values
 */
void learnNormalValues(sensors_event_t a) {
  
  if(systemState.learnCount > 0) {
    systemState.learnCount--;

    systemState.sumX += a.acceleration.x;
    systemState.sumY += a.acceleration.y;
    systemState.sumZ += a.acceleration.z;

    if(a.acceleration.x > systemState.runningMaxX) {
      systemState.runningMaxX = a.acceleration.x;
    }
    if(a.acceleration.x < systemState.runningMinX) {
      systemState.runningMinX = a.acceleration.x;
    }

    if(a.acceleration.y > systemState.runningMaxY) {
      systemState.runningMaxY = a.acceleration.y;
    }
    if(a.acceleration.y < systemState.runningMinY) {
      systemState.runningMinY = a.acceleration.y;
    }

    if(a.acceleration.z > systemState.runningMaxZ) {
      systemState.runningMaxZ = a.acceleration.z;
    }
    if(a.acceleration.z < systemState.runningMinZ) {
      systemState.runningMinZ = a.acceleration.z;
    }  
  }
}

/**
 * This method reads in the signal from the
 * 433mhz remote control that is used to
 * arm/disarm the alarm
 */
void readRemoteControl() {
  int sensorValue = digitalRead(REMOTE_CONTROL_INPUT_PIN);

  // System just turned on state
  if (INITIAL_REMOTE_CONTROL_READ_STATE == systemState.remoteControlInput) {
        systemState.remoteControlInput = sensorValue;
        DebugPrintln("Remote is now NOT_ARMED because we just started\n");
        systemState.state = NOT_ARMED;
        DebugPrintln("NOT_ARMED\n");
        return;
  }

  if (sensorValue != systemState.remoteControlInput) {
    systemState.remoteControlInput = sensorValue;
    if (sensorValue == HIGH) {
      DebugPrintln("Remote is now HIGH");
      systemState.state = NOT_ARMED;
      DebugPrintln("NOT_ARMED\n");
      digitalWrite(ONBOARD_LED_OUTPUT_PIN, LOW);
      flashCarLights(1000);
      delay(500);
      flashCarLights(1000);
    } else {
      DebugPrintln("Remote is now LOW\n");
      resetSystemStateToCalibrate();
      systemState.timeOfLastEvent = 0;
      systemState.state = LEARNING;
      DebugPrintln("LEARNING\n");
      digitalWrite(ONBOARD_LED_OUTPUT_PIN, HIGH);
      flashCarLights(1000);
    }
  }
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
