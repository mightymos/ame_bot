/*
 * robot stuff
 */

#include <Arduino.h>

#define VERSION_EXAMPLE "0.1"

/*
 * Set input pin and output pin definitions etc.
 */
//#include "PinDefinitionsAndMore.h"

// infrared remote library
// only include support for remote that comes in sparkfun infrared control kit
// https://www.sparkfun.com/products/14677
#define IRMP_PROTOCOL_NAMES 1
#define IRMP_USE_COMPLETE_CALLBACK 1
#define IRMP_ENABLE_PIN_CHANGE_INTERRUPT

#define IRMP_SUPPORT_NEC_PROTOCOL 1
#define IRMP_INPUT_PIN 2

#include <irmp.hpp>

// adafruit motor driver library
#include <AFMotor.h>

// infrared packet?
IRMP_DATA irmp_data;
bool sJustReceived;
void handleReceivedIRData();

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

// LED for "breathing" pattern
//const int LED_BREATH_PIN = 5;

// pin definitions in adafruit motor library
// MOTORCLK 4, MOTORENABLE 7, MOTORDATA 8, MOTORLATCH 12
// PWM1 11, PWM2 3, PWM3 6, PWM4 5
// create motor #1, 64KHz pwm

// back left
AF_DCMotor motor1(1, MOTOR12_64KHZ);
// back right
AF_DCMotor motor2(2, MOTOR12_1KHZ);
// front right
AF_DCMotor motor3(3, MOTOR34_1KHZ);
// front left
AF_DCMotor motor4(4, MOTOR34_1KHZ); 

void setup() {
    // initialize the digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(115200);
    while (!Serial)
    
    // just to know which program was flasehd on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));
  
    irmp_init();
    //irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at LED_BUILTIN
    irmp_register_complete_callback_function(&handleReceivedIRData);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    irmp_print_active_protocols(&Serial);
    Serial.println(F("at pin " STR(IRMP_INPUT_PIN)));

    // set the speed to 200/255
    Serial.println("Motor: set speed");
    motor1.setSpeed(200);
    motor2.setSpeed(200);
    motor3.setSpeed(200);
    motor4.setSpeed(200);

    // necessary?
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(RELEASE);
}

void loop() {
  volatile bool runMotorForward = false;
  volatile bool runMotorBackward = false;
  volatile bool stopMotor = false;
  
    if (sJustReceived) {
        sJustReceived = false;

        // skip repetitions of command
        //if (!(irmp_data.flags & IRMP_FLAG_REPETITION)) {
          switch(irmp_data.command)
          {
            case 0x18:
              Serial.println("stop motor");
              stopMotor = true;
              break;
            case 0x19:
              Serial.println("motor forward");
              runMotorForward = true;
              break;
            case 0x1C:
              Serial.println("motor backward");
              runMotorBackward = true;
              break;
            default:
              Serial.println("ir: unmapped command");
              break;
          }
        //}

        // this is not allowed in ISR context for any kind of RTOS
        irmp_result_print(&irmp_data);
    }

    if (runMotorForward)
    {
      Serial.println("forward");

      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);

      delay(100);
      
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);

      runMotorForward = false;
    }

    if (runMotorBackward)
    {
      Serial.println("backward");

      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);

      delay(100);
      
      motor1.run(RELEASE);
      motor2.run(RELEASE);
      motor3.run(RELEASE);
      motor4.run(RELEASE);

      runMotorBackward = false;
    }
}


/*
 * Here we know, that data is available.
 * Since this function is executed in Interrupt handler context, make it short and do not use delay() etc.
 * In order to enable other interrupts you can call sei() (enable interrupt again) after getting data.
 */
#if defined(ESP8266) || defined(ESP32)
void IRAM_ATTR handleReceivedIRData()
#else
void handleReceivedIRData()
#endif
{
    irmp_get_data(&irmp_data);
#if defined(ARDUINO_ARCH_MBED) || defined(ESP32)
    sJustReceived = true; // Signal new data for main loop, this is the recommended way for handling a callback :-)
#else
    //interrupts(); // enable interrupts
    //irmp_result_print(&irmp_data); // this is not allowed in ISR context for any kind of RTOS
    sJustReceived = true;
#endif
}
