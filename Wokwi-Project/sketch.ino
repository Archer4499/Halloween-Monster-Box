// Inputs
//   Mode switch - stop|auto|growl only|random activation
//   PIR sensor
//   Lid switch (to detect the lid is down)

// Outputs
//   Smoke machine
//   Motor (to make the lid jump)
//   Audio trigger - standby track (growling) Activation Track
//   Front LED strip Red and White (run from a digital led mosfet)
//   Internal LED Strip red and white (run from a digital mosfet)

// Standby:
//   audio track loops
//   front white leds candle flicker
//   front red leds at 30%
//   inside red leds "breathe" between 30% and 70% brightness
//   (might have motor pulse every now and again to move the lid up and down havent decided)

// Activated:
//   all lights go dark
//   activation track plays

// If Lid open:
//   motor to pulse until the lid is shut


////////    Config    ////////

//// Constants/Settings ////
#define DEBUG    // Print debug lines to serial, comment out to disable

#define LOOP_TIME 50  // Time in milliseconds between animation loops

// Random Mode
#define RANDOM_START_MIN 10000  // Minimum time before random mode could activate
#define RANDOM_START_MAX 15000  // Maximum time before random mode could activate
#define RANDOM_STOP_MIN  10000  // Minimum time before random mode could deactivate
#define RANDOM_STOP_MAX  15000  // Maximum time before random mode could deactivate

// PIR
#define PIR_TIMEOUT  10000  // Timeout in miliseconds after the PIR was last activated

// Motor
// PWM Frequency/Bit Depth/Steps
// 1220Hz   16  65536
// 2441Hz   15  32768
// 4882Hz   14  16384
// 9765Hz   13  8192
// 19531Hz  12  4096
// The code is using a bit depth of 10
// Not sure what frequency is best for the motor, 16-20kHz is claimed to reduce noise, but it might not handle high frequencies and the mosfet might run hot
#define MOTOR_PWM_FREQ 16000
#define MOTOR_PWM_CHANNEL 0    // Only matters if using other PWM channels as well

// Audio
#define AUDIO                  // Enable the DFPlayer Pro audio player, comment out to disable
#define AUDIO_VOLUME 15        // 0-30
// Audio tracks can by any of MP3, WAV, WMA, FLAC, AAC, APE formats
// Stored on the DFPlayer Pro from its USB port
#define ACTIVATION_TRACK "activation_track.wav"
#define STANDBY_TRACK "standby_track.wav"
#define DF1201SSerial Serial2  // Serial 2 is the only completely unused one on ESP32

// LEDs
#define NUM_LED_BOARDS 2
// Orders of the LED boards and strip wiring
#define LED_FRONT_INDEX  0
#define LED_INSIDE_INDEX 1
#define LED_RED_INDEX   0
#define LED_WHITE_INDEX 1

// Remote
#define WIZMOTE     // Enable the WiZmote ESP-NOW remote control, comment out to disable
                    //  https://www.wizconnected.com/en-us/p/accessory-wizmote/046677603595
////////

//// Pins ////
// It is recommended to use these pins for the audio UART, but any should be able to be used if needed:
// 16/RX2  17/TX2
//
// For the LEDs to use hardware SPI it needs to be either of the following sets of pins:
//  23(Data) and 18(Clk) (May also limit use of pins 5 and 19)
//   or
//  13(Data) and 14(Clk) (May also limit use of pins 12 and 15)
// Alternatively software SPI can be used on any other set of pins
//
// Any of the following pins can be used for the rest of the functions:
// 4   13  18  21  22  23
// 25  26  27  32  33
//
// The following can only be used for input, and have no pullups, so not really recommended:
// 34  35  36/VP

// Inputs:
//   Digital In Pullup    - Mode switch (Alternatively 1 analog pin with resistors)
#define MODE_PIN_1 27  // Stop
#define MODE_PIN_2 26  // Auto
#define MODE_PIN_3 25  // GrowlOnly
#define MODE_PIN_4 33  // Random

//   Digital In Interrupt - PIR sensor
#define PIR_PIN    13

//   Digital In Pullup    - Lid switch
#define LID_PIN    32

// Outputs:
//   Digital Out  - Smoke machine
#define SMOKE_PIN    21

//   PWM          - Motor
#define MOTOR_PIN    22

//   UART         - Audio
#define AUDIO_RX_PIN 16  // RX2
#define AUDIO_TX_PIN 17  // TX2

//   P9813(1 PWM) - Front LEDs Red/White + Internal LEDs red/white
#define FASTLED_ALL_PINS_HARDWARE_SPI  // Use hardware SPI. This will be on pins 23(Data) and 18(Clk) unless the next line is also uncommented
// #define FASTLED_ESP32_SPI_BUS HSPI  // Uncomment to use hardware SPI on pins 13(Data) and 14(Clk) instead
// Changing these 2 pin defines doesn't do anything when using hardware SPI
#define LED_DATA_PIN 23
#define LED_CLK_PIN  18
////////

////////    End Config    ////////

// Imports
#include <FastLED.h>
#include <DFRobot_DF1201S.h>
////

// TODO:
//
// Method of learning new WiZmote remotes, maybe when the moon key is pressed 5 times within x sec?


// Globals
#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print (x)
  #define DEBUG_PRINTLN(x) Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
#endif

enum ActivationMode {
  Stop,
  Auto,
  GrowlOnly,
  Random
};
ActivationMode currentMode = Auto;

bool activatedState = false;
volatile bool PIRDetected = false;
volatile long lastActivationTime = 0;
long lastDeactivationTime = 0;
long randomStartTime = random(RANDOM_START_MIN, RANDOM_START_MAX);
long randomStopTime = random(RANDOM_STOP_MIN, RANDOM_STOP_MAX);

// Audio
DFRobot_DF1201S DF1201S;
bool audioInit = false;

// LEDS
CRGB leds[NUM_LED_BOARDS];

////


void IRAM_ATTR PIRInterrupt() {
  PIRDetected = true;
}

void ledsOff() {
  leds[LED_FRONT_INDEX] =  CRGB(0, 0, 0);
  leds[LED_INSIDE_INDEX] = CRGB(0, 0, 0);
  FastLED.show();
}

ActivationMode readMode() {
  ActivationMode result = Auto;

  if (digitalRead(MODE_PIN_1) == LOW) {
    result = Stop;
  } else if (digitalRead(MODE_PIN_2) == LOW) {
    result = Auto;
  } else if (digitalRead(MODE_PIN_3) == LOW) {
    result = GrowlOnly;
  } else if (digitalRead(MODE_PIN_4) == LOW) {
    result = Random;
  }
  return result;
}

bool checkActivatedState() {
  // PIRDetected is always set back to false so if it's true then we know it is a new activation
  if (PIRDetected) {
    PIRDetected = false;
    // Test for if we actually care about the activation
    if (currentMode == Auto) {
      lastActivationTime = millis();
      return true;
    }
  }

  if (activatedState) {
    // Should deactivate?
    if (currentMode == Auto) {
      if (millis() - lastActivationTime > PIR_TIMEOUT) {
        lastDeactivationTime = millis();
        return false;
      }
    } else if (currentMode == Random) {
      if (millis() - lastActivationTime > randomStopTime) {
        lastDeactivationTime = millis();
        randomStopTime = random(RANDOM_STOP_MIN, RANDOM_STOP_MAX);
        return false;
      }
    }
    return true;
  }
   
  // Should Activate?
  if (currentMode == Random) {
    if (millis() - lastDeactivationTime > randomStartTime) {
      lastActivationTime = millis();
      randomStartTime = random(RANDOM_START_MIN, RANDOM_START_MAX);
      return true;
    }
  }

  return false;
}


void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  randomSeed(analogRead(39));  // Use random noise from unconnected pin read

  // Inputs:
  // Mode switch
  pinMode(MODE_PIN_1, INPUT_PULLUP);
  pinMode(MODE_PIN_2, INPUT_PULLUP);
  pinMode(MODE_PIN_3, INPUT_PULLUP);
  pinMode(MODE_PIN_4, INPUT_PULLUP);
  // PIR sensor
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(PIR_PIN, PIRInterrupt, RISING);
  // Lid switch
  pinMode(LID_PIN, INPUT_PULLUP);

  // Outputs:
  // Smoke machine
  pinMode(SMOKE_PIN, OUTPUT);
  // Motor
  pinMode(MOTOR_PIN, OUTPUT);
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, 10);
  ledcWrite(MOTOR_PWM_CHANNEL, 0);  // Make sure 0% PWM output to start with
  ledcAttachPin(MOTOR_PIN, MOTOR_PWM_CHANNEL);

  // Audio
#ifdef AUDIO
  DF1201SSerial.begin(115200, SERIAL_8N1, AUDIO_RX_PIN, AUDIO_TX_PIN);
  if (DF1201S.begin(DF1201SSerial)) {
    audioInit = true;

    DF1201S.setVol(AUDIO_VOLUME);
    DF1201S.switchFunction(DF1201S.MUSIC);
    // Not sure what it means by the prompt tone from its example?:
    // /*Wait for the end of the prompt tone */
    // delay(2000);
    // DF1201S.setPlayMode(DF1201S.SINGLECYCLE);
  } else {
    DEBUG_PRINTLN("Audio init failed, please check audio wire connection");
  }
#endif

  // Front LEDs Red/White + Internal LEDs red/white
  // FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LED_BOARDS);
  FastLED.addLeds<P9813, LED_DATA_PIN, LED_CLK_PIN>(leds, NUM_LED_BOARDS);
}


void loop() {
  // Check what state we should be in from the switch
  ActivationMode newMode = readMode();
  if (newMode != currentMode) {
    // Mode changed, so reset and setup anything the new mode needs
    currentMode = newMode;

    activatedState = false;
    PIRDetected = false;
    lastDeactivationTime = millis();

    if (newMode == Stop) {
      // Stop Smoke
      digitalWrite(SMOKE_PIN, LOW);

      // Stop Motor
      ledcWrite(MOTOR_PWM_CHANNEL, 0);

      // Stop Audio
      if (audioInit) {
        DF1201S.pause();
      }

      // Turn off LEDs
      ledsOff();
    }
  }

  if (currentMode != Stop) {
    bool newActivatedState = checkActivatedState();

    if (newActivatedState != activatedState) {
      // State changed, so do one time actions that the new state needs
      activatedState = newActivatedState;
      
      if (newActivatedState) {
        // On activation actions
        ledsOff();

        if (audioInit) {
          DF1201S.setPlayMode(DF1201S.SINGLE);
          DF1201S.playSpecFile(ACTIVATION_TRACK);
          // Maybe needs this start command?
          // DF1201S.start();
        }
      } else {
        // On standby actions
        if (audioInit) {
          DF1201S.setPlayMode(DF1201S.SINGLECYCLE);
          DF1201S.playSpecFile(STANDBY_TRACK);
          // Maybe needs this start command?
          // DF1201S.start();
        }
        
        leds[LED_FRONT_INDEX][LED_RED_INDEX] = 255*0.3;  // 30%
      }
    }

    // Do the continuous actions such as animations
    if (activatedState) {
      // Activated loop
      
    } else {
      // Standby loop

      // TODO: Candle filcker
      leds[LED_FRONT_INDEX][LED_WHITE_INDEX] = 200;
      // TODO: Breathe
      leds[LED_INSIDE_INDEX][LED_RED_INDEX] = 100;
      // TODO: ?
      leds[LED_INSIDE_INDEX][LED_WHITE_INDEX] = random(255);
      FastLED.show();

    }


    // if (digitalRead(LID_PIN) == HIGH) {
    //   // Lid open
    // }

    // ledcWrite(MOTOR_PWM_CHANNEL, 0);

    // leds[LED_FRONT_INDEX][LED_RED_INDEX] = 100;
    // leds[LED_FRONT_INDEX][LED_WHITE_INDEX] = 200;
    // FastLED.show();
    // delay(500);
    
    // leds[LED_FRONT_INDEX][LED_RED_INDEX] = 0;
    // leds[LED_FRONT_INDEX][LED_WHITE_INDEX] = 100;

    // ledcWrite(MOTOR_PWM_CHANNEL, 1024);

    // FastLED.show();
  }

  delay(LOOP_TIME);
}
