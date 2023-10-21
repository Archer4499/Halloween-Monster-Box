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
//   Then synced to the audio:
//    Red fade in/out
//    White fade in/out
//    Flickers red
//    Flickers white
//    Motor and smoke activates
//    Flickers between red/white
//    Wait until 10 sec point
//    Goes dark/motor and smoke stops
//    Run motor slowly until lid is closed
//    Wait
//    Little red flickers
//   Deactivate as the audio ends

// Activated: (internal leds only)
//   all lights go dark
//   activation track plays
//   Then synced to the audio:
//    0sec to 1sec Red fade in/out (monster is Red)
//    1-1.5sec White fade in/out (cat is white)
//    strobe red 1.5sec
//    strobe white 1.5sec
//    strobe white and red + Motor and smoke activates
//    Wait until 10 sec point
//    white to full then fade out over 800ms
//    Goes dark/motor and smoke stops
//    Run motor slowly until lid is closed
//    Wait
//    Little red flickers
//   Deactivate as the audio ends

// TODO:
//
// Pushbuttons for the mode select (with debounce)
// Change order of buttons stop(red)|auto(green)|random activation (white)|growl only(blue)
// LED to show the mode status
// 



////////    Config    ////////

//// Constants/Settings ////
#define DEBUG    // Print debug lines to serial, comment out to disable

#define LOOP_TIME 20  // Time in milliseconds between animation loops

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
#define MOTOR_NORMAL_SPEED 4095  // Integer 0-4095
#define MOTOR_SLOW_SPEED 3000  // Integer 0-4095

// Audio
#define AUDIO                  // Enable the DFPlayer Pro audio player, comment out to disable
#define AUDIO_VOLUME 15        // Integer: 0-30
// Audio tracks can by any of MP3, WAV, WMA, FLAC, AAC, APE formats
// Stored on the DFPlayer Pro from its USB port
#define ACTIVATION_TRACK "activation_track.wav"
#define STANDBY_TRACK "standby_track.wav"

// LEDs
#define NUM_LED_BOARDS 2
// Order of the LED boards
#define LED_FRONT_INDEX  0
#define LED_INSIDE_INDEX 1
// Order of the strip wiring to the RGB on each board
#define LED_RED_INDEX   0
#define LED_WHITE_INDEX 1

#define PERCENT(x) static_cast<uint8_t>(x * 255/100)

// LED Effects
#define GLOW_RED_BRIGHTNESS PERCENT(30)

#define CANDLE_BRIGHTNESS   PERCENT(70)
#define CANDLE_INTERVAL     400
#define CANDLE_INTENSITY    0.07f
#define CANDLE_ALPHA        PERCENT(90)

#define BREATHING_MIN       PERCENT(30)
#define BREATHING_MAX       PERCENT(70)
#define BREATHING_INTERVAL  5000

// Remote
// #define WIZMOTE     // Enable the WiZmote ESP-NOW remote control, comment out to disable
                    //  https://www.wizconnected.com/en-us/p/accessory-wizmote/046677603595
// WIZMOTE_BUTTON_ON           On
// WIZMOTE_BUTTON_OFF          Off
// WIZMOTE_BUTTON_NIGHT        Disable Activation (OnlyGrowl)
// WIZMOTE_BUTTON_ONE          Auto (PIR activation)
// WIZMOTE_BUTTON_TWO          Random activation
//  These two aren't intuitive:
// WIZMOTE_BUTTON_THREE        Trigger smoke
// WIZMOTE_BUTTON_FOUR         Trigger activation

// WIZMOTE_BUTTON_BRIGHT_UP    Volume up   (Or Led brightness)
// WIZMOTE_BUTTON_BRIGHT_DOWN  Volume down (Or Led brightness)
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
//   Digital Out - Smoke machine
#define SMOKE_PIN    21

//   PWM         - Motor
#define MOTOR_PIN    22

//   UART        - Audio
#define AUDIO_RX_PIN 16  // RX2
#define AUDIO_TX_PIN 17  // TX2

//   P9813(SPI)  - Front LEDs Red/White + Internal LEDs red/white
#define FASTLED_ALL_PINS_HARDWARE_SPI  // Use hardware SPI. This will be on pins 23(Data) and 18(Clk) unless the next line is also uncommented
// #define FASTLED_ESP32_SPI_BUS HSPI  // Uncomment to use hardware SPI on pins 13(Data) and 14(Clk) instead
// Changing these 2 pin defines doesn't do anything when using hardware SPI
#define LED_DATA_PIN 23
#define LED_CLK_PIN  18
////////

////////    End Config    ////////

// Imports
#include <FastLED.h>
#include <HardwareSerial.h>
#include <DFRobot_DF1201S.h>
////

// Constants / Globals
#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print (x)
  #define DEBUG_PRINTLN(x) Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
#endif

#define MOTOR_PWM_CHANNEL 0    // Only matters if using other PWM channels as well


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
HardwareSerial DF1201SSerial(2);
DFRobot_DF1201S DF1201S;
bool audioInit = false;

// LEDS
CRGB leds[NUM_LED_BOARDS];
struct ledInternal {
  CRGB initial;
  CRGB current;
  CRGB target;
  long startTimes[3];
  long durations[3];
};
ledInternal ledsInternal[NUM_LED_BOARDS];

////


void IRAM_ATTR PIRInterrupt() {
  PIRDetected = true;
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

float random_float() {
  return static_cast<float>(random(UINT32_MAX-1)) / static_cast<float>(UINT32_MAX);
}

inline static float random_cubic_float() {
  const float r = random_float() * 2.0f - 1.0f;
  return r * r * r;
}

bool ledCurrentlyFading(int boardIndex) {
  return (ledsInternal[boardIndex].current != ledsInternal[boardIndex].target);
}

bool ledCurrentlyFading(int boardIndex, int ledIndex) {
  return (ledsInternal[boardIndex].current[ledIndex] != ledsInternal[boardIndex].target[ledIndex]);
}

void ledProcessFades() {
  long currTime = millis();

  for (int i = 0 ; i < NUM_LED_BOARDS; i++) {
    // CRGB newColour = leds[i];
    for (int j = 0 ; j < 3; j++) {
      if (ledCurrentlyFading(i, j)) {
        uint8_t initial = ledsInternal[i].initial[j];
        // uint8_t current = ledsInternal[i].current[j];
        uint8_t target  = ledsInternal[i].target[j];
        long startTime  = ledsInternal[i].startTimes[j];
        long duration   = ledsInternal[i].durations[j];
        long currDuration = currTime - startTime;

        // Instant animation or finished animation
        if (duration == 0 || currDuration >= duration) {
          ledsInternal[i].current[j] = target;
          break;
        }

        // Lerp
        ledsInternal[i].current[j] = ((target - initial) * currDuration/duration) + initial;
      }

      leds[i][j] = dim8_video(ledsInternal[i].current[j]);
      // leds[i][j] = dim8_lin(ledsInternal[i].current[j]);
    }
  }
}

void ledFade(int boardIndex, int ledIndex, uint8_t target, long duration) {
  ledsInternal[boardIndex].initial[ledIndex] = ledsInternal[boardIndex].current[ledIndex];
  ledsInternal[boardIndex].target[ledIndex] = target;
  ledsInternal[boardIndex].startTimes[ledIndex] = millis();
  ledsInternal[boardIndex].durations[ledIndex] = duration;
}

void ledSet(int boardIndex, int ledIndex, uint8_t target) {
  ledFade(boardIndex, ledIndex, target, 0);
}

void ledsOff(long duration) {
  for (int i = 0 ; i < NUM_LED_BOARDS; i++) {
    for (int j = 0 ; j < 3; j++) {
      ledFade(i, j, 0, duration);
    }
  }
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
      // TODO: possibly move motor until lid is closed first
      ledcWrite(MOTOR_PWM_CHANNEL, 0);

      // Stop Audio
      if (audioInit) {
        DF1201S.pause();
      }

      // Turn off LEDs
      ledsOff(1000);
    }
  }

  if (currentMode != Stop) {
    bool newActivatedState = checkActivatedState();

    if (newActivatedState != activatedState) {
      // TODO: need to also run this when the mode changes and when program starts
      // State changed, so do one time actions that the new state needs
      activatedState = newActivatedState;
      
      if (newActivatedState) {
        // On activation actions
        ledsOff(0);

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

        ledFade(LED_FRONT_INDEX, LED_RED_INDEX, GLOW_RED_BRIGHTNESS, 500);
        ledFade(LED_FRONT_INDEX, LED_WHITE_INDEX, CANDLE_BRIGHTNESS, 200);
      }
    }

    // Do the continuous actions such as animations
    if (activatedState) {
      // Activated loop

    } else {
      // Standby loop

      // Candle flicker
      if (!ledCurrentlyFading(LED_FRONT_INDEX, LED_WHITE_INDEX)) {
        // ledFade(LED_FRONT_INDEX, LED_WHITE_INDEX, inoise8(millis()), 200);
        // uint8_t flicker = qadd8(CANDLE_BRIGHTNESS, inoise8(millis())/10);
        // uint8_t flicker = qadd8(CANDLE_BRIGHTNESS, dim8_video(inoise8(millis())/2));
        // uint8_t flicker = qadd8(blend8(CANDLE_BRIGHTNESS,
        //                                ledsInternal[LED_FRONT_INDEX].current[LED_WHITE_INDEX],
        //                                CANDLE_ALPHA),
        //                         dim8_video(inoise8(millis())/3));
        uint8_t starting_blend = blend8(CANDLE_BRIGHTNESS,
                                  ledsInternal[LED_FRONT_INDEX].current[LED_WHITE_INDEX],
                                  CANDLE_ALPHA);
        int8_t random_flicker = static_cast<int8_t>(ease8InOutCubic(random8())) * CANDLE_INTENSITY;
        uint8_t flicker;
        if (random_flicker < 0) {
          flicker = qsub8(starting_blend, -random_flicker);
        } else {
          flicker = qadd8(starting_blend, random_flicker);
        }

        ledFade(LED_FRONT_INDEX, LED_WHITE_INDEX, flicker, CANDLE_INTERVAL);
      }

      // Breathing animation
      // TODO: maybe use easing on the breathing
      // TODO: maybe slight pause when dark
      if (!ledCurrentlyFading(LED_INSIDE_INDEX, LED_RED_INDEX)) {
        if (ledsInternal[LED_INSIDE_INDEX].current[LED_RED_INDEX] < BREATHING_MAX) {
          ledFade(LED_INSIDE_INDEX, LED_RED_INDEX, BREATHING_MAX, BREATHING_INTERVAL);
        } else {
          ledFade(LED_INSIDE_INDEX, LED_RED_INDEX, BREATHING_MIN, BREATHING_INTERVAL);
        }
      }
    }

    // TODO: white inside?
    // ledSet(LED_INSIDE_INDEX, LED_WHITE_INDEX, random(255));
    
    // if (digitalRead(LID_PIN) == HIGH) {
    //   // Lid open
    // }

    // ledcWrite(MOTOR_PWM_CHANNEL, 0);

    // ledSet(LED_FRONT_INDEX, LED_RED_INDEX, 100);
    // ledSet(LED_FRONT_INDEX, LED_WHITE_INDEX, 200);
    // FastLED.show();
    // delay(500);
    
    // ledSet(LED_FRONT_INDEX, LED_RED_INDEX, 0);
    // ledSet(LED_FRONT_INDEX, LED_WHITE_INDEX, 100);

    // ledcWrite(MOTOR_PWM_CHANNEL, 1024);

  }

  ledProcessFades();
  FastLED.show();
  delay(LOOP_TIME);
}
