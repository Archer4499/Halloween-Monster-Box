// Inputs
//   Mode buttons - stop|auto|random activation|growl only
//   PIR sensor
//   Lid closed reed switch
//   WiZmote remote control

// Outputs
//   Mode LED - stop(red)|auto(green)|random activation (white)|growl only(blue)
//   Smoke machine trigger
//   Motor (to make the lid jump)
//   Audio (Standby track (growling) & activation track)
//   Front LED strip Red and White (run from a digital led mosfet)
//   Internal LED strip Red and White (run from a digital led mosfet)

// Actions during standby:
//   Audio track loops
//   Front white leds candle flicker
//   Front red leds at 30%
//   Inside red leds "breathe" between 30% and 70% brightness
//   (might have motor pulse every now and again to move the lid up and down havent decided)

// Activated process:
//  Stage 0 (0s):
//   All lights go dark
//   Activation track plays
//  Stage 1 (0s-1.2s):
//   Internal Red fade in/out (monster is Red)
//  Stage 2 (1.2s-2.2s):
//   Internal White fade in/out (cat is white)
//  Stage 3 (2.2s-3.5s):
//   Internal red strobe
//  Stage 4 (3.5s-4.2s):
//   Internal white strobe
//  Stage 5 (4.2s-10.5s):
//   Internal white and red strobe + Motor and smoke activates
//  Stage 6 (10.5s-11.5s):
//   Internal white to full then fade out
//   Everything stops
//  Stage 7 (11.5s-15s):
//   Run motor slowly until lid is closed then wait
//  Stage 8 (15s-18s):
//   Little red flickers
//  Stage 9 (18s-Audio end (19.632s)):
//   Wait then switch back to standby


////////    Config    ////////

//// Constants/Settings ////
#define DEBUG  // Print debug lines to serial, comment out to disable
// #define DEBUG_INTERRUPT  // Print debug lines for the interrupt functions, this can have major performance issues if buttons bounce too much, comment out to disable
// #define WOKWI  // Uncomment if running in the Wokwi simulator (Changes LED output method)

#define LOOP_TIME      20   // Time in milliseconds between animation loops

// Random Mode
#define RANDOM_START_MIN 100000  // Minimum time before random mode could activate
#define RANDOM_START_MAX 200000  // Maximum time before random mode could activate

// Auto/PIR
#define AUTO_ACTIVATION_COOLDOWN 10000  // Cooldown after activation finishes before the PIR will trigger another actiavtion

// Smoke
#define SMOKE_DURATION 2500  // Duration in ms of the smoke to run during an activation

// Motor
#define MOTOR_PULSE_DURATION 200   // Time in ms of each pulse when making sure the lid is closed
#define MOTOR_PULSE_INTERVAL 600   // Time in ms between each pulse when making sure the lid is closed
#define MOTOR_DURATION       7300  // Duration in ms of the motor to run during an activation

// Audio
#define AUDIO                  // Enable the DFPlayer Pro audio player, comment out to disable
#define AUDIO_VOLUME 25        // Integer: 0-30
// Audio tracks can by any of MP3, WAV, WMA, FLAC, AAC, APE formats
// Stored on the DFPlayer Pro from its USB port
#define ACTIVATION_TRACK_LENGTH 19632  // Length in ms of the activation audio file
#define ACTIVATION_TRACK        "/02.mp3"
#define STANDBY_TRACK           "/01.mp3"
#define BLANK_TRACK             "/03.mp3"

// LEDs
#define GAMMA_CORRECT  // Gamma correction on the birghtness of all LEDs, comment out to disable
#define NUM_LED_BOARDS   2
// Order of the LED boards
#define LED_INSIDE_INDEX 0
#define LED_FRONT_INDEX  1
// Order of the strip wiring to the RGB on each board
#define LED_RED_INDEX    0
#define LED_WHITE_INDEX  1

#define PERCENT(x) static_cast<uint8_t>(x * 255/100)  // Allows using PERCENT(x) to change a brightness from 0-100 into 0-255

// LED Effects
#define GLOW_RED_BRIGHTNESS PERCENT(30)

#define CANDLE_BRIGHTNESS   PERCENT(30)
#define CANDLE_INTERVAL     200
#define CANDLE_INTENSITY    0.12f
#define CANDLE_ALPHA        PERCENT(90)

#define BREATHING_MIN       PERCENT(30)
#define BREATHING_MAX       PERCENT(80)
#define BREATHING_INTERVAL  3000

#define STROBE_MAX          PERCENT(90)

// Remote
#define WIZMOTE     // Enable the WiZmote ESP-NOW remote control, comment out to disable
                    //  https://www.wizconnected.com/en-us/p/accessory-wizmote/046677603595
// Configured button actions:
//   Button:      Action:
//   ON           On (Auto)
//   OFF          Off
//   NIGHT        Disable Activation (OnlyGrowl)

//   ONE          Auto (PIR activation)
//   TWO          Random activation
//   THREE        Trigger activation
//   FOUR         Trigger smoke

//   BRIGHT_UP    Volume up   (Or Led brightness)
//   BRIGHT_DOWN  Volume down (Or Led brightness)
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
//  4  13  18  21  22  23
// 25  26  27  32  33
//
// The following can only be used for input, and have no pullups, so not really recommended:
// 34  35  36/VP

// Inputs:
//   Digital In Pullup    - Mode buttons
#define MODE_PIN_1 27  // Stop
#define MODE_PIN_2 14  // Auto
#define MODE_PIN_3 26  // Random
#define MODE_PIN_4 25  // Growl only

//   Digital In Interrupt - PIR sensor
#define PIR_PIN    13

//   Digital In Pullup    - Lid switch
#define LID_PIN    32

// Outputs:
//   Digital Out - Mode LED
#define MODE_LED_PIN    4

//   Digital Out - Smoke machine
#define SMOKE_PIN    21

//   Digital Out - Motor
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

#ifdef WIZMOTE
  #include <WiFi.h>
  #include <esp_now.h>
#endif
////

// Constants / Globals
#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print (x)
  #define DEBUG_PRINTLN(x) Serial.println (x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x) 
#endif
#ifdef DEBUG_INTERRUPT
  #define DEBUG_INT(x) x
#else
  #define DEBUG_INT(x)
#endif

#ifdef WIZMOTE
  #define WIZMOTE_BUTTON_ON          1
  #define WIZMOTE_BUTTON_OFF         2
  #define WIZMOTE_BUTTON_NIGHT       3
  #define WIZMOTE_BUTTON_ONE         16
  #define WIZMOTE_BUTTON_TWO         17
  #define WIZMOTE_BUTTON_THREE       18
  #define WIZMOTE_BUTTON_FOUR        19
  #define WIZMOTE_BUTTON_BRIGHT_UP   9
  #define WIZMOTE_BUTTON_BRIGHT_DOWN 8

  // Pulled from the WiZmote product spec.
  typedef struct message_structure {
    uint8_t program;      // 0x91 for ON button, 0x81 for all others
    uint8_t seq[4];       // Incremetal sequence number 32 bit unsigned integer LSB first
    uint8_t byte5 = 32;   // Unknown
    uint8_t button;       // Identifies which button is being pressed
    uint8_t byte8 = 1;    // Unknown, but always 0x01
    uint8_t byte9 = 100;  // Unnkown, but always 0x64

    uint8_t byte10;  // Unknown, maybe checksum
    uint8_t byte11;  // Unknown, maybe checksum
    uint8_t byte12;  // Unknown, maybe checksum
    uint8_t byte13;  // Unknown, maybe checksum
  } message_structure;

  static message_structure incoming;
  char linked_remote[13]   = "";
  char last_signal_src[13] = "";
  static uint32_t last_seq = UINT32_MAX;
#endif


enum ActivationMode {
  MODE_STOP,
  MODE_AUTO,
  MODE_RANDOM,
  MODE_GROWL_ONLY,
};
ActivationMode currentMode  = MODE_AUTO;
volatile ActivationMode newMode = MODE_AUTO;

enum AnimationState {
  ANIM_STANDBY_INIT,
  ANIM_STANDBY,
  ANIM_STAGE_0,
  ANIM_STAGE_1,
  ANIM_STAGE_2,
  ANIM_STAGE_3,
  ANIM_STAGE_4,
  ANIM_STAGE_5,
  ANIM_STAGE_6,
  ANIM_STAGE_7,
  ANIM_STAGE_8,
  ANIM_STAGE_9,
};
volatile AnimationState currentState = ANIM_STANDBY_INIT;

long activationTime = 0;
long randomStartTime = random(RANDOM_START_MIN, RANDOM_START_MAX);
long lastStandbyTime = 0;

long smokeStartTime = 0;
long smokeDuration  = 0;

long motorStartTime = 0;
long motorDuration  = 0;

// Audio
HardwareSerial DF1201SSerial(2);
DFRobot_DF1201S DF1201S;
bool audioInit = false;

// LEDS
CRGB modeLed = CRGB::Green;

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
  DEBUG_INT(DEBUG_PRINTLN("PIR"));
  if (currentMode == MODE_AUTO && currentState == ANIM_STANDBY) {
    if (millis() - lastStandbyTime > AUTO_ACTIVATION_COOLDOWN) {
      DEBUG_PRINTLN("PIR trigger valid");
      currentState = ANIM_STAGE_0;
    }
  }
}

void IRAM_ATTR modeButton1Interrupt() {
  DEBUG_INT(DEBUG_PRINTLN("B1"));
  newMode = MODE_STOP;
}
void IRAM_ATTR modeButton2Interrupt() {
  DEBUG_INT(DEBUG_PRINTLN("B2"));
  newMode = MODE_AUTO;
}
void IRAM_ATTR modeButton3Interrupt() {
  DEBUG_INT(DEBUG_PRINTLN("B3"));
  newMode = MODE_RANDOM;
}
void IRAM_ATTR modeButton4Interrupt() {
  DEBUG_INT(DEBUG_PRINTLN("B4"));
  newMode = MODE_GROWL_ONLY;
}


#ifdef WIZMOTE
  // Callback function that will be executed when ESP-NOW data is received
  void wizmoteDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (len != sizeof(incoming)) {
      DEBUG_PRINT("Unknown incoming ESP Now message received of length ");
      DEBUG_PRINTLN(len);
      return;
    }

    memcpy(&(incoming.program), incomingData, sizeof(incoming));

    // Reject duplicate messages  
    uint32_t cur_seq = incoming.seq[0] | (incoming.seq[1] << 8) | (incoming.seq[2] << 16) | (incoming.seq[3] << 24);
    if (cur_seq == last_seq) {
      DEBUG_PRINT("ESP Now Duplicate Message Received with sequence number: ");
      DEBUG_PRINTLN(cur_seq);
      return;
    }

    sprintf(last_signal_src, "%02x%02x%02x%02x%02x%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (strcmp(last_signal_src, linked_remote) != 0) {
      DEBUG_PRINT("ESP Now Message Received from Unlinked Sender: ");
      // TODO: learn code if button sequence, maybe when the moon key is pressed 5 times within x sec?
      DEBUG_PRINTLN(last_signal_src);
      return;
    }

    DEBUG_PRINT("Incoming ESP Now Packet["); DEBUG_PRINT(cur_seq);
    DEBUG_PRINT("] from sender[");   DEBUG_PRINT(last_signal_src);
    DEBUG_PRINT("] button: ");     DEBUG_PRINTLN(incoming.button);
    switch (incoming.button) {
      case WIZMOTE_BUTTON_ON             : newMode = MODE_AUTO;       break;  // TODO: maybe return to last on mode?
      case WIZMOTE_BUTTON_OFF            : newMode = MODE_STOP;       break;
      case WIZMOTE_BUTTON_NIGHT          : newMode = MODE_GROWL_ONLY; break;
      case WIZMOTE_BUTTON_ONE            : newMode = MODE_AUTO;       break;
      case WIZMOTE_BUTTON_TWO            : newMode = MODE_RANDOM;     break;
      case WIZMOTE_BUTTON_THREE          : if(currentState == ANIM_STANDBY) currentState = ANIM_STAGE_0;  break;
      case WIZMOTE_BUTTON_FOUR           : smokeRun(SMOKE_DURATION);  break;
      case WIZMOTE_BUTTON_BRIGHT_UP      : audioVolumeChange(2);      break;
      case WIZMOTE_BUTTON_BRIGHT_DOWN    : audioVolumeChange(-2);     break;
    }

    last_seq = cur_seq;
  }
#endif


bool isLidClosed() {
  return (digitalRead(LID_PIN) == LOW);
}

void smokeProcess() {
  long currTime = millis();
  long currDuration = currTime - smokeStartTime;

  // Instant animation or finished animation
  if (smokeDuration == 0 || currDuration >= smokeDuration) {
    digitalWrite(SMOKE_PIN, LOW);  // Stop
    return;
  }

  // Else we should be running
  digitalWrite(SMOKE_PIN, HIGH);   // Start
}
void motorProcess() {
  long currTime = millis();
  long currDuration = currTime - motorStartTime;

  // Instant animation or finished animation
  if (motorDuration == 0 || currDuration >= motorDuration) {
    // Pulse the motor until it is closed if we aren't in stop mode
    if (currDuration >= (motorDuration + MOTOR_PULSE_INTERVAL)) {
      if (currentMode != MODE_STOP && !isLidClosed()) {
        motorRun(MOTOR_PULSE_DURATION);
      }
    }
    digitalWrite(MOTOR_PIN, LOW);  // Stop
    return;
  }

  // Else we should be running
  digitalWrite(MOTOR_PIN, HIGH);   // Start
}

void smokeRun(long duration) {
  smokeStartTime = millis();
  smokeDuration = duration;
}

void motorRun(long duration) {
  motorStartTime = millis();
  motorDuration = duration;
}


void audioPlayStandbyTrack() {
  if (audioInit) {
    if (!DF1201S.setPlayMode(DF1201S.SINGLECYCLE))
      DEBUG_PRINTLN("Failed to set play mode: repeat, before standby");
    if (!DF1201S.playSpecFile(STANDBY_TRACK))
      DEBUG_PRINTLN("Failed to play standby track");
  }
}
void audioPlayActivationTrack() {
  if (audioInit) {
    if (!DF1201S.setPlayMode(DF1201S.SINGLE))
      DEBUG_PRINTLN("Failed to set play mode: single, before activation");
    if (!DF1201S.playSpecFile(ACTIVATION_TRACK))
      DEBUG_PRINTLN("Failed to play activation track");
  }
}
bool audioStop() {
  if (audioInit) {
    if (!DF1201S.setPlayMode(DF1201S.SINGLECYCLE))
      DEBUG_PRINTLN("Failed to set play mode: repeat, before stopping");
    if (!DF1201S.playSpecFile(BLANK_TRACK)) {
      DEBUG_PRINTLN("Failed to stop audio by playing blank track");
      return false;
    }
  }
  return true;
}
void audioVolumeChange(int8_t change) {
  if (audioInit) {
    int8_t volume = (int8_t)DF1201S.getVol();
    volume += change;
    volume = constrain(volume, 0, 30);
    if (!DF1201S.setVol(volume))
      DEBUG_PRINTLN("Failed to set audio volume");
  }
}

bool ledCurrentlyFading(int boardIndex) {
  return (ledsInternal[boardIndex].current != ledsInternal[boardIndex].target);
}

bool ledCurrentlyFading(int boardIndex, int ledIndex) {
  return (ledsInternal[boardIndex].current[ledIndex] != ledsInternal[boardIndex].target[ledIndex]);
}

void ledCandleFlicker(int boardIndex, int ledIndex) {
  if (!ledCurrentlyFading(boardIndex, ledIndex)) {
    // Other candle flicker animation attempts:
    //   uint8_t flicker = inoise8(millis());
    //   uint8_t flicker = qadd8(CANDLE_BRIGHTNESS, inoise8(millis())/10);
    //   uint8_t flicker = qadd8(CANDLE_BRIGHTNESS, dim8_video(inoise8(millis())/2));
    //   uint8_t flicker = qadd8(blend8(CANDLE_BRIGHTNESS,
    //                                  ledsInternal[boardIndex].current[ledIndex],
    //                                  CANDLE_ALPHA),
    //                           dim8_video(inoise8(millis())/3));
    uint8_t starting_blend = blend8(CANDLE_BRIGHTNESS,
                              ledsInternal[boardIndex].current[ledIndex],
                              CANDLE_ALPHA);
    int8_t random_flicker = static_cast<int8_t>(ease8InOutCubic(random8())) * CANDLE_INTENSITY;
    uint8_t flicker;
    if (random_flicker < 0) {
      flicker = qsub8(starting_blend, -random_flicker);
    } else {
      flicker = qadd8(starting_blend, random_flicker);
    }

    ledFade(boardIndex, ledIndex, flicker, CANDLE_INTERVAL);
  }
}

void ledBreathing(int boardIndex, int ledIndex) {
  // TODO: maybe use easing on the breathing
  // TODO: maybe slight pause when dark
  if (!ledCurrentlyFading(boardIndex, ledIndex)) {
    if (ledsInternal[boardIndex].current[ledIndex] < BREATHING_MAX) {
      ledFade(boardIndex, ledIndex, BREATHING_MAX, BREATHING_INTERVAL);
    } else {
      ledFade(boardIndex, ledIndex, BREATHING_MIN, BREATHING_INTERVAL);
    }
  }
}

void ledStrobe(int boardIndex, int ledIndex) {
  if (!ledCurrentlyFading(boardIndex, ledIndex)) {
    if (ledsInternal[boardIndex].current[ledIndex] < 128) {
      ledSet(boardIndex, ledIndex, STROBE_MAX);
      ledFade(boardIndex, ledIndex, STROBE_MAX, 100);
    } else {
      ledFade(boardIndex, ledIndex, 0, random(100, 200));
    }
  }
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

#ifdef GAMMA_CORRECT
      leds[i][j] = dim8_video(ledsInternal[i].current[j]);
#else
      leds[i][j] = ledsInternal[i].current[j];
      // leds[i][j] = dim8_lin(ledsInternal[i].current[j]);
#endif
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
  // Mode buttons
  pinMode(MODE_PIN_1, INPUT_PULLUP);
  attachInterrupt(MODE_PIN_1, modeButton1Interrupt, RISING);
  pinMode(MODE_PIN_2, INPUT_PULLUP);
  attachInterrupt(MODE_PIN_2, modeButton2Interrupt, RISING);
  pinMode(MODE_PIN_3, INPUT_PULLUP);
  attachInterrupt(MODE_PIN_3, modeButton3Interrupt, RISING);
  pinMode(MODE_PIN_4, INPUT_PULLUP);
  attachInterrupt(MODE_PIN_4, modeButton4Interrupt, RISING);
  // PIR sensor
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  attachInterrupt(PIR_PIN, PIRInterrupt, RISING);
  // Lid switch
  pinMode(LID_PIN, INPUT_PULLUP);

  // Outputs:
  // Smoke machine
  pinMode(SMOKE_PIN, OUTPUT);
  // Motor
  pinMode(MOTOR_PIN, OUTPUT);

  // Audio
#ifdef AUDIO
  DF1201SSerial.begin(115200, SERIAL_8N1, AUDIO_RX_PIN, AUDIO_TX_PIN);
  if (DF1201S.begin(DF1201SSerial)) {
    audioInit = true;

    DF1201S.setPrompt(false);  // Make sure the automatic startup audio prompt is disabled
    if (!DF1201S.setVol(AUDIO_VOLUME))
      DEBUG_PRINTLN("Failed to set audio volume");
    if (!DF1201S.switchFunction(DF1201S.MUSIC))
      DEBUG_PRINTLN("Failed to set music mode");

    while(!audioStop()) delay(200); // TODO: needed?
    delay(200);  // Let the audio module start fully?
  } else {
    DEBUG_PRINTLN("Audio init failed, please check audio wire connection");
  }
#endif

  // WiZmote
#ifdef WIZMOTE
  // Set station mode, though we won't be connecting to another AP
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0) {
    DEBUG_PRINTLN("Error initializing ESP-NOW");
  }
  esp_now_register_recv_cb(wizmoteDataRecv);
#endif

  // LEDs
#ifdef WOKWI
  // Use neopixel leds instead so that the simulator can show the output
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LED_BOARDS);
#else
  // TODO: This makes the wokwi simulator get stuck in the FastLED.show() function
  FastLED.addLeds<NEOPIXEL, MODE_LED_PIN>(&modeLed, 1);

  FastLED.addLeds<P9813, LED_DATA_PIN, LED_CLK_PIN>(leds, NUM_LED_BOARDS);
#endif
}


void loop() {
  // Check what state we should be in from the switch
  if (newMode != currentMode) {
    // Mode changed, so reset and setup anything the new mode needs
    currentMode = newMode;
    switch (newMode) {
      case MODE_STOP:
        DEBUG_PRINTLN("New mode: 1 (STOP)");
        modeLed = CRGB::Red;
        break;
      case MODE_AUTO:
        DEBUG_PRINTLN("New mode: 2 (AUTO)");
        modeLed = CRGB::Green;
        break;
      case MODE_RANDOM:
        DEBUG_PRINTLN("New mode: 3 (RANDOM)");
        modeLed = CRGB::White;
        break;
      case MODE_GROWL_ONLY:
        DEBUG_PRINTLN("New mode: 4 (GROWL_ONLY)");
        modeLed = CRGB::Blue;
        break;
    }

    currentState = ANIM_STANDBY_INIT;

    if (newMode == MODE_STOP) {
      // Stop Smoke
      smokeRun(0);

      // Stop Motor
      motorRun(0);

      audioStop();

      ledsOff(1000);
    }
  }

  // Perform animations if we aren't stopped
  if (currentMode != MODE_STOP) {
    long currAnimationTime = millis() - activationTime;

    switch(currentState) {
      case ANIM_STANDBY_INIT:
        ledFade(LED_FRONT_INDEX, LED_RED_INDEX, GLOW_RED_BRIGHTNESS, 500);
        ledFade(LED_FRONT_INDEX, LED_WHITE_INDEX, CANDLE_BRIGHTNESS, 500);
        audioPlayStandbyTrack();

        lastStandbyTime = millis();
        currentState = ANIM_STANDBY;
        break;

      case ANIM_STANDBY:
        ledCandleFlicker(LED_FRONT_INDEX, LED_WHITE_INDEX);
        ledBreathing(LED_INSIDE_INDEX, LED_RED_INDEX);

        // Check for random activation
        if (currentMode == MODE_RANDOM) {
          if (millis() - lastStandbyTime > randomStartTime) {
            randomStartTime = random(RANDOM_START_MIN, RANDOM_START_MAX);
            currentState = ANIM_STAGE_0;
          }
        }
        break;

      case ANIM_STAGE_0:
        ledsOff(0);
        audioPlayActivationTrack();

        activationTime = millis();

        DEBUG_PRINT("Stage0: ");
        DEBUG_PRINTLN(activationTime);
        
        currentState = ANIM_STAGE_1;
        ledFade(LED_INSIDE_INDEX, LED_RED_INDEX, 255, 600);
        break;

      case ANIM_STAGE_1:
        // Internal Red fade in/out
        if (!ledCurrentlyFading(LED_INSIDE_INDEX, LED_RED_INDEX)) {
          ledFade(LED_INSIDE_INDEX, LED_RED_INDEX, 0, 600);
        }

        if (currAnimationTime > 1200) {
          currentState = ANIM_STAGE_2;
          ledFade(LED_INSIDE_INDEX, LED_WHITE_INDEX, 255, 500);
        }
        break;

      case ANIM_STAGE_2:
        // Internal White fade in/out
        if (!ledCurrentlyFading(LED_INSIDE_INDEX, LED_WHITE_INDEX)) {
          ledFade(LED_INSIDE_INDEX, LED_WHITE_INDEX, 0, 500);
        }

        if (currAnimationTime > 2200) {
          currentState = ANIM_STAGE_3;
        }
        break;

      case ANIM_STAGE_3:
        // Internal red strobe
        ledStrobe(LED_INSIDE_INDEX, LED_RED_INDEX);

        if (currAnimationTime > 3500) {
          ledSet(LED_INSIDE_INDEX, LED_RED_INDEX, 0);  // Cleanup strobe
          currentState = ANIM_STAGE_4;
        }
        break;

      case ANIM_STAGE_4:
        // Internal white strobe
        // END: Motor and smoke activates
        ledStrobe(LED_INSIDE_INDEX, LED_WHITE_INDEX);

        if (currAnimationTime > 4200) {
          ledSet(LED_INSIDE_INDEX, LED_WHITE_INDEX, 0);  // Cleanup strobe
          currentState = ANIM_STAGE_5;
          smokeRun(SMOKE_DURATION);
          motorRun(MOTOR_DURATION);
        }
        break;

      case ANIM_STAGE_5:
        // Internal white and red strobe
        // END: Internal white to full then fade out
        ledStrobe(LED_INSIDE_INDEX, LED_RED_INDEX);
        ledStrobe(LED_INSIDE_INDEX, LED_WHITE_INDEX);

        if (currAnimationTime > 10500) {
          ledSet(LED_INSIDE_INDEX, LED_RED_INDEX, 0);  // Cleanup strobe
          currentState = ANIM_STAGE_6;
          ledSet(LED_INSIDE_INDEX, LED_WHITE_INDEX, 255);
          ledFade(LED_INSIDE_INDEX, LED_WHITE_INDEX, 0, 1000);
        }
        break;

      case ANIM_STAGE_6:
        // END: LEDs off, motor runs slowly to make sure it closes the lid

        if (currAnimationTime > 11500) {
          ledsOff(0);
          currentState = ANIM_STAGE_7;
        }
        break;

      case ANIM_STAGE_7:
        // Wait
        if (currAnimationTime > 15000) {
          currentState = ANIM_STAGE_8;
        }
        break;

      case ANIM_STAGE_8:
        // Little red flickers
        ledCandleFlicker(LED_INSIDE_INDEX, LED_RED_INDEX);

        if (currAnimationTime > 18000) {
          currentState = ANIM_STAGE_9;
        }
        break;

      case ANIM_STAGE_9:
        // Wait
        if (currAnimationTime > ACTIVATION_TRACK_LENGTH) {
          currentState = ANIM_STANDBY_INIT;
        }
        break;
    }
  }

  smokeProcess();
  motorProcess();
  ledProcessFades();
  FastLED.show();
  delay(LOOP_TIME);
}
