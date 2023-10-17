#define FASTLED_ALL_PINS_HARDWARE_SPI
// #define FASTLED_ESP32_SPI_BUS HSPI
#include <FastLED.h>

#ifdef AUDIO
#include <DFRobot_DF1201S.h>
#endif

////////    Config    ////////

//// Constants/Settings ////
#define DEBUG    // Print debug lines to serial, comment out to disable

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

// Audio
// #define AUDIO       // Enable the DFPlayer Pro audio player, comment out to disable
#define DF1201SSerial Serial2  // Serial 2 is the only completely free one on ESP32

// LEDs
#define NUM_LED_BOARDS 2
#define LED_FRONT_INDEX 0
#define LED_INSIDE_INDEX 1
#define LED_RED_INDEX 0
#define LED_WHITE_INDEX 1

// Remote
#define WIZMOTE     // Enable the WiZmote ESP-NOW remote control, comment out to disable
                    //  https://www.wizconnected.com/en-us/p/accessory-wizmote/046677603595
////////

//// Pins ////
// It is recommended to use these pins for the audio UART, but any should be able to be used if needed:
// 16/RX2  17/TX2
//
// For hardware SPI for the LEDs it needs to be either of the following sets of pins:
// To choose between them either comment or uncomment the following line from the top of this file
//   "#define FASTLED_ESP32_SPI_BUS HSPI"
//  23(Data)  18(Clk)  (Comment out the line)
//   or
//  13(Data)  14(Clk)  (Uncomment the line)
// Hardware SPI may also take over pins 5 and 19 or 12 and 15 respectively
// Alternatively software SPI can be used on any other set of pins if
//  "#define FASTLED_ALL_PINS_HARDWARE_SPI" is commented out
//
// Any of the following pins can be used for the rest of the functions:
// 4   13  18  19? 21  22
// 23  25  26  27  32  33
//
// The following can only be used for input, and have no pullups:
// 34  35  36/VP  39/VN

// Inputs:
//   4 Digital pins - Digital In Pullup    - Mode switch (Alternatively 1 analog pin with resistors)
#define MODE_PIN_1 27
#define MODE_PIN_2 26
#define MODE_PIN_3 25
#define MODE_PIN_4 33
//   1 Digital pin  - Digital In Interrupt - PIR sensor
#define PIR_PIN    13
//   1 Digital pin  - Digital In Pullup    - Lid switch
#define LID_PIN    32

// Outputs:
//   1 Digital pin  - Digital Out  - Smoke machine
#define SMOKE_PIN    21
//   1 Digital pin  - PWM          - Motor
#define MOTOR_PIN    22
#define MOTOR_PWM_CHANNEL 0
//   2 Digital pins - UART         - Audio
#define AUDIO_RX_PIN 16  // RX2
#define AUDIO_TX_PIN 17  // TX2
//   2 Digital pins - P9813(1 PWM) - Front LEDs Red/White + Internal LEDs red/white
//     Changing these 2 pin defines doesn't do anything when using hardware SPI
#define LED_DATA_PIN 23
#define LED_CLK_PIN  18
////////

////////    End Config    ////////


// TODO:
//
// Method of learning new WiZmote remotes, maybe when the moon key is pressed 5 times within x sec?
//
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



// Globals

enum ActivationMode {
  Stop,
  Auto,
  GrowlOnly,
  Random
};
ActivationMode currentMode = Auto;

bool Activated = false;

// Audio
#ifdef AUDIO
DFRobot_DF1201S DF1201S;
#endif

// LEDS
CRGB leds[NUM_LED_BOARDS];

////


void IRAM_ATTR PIRInterrupt() {
  // TODO
}


void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

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
  while (!DF1201S.begin(DF1201SSerial)) {
 #ifdef DEBUG
    Serial.println("Init failed, please check audio wire connection!");
 #endif
    delay(1000);
  }
  DF1201S.setVol(15);
  DF1201S.switchFunction(DF1201S.MUSIC);
  // Not sure what this means?:
  // /*Wait for the end of the prompt tone */
  // delay(2000);
  DF1201S.setPlayMode(DF1201S.SINGLECYCLE);
#endif

  // Front LEDs Red/White + Internal LEDs red/white
  // FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LED_BOARDS);
  FastLED.addLeds<P9813, LED_DATA_PIN, LED_CLK_PIN>(leds, NUM_LED_BOARDS);


}

void loop() {
  // Check what state we should be in from the switch
  if (digitalRead(MODE_PIN_1) == HIGH) {
    currentMode = Stop;
  } else if (digitalRead(MODE_PIN_2) == HIGH) {
    currentMode = Auto;
  } else if (digitalRead(MODE_PIN_3) == HIGH) {
    currentMode = GrowlOnly;
  } else if (digitalRead(MODE_PIN_4) == HIGH) {
    currentMode = Random;
  }



  ledcWrite(MOTOR_PWM_CHANNEL, 0);

  leds[LED_FRONT_INDEX][LED_RED_INDEX] = 100;
  leds[LED_FRONT_INDEX][LED_WHITE_INDEX] = 200;
  FastLED.show();
  delay(500);
  
  leds[LED_FRONT_INDEX][LED_RED_INDEX] = 0;
  leds[LED_FRONT_INDEX][LED_WHITE_INDEX] = 100;

  ledcWrite(MOTOR_PWM_CHANNEL, 1024);



  FastLED.show();

  delay(500);
}