/*   Skeeter Synth - OLED Display
      by Janis Wilson Hughes aka Jupertronic aka vitalWho aka J Dub
      Shows the sequence currently selected on OLED so you can tell what the heck you're doing
      Shows when Drone is engaged
      Blink low line on last note of sequence for timing cue
      Blink high line on first note of sequence for timing cue
      Programmed for Arduino Nano. Different boards use different pins for the OLED
      so definine them accordingly in the code (SDA & SCL)
      Be sure to wire 2nd Arduino to common ground with the first board
*/


#include <ss_oled.h>

// if your system doesn't have enough RAM for a back buffer, comment out
// this line (e.g. ATtiny85)
#define USE_BACKBUFFER

#ifdef USE_BACKBUFFER
static uint8_t ucBackBuffer[1024];
#else
static uint8_t *ucBackBuffer = NULL;
#endif

// Use -1 for the Wire library default pins
// or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
// These are the pin numbers for the M5Stack Atom default I2C
#define SDA_PIN 18   //Pro Micro 2
#define SCL_PIN 19   //Pro Micro 3
// Set this to -1 to disable or the GPIO pin number connected to the reset
// line of your display if it requires an external reset
#define RESET_PIN -1
// let ss_oled figure out the display address
#define OLED_ADDR -1
// don't rotate the display
#define FLIP180 0
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 0

// Change these if you're using a different OLED display
#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
//#define MY_OLED OLED_64x32
//#define OLED_WIDTH 64
//#define OLED_HEIGHT 32

SSOLED ssoled;

// digital pins
const int upButtonPin = 8;    // Up push button
const int downButtonPin = 9;  // Down push button
const int droneButtonPin = 6; // Button to sustain glitch drone of the root note for a jam breakdown
const int lastNotePin = 3;    // Input high on last note of sequence
const int firstNotePin = 4;   // Input high on first note of sequence

// global vars to handle push button functionality and debouncing
int droneButtonState = 0;
int lastDroneState = HIGH;
int upButtonState;
int lastUpState = HIGH;   //HIGH because using pullup resistor
int downButtonState;
int lastDownState = HIGH; //HIGH because using pullup resistor
int whatSequence;
unsigned long lastUpDebounceTime = 0;
unsigned long lastDownDebounceTime = 0;
unsigned long lastDroneDebounceTime = 0;
unsigned long debounceDelay = 50;        // the debounce time; increase if the output flickers


// global vars for the sequence selection
int sequence = 0;               // current sequence playing
const int numSequences = 10;    // total number of sequences (***must match number of sequences in NOTES array*** from synth Arduino)


//--------------------------------------------------------------------------------------------
//
// Setup
//
//--------------------------------------------------------------------------------------------

void setup() {
  //  Serial.begin(9600);  //For debugging

  //initialize buttons
  pinMode(upButtonPin, INPUT_PULLUP);   //Activate pullup resistors for buttons to eliminate need to hardwire in a resistors
  pinMode(downButtonPin, INPUT_PULLUP);
  pinMode(droneButtonPin, INPUT_PULLUP);
  pinMode(lastNotePin, INPUT);
  pinMode(firstNotePin, INPUT);

  //Initialize OLED display
  int rc;
  // The I2C SDA/SCL pins set to -1 means to use the default Wire library
  // If pins were specified, they would be bit-banged in software
  // This isn't inferior to hw I2C and in fact allows you to go faster on certain CPUs
  // The reset pin is optional and I've only seen it needed on larger OLEDs (2.4")
  //    that can be configured as either SPI or I2C
  //
  // oledInit(SSOLED *, type, oled_addr, rotate180, invert, bWire, SDA_PIN, SCL_PIN, RESET_PIN, speed)

  rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz


  oledFill(&ssoled, 1, 1);
  oledFill(&ssoled, 0, 1);
  //  oledDumpBuffer(&ssoled, NULL);

  oledWriteString(&ssoled, 0, 0, 1, (char *)"   Jupertronic       ", FONT_NORMAL, 1, 1);
  oledWriteString(&ssoled, 0, 10, 3, (char *)"Skeeter", FONT_STRETCHED, 0, 1);
  oledWriteString(&ssoled, 0, 0, 5, (char *)"     Sequence", FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0, 0, 6, (char *)"        Synth   ", FONT_NORMAL, 0, 1);

  delay(1000);

  oledFill(&ssoled, 0, 1);
  oledDumpBuffer(&ssoled, NULL);

}
//--------------------------------------------------------------------------------------------
//
// Loop
//
//--------------------------------------------------------------------------------------------

void loop () {
  char printSequence[2];  // Create a character array of 2 characters to store the sequence number i.e.

  // read sequence selector buttons and debounce them
  int upreading = digitalRead(upButtonPin);
  int downreading = digitalRead(downButtonPin);

  if ((millis() - lastUpDebounceTime) > debounceDelay) {
    if (upreading != upButtonState) {
      lastUpDebounceTime = millis();
      if (upButtonState == LOW) { //LOW for internal pullup resistor, LOW = pressed
        sequence += 1;
        if (sequence >= numSequences) {
          // if we get to the end of the sequences we'll rollover back to the first sequence
          sequence = 0;
        }
      }
      upButtonState = upreading;
    }
  }

  if ((millis() - lastDownDebounceTime) > debounceDelay) {
    if (downreading != downButtonState) {
      lastDownDebounceTime = millis();
      if (downButtonState == LOW) { //LOW for internal pullup resistor, LOW = pressed
        sequence -= 1;
        if (sequence < 0) {
          // if we get to the first sequence we'll rollover to the last sequence
          sequence = (numSequences - 1);
        }
      }
      downButtonState = downreading;
    }
  }

  whatSequence = (sequence + 1);
  dtostrf(whatSequence, 2, 0, printSequence);
  oledWriteString(&ssoled, 0, 44, 3, (char *)printSequence, FONT_STRETCHED, 0, 1);
  //  Serial.println(whatSequence);  //For debugging

  // Check if drone button is pressed and display on OLED if so
  // since latching button no debouncing; add debounce if you need it
  int dronereading = digitalRead(droneButtonPin);
  if (dronereading == LOW) {
    oledWriteString(&ssoled, 0, 0, 0, (char *)"   Jupertronic       ", FONT_NORMAL, 0, 1);
    oledWriteString(&ssoled, 0, 5, 7, (char *)"SkeeterSeqSynth  ", FONT_NORMAL, 0, 1);
    oledWriteString(&ssoled, 0, 00, 2, (char *)"    D R O N E          ", FONT_NORMAL, 1, 1);
    oledWriteString(&ssoled, 0, 00, 5, (char *)"    D R O N E          ", FONT_NORMAL, 1, 1);
  }
  else {
    oledWriteString(&ssoled, 0, 0, 0, (char *)"   Jupertronic       ", FONT_NORMAL, 0, 1);
    oledWriteString(&ssoled, 0, 5, 7, (char *)"SkeeterSeqSynth  ", FONT_NORMAL, 0, 1);
    oledWriteString(&ssoled, 0, 00, 2, (char *)"                   ", FONT_NORMAL, 0, 1);
    oledWriteString(&ssoled, 0, 00, 5, (char *)"                   ", FONT_NORMAL, 0, 1);
  }
  int lastNote = digitalRead(lastNotePin);
  int firstNote = digitalRead(firstNotePin);
  if (lastNote == HIGH) {
    oledWriteString(&ssoled, 0, 40, 6, (char *)" * * * ", FONT_NORMAL, 1, 1);
  }
  else {
    oledWriteString(&ssoled, 0, 40, 6, (char *)"                   ", FONT_NORMAL, 0, 1);
  }
  if (firstNote == HIGH) {
    oledWriteString(&ssoled, 0, 40, 1, (char *)" * * * ", FONT_NORMAL, 1, 1);
  }
  else {
    oledWriteString(&ssoled, 0, 40, 1, (char *)"                   ", FONT_NORMAL, 0, 1);
  }
}
