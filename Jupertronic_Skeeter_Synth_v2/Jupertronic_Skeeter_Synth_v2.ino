/*   Skeeter Sequence Synth - Arpeggio Box
      by Janis Wilson Hughes aka Jupertronic aka vitalWho aka J Dub aka Evolution Stoneware (youtube)
      Mozzi synthesizer with pre-programmed sequences
      Low pass filter, phasor, rate, and legato adjustments via potentiometers
      Select sequences with up / down buttons
      Drone Glitch latching button to hold root note for jamming breaks
      Ability to change sequence while in drone glitch mode to come out into a new sequence
      MIDI input to change root note of arpeggio sequence on the fly
      MIDI output to send notes to external synth
      MIDI out toggle to select whether to send full sequence or drone
      MIDI out pitch adjustment +/- 1 octave via potentiometer
      Sync output for Korg Volca, Pocket Operator, etc
      Outputs to sencond Arduino to run OLED display
      Blinking light on LED equiped latching drone botton indicating last note of arpeggio
      Originally built on Arduino Nano
      Build your own or modify with attribution to this project
      Inspired by:
      -Mosquito I - by Tim D... analog.sketchbook.101@gmail.com
      -Audiuno with MIDI mods by Notes and Volts - https://www.notesandvolts.com/2015/05/auduino-synth-midi.html

*/
#include <MIDI.h> //Necessary for MIDI input to control root note of arpeggios
#include <MozziGuts.h>
#include <Oscil.h>

#include <tables/saw8192_int8.h>
#include <tables/smoothsquare8192_int8.h>

#include <EventDelay.h>
#include <mozzi_midi.h>
#include <LowPassFilter.h>

MIDI_CREATE_DEFAULT_INSTANCE();
//#define MIDICHANNEL 1 // Can be a value from 1 to 16

#define CONTROL_RATE 128 // powers of 2 please

Oscil<SAW8192_NUM_CELLS, AUDIO_RATE> oscOne(SAW8192_DATA);       //audio oscillator
Oscil<SMOOTHSQUARE8192_NUM_CELLS, AUDIO_RATE> oscTwo(SMOOTHSQUARE8192_DATA); //audio oscillator
LowPassFilter lpf;                                                               //low pass filter

// for sequencing gain
EventDelay kGainChangeDelay;
char gain = 1;

// analog pins (for control pots)
// Note: Pin assignment is a bit arbitrary. This order worked fine for the way I laid out the pots
//       on my front panel, but you might find you need a different order to help with wire routing,
//       panel layout etc
const int OSC_ONE_PIN = 2;    // pitch
const int OSC_TWO_PIN = 3;    // phase
const int MODA_PIN = 0;       // rate
const int MODB_PIN = 1;       // legato
const int MODC_PIN = 4;       // filter

// digital pins
//const int ledPin=4;           // LED
const int upButtonPin = 2;    // up push button
const int downButtonPin = 3;  // down push button
const int syncOutPin = 7;     // jack to send pulse out to sync other devices like drum sequencer
const int droneButtonPin = 6; // Button to sustain glitch drone of the root note for a jam breakdown
const int droneLEDpin = 8;    // Pin to light the LED on the latching drone glitch button
const int lastNotePin = 12;   // Pin to send pulse on last note of sequence
const int firstNotePin = 13;  // Pin to send pulse on first note of sequence
const int MIDItogglePin = 5;  // Pin to select whether to send MIDI for full sequence or root only

// global vars to handle push button functionality and debouncing
int droneButtonState = 0;
int lastDroneState = HIGH;
int upButtonState;
int lastUpState = HIGH;   //HIGH because using pullup resistor
int downButtonState;
int lastDownState = HIGH; //HIGH because using pullup resistor
unsigned long lastUpDebounceTime = 0;
unsigned long lastDownDebounceTime = 0;
unsigned long lastDroneDebounceTime = 0;
unsigned long debounceDelay = 50;        // the debounce time; increase if the output flickers

// division constants
// We're doing some common division operations ahead of time so that we can avoid division in the main loops
// which will slow things down when Mozzi is trying to do its thing. Doing the division here allows us to use
// multiplication in the main loop which is much faster
const float DIV1023 = 1.0 / 1023.0; // division constant for pot value range
const float DIV10 = 1.0 / 10.0;     // division constant for division by 10
const float DIV2 = 1.0 / 2.0;       // division constant for division by 2


// global vars for the arpeggiator stuff
int sequence = 0;               // current sequence playing
int newSequence = 0;            // New sequence chosen with buttons stored to change on first note
int note = 0;                   // current note playing
int lastNote;                   // Last note played
int initialRoot = 42;           // Initialize a root note for arpeggio sequences when no MIDI input present
int currentRoot;                // MIDI input to set root changes
int lastRoot;                   // MIDI out toggle option to send to bass synth
int newRoot = 42;                    // Reading from MIDI input
const int numSequences = 10;    // total number of sequences (must match number of sequences in NOTES array)
const int numNotes = 8;         // total number of notes in each sequence (must all be the same and match length of each sequence in NOTES)
int syncState;                  // Sending a sync pules out for drum machine
int lastSyncState = HIGH;       // Toggle sync pulse
int cSequenceNote;
int pSequenceNote;
int cDroneNote;
int pDroneNote;

// Stuff for drone settings
const int numDroneNotes = 8;
const int DRONE[numNotes] = {0, 0, 0, 0, 0, 0, 0, 0};
//Drone length settings for option to lengthen the drone notes to vary tempo while droning
//int droneLength = 4;  //makes the notes played while in drone mode last 4x as long as the arpeggio notes for cool jamming
//const float DIVDRONE = 1.0 / droneLength;
//const int numDroneLengths = 4;
//const int DRONELENGTH[numDroneLengths] = {2, 8, 16, 4};

// Arpeggio sequences of MIDI notes to play back
// add new sequences or modify existing ones to suit
// just make sure you update the numSequences and numNotes variables above to match what you've done
const int NOTES[numSequences][numNotes] = {

  {0, 12, 0, 12, 0, 12, 0, 12},                     // octave up on two
  {0, 0, 0, 12, 0, 0, 0, 12},                       // octave up on four
  {0, 0, 12, 12, 0, 0, 12, 12},                     // down down up up
  {0, 12, 12, 0, 12, 12, 0, 12},                    // da do do da
  {0, 0, 12, 0, 0, 12, 0, 12},                      // da da do da do
  {0, 0, 0, 12, 0, 0, 0, 12},                       // octave up on four
  {0, 0, 12, 12, 0, 12, 7, 7},                      // Octave pairs plus fifth power chord
  {0, 0, 7, 7, 12, 7, 12, 7},                       // Power chord Trance 1
  {12, 12, 7, 7, 0, 7, 7, 0},                       // Power chord Trance 2
  {0, 7, 12, 7, 12, 7, 0, 12}
  //  {0, 3, 0, 7, 12, 7, 12, 3}                        // Minor Bounce

};

// global control params
int OSC_ONE_OFFSET = 12;      // amount to offset the original midi note (12 is one octave)
int OSC_TWO_OFFSET = 7;       // amount to offset the second midi note from the osc one's midi note (7 is a fifth,this is a power chord - SWEET)
int ARP_RATE_MIN = 32;        // minimum arpeggiator rate (in millisecs)
int ARP_RATE_MAX = 1024;      // maximum arpeggiator rate (in millisecs)
int LEGATO_MIN = 32;          // minimum note length (capping at 32 to avoid rollover artifacts)
int LEGATO_MAX = 1024;        // maximum note length
int LPF_CUTOFF_MIN = 10;      // low pass filter min cutoff frequency
int LPF_CUTOFF_MAX = 245;     // low pass filter max cutoff frequency

//--------------------------------------------------------------------------------------------
//
// Setup
//
//--------------------------------------------------------------------------------------------

void setup() {
  //initialize buttons and toggle switch
  pinMode(upButtonPin, INPUT_PULLUP);   //Activate pullup resistors to eliminate need to hardwire in resistors
  pinMode(downButtonPin, INPUT_PULLUP);
  pinMode(droneButtonPin, INPUT_PULLUP);
  pinMode(MIDItogglePin, INPUT_PULLUP);

  //intialize output pins
  pinMode(syncOutPin, OUTPUT);          //Pin to send pulse to sync other audio devices
  pinMode(droneLEDpin, OUTPUT);         //Pin to light up the latching drone button
  pinMode(lastNotePin, OUTPUT);         //Pin to send pulse on last note of sequence for visual cue
  pinMode(firstNotePin, OUTPUT);        //Pin to send pulse on first note of sequence for visual cue

  //initialize Mozzi objects
  startMozzi(CONTROL_RATE);
  oscOne.setFreq(48);
  oscTwo.setFreq(51);
  lpf.setResonance(128u);
  kGainChangeDelay.set(1000);
  //Serial.begin(9600); //Serial output can cause audio artifacting so this is commented out by default. Can be uncommented for debugging purposes.

  //Set an initial root note value so Skeeter will play as standalone without MIDI input
  currentRoot = initialRoot;

  //Initialize MIDI input
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(NoteOnMidi);
  MIDI.setHandleNoteOff(NoteOffMidi);
}

//--------------------------------------------------------------------------------------------
//
// MIDI Commands
//
//--------------------------------------------------------------------------------------------

// Set the arpeggio sequence root note via the MIDI input
void NoteOnMidi(byte channel, byte pitch, byte velocity) {
  if (pitch != currentRoot) {
    //    currentRoot = pitch;
    newRoot = pitch;
  }
}

// Leave the arpeggio root note set to last MIDI input until a different MIDI input is received
// No need for continuous MIDI note input. Root won't change until new note received.
void NoteOffMidi(byte channel, byte pitch, byte velocity) {
  if (pitch != lastRoot) {
    //    currentRoot = pitch;
    lastRoot = pitch;
  }
}

//--------------------------------------------------------------------------------------------
//
// Arpeggio Sequence Controls
//
//--------------------------------------------------------------------------------------------

void updateControl() {

  // read sequence selector buttons and debounce them
  // using mozziMicros() instead of millis() for timing calls because Mozzi disables millis() timer
  int upreading = digitalRead(upButtonPin);
  int downreading = digitalRead(downButtonPin);
  if (upreading != lastUpState) {
    lastUpState = mozziMicros();
  }
  if ((mozziMicros() - lastUpDebounceTime) > debounceDelay) {
    if (upreading != upButtonState) {
      lastUpDebounceTime = mozziMicros();
      upButtonState = upreading;
      if (upButtonState == LOW) { //LOW for internal pullup resistor, LOW = pressed
        newSequence += 1;
        if (newSequence >= numSequences) {
          // if we get to the end of the sequences we'll rollover back to the first sequence
          newSequence = 0;
        }
      }
    }
  }
  if (downreading != lastDownState) {
    lastDownState = mozziMicros();
  }
  if ((mozziMicros() - lastDownDebounceTime) > debounceDelay) {
    if (downreading != downButtonState) {
      lastDownDebounceTime = mozziMicros();
      downButtonState = downreading;
      if (downButtonState == LOW) { //LOW for internal pullup resistor, LOW = pressed
        newSequence -= 1;
        if (newSequence < 0) {
          // if we get to the first sequence we'll rollover to the last sequence
          newSequence = numSequences - 1;
        }
      }
    }
  }

  // Check if drone button is pressed, turn the button light on if yes or off if the button is not pressed
  int dronereading = digitalRead(droneButtonPin);
  if (dronereading != lastDroneState) {
    lastDroneState = mozziMicros();
  }
  if ((mozziMicros() - lastDroneDebounceTime) > debounceDelay) {
    if (dronereading != droneButtonState) {
      lastDroneDebounceTime = mozziMicros();
      droneButtonState = dronereading;
      note = 0; // resets arp sequence to note 1 when drone released so you don't start in middle of arp
    }
  }

  if (droneButtonState == LOW) {
    digitalWrite(droneLEDpin, HIGH);
  }
  else {
    digitalWrite(droneLEDpin, LOW);
  }

  // Check for MIDI input
  MIDI.read();


  // read pot values
  int oscOne_val = mozziAnalogRead(OSC_ONE_PIN);
  int oscTwo_val = mozziAnalogRead(OSC_TWO_PIN);
  int modA_val = mozziAnalogRead(MODA_PIN);
  int modB_val = mozziAnalogRead(MODB_PIN);
  int modC_val = mozziAnalogRead(MODC_PIN);

  // map pot vals
  // These formulas set the range of values coming from the pots to value ranges that work well with the various control functionality.
  // You'll probably only need to mess with these if you want to expand or offset the ranges to suit your own project's needs
  int oscOne_offset = (OSC_ONE_OFFSET * 2) * ((oscOne_val * DIV1023) - 0.5);              // offset of original midi note number +/- 1 octave
  float oscTwo_offset = ((oscTwo_val * DIV1023) * DIV10) * OSC_TWO_OFFSET;                // frequency offset for second oscillator +/- 0.2 oscOne freq
  float modA_freq = ARP_RATE_MIN + (ARP_RATE_MAX * (1 - (modA_val * DIV1023)));           // arpeggiator rate from 32 millisecs to ~= 1 sec
  float modB_freq = 1 - (modB_val * DIV1023);                                             // legato from 32 millisecs to full on (1 sec)
  int modC_freq = LPF_CUTOFF_MIN + (LPF_CUTOFF_MAX * (modC_val * DIV1023));               // lo pass filter cutoff freq ~=100Hz-8k

  // Check whether to send MIDI out for full sequence or root note only
  int outMIDItoggle = digitalRead(MIDItogglePin);

  // Use an EventDelay to cycle through the sequence and play each note
  kGainChangeDelay.set(modA_freq);                                        // set the delay frequency
  if (kGainChangeDelay.ready()) {

    // Send sync out pulse on for drum machine
    syncState = lastSyncState;
    digitalWrite(syncOutPin, syncState);

    // If drone button pressed, glitch hold note on current root
    if (droneButtonState == LOW) {

      if (gain == 0) {                    // we'll make changes to the oscillator freq when the note is off
        if (note >= numNotes) {           // if we've reached the end of the sequence, loop back to the beginning
          note = 0;
        }
        if (note == 0) {
          currentRoot = newRoot;    // Update the root note from MIDI on start of sequence so no changes in the middle
          sequence = newSequence;   // Update the sequnce selected from buttons on the start of sequence so no changes in the middle

        }
        cSequenceNote = (DRONE[note] + currentRoot + oscOne_offset);
        //If toggle set for every note send every note
        if (outMIDItoggle == HIGH) {
          //            MIDI.sendNoteOn(pSequenceNote, 0, 1);
          MIDI.sendNoteOn(cSequenceNote, 127, 1);   // send current note to MIDI out for external synth, full velocity, MIDI channel 1
        }
        //If MIDI out toggle set for bass root only, send MIDI note out for the current root note minus 1 octave
        if (outMIDItoggle == LOW) {
          //            MIDI.sendNoteOn(lastRoot, 0, 1);
          MIDI.sendNoteOn(currentRoot + oscOne_offset, 127, 1);  //Enable if you want the pitch pot to affect the note out
          //           lastRoot = currentRoot + oscOne_offset;                   //Enable if you want the pitch pot to affect the note out

        }
        pSequenceNote = cSequenceNote;
        lastRoot = (currentRoot + oscOne_offset);

        // set oscillator notes based on current note in sequence
        float noteOne = mtof(DRONE[note] + currentRoot - 12);  // osc one's freq will stay on the current root to drone for jam breaks
        float noteTwo = noteOne + oscTwo_offset;                          // osc two's freq = osc one's freq plus user offset
        oscOne.setFreq(noteOne);
        oscTwo.setFreq(noteTwo);

        note += 1;
        // setting length of note
        gain = 1;
        kGainChangeDelay.set(modA_freq * (1 - modB_freq));                // set length that note is on based on user legato settings
      }
      else {
        gain = 0;
        kGainChangeDelay.set(modA_freq * modB_freq);                    // set length that note is off based on user legato settings
        //        if (outMIDItoggle == HIGH) {      //If we're sending every note MIDI out, turn the last note off to prep to send new note when loop starts over
        //          MIDI.sendNoteOn(NOTES[sequence][note] + currentRoot + oscOne_offset, 0, 1);   // send a MIDI note off to external synth, zero velocity, MIDI channel 1
        //        }
        //        if (outMIDItoggle == LOW) {
        //          MIDI.sendNoteOn(lastRoot, 0, 1);    //Enable if you want the pitch pot to affect the note out
        //        }
        syncState = !lastSyncState;                                     // toggle sync pulse on and off
        lastSyncState = syncState;                                      // for sending pulse to sync external drum machine, PO, Volca
      }
      kGainChangeDelay.start();                                             // execute the delay specified above

    }
    else {
      // If drone not pressed set oscillator notes based on current note in sequence
      // if (droneButtonState == HIGH) {
      if (gain == 0) {                                                    // we'll make changes to the oscillator freq when the note is off
        if (note >= numNotes) {                                           // if we've reached the end of the sequence, loop back to the beginning
          note = 0;
        }
        if (note == 7) {
          digitalWrite(lastNotePin, HIGH);
        } else {
          digitalWrite(lastNotePin, LOW);
        }
        if (note == 0) {
          // According to MIDI input, change root note only at start of sequence
          currentRoot = newRoot;    // Update the root note from MIDI on start of sequence so no changes in the middle
          sequence = newSequence;   // Update the sequence selected from buttons on start of sequence so no changes in the middle
          digitalWrite(firstNotePin, HIGH);
        } else {
          digitalWrite(firstNotePin, LOW);
        }
        // flash drone button LED on last note of sequence only
        if (note == 7) {
          digitalWrite(droneLEDpin, HIGH);
        } else {
          digitalWrite(droneLEDpin, LOW);
        }

        cSequenceNote = (NOTES[sequence][note] + currentRoot + oscOne_offset);

        // set oscillator notes based on current note in sequence and current root note
        float noteOne = mtof(NOTES[sequence][note] + currentRoot - 12);      // osc one's freq = step change + root + user offset from pitch pot
        float noteTwo = noteOne + oscTwo_offset;                          // osc two's freq = osc one's freq plus user offset
        oscOne.setFreq(noteOne);
        oscTwo.setFreq(noteTwo);

        // Send MIDI out note on and off messages
        int MIDIcNote = (NOTES[sequence][note] + currentRoot + oscOne_offset); // Current note
        int MIDIpNote;  // Previous note
        if (outMIDItoggle == HIGH) {
          MIDI.sendNoteOn(pSequenceNote, 0, 1);
          MIDI.sendNoteOn(cSequenceNote, 127, 1);   // send current note to MIDI out for external synth, full velocity, MIDI channel 1
          //          MIDIpNote = MIDIcNote;                // set previous note equal to current note for next loop
        }
        //If MIDI out toggle set for bass root only, send MIDI note out for the current root note
        if (outMIDItoggle == LOW) {
          MIDI.sendNoteOn(lastRoot, 0, 1);
          MIDI.sendNoteOn(currentRoot + oscOne_offset, 127, 1);  // send current root note to MIDI out for external synth, full velocity, MIDI channel 1
          //          lastRoot = (currentRoot + oscOne_offset);                   // set last root equal to current root for next loop
        }
        pSequenceNote = cSequenceNote;
        lastRoot = (currentRoot + oscOne_offset);
        note += 1;

        // setting length of note
        gain = 1;
        kGainChangeDelay.set(modA_freq * (1 - modB_freq));                // set length that note is on based on user legato settings
      }
      else {
        gain = 0;
        kGainChangeDelay.set(modA_freq * modB_freq);                    // set length that note is off based on user legato settings

        syncState = !lastSyncState;                                     // toggle sync pulse on and off
        lastSyncState = syncState;                                      // for sending pulse to sync external drum machine, PO, Volca

      }

      kGainChangeDelay.start();                                             // execute the delay specified above
      //      }
    }
  }



  // setting lo pass cutoff freq
  lpf.setCutoffFreq(modC_freq);                                           // set the lo pass filter cutoff freq per user settings

}

//--------------------------------------------------------------------------------------------
//
// Set Audio for Mozzi
//
//--------------------------------------------------------------------------------------------

int updateAudio() {
  // calculating the output audio signal as follows:
  // 1. Summing the waveforms from the two oscillators
  // 2. Shifting their bits by 1 to keep them in a valid output range
  // 3. Multiplying the combined waveform by the gain (volume)
  // 4. Passing the signal through the low pass filter
  return (char)lpf.next((((oscOne.next() + oscTwo.next()) >> 2) * gain));
}

//--------------------------------------------------------------------------------------------
//
// Main Loop for Mozzi
//
//--------------------------------------------------------------------------------------------

void loop() {
  audioHook();

}
