#include "CapacitiveSensor.h"
#include <Bounce.h>

#define NUM_SENSORS 10

#define NUM_SAMPLES_FOR_SENSORS 30
#define NUM_BUTTONS 7
#define NUM_BUTTON_COMBOS 12
#define BOUNCE_TIME 10 // in milliseconds
#define CHANNEL 1
#define MIN_SENSOR_VAL 100
#define MAX_SENSOR_VAL 2000
#define NUM_SCALES 3


Bounce bounceButtons[NUM_BUTTONS] = {Bounce(12, BOUNCE_TIME), Bounce(11, BOUNCE_TIME), Bounce(10, BOUNCE_TIME), Bounce(9, BOUNCE_TIME), Bounce(8, BOUNCE_TIME), Bounce(7, BOUNCE_TIME), Bounce(6, BOUNCE_TIME)};
int buttonPins[NUM_BUTTONS] = {12, 11, 10, 9, 8, 7, 6};

int key_on_threshold = 500;
int scale_num = 0;

enum buttonPressType {
  BUTTON0,
  BUTTON0_AND_BUTTON1,
  BUTTON1,
  BUTTON1_AND_BUTTON2,
  BUTTON2,
  BUTTON3,
  BUTTON3_AND_BUTTON4,
  BUTTON4,
  BUTTON4_AND_BUTTON5,
  BUTTON5,
  BUTTON5_AND_BUTTON6,
  BUTTON6,
};


enum buttonPressType buttonPressState;

byte buttonStates[NUM_BUTTONS] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // LOW = button pressed down
byte previousButtonStates[NUM_BUTTONS];

int sensePin = 19;

int sensorPins[NUM_SENSORS] = {15, 16, 17, 18, 19, 22, 23, 25, 33, 32};
int potPins[3] = {14, 21, 20};
int minBrightness = 0;
int maxBrightness = 255;

int midiNotes[NUM_BUTTON_COMBOS][NUM_SENSORS] = {{40, 42, 47, 50, 52, 53, 55, 57, 58, 60}};


int scale[NUM_SENSORS] = {0, 4, 7, 8, 10, 12, 14, -5, -6, -10};

int baseNoteOffset = 60;
int currentBaseNote = 60;



unsigned long chordingTimeLimit = 30; // the time we need to wait
int firstButtonPress;
int buttonPresses[2] = {};
int naturalsPattern[7] = {0, 2, 4, 5, 7, 9, 11};

int secondButtonPress;
unsigned long timeSinceFirstButtonPress = 0; // for chording
int numButtonsPressed = 0;

bool sensorsInitialized = false;
bool keyIsOn[10] = {false, false, false, false, false, false, false, false, false, false};
int lastPressureReading[10] = {0, 0, 0, 0, 0, 0 , 0, 0, 0, 0};

int lastPitchWheelMessage = 0;

int tares[NUM_SENSORS] = {0, 0, 0, 0, 0, 0 , 0, 0, 0, 0};

void setup()
{
  pinMode(13, OUTPUT);
  Serial.begin(9600);

  for (int i = 0; i < NUM_BUTTONS; i++)
  {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
}

void loop() {
  if (sensorsInitialized == false) {
    initializeSensors();
    sensorsInitialized = true;
  }
  //unsigned long currentMillis = millis();

  for (int i = 0; i < NUM_SENSORS; i++) {
    int sensorVal = touchRead(sensorPins[i]) - tares[i];
    int remapped127 = remap(sensorVal, MIN_SENSOR_VAL, MAX_SENSOR_VAL, 1, 127);
    int midiNote = currentBaseNote + scale[i];

    Serial.print(sensorVal);
    Serial.print("\t");

    lastPressureReading[i] = remapped127;
    if (keyIsOn[i] == true)
    {
      if (sensorVal < key_on_threshold - 100){
        keyIsOn[i] = false;
        usbMIDI.sendNoteOff(midiNote, remapped127, i + 1);
      }
      else {
        usbMIDI.sendAfterTouchPoly(midiNote, remapped127, i + 1);
      }
    }

    else {
      if (sensorVal > key_on_threshold){
        Serial.print("ON");
        keyIsOn[i] = true;
        usbMIDI.sendNoteOn(midiNote, remapped127, i + 1); // TODO: replace
      }
    }
  }

  Serial.print(baseNoteOffset);
  Serial.print("\t");

  for (int i = 0; i < 3; i++){
    int potVal = analogRead(potPins[i]);
    switch (i){
      case 0:
        key_on_threshold = potVal;
      case 1:
        if ((lastPitchWheelMessage > potVal + 1) || (lastPitchWheelMessage < potVal - 1)){
          usbMIDI.sendPitchBend(potVal, CHANNEL);
          lastPitchWheelMessage = potVal;
        }
        break;
      case 2:
        baseNoteOffset = remap(potVal, 0, 1023, 0, 127);
        break;
    }
    Serial.print(potVal);
    Serial.print("\t");
  }

  Serial.println("");
  for (int i = 0; i < NUM_BUTTONS; i++){
    if (bounceButtons[i].update()){
      if (bounceButtons[i].fallingEdge()){
        buttonStates[i] = HIGH;
        //Serial.print(i);

        //handleButtonPress(i);
        if (numButtonsPressed == 0){
          firstButtonPress = i;
          numButtonsPressed = 1;
        }

        else {
          secondButtonPress = i;
          numButtonsPressed = 2;
        }
      }

      if (bounceButtons[i].risingEdge() && buttonStates[i] == HIGH){

        buttonPressType pressedButtonType;
        if (numButtonsPressed == 2)
          {
          pressedButtonType = getButtonPressType();
          buttonStates[firstButtonPress] = LOW;
          buttonStates[secondButtonPress] = LOW;
        }
        else  // else, just one button pressed
        {
          pressedButtonType = getButtonPressType();
          buttonStates[firstButtonPress] = LOW;
        }

        // now change the key of the board
        int lastBaseNote = currentBaseNote;
        currentBaseNote = baseNoteOffset + pressedButtonType;

        for (int j = 0; j < NUM_SENSORS; j++)
        {
          if (keyIsOn[j] == true)
          {
            int lastMidiNote = lastBaseNote + scale[j];
            int newMidiNote = currentBaseNote + scale[j];
            usbMIDI.sendNoteOff(lastMidiNote, 0, j + 1); // turn off all notes when going to next scale
            usbMIDI.sendNoteOn(newMidiNote, lastPressureReading[j], j + 1); // turn on the note in the new scale
          }

        }

        numButtonsPressed = 0;
      }
    }
  }

  memcpy(previousButtonStates, buttonStates, NUM_BUTTONS);
  //long avgSensorVal = sensorSum/numSensorsOn;
  //int remapped255 = remap(avgSensorVal, minSensorVal, maxSensorVal, 0, 255);
  //analogWrite(5, remapped255);
  //Serial.print("\n");
  delay(50);
}


buttonPressType getButtonPressType()
{
  buttonPressType pressedButtonPressType = BUTTON0; // default it to this to avoid warning

  if (numButtonsPressed == 1)
  {
    switch (firstButtonPress) {
      case 0:
        pressedButtonPressType = BUTTON0;
        break;
      case 1:
        pressedButtonPressType = BUTTON1;
        break;
      case 2:
        pressedButtonPressType = BUTTON2;
        break;
      case 3:
        pressedButtonPressType = BUTTON3;
        break;
      case 4:
        pressedButtonPressType = BUTTON4;
        break;
      case 5:
        pressedButtonPressType = BUTTON5;
        break;
      case 6:
        pressedButtonPressType = BUTTON6;
        break;
    }
    //newBaseNote = naturalsPattern[firstButtonPress];
  }
  else
  {
    // dunno if this will work
    int sortedButtonPresses[2] = {min(firstButtonPress, secondButtonPress), max(firstButtonPress, secondButtonPress)};

    if ((sortedButtonPresses[0] == 0) && (sortedButtonPresses[1] == 1))
    {
      pressedButtonPressType = BUTTON0_AND_BUTTON1;
    }
    else if ((sortedButtonPresses[0] == 1) && (sortedButtonPresses[1] == 2))
    {
      pressedButtonPressType = BUTTON1_AND_BUTTON2;
    }
    else if ((sortedButtonPresses[0] == 3) && (sortedButtonPresses[1] == 4))
    {
      pressedButtonPressType = BUTTON3_AND_BUTTON4;
    }
    else if ((sortedButtonPresses[0] == 4) && (sortedButtonPresses[1] == 5))
    {
      pressedButtonPressType = BUTTON4_AND_BUTTON5;
    }
    else if ((sortedButtonPresses[0] == 5) && (sortedButtonPresses[1] == 6))
    {
      pressedButtonPressType = BUTTON5_AND_BUTTON6;
    }
  }
  return pressedButtonPressType;
}



void handleButtonPress(int pressedButtonNum)
{

  // if ((pressedButtonNum == firstButtonPress) || (pressedButtonNum == secondButtonPress))
  // {

  // }

  unsigned long currentTime = millis();
  //Serial.print("pressed button num ");
  //Serial.print(pressedButtonNum);
  //Serial.print("\t");
  //Serial.print(currentTime);
  // Serial.print("\t");
  // Serial.print(timeSinceFirstButtonPress);
  //Serial.print("\t");



  if (numButtonsPressed == 0)
  {
    firstButtonPress = pressedButtonNum;
    timeSinceFirstButtonPress = currentTime;
    numButtonsPressed = 1;
  }

  else if (pressedButtonNum != firstButtonPress)
  {
    secondButtonPress = pressedButtonNum;
    numButtonsPressed += 1;
  }


  //  Serial.print(numButtonsPressed);
  //  Serial.print("\n");

}



boolean array_compare(byte *a, byte *b, int array_length)
{
  // https://forum.arduino.cc/index.php?topic=5157.0
  int n;
  for (n = 0; n < array_length; n++) if (a[n] != b[n]) return false;
  return true;
}

int remap(int val, int inMin, int inMax, int outMin, int outMax)
{
  if (val < inMin)
  {
    return outMin;
  }
  if (val > inMax)
  {
    return outMax;
  }
  else
  {
    return map(val, inMin, inMax, outMin, outMax);
  }
}


void initializeSensors() {
    for (int i=0; i < NUM_SENSORS; i++) {
      delay(200);
      int sensorVal = touchRead(sensorPins[i]);
      delay(200);
      tares[i] = sensorVal;
  }
}


void removeIndex(uint8_t array[], size_t length, size_t index) {
  // Sanity check.
  if (array != nullptr && length > 0) {
    for (size_t i = index; i < (length - 1); i++) {
      array[i] = array[i + 1];
    }
    array[length - 1] = 0;
  }
}
