/* 
Copyright (c) 2020 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include <math.h>
#include <TMC5160.h>

// Pins for controlling stepper motor.
const uint8_t SPI_CS = 5; // CS pin in SPI mode
const uint8_t SPI_DRV_ENN = 8;  // DRV_ENN pin in SPI mode

//Use default SPI peripheral and SPI settings.
TMC5160_SPI motor = TMC5160_SPI(SPI_CS);

// Parameters.
char controlCode = 0;
int receivedCode = 0;
int currentPosition = 0;
int currentSpeed = 0;
int deltaSpeed = 50;
int maxSpeed = 9 * deltaSpeed;
int maxPosition = 1800;
int defaultAcceleration = 2000;
bool allowRemote = true;
volatile bool confirmed = false;
volatile byte currentPositionAsByte = 0;

// Parameters for shutter glasses.
static int pinGlasses = 19;
bool glasses = false;

// Miscellaneous.
int noteDirection = 1;

// Parameters for rotary encoder.
// (From: https://www.instructables.com/Improved-Arduino-Rotary-Encoder-Reading/)
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
static int pinConfirmSwitch = 4; // Pin for the switch.
volatile byte aFlag = 0; // Let's us know when we're expecting a rising edge on pinA.
volatile byte bFlag = 0; // Let's us know when we're expecting a rising edge on pinB.
volatile int encoderPos = 0; // Current value of encoder position.
volatile int oldEncPos = 0; // Last encoder position value.
volatile byte reading = 0; // Somewhere to store the direct values we read from our interrupt pins before checking.


void moveToPosition(int);
void PinA();
void PinB();
void confirm();
void playNote(int, int, int);
void playSong();
void playSongPlotter();

void setup()
{
  // USB/debug serial coms
  Serial.begin(9600); // 115200

  pinMode(SPI_DRV_ENN, OUTPUT); 
  digitalWrite(SPI_DRV_ENN, LOW); // Active low

  // This sets the motor & driver parameters /!\ run the configWizard for your driver and motor for fine tuning !
  TMC5160::PowerStageParameters powerStageParams; // defaults.
  TMC5160::MotorParameters motorParams;
  motorParams.globalScaler = 98; // Adapt to your driver and motor (check TMC5160 datasheet - "Selecting sense resistors")
  motorParams.irun = 31;
  motorParams.ihold = 0; // 16

  SPI.begin();
  motor.begin(powerStageParams, motorParams, TMC5160::NORMAL_MOTOR_DIRECTION);

  motor.setRampMode(TMC5160::VELOCITY_MODE);
  motor.setMaxSpeed(0); // Needs to be 0 if VELOCITY_MODE is used.
  motor.setAcceleration(defaultAcceleration);

  // Setup for glasses.
  pinMode(pinGlasses, OUTPUT);

  // Setup for rotary encoder.
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinConfirmSwitch, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(pinA), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(pinB), PinB, RISING);
  attachInterrupt(digitalPinToInterrupt(pinConfirmSwitch), confirm, FALLING);

  delay(1000); // Standstill for automatic tuning
}

void loop()
{
  if(oldEncPos != encoderPos) {
    // Update encoder position.
    oldEncPos = encoderPos;

    // Calculate and set the new speed.
    if(allowRemote) {
      currentSpeed = max(-maxSpeed, min(maxSpeed, encoderPos * deltaSpeed));
      motor.setMaxSpeed(currentSpeed);

      // Set 'confirmed' to false.
      confirmed = false;
    }
  }

  currentPositionAsByte = (byte)(motor.getCurrentPosition()/((float)maxPosition) * 255);

  // Prevent from going to far.
  if(motor.getCurrentPosition() < 0) {
    encoderPos = 0;
    moveToPosition(0); // The speed is always set to 0 in this function.
  }

  if(motor.getCurrentPosition() > maxPosition) {
    encoderPos = 0;
    moveToPosition(maxPosition); // The speed is always set to 0 in this function.
  }
  
  // When sending data to the RAIL AWAY device, you should send two bytes:
  // - 1st: Control char: e.g. 'p' (position), 'g' (shutter glasses)
  // - 2nd: Value (for position or glasses)
  // The code below is not the best way to receive multiple bytes. But yeah.
  if (Serial.available() >= 2) {
    // Read the control code.
    controlCode = Serial.read();

    // Read the value.
    receivedCode = Serial.read();

    // Serial.print(controlCode);
    // Serial.println(receivedCode);

    // Action based on the control code.
    switch(controlCode) {
      case 'p': // Set position.
        moveToPosition(maxPosition * receivedCode/255);
        break;

      case 'q': // Request position.
        Serial.print("r"); // 'r' for 'response'.
        Serial.println(currentPositionAsByte);
        break;

      case 'l': // Allow responses.
        confirmed = false;
        break;

      case 'u': // Allow remote control.
        if(receivedCode == 'e') {
          allowRemote = true;
        } else if(receivedCode == 'd') {
          allowRemote = false;
        }
        break;
      
      case 'g': // Shutter glasses.
        if(receivedCode == 'o') {
          digitalWrite(pinGlasses, LOW);
          glasses = !glasses;
        } else if(receivedCode == 'c') {
          digitalWrite(pinGlasses, HIGH);
          glasses = !glasses;
        }
        break;
      
      case 'm': // Play music.
        if(receivedCode == 'p') playSongPlotter();
        if(receivedCode == 's') playSong();
        break;
      
      default:
        break;
    }

    // Reset the encoder position (and the speed) if a value is received from the serial interface.
    encoderPos = 0;
    currentSpeed = 0;
  }
}


void moveToPosition(int newPosition)
{
  // Stop motor and reset speed.
  motor.stop();
  currentSpeed = 0;

  // For the motor to move to a specified position, the RampMode has to be set
  // to 'POSITIONING_MODE'. Then, the maximum allowed speed has to be set as well.
  motor.setRampMode(TMC5160::POSITIONING_MODE);
  motor.setMaxSpeed(500);
  motor.setTargetPosition(newPosition);
  
  // Wait until the motor has reached the taregt position.
  while(motor.getCurrentPosition() != newPosition);

  // Set the motor back to VELOCITY_MODE. For this, first set the maximum speed
  // to 0 (as this will be set as the current speed when in VELOCITY_MODE).
  motor.setMaxSpeed(0);
  motor.setRampMode(TMC5160::VELOCITY_MODE);

  // Calculate current position as byte.
  currentPositionAsByte = (byte)(motor.getCurrentPosition()/((float)maxPosition) * 255);

  // Send a 'target reached' message.
  Serial.print("t"); // 't' for 'target reached'.
  Serial.println(currentPositionAsByte);

  // Reset confirmation.
  //confirmed = false;
}


// Interrupt function for Pin A.
void PinA() {
  cli(); // Stop interrupts happening before we read pin values.
  reading = PIND & 0xC; // Read all eight pin values then strip away all but pinA and pinB's values.
  if(reading == B00001100 && aFlag) { // Check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge.
    encoderPos --; // Decrement the encoder's position count.
    bFlag = 0; // Reset flags for the next turn.
    aFlag = 0; // Reset flags for the next turn.
  }
  else if (reading == B00000100) bFlag = 1; // Signal that we're expecting pinB to signal the transition to detent from free rotation.
  sei(); //restart interrupts
}


// Interrupt function for Pin B.
void PinB() {
  cli(); // Stop interrupts happening before we read pin values.
  reading = PIND & 0xC; // Read all eight pin values then strip away all but pinA and pinB's values.
  if (reading == B00001100 && bFlag) { // Check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge.
    encoderPos ++; // Increment the encoder's position count.
    bFlag = 0; // Reset flags for the next turn.
    aFlag = 0; // Reset flags for the next turn.
  }
  else if (reading == B00001000) aFlag = 1; // Signal that we're expecting pinA to signal the transition to detent from free rotation.
  sei(); //restart interrupts
}


// The user confirms the position.
void confirm() {
  if(!confirmed && currentSpeed == 0) {
    confirmed = true;

    Serial.print("r"); // 'r' for 'response'.
    Serial.println(currentPositionAsByte);
  }
}

// Play a musical note with a given pitch (as MIDI note number, 1--127), noteValue (1, 2nd, 4th, 8th, 16th)
// and tempo (in BPM)!
void playNote(int midiNumber, int noteValue, int bpm) {
  // Calculate pitch based on MIDI note number.
  double frequency = 440.0 * pow(2.0, (midiNumber-69)/12.0);
  Serial.println(frequency);

  // Calculate duration based on length and tempo.
  double duration = 1000 * (240.0 / noteValue) / bpm;
  Serial.println(duration);

  // PLAY!
  motor.setMaxSpeed(noteDirection * frequency);
  delay(duration);
  motor.setMaxSpeed(0);
  delay(100); 
}

void playSong() {
  // Set to high acceleration.
  motor.setAcceleration(3000);

  int tempo = 110;

  playNote(69, 4, tempo);
  playNote(69, 4, tempo);
  playNote(69, 4, tempo);
  playNote(65, 8, tempo);
  playNote(72, 8, tempo);
  playNote(69, 4, tempo);
  playNote(65, 8, tempo);
  playNote(72, 8, tempo);
  playNote(69, 4, tempo);

  noteDirection *= -1;
  delay (500);

  playNote(76, 4, tempo);
  playNote(76, 4, tempo);
  playNote(76, 4, tempo);
  playNote(77, 8, tempo);
  playNote(72, 8, tempo);
  playNote(68, 4, tempo);
  playNote(65, 8, tempo);
  playNote(72, 8, tempo);
  playNote(69, 4, tempo);

  noteDirection = 1;

  // Reset to default acceleration.
  motor.setAcceleration(defaultAcceleration);
}

void playSongPlotter() {
  // Set to high acceleration.
  motor.setAcceleration(3000);

  int tempo = 105;

  noteDirection = 1;

  playNote(64, 8, tempo);
  playNote(69, 4, tempo);
  playNote(72, 16, tempo);
  playNote(71, 8, tempo);
  playNote(69, 4, tempo);
  playNote(76, 8, tempo);

  playNote(74, 4, tempo);
  playNote(71, 4, tempo);

  noteDirection = -1;
  delay (500);

  playNote(69, 4, tempo);
  playNote(72, 16, tempo);
  playNote(71, 8, tempo);
  playNote(67, 4, tempo);

  playNote(70, 8, tempo);
  playNote(64, 4, tempo);

  noteDirection = 1;
  

  // Reset to default acceleration.
  motor.setAcceleration(defaultAcceleration);
}