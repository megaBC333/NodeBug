# NodeBug
Covert audio recording device

## Features
Small size.
Records audio at 16 kHz with 16-bit depth in WAV format.
Blinks the onboard LED to indicate recording in progress.
Recharcheable battery.
A single button toggles recording on and off.
Saves recordings to an SD card (FAT16/FAT32 supported).

## Hardware Components
Arduino Nano

MAX9814 Microphone Amplifier Module

SD Card Module

TP4056 Module

LiPo Battery

Push Button

SPDT on/off Switch

## Connections

### MAX9814:
VDD → 5V (Nano)

GND → GND (Nano)

OUT → A0 (Nano)

### SD Card Module:
CS → D10

MOSI → D11

MISO → D12

SCK → D13

VCC → 5V

GND → GND

### Push Button:
One leg → D2 (interrupt pin)

Other leg → GND

### Battery, TP4056, SPDT Switch:
*LiPo Battery:*

BAT+ → TP4056 BAT+

BAT- → TP4056 BAT-

*TP4056 Module:*

BAT+ → LiPo battery positive terminal

BAT- → LiPo battery negative terminal

OUT+ → Middle pin of SPDT switch

OUT- → Ground (common GND for the entire circuit)

*SPDT Switch:*

Middle Terminal → TP4056 OUT+

One Side Terminal → Nano 5V pin

Other Side Terminal → Leave unconnected


# Code

```
#include <SD.h>
#include <SPI.h>
#include <TMRpcm.h>

#define SD_ChipSelectPin 10 
#define MIC_PIN A0 
#define BUTTON_PIN 2  // button pin for toggling recording

TMRpcm audio;
int audioFileCounter = 0;  // file numbering for recordings
volatile bool recMode = false;

void setup() {
  // debug
  Serial.begin(9600);
  delay(2000);
  Serial.println("NodeBug starting...");

  // initialize SD card
  if (!SD.begin(SD_ChipSelectPin)) {
    Serial.println("SD Card initialization failed!");
    while (1); 
  }
  Serial.println("SD Card initialized successfully.");

  // set TMRpcm chip select pin
  audio.CSPin = SD_ChipSelectPin;

  // configure pins
  pinMode(MIC_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // button interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), toggleRecording, FALLING);
}

void loop() {
  // blink when recording
  if (recMode) {
    static unsigned long lastBlinkTime = 0;

    if (millis() - lastBlinkTime >= 1000) { 
      lastBlinkTime = millis();
      Serial.print(".");
    }
  }
}

// Interrupt Service Routine (ISR) for button
void toggleRecording() {
  static unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 200; // debounce time

  // debounce button press
  if (millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    recMode = !recMode; // toggle recording mode

    Serial.print("Recording mode: ");
    Serial.println(recMode ? "ON" : "OFF");

    if (recMode) {
      // start recording
      audioFileCounter++;
      Serial.print("Starting recording: ");
      Serial.print(audioFileCounter);
      Serial.println(".wav");

      String fileName = String(audioFileCounter) + ".wav";
      audio.startRecording(fileName.c_str(), 16000, MIC_PIN);
      Serial.println("Recording started.");
    } else {
      // stop recording
      Serial.print("Stopping recording: ");
      Serial.print(audioFileCounter);
      Serial.println(".wav");

      String fileName = String(audioFileCounter) + ".wav";
      audio.stopRecording(fileName.c_str());
      Serial.println("Recording stopped.");
    }
  }
}
```

# Protoype

![20241125_180449](https://github.com/user-attachments/assets/0b323708-a869-4a05-b75c-98247eb407c0)

![20241126_092639](https://github.com/user-attachments/assets/95169290-fde6-48f0-b600-c3b628923322)

![20241126_100604](https://github.com/user-attachments/assets/aa622b1f-a5c7-499e-807b-64cd3c3746d8)
