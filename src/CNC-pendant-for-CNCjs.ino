// CNC pendant interface to CNCjs
// D Crocker, started 2020-05-04
// Nyarurato changed 2020-09-24

/* This Arduino sketch can be run on either Arduino Nano or Arduino Pro Micro.
   It should alo work on an Arduino Uno (using the same wiring scheme as for the Nano) or Arduino Leonardo (using the same wiring scheme as for the Pro Micro).
   The recommended board is the Arduino Pro Micro because the passthrough works without any modificatoins to the Arduino.

*** Pendant to Arduino Pro Micro connections ***

  Pro Micro Pendant   Wire colours
  VCC       +5V       red
  GND       0V,       black
          COM,      orange/black
          CN,       blue/black
          LED-      white/black

  D2        A         green
  D3        B         white
  D4        X         yellow
  D5        Y         yellow/black
  D6        Z         brown
  D7        4         brown/black
  D8        5         powder (if present)
  D9        6         powder/black (if present)
  D10       LED+      green/black
  A0        STOP      blue
  A1        X1        grey
  A2        X10       grey/black
  A3        X100      orange

  NC        /A,       violet
          /B        violet/black

  Arduino Nano to Duet PanelDue connector connections:

  Pro Micro Duet
  VCC       +5V
  GND       GND
  TX1/D0    Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

  To connect a PanelDue as well:

  PanelDue +5V to +5V/VCC
  PanelDue GND to GND
  PanelDue DIN to Duet UTXD or IO_0_OUT
  PanelDue DOUT to /Pro Micro RX1/D0.

*** Pendant to Arduino Nano connections ***

  Nano    Pendant   Wire colours
  +5V     +5V       red
  GND     0V,       black
        COM,      orange/black
        CN,       blue/black
        LED-      white/black

  D2      A         green
  D3      B         white
  D4      X         yellow
  D5      Y         yellow/black
  D6      Z         brown
  D7      4         brown/black
  D8      5         powder (if present)
  D9      6         powder/black (if present)
  D10     X1        grey
  D11     X10       grey/black
  D12     X100      orange
  D13     LED+      green/black
  A0      STOP      blue

  NC      /A,       violet
        /B        violet/black

  Arduino Nano to Duet PanelDue connector connections:

  Nano    Duet
  +5V     +5V
  GND     GND
  TX1/D0  Through 6K8 resistor to URXD, also connect 10K resistor between URXD and GND

  To connect a PanelDue as well:

  PanelDue +5V to +5V
  PanelDue GND to GND
  PanelDue DIN to Duet UTXD or IO_0_OUT
  PanelDue DOUT to Nano/Pro Micro RX1/D0.

  On the Arduino Nano is necessary to replace the 1K resistor between the USB interface chip by a 10K resistor so that PanelDiue can override the USB chip.
  On Arduino Nano clones with CH340G chip, it is also necessary to remove the RxD LED or its series resistor.

*/

// Configuration constants
const int PinA = 2;
const int PinB = 3;
const int PinX = 4;
const int PinY = 5;
const int PinZ = 6;
const int PinAxis4 = 7;
const int PinAxis5 = 8;
const int PinAxis6 = 9;
const int PinStop = A0;

#if defined(__AVR_ATmega32U4__)     // Arduino Micro, Pro Micro or Leonardo
const int PinTimes1 = A1;
const int PinTimes10 = A2;
const int PinTimes100 = A3;
const int PinLed = 10;
#endif

#if defined(__AVR_ATmega328P__)     // Arduino Nano or Uno
const int PinTimes1 = 10;
const int PinTimes10 = 11;
const int PinTimes100 = 12;
const int PinLed = 13;
#endif


const unsigned long BaudRate = 115200;
const int PulsesPerClick = 1;
const unsigned long MinCommandInterval = 20;

// Table of commands we send, one entry for each axis
const String MoveCommands[] =
{
  "X",     // X axis
  "Y",     // Y axis
  "Z",      // Z axis
  "U",     // axis 4
  "V",     // axis 5
  "W"      // axis 6
};

#include "RotaryEncoder.h"

RotaryEncoder encoder(PinA, PinB, PulsesPerClick);

int serialBufferSize;
int distanceMultiplier;
int axis;
uint32_t whenLastCommandSent = 0;

const int axisPins[] = { PinX, PinY, PinZ, PinAxis4, PinAxis5, PinAxis6 };
const int feedAmountPins[] = { PinTimes1, PinTimes10, PinTimes100 };
String s;

void setup()
{
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  pinMode(PinX, INPUT_PULLUP);
  pinMode(PinY, INPUT_PULLUP);
  pinMode(PinZ, INPUT_PULLUP);
  pinMode(PinAxis4, INPUT_PULLUP);
  pinMode(PinAxis5, INPUT_PULLUP);
  pinMode(PinAxis6, INPUT_PULLUP);
  pinMode(PinTimes1, INPUT_PULLUP);
  pinMode(PinTimes10, INPUT_PULLUP);
  pinMode(PinTimes100, INPUT_PULLUP);
  pinMode(PinStop, INPUT_PULLUP);
  pinMode(PinLed, OUTPUT);

  Serial.begin(BaudRate);
}


void loop()
{
  // 0. Poll the encoder. Ideally we would do this in the tick ISR, but after all these years the Arduino core STILL doesn't let us hook it.
  // We could possibly use interrupts instead, but if the encoder suffers from contact bounce then that isn't a good idea.
  // In practice this loop executes fast enough that polling it here works well enough
  encoder.poll();

  // 1. Check for emergency stop
  if (digitalRead(PinStop) == HIGH)
  {
    // Send emergency stop command every 2 seconds
    do
    {
      s = "{\"ax\":\"None\",\"rt\":0,\"mv\":0,\"emg\":1}" ;
      Serial.println(s);
      digitalWrite(PinLed, LOW);

      encoder.getChange();      // ignore any movement
    } while (digitalRead(PinStop) == HIGH);
  }

  digitalWrite(PinLed, HIGH);

  // 2. Poll the feed amount switch
  distanceMultiplier = 0;
  int localDistanceMultiplier = 1;
  for (int pin : feedAmountPins)
  {
    if (digitalRead(pin) == LOW)
    {
      distanceMultiplier = localDistanceMultiplier;
      break;
    }
    localDistanceMultiplier *= 10;
  }
  
  // 3. Poll the axis selector switch
  axis = -1;
  int localAxis = 0;
  for (int pin : axisPins)
  {
    if (digitalRead(pin) == LOW)
    {
      axis = localAxis;
      break;
    }
    ++localAxis;
  }

  // 5. If the serial output buffer is empty, send a G0 command for the accumulated encoder motion.

  const uint32_t now = millis();
  if (now - whenLastCommandSent >= MinCommandInterval)
  {
    int distance = encoder.getChange();
    if (axis >= 0 && distance != 0)
    {
      whenLastCommandSent = now;

      s = "{\"ax\":\"" + MoveCommands[axis] + "\",\"rt\":" + String(distanceMultiplier)
      +",\"mv\":" + String(distance) +
          ",\"emg\":0}" ;
      Serial.println(s);

    }
  }


}

// End
