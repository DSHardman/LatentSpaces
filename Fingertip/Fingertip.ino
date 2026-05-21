// 3 Task Arduino code: control motors, return DC data, and measure normal forces

#include "HX711.h"
#include "TMAG5273.h"

#define DOUT  8
#define CLK  9

#define TMAG_SDA 19
#define TMAG_SCL 18

TMAG5273 tmag5273(&Wire);

HX711 scale;

char inByte = 'p';
char readmode = 'p';

int toolchangeA = 6;
int toolchangeB = 7;
int fingerjointA = 4;
int fingerjointB = 5;
int mountjointA = 2;
int mountjointB = 3;

float Bx, By, Bz, T;
uint8_t res;

void setup() {
  pinMode(toolchangeA, OUTPUT);
  digitalWrite(toolchangeA, 0);
  pinMode(toolchangeB, OUTPUT);
  digitalWrite(toolchangeB, 0);
  pinMode(fingerjointA, OUTPUT);
  digitalWrite(fingerjointA, 0);
  pinMode(fingerjointB, OUTPUT);
  digitalWrite(fingerjointB, 0);
  pinMode(mountjointA, OUTPUT);
  digitalWrite(mountjointA, 0);
  pinMode(mountjointB, OUTPUT);
  digitalWrite(mountjointB, 0);

  // Ensure analog pins are floating at first
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  Wire.begin();
  tmag5273.modifyI2CAddress(0x35);
  
  tmag5273.configOperatingMode(TMAG5273_OPERATING_MODE_MEASURE);
  tmag5273.configReadMode(TMAG5273_READ_MODE_STANDARD);
  tmag5273.configMagRange(TMAG5273_MAG_RANGE_40MT);
  tmag5273.configLplnMode(TMAG5273_LOW_NOISE);
  tmag5273.configMagTempcoMode(TMAG5273_MAG_TEMPCO_NdBFe);
  tmag5273.configConvAvgMode(TMAG5273_CONV_AVG_1X);
  tmag5273.configTempChEnabled(true);
  tmag5273.init();

  // digitalWrite(A0, HIGH);
  
  // start serial port at 9600 bps:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  establishContact();  // send a byte to establish contact until receiver responds

}

void loop() {
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    Serial.print(inByte);

    switch (inByte) {
      case 'a': // attach
        digitalWrite(toolchangeA, 0);
        digitalWrite(toolchangeB, 1);
        break;
      case 'r': // release
        digitalWrite(toolchangeA, 1);
        digitalWrite(toolchangeB, 0);
        break;
      case 's': // straighten
        digitalWrite(fingerjointA, 1);
        digitalWrite(fingerjointB, 0);
        break;
      case 'b': // bend
        digitalWrite(fingerjointA, 0);
        digitalWrite(fingerjointB, 1);
        break;
      case 'd': // down
        digitalWrite(mountjointA, 0);
        digitalWrite(mountjointB, 1);
        break;
      case 'u': // up
        digitalWrite(mountjointA, 1);
        digitalWrite(mountjointB, 0);
        break;
      case 'f': // FSR mode;
        pinMode(A0, OUTPUT);
        pinMode(A1, INPUT);
        pinMode(A2, OUTPUT);
        pinMode(A3, INPUT);
        pinMode(A4, INPUT);
        pinMode(A5, INPUT);

        digitalWrite(A0, HIGH);
        digitalWrite(A2, HIGH);
        readmode = 'f';
        break;
      case 'n': // Benhui mode;
        pinMode(A0, INPUT);
        pinMode(A1, OUTPUT);
        pinMode(A2, OUTPUT);
        pinMode(A3, INPUT);
        pinMode(A4, INPUT);
        pinMode(A5, INPUT);

        digitalWrite(A0, HIGH);
        digitalWrite(A1, HIGH);
        readmode = 'n';
        break;
      case 'p': // pneumatic mode;
        pinMode(A0, OUTPUT);
        pinMode(A1, INPUT);
        pinMode(A2, INPUT);
        pinMode(A3, INPUT);
        pinMode(A4, INPUT);
        pinMode(A5, OUTPUT);
        digitalWrite(A0, LOW);
        digitalWrite(A5, HIGH);
        readmode = 'p';
        break;
      case 'h': // Hall effect mode;
        readmode = 'h';
        break;
      case 'e': // EIT mode;
        pinMode(A0, INPUT);
        pinMode(A1, INPUT);
        pinMode(A2, INPUT);
        pinMode(A3, INPUT);
        pinMode(A4, INPUT);
        pinMode(A5, INPUT);
        readmode = 'e';
        break;
      case 'o': // Motors off
        digitalWrite(toolchangeA, 0);
        digitalWrite(toolchangeB, 0);
        digitalWrite(fingerjointA, 0);
        digitalWrite(fingerjointB, 0);
        digitalWrite(mountjointA, 0);
        digitalWrite(mountjointB, 0);
        break;
      default:
        break;
    }
    delay(10);
  }
  
  if (readmode=='p') { // Return pneumatic measurements before force data
    Serial.println(analogRead(A4));
    delay(10);
  } else if (readmode=='f') { // Return FSR measurements before force data
    Serial.print(analogRead(A1));
    Serial.print(", ");
    Serial.print(analogRead(A3));
    Serial.print(", ");
    delay(10);
  } else if (readmode=='n') { // Return Benhui measurements before force data
    Serial.print(analogRead(A0));
    Serial.print(", ");
    Serial.print(analogRead(A3));
    Serial.print(", ");
    delay(10);
  } else if (readmode=='h') { // I2C communication with hall effect sensor
    res = tmag5273.readMagneticField(&Bx, &By, &Bz, &T);
    Serial.print(String(Bx));
    Serial.print(", ");
    Serial.print(String(By));
    Serial.print(", ");
    Serial.println(String(Bz));
    // Serial.print(", ");
    delay(10);
  }
}


void establishContact() {
  while (Serial.available() <= 0) {
    delay(300);
  }
}
