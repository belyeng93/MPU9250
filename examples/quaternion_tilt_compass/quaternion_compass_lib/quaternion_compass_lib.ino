#include "mpu9250_lib.h"


// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value


MCU9250::MCU9250 *mpu;
// -----------------
// setup()
// -----------------
void setup()
{
  mpu = new MCU9250::MCU9250();
 
}

// ----------
// loop()
// ----------
void loop()
{
  mpu->update();
  // ----- Perform these tasks every 500mS
  delt_t = millis() - count;
  if (delt_t > 500)
  {
    compass_rose();                         // View compass heading using Processing "compass_rose.pde"
    count = millis();                          // Reset timer
  }
}

// ------------------------
// compass_rose()
// ------------------------
/* View heading using Processing "compass_rose.pde" */
void compass_rose()
{
  // ----- read input character
  if (Serial.available()) {
    InputChar = Serial.read();
    if ((InputChar == 's') || (InputChar == 'S')) {
      LinkEstablished = true;
    }
  }

  float heading;
  mpu->getHeading(heading);

  // ------ Create output data string
  OutputString = String(heading);

  // ----- Send string if link established
  if (LinkEstablished && ((InputChar == 's') || (InputChar == 'S'))) {
    Serial.println(OutputString);
  }

  // ----- send 'S' if link not established
  if (micros() > Stop1) {
    Stop1 += Timer1;
    if (!LinkEstablished) {
      Serial.println('S');
    }
  }
}

// ------------------------
// view_heading_SM()
// ------------------------
/* View heading on serila monitor */
void view_heading_SM()
{
  float yaw, pitch, roll, heading, temperature;
  mpu->getYawPitchRoll(yaw, pitch, roll);
  mpu->getHeading(heading);
  mpu->getTemperature(temperature);

  // ----- send the results to the Serial Monitor
  Serial.print("        Pitch ");
  print_number((short)pitch);
  Serial.print("        Roll ");
  print_number((short)roll);
  Serial.print("        Heading ");
  print_number((short)heading);

  // ----- Print temperature in degrees Centigrade
  tempCount = readTempData();                               // Read the temperature registers
  temperature = ((float) tempCount) / 333.87 + 21.0;        // Temp in degrees C
  Serial.print("        Temp(C) ");
  Serial.print(temperature, 1);
  
  Serial.println("");
  count = millis();

  sumCount = 0;
  sum = 0;
}
// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop integer numbers jumping around */
long print_number(short number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}

// ------------------------
// print_number()
// ------------------------
/* Overloaded routine to stop float numbers jumping around */
float print_number(float number) {
  String myString = String(number);
  short numberChars = myString.length();
  for (short i = 0; i < 6 - numberChars; i++) {
    Serial.print(" ");
  }
  Serial.print(myString);
}
