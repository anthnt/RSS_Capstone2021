/*
   Callibrate Load cell and HX711 using Arduino and known masses.
   Based off of Olkal HX711_ADC library and examples.

   Written by: Anthony Newton, Nolan Cain
   Project: RSS Capstone 2021-2022
   Updated: 2022-03-24
*/

// Load Cell
#include <HX711_ADC.h>
#include <curveFitting.h> //Used to generate linear interpolation curve for callibration

const int HX711_dat = A4; // HX711 dat pin, refered to as HX711_dout in HX711_ADC library
const int HX711_clk = A5; // HX711 clk pin, refered to as HX711_sck in HX711_ADC library
HX711_ADC LoadCell(HX711_dat, HX711_clk);

unsigned long t = 0;
float a = 0.000262; // Calibration value a (slope)
float b = 0.008974; // Calibration value b (y-axis intercept)

// Pressure Transducer
const int pressureInputPin = A6;
const float maxPressure = 200; // Max pressure of transducer (PSI)
const float minV = 0.5; // Min output voltage of transducer (PSI)
const float maxV = 4.5; // Max output voltage of transducer (PSI)
const float supplyV = 5.0; // Supply voltage of Arduino
const float bitSize = 1023.0; // Int resolution of Arduino
const int avgCount = 64; // Amount of samples to average

// Solenoid

void setup()
{
  Serial.begin(57600);
  Serial.println();
  DDRD = B11111110;
  LoadCell.begin();

  unsigned int stabilizingtime = 2000; //Stabilizing time to improve precision, minimum 2 seconds
  bool _tare = true; //set this to false if you don't want tare to be performed in the next step

  LoadCell.start(stabilizingtime, _tare);

  // Check whether HX711 is connected properly
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check HX711 wiring and pin designations");
    while (1);
  }
  else {
    // Not using default calibration method, keep to 1.0
    LoadCell.setCalFactor(1.0);
    Serial.println("Loading complete.");
  }

  while (!LoadCell.update());
}

void loop()
{
  Serial.println("To calibrate load cell, enter 'c' in serial monitor.");
  Serial.println("To read load/pressure or actuate solenoids, enter 'r' in serial monitor.");
  Serial.println(" - Uses hardcoded calibration values for load cell reading.");

  while (1)
  {
    if (Serial.available() > 0)
    {
      char inByte = Serial.read(); // read user entry in serial
      if (inByte == 'c')
      {
        calibrate();
        break;
      }
      if (inByte == 'r')
      {
        readTransducer();
        break;
      }
    }
  }
}

void readTransducer()
{
  // Load Cell
  LoadCell.setSamplesInUse(1);
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; // increase value to slow down serial print activity
  // Pressure Transducer
  int rawPressVal = 0; // Value to read raw sample into
  float psi = 0;
  float V = 0;

  const int avgNum = 80; // amount of times to average transducer data

  tareLoad();
  Serial.println("To return to options, enter 'e' in serial monitor.");
  Serial.println("Optionally enter number of solenoids/thrusters to actuate for 2 sec (ex. '1-4'): ");
  Serial.println("Mass (g), Pressure (PSI), Time (millis)");

  while (1)
  {
    int counter = 1;
    float avgLoad = 0;

    while (counter <= avgNum)
    {
      if (LoadCell.update()) newDataReady = true; // check for new data/start next conversion
      if (newDataReady)
      {
        if (millis() > t + serialPrintInterval)
        {
          float i = LoadCell.getData();
          t = millis();
          newDataReady = false;
          avgLoad = avgLoad + i;
          rawPressVal += analogRead(pressureInputPin); // Read in samples
          counter = counter + 1;
        }
      }
    }
    rawPressVal = rawPressVal / counter;
    V = (supplyV * rawPressVal) / bitSize;
    psi = mapFloat(V, minV, maxV, 0, maxPressure);
    // calculate average load cell value
    avgLoad = avgLoad / ((float)counter);
    avgLoad = a * avgLoad + b;
    Serial.print(avgLoad);
    Serial.print(" , ");
    Serial.print(psi);
    Serial.print(" , ");
    Serial.println(t);

    // receive command from serial terminal
    if (Serial.available() > 0)
    {
      char inByte = Serial.read();
      if (inByte == 't')
      {
        tareLoad();        
      }
      if (inByte == '1' || inByte == '2' || inByte == '3'|| inByte == '4')
      {
        solenoids(inByte);
      }
      else if (inByte == 'e')
      {
        return;
      }
    }
  }
}

void calibrate()
{
  Serial.println("Remove any load applied to the load cell."); // Taring zeros the scale
  delay(2000);
  boolean _resume = false;
  tareLoad();

  Serial.println("Enter number of callibration trials to carry out, a minimum of 2 is required (ex. '2'): ");

  int n_mass; // Determine number of masses/trials used to calibrate the load cell
  _resume = false;
  while (_resume == false)
  {
    if (Serial.available() > 0)
    {
      n_mass = Serial.parseInt(); // read user entry in serial
      _resume = true;
    }
  }
  Serial.println(n_mass);

  // Read in the known callibration masses from user, and read in actual value read by load cell
  // Save values to known_mass and read_mass respectively
  double known_mass[n_mass]; // Create array to store known masses for callibration
  double read_mass[n_mass];  // Create array to store load cell readings of known masses
  double avgRead_mass = 0.0; // Mass values are averaged for amount avgTrial
  int avgTrial = 80;         // Amount of times to read in a load cell value for each mass which is then averaged

  for (int i = 0; i < n_mass; i++)
  {
    _resume = false;
    Serial.println("***");
    Serial.print("Place known mass ");
    Serial.print(i + 1);
    Serial.println(" on the scale.");
    Serial.println("Then send the known mass value in grams (ex.'100.00') from serial monitor:");

    while (_resume == false)
    {
      LoadCell.update();
      if (Serial.available() > 0)
      {
        known_mass[i] = Serial.parseFloat();
        if (known_mass[i] != 0)
        {
          Serial.print("Known mass is ");
          Serial.print(known_mass[i]);
          Serial.println(" (g).");
          LoadCell.refreshDataSet(); // refresh the dataset to be sure that the known mass is measured correct
          read_mass[i] = LoadCell.getData();
          Serial.print("The load cell reads: ");
          Serial.print(read_mass[i]);
          Serial.println(".");
          _resume = true;
        }
      }
    }
  }

  Serial.println("Reading known masses is complete, determining calibration fit...");

  // Calculate calibration value using a linear fit. Order is defined at start of script.
  // for a first order system: y = mx + b
  int order = 1; // Determines the order of the polynomial curve fit
  char buf[100];
  double coeffs[order + 1];
  int ret = fitCurve(order, sizeof(read_mass) / sizeof(float), read_mass, known_mass, sizeof(coeffs) / sizeof(double), coeffs);

  // Print calculated coefficients of calibration curve
  if (ret == 0)
  { // Returned value is 0 if no error
    uint8_t c = 'a';
    Serial.println("Coefficients are");
    for (int i = 0; i < sizeof(coeffs) / sizeof(double); i++)
    {
      snprintf(buf, 100, "%c=", c++);
      Serial.print(buf);
      Serial.print(coeffs[i], 6);
      Serial.print('\t');
    }
  }
  else
  {
    Serial.println("An error has been encountered with the calibration curve fitting, please try again");
    return;
  }
  Serial.println();

  // This code has to be modified if the order of the calibration fit is increased
  a = coeffs[0];
  b = coeffs[1];

  Serial.println("Please record the calibration values. Enter 'e' in serial monitor to return to options.");
  Serial.println("***");
}

void tareLoad()
{
  LoadCell.update();
  LoadCell.tare();
  long _offset = LoadCell.getTareOffset();
  LoadCell.setTareOffset(_offset);
  Serial.println("Tare Completed.");
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void solenoids(char numSolnd)
{
  // Actuates solenoids 1,2,3,4 (or any combination thereof)
  // once the time to read before actuation has ended
  if (numSolnd == '1')
  {
    PORTD = B00100000; // 1 (Load cell solenoid)
  }
  else if (numSolnd == '2')
  {
    PORTD = B00110000;  // 1,2
  }
  else if (numSolnd == '3')
  {
    PORTD = B00111000; // 1,2,3
  }
  else if (numSolnd == '4')
  {
    PORTD = B00111100; // 1,2,3,4 (all)
  }
  else
  {
    Serial.println("Invalid number of solenoids selected");
    Serial.println("Please restart");
    return;
  }
  delay(2000);
  PORTD = B00000000;
}
