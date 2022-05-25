/*
 * Summary:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * This program reads load cell and pressure transducer data using Arduino.
 * The program, in conjuction with a Python serial monitor script, takes from
 * the user:
 * 
 * - How many solenoids to actuate
 * - How long to actuate the solenoids
 * - How long to read data from the load cells
 * 
 * It then actuates the solenoids, reads data from the load cell and pressure
 * transducer, and prints it to serial monitor. The python script saves the data
 * to a CSV file. A second python script converts the CSV data to the desired units.
 * 
 * NOTE: 
 * 
 * If using a laptop battery as source of power for Arduino and load cell,
 * please disconect the laptop from it's charger. This will reduce noise in
 * the load cell signal.
 *  
 * The HX711 is set to 80 Hz reading by removing a connection on the board itself. 
 * Its default is 10 Hz.
 * 
 * Conversion of the load cell signal (counts) and pressure transducer signal (V)
 * to grams and PSI is performed in the python script.
 * 
 * The user should determine the load cell calibrations beforehand using the
 * Arduino script, and import the values into the Python unit conversion script.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

 * Settings for RSS apparatus:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Board     -> "Arduino Nano"
 * Processor -> "ATmega168"
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * Arduino -> solenoid output pin mapping:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   terminal 1       terminal 2      terminal 3
 * [[+][2][10][9]]  [[+][3][8][7]]  [[+][4][6][5]]
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 * Open pins left for DAQ: 1,11,12,13,14
 * 
 * Written by: Anthony Newton, Nolan Cain
 * Project: RSS Capstone 2021-2022
 * 
 * Source for HX711 library: https://github.com/olkal/HX711_ADC
*/

#include <HX711_ADC.h>

// solenoid control pins (here for circuit wiring, don't need to be executed)
//const int pinSolnd1 = 5; // load cell solenoid
//const int pinSolnd2 = 4; // solenoid 2
//const int pinSolnd3 = 3; // solenoid 3
//const int pinSolnd4 = 2; // solenoid 4

//HX711 pins:
const int HX711_dat = A4; //HX711 dat pin, refered to as HX711_dout in HX711_ADC library
const int HX711_clk = A5; //HX711 clk pin, refered to as HX711_sck in HX711_ADC library
const int pinPressure = A6; // pressure read pin

//constructors:
HX711_ADC LoadCell(HX711_dat, HX711_clk);

//constants and global var
boolean _resume = false;
unsigned long t = 0;
unsigned long air_on = 1000; //How long solenoid valve is to be actuated for (s)
unsigned long before_air = 1000; //Reading data before solenoid is actuated.
unsigned long after_air = 1000; //Reading data after the solenoid is actuated
unsigned int numSolnd; //How many solenoids to actuate

void setup() 
{
  // variable used for port manipulation
  // initialize pins 2,3,4 and 5 as output for actuating solenoids
  DDRD = B1111110;

  Serial.begin(115200); delay(10);

  // initialize pressure transducer read pin
  pinMode(pinPressure,INPUT);

  // Load cell initialization
  LoadCell.begin();
  unsigned long stabilizingtime = 5000;
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // We are not using the libraries default calibration method. Therefore set to 1.
    // Succesfully initialized
  }
}

void loop() 
{
  Serial.flush();
  _resume = false;

  if(Serial.available() > 0) 
  {
    String inBytes = Serial.readString();

    if(inBytes == "AIRON") 
    {
      // Reads from user how long to actuate solenoids
      while(_resume == false) {
        if(Serial.available() > 0) 
        {
          air_on = Serial.parseInt();
          Serial.println(air_on);
          Serial.flush();
          _resume = true;
        }
      }
    }

    else if(inBytes == "BEFOREAIR") 
    {
      // Reads from user how long to read data before solenoids are actuated
      while(_resume == false) 
      {
        if(Serial.available() > 0) 
        {
          before_air = Serial.parseInt();
          Serial.println(before_air);
          Serial.flush();
          _resume = true;
        }
      }
    }

    else if(inBytes == "AFTERAIR") 
    {
      // Reads from user how long to read data after solenoids are actuated
      while(_resume == false) 
      {
        if(Serial.available() > 0) 
        {
          after_air = Serial.parseInt();
          Serial.println(after_air);
          Serial.flush();
          _resume = true;
        }
      }
    }

    else if(inBytes == "SOLENOIDS") 
    {
      // Reads from user how many solenoids to actuate
      while(_resume == false) 
      {
        if(Serial.available() > 0) 
        {
          numSolnd = Serial.parseInt();
          Serial.println(numSolnd);
          Serial.flush();
          _resume = true;
        }
      }
    }

    else if(inBytes == "TAREGO") 
    {
      // User commands script to start actuation and transducer reading
      Serial.println("Preparing experiment...");
      delay(5000);
      actuate();
    }
  }
}

// actuate() actuates the solenoid valves and reads transducer data to serial monitor
void actuate()
{
  unsigned long time_on = 0; //How long the solenoid valve has been on for
  unsigned long time_on_prev = 0;
  unsigned long tStarted = 0; // Records when actuation actually started
  unsigned long tEnded = 0; // Records when actuation actually ended
  unsigned int sample = 0; // Records transudcer sample number
  // Variable used to determine whether data from load cell is ready to read in
  static boolean newDataReady = 0;
  // Variable to control how much data from load cell is recieved (keep at 0)
  const int serialPrintInterval = 0;

  // Reset pressure
  PORTD = B00111100; // 1,2,3,4 (all)
  delay(1000);
  PORTD = B00000000;
  delay(2000);
     
  Serial.println("CSVSTART"); //Indicates start of data to Python script
  Serial.println("Sample,RawLoadReading,RawPressureReading,Time,AirOn,TimeStarted,TimeEnded"); // CSV Column headers

  boolean turnedOn = false;
  _resume = false;
  time_on_prev = millis(); // Reads Arduino clock time to determine when the reading loop has started
  
  while(time_on < (air_on + before_air + after_air)) 
  {

    // Turn solenoids on
    if (_resume == false && time_on > before_air) 
    {   
      // Actuates solenoids 1,2,3,4 (or any combination thereof) 
      // once the time to read before actuation has ended
      if (numSolnd == 1)
      {
        tStarted = millis();
        PORTD = B00100000; // 1 (Load cell solenoid)
      }
      else if (numSolnd == 2)
      {
        tStarted = millis();
        PORTD = B00110000;  // 1,2
      }
      else if (numSolnd == 3)
      {
        tStarted = millis();
        PORTD = B00111000; // 1,2,3
      }
      else if (numSolnd == 4) 
      {
        tStarted = millis();
        PORTD = B00111100; // 1,2,3,4 (all)
      }
      else
      {
        Serial.println("Invalid number of solenoids selected");
        Serial.println("Please restart");
        return;
      }
      turnedOn = true;
      _resume = true;
    }

    if (time_on > (before_air + air_on) && turnedOn == true)
    {
        PORTD = B00000000; //Turn off solenoids
        tEnded = millis(); //Record actual time the solenoids were turned off
        turnedOn = false;
    }
    
    if (LoadCell.update())
    {
      newDataReady = true; // Check if data is ready to be read from load cell
    }

    if (newDataReady) 
    {
      if (millis() > t + serialPrintInterval) 
      {
        float i = LoadCell.getData(); // Read data from load cell. By default, the moving average data set of 1 sample is used.
        float p = analogRead(pinPressure);
        sample = sample + 1;
        Serial.print(sample);
        Serial.print(",");
        Serial.print(i);
        Serial.print(",");
        Serial.print(p);
        Serial.print(",");
        t = millis();
        Serial.println(t);
        newDataReady = 0;
      }
    }
    else
    {
      // comment this block out to match frequency of pressure reading
      // to frequency of load cell reading
      float p = analogRead(pinPressure);
      sample = sample + 1;
      Serial.print(sample);
      Serial.print(",,");
      Serial.print(p);
      Serial.print(",");
      Serial.println(millis());
    }

    time_on = millis() - time_on_prev; // update time counter
  }
  // close all solenoids
  PORTD = B00000000; //Ensure solenoids are off

  Serial.print(",,,,");
  Serial.print(air_on);
  Serial.print(",");
  Serial.print(tStarted);
  Serial.print(",");
  Serial.println(tEnded);
  Serial.println("CSVEND"); //Tells the python script to stop reading
}
