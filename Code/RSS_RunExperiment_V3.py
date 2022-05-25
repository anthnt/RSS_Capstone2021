# SUMMARY:
# This script reads transducer data from an Arduino's serial output
# and reads it into a CSV file. It then converts the raw transducer data
# into the proper units and saves it to a new CSV file, with an accompanying
# plot. Parameters to open the proper serial port, CSV file names, transducer
# conversion variables are set in 'if __name__ == "__main__"' section.

# SOURCE:
# Written by: Anthony Newton, an768460@dal.ca
# Project: RSS Thruster Testing, 2021-2022

from serial.tools.list_ports import comports
import serial
import time
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from datetime import datetime


def usrOpenSerial(selectBaud, selectTimeout):
    # View open serial ports
    print('------------------------------------------------------------------')
    print('The following serial ports are open: ')
    portList = [p.device for p in comports()] # Lists active COM ports
    for onePort in portList:
        print(str(onePort))

    # User selects COM port connected to Arduino
    resume = 0
    while (resume == 0):
        selectPort = input("Select Arduino serial port (ex.'/dev/ttyS0'): ")
        for i in range(0, len(portList)):
            if (portList[i] == str(selectPort)):
                print('Port ', portList[i], ' selected.')
                selectPort = portList[i]
                resume = 1
        if (resume == 0):
            print('No matching serial port detected, try again')

    print('------------------------------------------------------------------')
    # Initialize serial connection with Arduino
    ser.baudrate = selectBaud
    ser.timeout = selectTimeout
    ser.port = selectPort
    ser.open()
    ser.reset_input_buffer
    time.sleep(5)


def readSerialMonitor():
    time.sleep(2)
    if ser.in_waiting: # Check to see if any message has been sent by Arduino
        inByte = ser.readline().strip() # Remove \n line endings from Arduino
        print(inByte.decode('utf-8'))
    else:
        print('Error, nothing returned.') # Error out if nothing received
        print('Press the reset button on Arduino and try again.')
        print('If further trouble is encountered, check HX711 wiring')
        quit()


def talkToArduino(usrInput, command):  
    
    resume = 'false'
    while (resume == 'false'):
        if(usrInput == 'n'):
            exit()

        if command == 'SOLENOIDS':
            print('Inputing amount of solenoids to actuate...')
            ser.write(command.encode('utf-8'))
            time.sleep(4)
            ser.write(usrInput.encode('utf-8'))
            readSerialMonitor()

        if command == 'AIRON':
            print('Inputting time to actuate...')
            ser.write(command.encode('utf-8'))
            time.sleep(4)
            ser.write(usrInput.encode('utf-8'))
            readSerialMonitor()

        if command == 'BEFOREAIR':
            print('Inputing time to read before actuation...')
            ser.write(command.encode('utf-8'))
            time.sleep(4)
            ser.write(usrInput.encode('utf-8'))
            readSerialMonitor()

        if command == 'AFTERAIR':
            print('Inputing time to read after actuation...')
            ser.write(command.encode('utf-8'))
            time.sleep(4)
            ser.write(usrInput.encode('utf-8'))
            readSerialMonitor()

        if command == 'TAREGO':
            print('Command sent...')
            ser.write(command.encode('utf-8'))
            time.sleep(4)
            readSerialMonitor()
            return

        ser.reset_input_buffer
        resume = 'true'


def usrCommands(numThruster):
    # Takes user inputs to send to Arduino

    command = 'SOLENOIDS'
    talkToArduino(numThruster, command)

    # Take user input
    usrInput = input('Enter time (ms) to actuate thrusters for (ex. "1000"): ')
    # Indicate what variable user input corresponds to on Arduino
    command = 'AIRON'
    # Send user input and what variable it corresponds to to Arduino
    talkToArduino(usrInput, command)

    usrInput = input(
        'Enter time (ms) to read transducer data before actuating (ex. "1000"): ')
    command = 'BEFOREAIR'
    talkToArduino(usrInput, command)

    usrInput = input(
        'Enter time (ms) to read transducer data after actuating (ex. "1000"): ')
    command = 'AFTERAIR'
    talkToArduino(usrInput, command)

    usrInput = input('Start experiment (y/n): ')
    command = 'TAREGO'
    talkToArduino(usrInput, command)


def serialToCSV(rawDataFileName):
    # Reads data from serial monitor to CSV file

    file = open(rawDataFileName, "w+")
    resume = 0  # true/false condition to record data
    while (resume == 0):
        if ser.in_waiting:
            incByte = ser.readline().strip()  # Read data from serial as byte format
            # Converts byte from serial monitor to string format
            incByte = incByte.decode('utf-8')
            incByte = incByte.strip()  # Strips string of \r\n characters that Arduino prints
            # print(incByte) # Uncomment this line to see if data is being read
            if (incByte == 'CSVSTART'):
                resume = 1

    while True:
        if ser.in_waiting:
            incByte = ser.readline().strip()  # Read data from serial as byte format
            # Converts byte from serial monitor to string format
            incByte = incByte.decode('utf-8')
            incByte = incByte.strip()  # Strips string of \r\n characters that Arduino prints
            # print(incByte) # Uncomment this line to see if data is being read
            if (incByte == 'CSVEND'):
                print('Data capture complete.')
                break
            file.write(incByte + "\n")

    file.close()  # Close CSV file


def processData(aCal, bCal, bitCount, maxP, minP, minV, maxV, Vs, rawDataFileName,
                processedDataFileName, processedDataPlotName, thrusterDia, numThruster):
    # PARSE RAW DATA INTO DATAFRAME:

    # Open CSV file containing raw data and generate dataframe.
    # Match the header names defined in Arduino RSS_LoadCell_Read script.
    df = pd.read_csv(rawDataFileName, usecols=['RawLoadReading',
                                               'RawPressureReading', 'Time', 'AirOn', 'TimeStarted',
                                               'TimeEnded'], low_memory=False)
    rawLoad = df['RawLoadReading'].to_numpy()
    rawTime = df['Time'].to_numpy()
    rawPressure = df['RawPressureReading'].to_numpy()
    AirOn = df['AirOn'].to_numpy()
    TimeStarted = df['TimeStarted'].to_numpy()
    TimeEnded = df['TimeEnded'].to_numpy()

    # Lists which contain processed data

    load = np.zeros(len(rawLoad)-1)
    pressure = np.zeros(len(rawLoad)-1)
    time = np.zeros(len(rawLoad)-1)

    # CONVERT RAW DATA TO PROPER UNITS:
    for i in range(len(rawTime)):

        # SAVE EXPERIMENT SETUP DATA:
        # These parameters are saved by Arduino and are entered into
        # serial monitor at the very end, and are captured here.
        if(i == (len(rawTime)-1)):
            AirOn = AirOn[i]
            TimeStarted = TimeStarted[i] - rawTime[0]
            TimeEnded = TimeEnded[i] - rawTime[0]
            break

        # CONVERT LOAD CELL DATA:
        # Convert raw load cell reading (in counts)
        # to a reading in grams using linear callibration curve fitting
        load[i] = aCal * rawLoad[i] + bCal

        # CONVERT PRESSURE DATA:
        # Convert raw pressure value reading (in counts) to a voltage value (V)
        pressure[i] = Vs * rawPressure[i] / (bitCount - 1)
        # Convert pressure voltage reading (in V) to pressure reading (PSI),
        # using linear interpolation
        pressure[i] = (pressure[i] - minV)*(maxP - minP)/(maxV - minV) + minP

        # CONVERT TIME DATA:
        # Convert arduino clocktime in millis
        # to time in millis which starts at 0
        if(i > 0):
            time[i] = rawTime[i] - rawTime[i-1] + time[i-1]

    maxLoad = np.nanmax(load)

    # OPTIONAL: APPLY SMOOTHING TO DATA
    #pressure_smoothed = savgol_filter(pressure, 7, 2)

    # SAVE PROCESSED DATA INTO NEW CSV FILE:

    # Saves processed data as dictionary
    processedData = {'Load_g': load,
                     'Pressure_PSI': pressure, 'time_millis': time}
    # Saves dictionary into pandas dataframe, to save as new CSV
    dfProcessed = pd.DataFrame(processedData)
    # Saves dataframe as CSV
    dfProcessed.to_csv(processedDataFileName)

    # MASK POINTS LOAD CELL DATA WHERE ELEMENTS ARE EQUAL TO NAN
    loadMask = np.isfinite(load)

    # PLOT PROCESSED DATA:
    fig, axs = plt.subplots(2, figsize=(15, 15))
    # fig.suptitle(f'Thruster Test on {strTime}: {AirOn} ms of Thrust \n \
    #    {numThruster} Thrusters, {thrusterDia} mm Outlet Diameter', fontsize=16 )

    fig.suptitle(f'{numThruster} Thrusters, {thrusterDia} mm Outlet Diameter\n \
        Maximum Thrust: {maxLoad:.4f} g', fontsize=16)

    axs[0].plot(time[loadMask], load[loadMask])
    # Plot time thruster started
    axs[0].axvline(x=TimeStarted, color='k', linestyle='--')
    # Plot time thruster ended
    axs[0].axvline(x=TimeEnded, color='k', linestyle='--')
    axs[0].legend(['Thrust', 'Solenoid On/Off'])
    axs[0].set_title('Thrust (g) versus Time (ms)')
    axs[0].set(xlabel='Time (ms)', ylabel='Grams of Force (g)')
    axs[0].grid()

    axs[1].plot(time, pressure, color='b', linestyle=':')
    # Comment this out if no smoothing applied
    #axs[1].plot(time, pressure_smoothed, color='r',)
    axs[1].axvline(x=TimeStarted, color='k', linestyle='--')
    axs[1].axvline(x=TimeEnded, color='k', linestyle='--')
    # axs[1].legend(['Pressure', 'Smoothed Pressure',
    #              'Solenoid On', 'Solenoid Off'])
    axs[1].legend(['Pressure', 'Solenoid On/Off'])
    axs[1].set_title('Gauge Pressure (PSI) versus Time (ms)')
    axs[1].set(xlabel='Time (ms)', ylabel='Gauge Pressure (PSI)')
    axs[1].grid()

    fig.savefig(processedDataPlotName)
    plt.show()


if __name__ == "__main__":
    # SET SERIAL MONITOR SPECIFICATIONS AND INITIALIZE:
    # Ensure baud rate matches baud rate set in Arduino code
    selectBaud = 115200
    selectTimeout = 60
    ser = serial.Serial()  # Initialize serial ports
    usrOpenSerial(selectBaud, selectTimeout)

    # SET LOAD CELL PARAMETERS:
    # Linear fit calibration values, determined
    # from RSS_LoadCell_Callibration_Order2
    calDate = '2022-03-23'  # Date calallibration values were last entered
    aCal = 0.000262  # slope of linear calibration fit
    bCal = 0.008974  # b point of linear calibration fit

    print('The following linear calibration values will be used:')
    print(f'a = {aCal}, b = {bCal}')
    print(f'They were entered on {calDate}.')

    # SET THRUSTER PARAMETERS
    print('------------------------------------------------------------------')
    numThruster = input(
        "Enter number of solenoids/thrusters to actuate (ex. '1-4'): ")
    thrusterDia = input("Enter diameter of thruster outlet (mm) (ex. '0.4'): ")

    # SET PRESSURE TRANSDUCER PARAMETERS:
    bitCount = 1024  # Maximum resolution of Arduino ADC
    maxPressure = 200  # Maximum pressure of pressure transducer (PSI)
    minPressure = 0  # Minimum pressure of pressure transducer (PSI)
    minPressureVoltage = 0.5  # Minimum output of pressure transducer (V)
    maxPressureVoltage = 4.5  # Maximum output of pressure transducer (V)
    pressureSupplyVoltage = 5.0  # pressure transducer supply voltage (V)

    # SPECIFICY CSV FILE NAME WHICH CONTAINTS RAW AND PROCESSED DATA:
    # The script will append the time of file save to the processed data file
    rawDataFileName = f'RawThrusterData'  # dont add .csv
    processedDataFileName = f'ProcessedThrusterData'  # dont add .csv'
    processedDataPlotName = f'ProcessedThrusterPlot'  # dont add .png

    # Get time at which CSV is generated to add to title of CSV
    strTime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    # Get location of files to save to
    cwd = os.path.dirname(os.path.abspath(__file__))
    # Update file names
    rawDataFileName = f'{cwd}\Plots\{rawDataFileName}_{numThruster}sol_D{thrusterDia}_{strTime}.csv'
    processedDataFileName = f'{cwd}\Plots\{processedDataFileName}_{numThruster}sol_D{thrusterDia}_{strTime}.csv'
    processedDataPlotName = f'{cwd}\Plots\{processedDataPlotName}_{numThruster}sol_D{thrusterDia}_{strTime}.png'

    # START PROGRAM AND SERIAL MONITOR:

    usrCommands(numThruster)
    serialToCSV(rawDataFileName)
    processData(aCal, bCal, bitCount, maxPressure, minPressure, minPressureVoltage,
                maxPressureVoltage, pressureSupplyVoltage, rawDataFileName,
                processedDataFileName, processedDataPlotName, thrusterDia, numThruster)

    # CLOSE SERIAL MONITOR:
    ser.reset_output_buffer()
    ser.close()  # Close serial port so it may be used by other programs
