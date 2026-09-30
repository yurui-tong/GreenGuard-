import threading
import time
import csv
from phidget_heading import PhidgetHeading
from receive import Corr_receive
from KalmanF import kal_fil
import numpy as np
import pymap3d as pm
from rover import GPS_rover
from data_processing import data_processing

## Input settings
Baudrate=115200
GPS_com='/dev/ttyACM0'
Radio_com='/dev/ttyUSB0'
IMU_serial=721561

## Initialize classes
print("Initializing devices and classes...")
Corr_receive_base = Corr_receive(PORT=Radio_com, BAUD_RATE=Baudrate)
GPS_rover = GPS_rover(PORT=GPS_com, BAUD_RATE=Baudrate, TIMEOUT=1)
IMU = PhidgetHeading(serial=IMU_serial, data_interval=10)
IMU.start()
kf=kal_fil()

## Run baseline collection
# Rover GPS baseline
GPS_rover_zero_data = GPS_rover.collect_gps_samples(duration_sec=10)
GPS_rover_zero = GPS_rover.average_gps(GPS_rover_zero_data)
print(f"Rover baseline set: {GPS_rover_zero}")
# GPS correction baseline
timeout_cor_base = 5.0
print(f"Waiting up to {timeout_cor_base}s for correction baseline...")
msg = Corr_receive_base.get_message(timeout=timeout_cor_base)           
cor_base =np.fromstring(msg, sep=',')
print(f"Correction baseline set: {cor_base}")

## Create data storage lists
GPS_rover_data_list = []
GPS_cor_list = []
cor_list = []
yaw_list = []
state_list = []
time_list = []


## Main loop
print("Starting main loop...")

start_time = time.time()
last_time = time.time()
try:
    while True:
        current_time = time.time()
        dt = current_time - last_time
        # print(f"Loop dt: {dt:.4f}s")
        last_time = current_time

        # Apply Kalman Filter Prediction Step
        kf.predict(dt)

        gps_data = GPS_rover.get_GPS()
       
        # Prepare placeholders for this step's data
        current_gps_x, current_gps_y = np.array([np.nan, np.nan])
        current_cor = np.array([np.nan, np.nan,np.nan])
        GPS_x, GPS_y = np.array([np.nan, np.nan])

        if gps_data is not None:
            # Get Correction
            try:
                msg = Corr_receive_base.get_message(timeout=0.01)
                cor = np.fromstring(msg, sep=',')
            except Exception:
                if len(cor_list) > 0:
                    cor = cor_list[-1]
                else:
                    cor = np.array([0.0, 0.0, 0.0])
            print("Correction Received:", cor)
            cor -= cor_base
            print("Correction Received:", cor)

            # Convert to Local ENU
            GPS_x, GPS_y, _ = pm.geodetic2enu(
                gps_data['lat'], gps_data['lon'], gps_data['height'],
                GPS_rover_zero['lat'], GPS_rover_zero['lon'], GPS_rover_zero['height']
            )
            print("Rover GPS Position:", GPS_x, GPS_y)
            # Convert to Local ENU
            rover_x = GPS_x - cor[0]
            rover_y = GPS_y - cor[1]
            print("Rover GPS Position with Correction:", rover_x, rover_y)

            # Apply Kalman Filter Update with GPS and Correction
            kf.update_position(rover_x, rover_y)
            # Update placeholders for logging
            current_gps_x, current_gps_y = rover_x, rover_y
            current_cor = cor
        # print("Rover GPS Position:", current_gps_x, current_gps_y)

        # Collect IMU Yaw
        yaw=IMU.get_heading(wait_for_data=False) # Get current heading from IMU
        if yaw is not None:
            kf.update_heading(yaw)
        # print("Rover Heading:", yaw)

        # Kalman Filter State
        state_val = kf.kf.x.ravel()
        # print("Kalman Filter State:", state_val)

        # Save data
        GPS_rover_data_list.append([GPS_x, GPS_y])
        GPS_cor_list.append([current_gps_x, current_gps_y])
        cor_list.append(current_cor)
        yaw_list.append(yaw if yaw is not None else np.nan)
        state_list.append(state_val)
        time_list.append(current_time - start_time)

        # Loop speed control (approx 100Hz max)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopping main loop...")
finally:
    # Close devices
    print("Closing...")
    try:
        GPS_rover.close()
        IMU.stop()
        Corr_receive_base.close()
    except:
        pass

    # Save Data & Plot
    if len(time_list) > 0:
        print(f"Processing {len(time_list)} data points...")
        data = {
            "GPS_rover_data_list": GPS_rover_data_list,
            "GPS_cor_list": GPS_cor_list,
            "cor_list": cor_list,
            "yaw_list": yaw_list,
            "state_list": state_list,
            "time_list": time_list
            }
        
        processor = data_processing(data)
        processor.plots()
        processor.save_data_to_csv(filename="test_data1.csv")
    else:
        print("No data collected.")