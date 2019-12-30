import pyfirmata
import time
import datetime
import csv
import os.path
import numpy as np
import transformations as tf
import re
from sweeppy import Sweep
import scan_setting
import sweep_helpers

board = pyfirmata.Arduino('COM15')      #Arduino assigned to COM15
it = pyfirmata.util.Iterator(board)           #
it.start()
pin9 = board.get_pin('d:9:s')


def main():    
    settings = scan_setting.ScanSettings()

    #Field names for the CSV
    field_names = ['X', 'Y', 'Z', 'SIGNAL_STRENGTH']

    #File name for the scan
    #file_name = "3Dscan " + datetime.datetime.fromtimestamp(time.time()).strftime('%Y-%m-%d %H-%M-%S') + '.csv'
    file_name = "3Dscan.csv"

    num_sweeps = 182    #sweep for 180 degree
    valid_scan_index = 0
    count =0
    baseAngle = 0

    #create the file, create a writer to write the CSV file and write header to the file
    with open(file_name, 'w', newline = '') as file:
        writer = csv.DictWriter(file, fieldnames = field_names)
        writer.writeheader()
        #setup the sweep and start scanning
        with Sweep('COM11') as sweep:
            sweep.set_motor_speed(3)
            print("Motor Speed: %s Hz" % (sweep.get_motor_speed()))
            print("Sample Rate: %s Hz" % (sweep.get_sample_rate()))
            #wait for the device to get ready
            while True:
                if sweep.get_motor_ready() is True:
                    print("Motor is ready!")
                    break
                time.sleep(0.5)
            print("Start scanning now...")
            startTime = time.time()
            sweep.start_scanning()
            
            while True:
                try:                                                    
                    #reading data    
                    for scan in sweep.get_scans():
                        remove_distance_extremes(scan, 50, 20000)
                        remove_angular_window(scan, 200, 310)

                        #if valid_scan_index >= num_sweeps -2:
                            #remove_angular_window(scan, settings.get_deadzone(), 360)

                        if contains_unordered_samples(scan):
                            continue
                        
                        converted_coords = transform_scan(scan, 90, baseAngle)  #90 is the mount angle       

                        for n, sample in enumerate(scan.samples):
                            writer.writerow({
                                'X': int(round(converted_coords[n,0])),
                                'Y': int(round(converted_coords[n,1])),
                                'Z': int(round(converted_coords[n,2])),
                                'SIGNAL_STRENGTH': sample.signal_strength
                                })

                        valid_scan_index += 1                    

                        count +=1
                        if count%7== 0:     #record data for 7 times then only increase base angle by 1 degree
                            baseAngle +=1
                            print("Base angle: %d" % (baseAngle))

                        move_servo(baseAngle)                        
                        time.sleep(0.6)
                        
                        if baseAngle==180:
                            break
                except:     #error handling for get_scans()
                    while True:
                        try:
                            sweep.stop_scanning()
                            while True:                        
                                if sweep.get_motor_ready() is True:
                                    #print("Reset done")
                                    sweep.start_scanning()
                                    break
                                time.sleep(0.5)
                            break
                        except:     #error handling for stop scanning
                            time.sleep(0.1)
                if baseAngle==180:      #slowly turn the servo back to original position
                    for i in range(baseAngle,0,-1):
                        move_servo(i)
                        time.sleep(0.05)
                    break

            while True:
                try:
                    sweep.stop_scanning()
                    break
                except:
                    time.sleep(0.1)
            
            stopTime = time.time()
        
    totalDuration = (stopTime - startTime)/60
    print("Time taken: %.3f mins" % (totalDuration))
    print("Scanning Completed!")   
    file.close()        #end of main function
        
def polar_to_cartesian(radius, angle):
    theta = np.deg2rad(angle)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return (x,y)

def get_scan_rotation_matrix(mount_angle, base_angle):
    alpha =  np.deg2rad(-mount_angle)       #x-axis
    beta = 0                                                       #y-axis
    gamma = np.deg2rad(-base_angle)        #z-axis
    return tf.euler_matrix(alpha, beta, gamma, 'rxyz')      #r stands for rotating plane, check the documentation for more info

def remove_distance_extremes(scan, low, high):
    scan.samples[:] = [sample for sample in scan.samples if(
        sample.distance>=low and sample.distance<=high)]
    
    scan.samples[:] = [sample for sample in scan.samples if(
        sample.signal_strength>10 or sample.signal_strength<30000)]          #save data between signal strength of 10 and 30000

def remove_angular_window(scan,low,high):
    scan.samples[:] = [sample for sample in scan.samples if(
        0.001*sample.angle<low or 0.001*sample.angle>high)]

def contains_unordered_samples(scan):
    previous_angle = -1
    for index, sample in enumerate(scan.samples):
        if sample.angle<= previous_angle:
            return True
        previous_angle = sample.angle
    return False

def transform_scan(scan, mountAngle, baseAngle):
    rot_mat = get_scan_rotation_matrix(mountAngle, baseAngle)

    numSamples = len(scan.samples)
    arraySize = (numSamples, 4)
    coords = np.zeros(arraySize)
    
    for index, sample in enumerate(scan.samples):        
        cartesianCoord = polar_to_cartesian(sample.distance, 0.001*sample.angle)
        homogeneousCoord = np.array([cartesianCoord[0], cartesianCoord[1], 0, 1])
        coords[index] = homogeneousCoord

    coords = np.dot(coords, rot_mat)

    return coords

def move_servo(a):
    pin9.write(a)

if __name__ == "__main__":
    move_servo(0)
    main()    
    exit()
    time.sleep(3600)

