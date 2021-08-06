#@package RoDeP statemachine, coded in Python.
#Documentation for the RoDeP's state machine use.

#Going through every state and algorithms governing the robot's behaviour.

import sys
from abc import ABC, abstractmethod
import cv2
import numpy as np
import matplotlib.pyplot as pl
import math
import time
import serial
import json
import os
import datetime
from statemachine import StateMachine, State
import pickle

import RPi.GPIO as GPIO

from pupil_apriltags import Detector
from io import BytesIO
from picamera import PiCamera
from PIL import Image
from rodep.camera import make_camera
from simple_pid import PID

# Configuration

rover_width_meters = 0.30
rover_length_meters = 0.39
rover_wheel_diameter = 0.15

# Stepper motor (product ID: 57STH56-2804MB)
# https://www.phidgets.com/?tier=3&catid=24&pcid=21&prodid=349
rover_motor_microsteps = 8
rover_motor_steps = 200 * rover_motor_microsteps

# Encoder (product ID: HKT2204-702C1-300B-5E)
# https://www.phidgets.com/?tier=3&catid=103&pcid=83&prodid=404
#
# SAME AS rover_motor_steps because not using an encoder, yet.
rover_encoder_steps = rover_motor_steps

camera_width = 640
camera_height = 480

Kp = 0.007 #proportional correction coeff
kDefaultVelocity = 300 #250 previously (stable)
kBackwardsVelocity = 50


class Rover:
    """
    The Rover class stores the physical properties of the rover,
    including its dimensions and the resolution of the motors.
    """
    def __init__(self, width, length, wheel_diameter,
                 motor_steps_per_revolution, encoder_steps_per_revolution):
        self.width = width
        self.length = length
        self.wheel_circumference = np.pi * wheel_diameter
        self.motor_steps_per_revolution = motor_steps_per_revolution
        self.encoder_steps_per_revolution = encoder_steps_per_revolution


class MotorController(ABC): #todo : messages a l'arduino
    @abstractmethod
    def moveat(self, speed_left, speed_right):
        pass

    @abstractmethod
    def move(self, duration, steps_left, steps_right):
        pass


class SerialMotorController(MotorController): #todo
#MotorController : folder for picocom
    def __init__(self, device):
        #establishing a serial link with the Arduino board, at 115200 bauds
        self.link = serial.Serial(device, 115200)
        #waiting for the connexion to be established
        time.sleep(2.0)
        self.link.write("\r\n".encode('ascii'))
        self.link.write("\r\n".encode('ascii'))
        self.link.write("#?:xxxx\r\n".encode('ascii'))
        s = self.link.readline().decode("ascii").rstrip()
        print("Init: %s" % s)


    def send_command(self, s): #sending a command as the Rasp-Pi
        #print("Command: %s" % s)
        command = "#" + s + ":xxxx\r\n"
        self.link.write(command.encode('ascii'))
        return self.assert_reply(self.read_reply())

    def read_reply(self): #reply from the Arduino
        while True:
            s = self.link.readline().decode("ascii").rstrip()
            if s[0] == "#":    
                if s[1] == "!":
                    print("Log: %s" % s)
                else: #not an error
                    break;
        return s

    def assert_reply(self, line):
        s = str(line)
        start = s.find("[")
        end = 1 + s.find("]")
        array_str = s[start:end]
        return_values = json.loads(array_str)
        status_code = return_values[0]
        success = (status_code == 0)
        if not success:
            raise RuntimeError(return_values[0])
        return return_values

    def moveat(self, left, right):
        self.send_command("V[%d,%d,0]" % (left, right))

    def move(self, duration, steps_left, steps_right):
        #sends a message automatically to the Arduino, including the time of
        #the move and the number of steps we are taking
        dt = int(1000.0 * duration) #we defined duration as 1/camera_framerate ? todo
        self.send_command("M[%d,%d,%d,0]" % (dt, steps_left, steps_right))

    def move_gripper(self, duration, steps_grip_x, steps_grip_y, steps_lift):
    	self.send_command("M[%d,%d,%d,%d]" % (duration, steps_grip_x, steps_grip_y, steps_lift))

    def home_gripper_horizontally(self):
        self.send_command("h[1,-1,-1]")
        self.send_command("H")
        time.sleep(2)
        
class Gripper():

    kStepsOpen = 900
    kStepsClose = 1100
    kStepsUp = 200#10000
    kStepsDown = 200#7500
    kMillisecsOpenClose = 2500
    kMillisecsUpDown = 2000#15000
    
    def __init__(self, rover, controller):
        self.rover = rover
        self.controller = controller
        self.move_to_default_position()
    
    def move_to_default_position(self):
        self.grip()
        #self.move(self.kMillisecsOpenClose, self.kStepsOpenClose, 0, 0) todo
        self.move(self.kMillisecsOpenClose, self.kStepsOpen, self.kStepsOpen, 0)
    
    def grip(self):
        #self.controller.home_gripper_horizontally()     #je sais pas pq ca bloque
        self.move(self.kMillisecsOpenClose, -self.kStepsClose, -self.kStepsClose, 0)

    def lift(self):
        self.move(self.kMillisecsUpDown, 0, 0, self.kStepsUp)

    def lift_without_waiting(self):
        self.move_without_waiting(self.kMillisecsUpDown, 0, 0, self.kStepsUp)

    def release(self):
        #self.controller.move(self.kMillisecsOpenClose, self.kStepsOpenClose, 0, 0) todo
        self.move(self.kMillisecsOpenClose, self.kStepsOpen, self.kStepsOpen, 0)
        
    def put_down(self):
        self.move(self.kMillisecsUpDown, 0, 0, -self.kStepsDown)

    def move(self, duration, steps_grip_x, steps_grip_y, steps_move):
        self.controller.move_gripper(duration, steps_grip_x, steps_grip_y, steps_move)
        time.sleep(duration/1000)

    def move_without_waiting(self, duration, steps_grip_x, steps_grip_y, steps_move):
        self.controller.move_gripper(duration, steps_grip_x, steps_grip_y, steps_move)

class Navigation():
    def __init__(self, rover, controller):
        self.rover = rover
        self.controller = controller
        self.speed_left = 0
        self.speed_right = 0

    def forward(self, velocity, differential):
        self.speed_left = velocity * (1 - differential)
        self.speed_right = velocity * (1 + differential)
        self.move(self.speed_left, self.speed_right)

    def move(self, left, right):
        self.controller.moveat(left, right)



class ISegmentation(ABC):
    @abstractmethod
    def compute_mask(self, image):
        pass
    
class SVM(ISegmentation):
    def __init__(self, coefficients_file, intercept_file):
        self.w = np.loadtxt(coefficients_file, dtype = np.float)
        self.intercept = np.loadtxt(intercept_file, dtype = np.float)
        
    def compute_mask(self, image):
        val = (self.w[0] * image[:,:,0]
               + self.w[1] * image[:,:,1]
               + self.w[2] * image[:,:,2]
               + self.intercept)
        mask = (val > 0) * 255
        return mask

class HSV(ISegmentation):
    def __init__(self, hsvmin, hsvmax):
        self.hsvmin = hsvmin
        self.hsvmax = hsvmax

    def compute_mask(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = 255 - cv2.inRange(hsv_image, self.hsvmin, self.hsvmax)
        return mask
    
class IFilter(ABC):
    @abstractmethod
    def filter(self, error):
        pass


class FirstOrderLowPassFilter():
    def __init__(self, cutoff_frequency):
        self.cutoff_frequency = cutoff_frequency
        #todo : définir le tableau d'erreurs, méthode filter...


#We use 57STH56 NEMA 23 Bipolar Precision Gearless Stepper

##Define the RoDeP's class, with its methods and states.
#
#The RoDeP is defined as a state machine, using the StateMachine package.
class RoDeP(StateMachine):

    ##Initial state of the robot. It searches through the list of plants in the
    #server, then looks for it in the environment.  If the list of plant is
    #empty, the robot stops (goes to last state).
    #@see plant_found.
    #@warning An error occurs when the plant is not found.
    #(out of bonds, or mispelled).
    seeking=State('Seeking a plant', initial=True)

    ##Transition state of the robot. After he found the right plant,
    #it starts to lift it.
    taking=State('Taking up a plant')

    ##After taking up a plant, the robot goes to the scanner.
    going_to_scanner=State('Going to scanner')

    ##When the robot arrives at the scanner, it waits for the scan to be completed.
    #Entering this state, the robot emits a signal to the server,
    #notifying the scanner that a scan can start, and the user that a scan
    #is starting.
    scanning=State('Waiting for scanning completion')

    ##After scan completion, the robot will receive a notification from the
    #server, and will take the plant to its place in the storage room.
    going_to_storage=State('Going to storage')

    ##When the robot has arrived to the correct place to put the plant,
    #it starts taking it off. After that, it returns to seeking state.
    depositing=State('Depositing a plant')

    ##When the list of plants to scan is empty, the robot stops, notifying
    #the server.
    completing=State('Mission complete')

    plant_found = seeking.to(taking)
    plant_taken = taking.to(going_to_scanner)
    plant_at_scanner = going_to_scanner.to(scanning)
    plant_scanned = scanning.to(going_to_storage)
    plant_to_storage = going_to_storage.to(depositing)
    plant_deposited = depositing.to(seeking)
    mission_complete = seeking.to(completing)

    #On manipule la machine à état dans la Rasberry, on envoie les commandes
    #sur les moteurs à l'Arduino (pour profiter de l'analogique)

    kTowards = 1
    kBackwards = -1
    kScannerTag = 0

    ##RoDeP startup
    #Gets data from the server and readies up.
    def __init__(self, number_of_plants_to_be_scanned):
        print("Initializing...")

        # Main components
        self.rover = Rover(rover_width_meters,
                           rover_length_meters,
                           rover_wheel_diameter,
                           rover_motor_steps,
                           rover_encoder_steps)

        print("Connecting to the controllers...")
        #Navigation controller
        self.navigationController = SerialMotorController("/dev/ttyACM0")
        self.navigationController.send_command("E[1]")
        self.navigation = Navigation(self.rover, self.navigationController)

        #Gripper controller
        self.gripperController = SerialMotorController("/dev/ttyACM1")
        self.gripperController.send_command("E[1]")
        self.gripper = Gripper(self.rover, self.gripperController)

        print("Initializing camera...")
        self.camera = make_camera()
        self.pid = PID(Kp, 0.0, 0.0, setpoint=0)
        self.pid.sample_time = 1.0 / 50.0  # 50 FPS ??? TODO

        #apriltag mask
        self.at_detector = Detector(families = 'tag36h11', nthreads = 1, 
            quad_decimate = 1.0, quad_sigma = 0.0, refine_edges = 1, 
            decode_sharpening = 0.25, debug = 0)

        #detectors
        #SVM
        #self.blue_line_detector = SVM("coef_blue_line", "intercept_blue_line")
        #self.green_line_detector = SVM("coef_green_line", "intercept_green_line")

        #HSV
        self.apriltag_detector = HSV((140, 110, 0), (180, 200, 255))
        self.blue_line_detector = HSV((110, 170, 0), (130, 255, 255))
        #self.green_line_detector = HSV((80, 100, 0), (110, 170, 255))
        self.green_line_detector = HSV((80, 60, 0), (110, 170, 255))

        self.direction = self.kTowards

        self.plant_count = 1
        self.plant_count_limit = int(number_of_plants_to_be_scanned)

        #self.current_line_detector = self.blue_line_detector
        self.current_line_detector = self.green_line_detector;
        self.output_directory = "rodep-output"
        self.debug_counter = 0
        self.debug_start_time = 0
        self.debug_store_images = 1
        self.velocity = kDefaultVelocity
        self.error = 0
        self.correction = 0

        StateMachine.__init__(self)
        
    def make_output_directory(self):
        if not os.path.exists(self.output_directory):
            os.mkdir(self.output_directory)

    def grab_image(self):   
        self.image = self.camera.grab()

    def grab_hsv_image(self):
        self.image = self.camera.grab_hsv()

    def update_direction(self):
        self.make_control_image()
        self.detect_line()
        self.estimate_error()
        self.assert_line_is_visible()
        self.compute_correction()
        self.adjust_direction()

    def detect_line(self): 
        self.line_mask = self.current_line_detector.compute_mask(self.control_image)

    def detect_apriltag(self, tag_to_detect):
        apriltag_mask = self.apriltag_detector.compute_mask(self.image)
        tags = self.at_detector.detect(apriltag_mask, estimate_tag_pose = False,
            camera_params = None, tag_size = None)
        #todo : pour le moment, renvoie un booléen; à terme, renvoyer l'id du tag ?
        if(len(tags) == 1):
            if(tags[0].tag_id == tag_to_detect):
                return True
            else:
                return False
        else:
            return False

        
    def estimate_error(self):
        _, width = self.line_mask.shape 
        idx = np.where(self.line_mask == 0)
        mean = np.mean(idx[1])
        self.error = mean - width / 2

    def make_control_image(self): 
        height, width, _ = self.image.shape
        # 1/2 width
        x, y, w, h = int(width/4), int(height/4), int(width/2), 8
        # 3/4 width
        #x, y, w, h = int(width/8), int(height/4), int(3*width/4), 8
        # Full width
        #x, y, w, h = 0, int(height/4), width, 8
        self.control_image = self.image[y:y+h, x:x+w, :]
        
    def assert_line_is_visible(self): 
        height, width = self.line_mask.shape
        if (math.isnan(self.error)):
            self.stop("Lost the line")
        elif (self.error <= -width/2 or self.error >= width/2):
            self.stop("Lost the line")

    def compute_correction(self): 
        self.correction = self.pid(self.error)

    def adjust_direction(self):
        self.navigation.forward(self.direction * self.velocity, self.correction) 
        
    
    # Infinite loop that sets the rover in motion according to the flags.
    # color_flag : filters colors on camera to follow the corresponding color ("R", "G", "B")
    # direction_flag : direction which the robot follows ("towards", "backwards")
    # stop_tag_flag : determines which tag the robot stops at.
    def exploration_loop(self, color_flag, direction_flag, stop_tag_flag): 
        self.select_line_detector(color_flag)
        self.select_direction(direction_flag)
        self.prepare_for_debugging_output()
        while(True):
            self.grab_image()
            self.update_direction() 
            self.store_debugging_data()
            is_tag = self.detect_apriltag(stop_tag_flag)
            if(is_tag):
                self.navigation.move(0,0)
                break
    
    def half_turn(self):
        self.navigationController.move(1, 0, 0)
        time.sleep(1)
        self.navigationController.move(10, -3200, 3200)
        time.sleep(10)

    def taking_steps_back(self, distance_in_cm):
        #moving backwards a bit
        wheel_circumference_in_cm = self.rover.wheel_circumference * 100
        number_of_steps = 3600 * distance_in_cm / wheel_circumference_in_cm
        self.navigationController.move(2, -number_of_steps, -number_of_steps)
        time.sleep(2)


    def select_line_detector(self, color_flag):
        if color_flag == 'b' or color_flag == 'B':
            self.current_line_detector = self.blue_line_detector
        elif color_flag == 'g' or color_flag == 'G':
            self.current_line_detector = self.green_line_detector
        else:
            raise RuntimeError(f"Unknown line selector: {color_flag}")

    def select_direction(self, direction_flag):
        if direction_flag == "towards":
            self.velocity = kDefaultVelocity
            self.direction = self.kTowards
        elif direction_flag == "backwards":
            self.velocity = kBackwardsVelocity
            self.direction = self.kBackwards
        else:
            raise RuntimeError(f"Unknown direction_flag : {direction_flag}")


    def prepare_for_debugging_output(self): 
        self.debug_counter = 0
        self.debug_start_time = time.time()
        self.make_output_directory()
        with open(self.log_path(), "w") as logfile:
            logfile.write(f'# velocity={self.velocity}\n')
            logfile.write(f'# Kp={self.pid.Kp} Ki={self.pid.Ki} Kd={self.pid.Kd}\n')
            logfile.write(f'# index timestamp error correction:\n')

    def store_debugging_data(self):
        timestamp = time.time() - self.debug_start_time
        with open(self.log_path(), "a") as logfile:
            logfile.write(f'{self.debug_counter} {timestamp} {self.error:.2f} {self.correction:.2f}\n')
        if (self.must_store_debugging_images()):
            self.store_debugging_images()
        self.debug_counter += 1

    def must_store_debugging_images(self):
        return (self.debug_store_images > 0
                and self.debug_counter % self.debug_store_images == 0)

    def store_debugging_images(self):
        cv2.imwrite(self.image_path(), self.image) 
        cv2.imwrite(self.mask_path(), self.line_mask)
        height, width = self.line_mask.shape 
        cv2.circle(self.line_mask,
                   (int(width/2 + self.error), int(height / 2)),
                   6, 200, 4)
        cv2.imwrite(self.control_path(), self.control_image)
    
    def image_path(self):
        return self.output_path("image")

    def mask_path(self):
        return self.output_path("mask")

    def control_path(self):
        return self.output_path("control")
            
    def output_path(self, name):
        return "%s/%05d-%s.jpg" % (self.output_directory,
                                   self.debug_counter,
                                   name) 

    def log_path(self):
        return f"{self.output_directory}/log.txt" 
            

    ##takes a plant
    def on_plant_found(self): 
        print("Taking up plant...")
        self.take_plant()
        self.on_plant_taken()


    def on_plant_taken(self):
        self.send_to_server("Plant taken; going to scanner...")
        #moving backwards a bit
        self.taking_steps_back(12) 
        self.half_turn()
        #going back (green line to scanner)
        self.exploration_loop('g', "towards", self.kScannerTag) #could be backwards if no half_turn
        self.on_plant_at_scanner()


    def on_plant_at_scanner(self):
        self.send_to_server("Arrived at scanner; sending a scanning request...") #HTTP function to send a request to the scanner via the server
        time.sleep(1)
        #lift for proper scan
        self.gripper.lift()
        self.gripper.lift()
        self.send_to_server("Request sent successfully; waiting for scan completion...")
        self.send_to_server("Scan complete; going to storage...")
        self.on_plant_scanned()

    def on_plant_scanned(self):
        #going to de-storage (blue line for scanner)
        self.half_turn()
        self.exploration_loop('b', "towards", self.plant_count)
        self.send_to_server("Storage space found; depositing the plant...")
        self.deposit_plant()
        time.sleep(3)
        self.send_to_server("Plant deposited.")
        self.plant_count += 1
        #going back to starting point
        self.taking_steps_back(12) 
        self.half_turn()
        #going back (blue line to scanner)
        self.exploration_loop('b', "towards", self.kScannerTag) #could be backwards if no half_turn
        self.half_turn()
        self.on_plant_deposited()

    ##seeks a plant for the RoDeP
    def on_plant_deposited(self):
        if(self.plant_count > self.plant_count_limit):
            self.on_mission_complete()
        #get_plants_list()
        #print("Browsing the plants list...")
        self.send_to_server("Seeking a plant...")
         #going to storage looking for a plant
        print("Going to storage looking for a plant...")
        self.exploration_loop('g', "towards", self.plant_count) 
        self.send_to_server("Plant found !")
        self.on_plant_found() #todo : il faut ce genre de transis à chaque état

    def on_mission_complete(self):
        #for the moment, stops on place; improvement : a go to base method
        self.gripper.grip()
        self.stop("All plants have been scanned")

    #Internal functions

    def send_to_server(self, message):
        print("Sending a message to the server...")
        print(message)

    def stop(self, message):        
        self.navigationController.send_command("V[0,0,0]")
        if (self.is_seeking):
            self.gripper.grip()
        elif (self.is_going_to_scanner):
            self.gripper.put_down()
        elif (self.is_going_to_storage):
            self.gripper.put_down()
            self.gripper.put_down()
            self.gripper.put_down()

        raise Exception(message)
        
    #checks if we still follow the line; stop the rover if it is out
    def error_stop(self, error, width, height):
        if (math.isnan(error)):
            self.stop("Lost the line")
        elif (error <= -width/2 or error >= width/2):
            self.stop("Lost the line")


    def sending_plant_id(self): #for the moment, only the scanner gets the id
        time.sleep(1)
        plant_id = math.floor(random.random() * 100 + 1) #id of the plant to scan
        print("Sending plant id to server...")
        self.send_to_server("Found plant #"+str(plant_id))

    def take_plant(self):
        self.send_to_server("Starting taking routine, taking place correctly...")
        self.gripper.grip()
        self.gripper.lift()

    def deposit_plant(self):
        self.send_to_server("Starting depositing routine, taking place correctly...")
        print("Putting the plant down...")
        self.gripper.put_down()
        self.gripper.put_down()
        self.gripper.put_down()
        print("Releasing the plant...")
        self.gripper.release()

        print("Plant deposited !")

    def going_to_scanner(self):
        #image treatment; depending on result, go to some way
        #always tries to go to the scanner (difference with exploration_loop)
        print("Going to scanner...")

GPIO.setmode(GPIO.BCM)
GPIO.setup(4, GPIO.OUT)

GPIO.output(4, GPIO.HIGH)

if len(sys.argv) == 1:
    raise Exception("Error : please enter a number of plants to scan when calling the script. Example : python3 rover.py 4")

try:
    rodep=RoDeP(sys.argv[1]);

    rodep.on_plant_deposited()

except KeyboardInterrupt:
    rodep.stop("User interruption.")
    rodep.navigationController.send_command("E[0]")
    rodep.gripperController.send_command("E[0]")
