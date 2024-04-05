#!/usr/bin/env python

# For reference. Color cube picking demo by Ryan. (Deprecated)

from serial.tools import list_ports

from pydobot import Dobot
import numpy as np
import cv2
import time

###########################################################################
#                                VARIABLES                                #
###########################################################################

# Define the number of inner corners of the checkerboard pattern in the horizontal and vertical directions
pattern_size = (6, 9)
camera_model = 2 

# Define the reference points and the point to be converted
point = np.array([2, 5], dtype=np.float32)
src_points_coor = [[0, 0], [5, 0], [0, 8]]
src_points = np.array(src_points_coor, dtype=np.float32)
dst_points_cam = np.array([[0, 0], [0, 0], [0, 0]], dtype=np.float32)
dst_points_arm = np.array([[0, 0], [0, 0], [0, 0]], dtype=np.float32)
dst_points_box = np.array([[216.781982421875, -131.3333282470703],[98.82752990722656, -237.96827697753906],[-43.38584518432617, -252.02626037597656]], dtype=np.float32)
value_list = [0, 0, 0, 0, 0, 0]
standard_z = -65
std_drop_height = 60
instruction = False

color_bound =  [[  0,  50,  50], # RED_LOWER
                [ 10, 255, 255], # RED_UPPER
                [105,  50,  50], # BLUE_LOWER
                [115, 255, 255], # BLUE_UPPER
                [ 36,  50,  50], # GREEN_LOWER
                [ 90, 255, 255]] # GREEN_UPPER
                # [ 15,  50,  50], # YELLOW_UPPER
                # [ 30, 255, 255]] # YELLOW_UPPER

###########################################################################

available_ports = list_ports.comports()
print(f'available ports: {[x.device for x in available_ports]}')
port = available_ports[0].device

device = pydobot.Dobot(port=port)

def draw_coordinates(img, corners):
    colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
    for i, (color, world_coord) in enumerate(zip(colors[:3], src_points_coor)):
        corner_index = world_coord[1] * pattern_size[0] + world_coord[0]
        cv2.circle(img, tuple(corners[corner_index][0].astype(int)), 5, color, -1)
        cv2.putText(img, f"World: {world_coord} | Pixel: {tuple(corners[corner_index][0].astype(int))}", (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
        dst_points_cam[i] = tuple(corners[corner_index][0].astype(int))
        storing_calibration_data(dst_points_cam[i][0], dst_points_cam[i][1], i, "calibration_coord_cam.txt")
        print("World:", world_coord, " | Pixel:", tuple(corners[corner_index][0].astype(int)), " | Testing:", dst_points_cam[i])

def convert_point_using_affine(point, src_points, dst_points):
    # Calculate the Affine Transformation matrix
    affine_matrix = cv2.getAffineTransform(src_points, dst_points)

    # Convert the point using the transformation matrix
    point_homogeneous = np.array([point[0], point[1], 1])
    point_transformed = np.dot(affine_matrix, point_homogeneous)

    return point_transformed[:2]

def storing_calibration_data(x, y, num_point, file_name):
    value_list[num_point*2 + 0] = x
    value_list[num_point*2 + 1] = y
    print(value_list)
    # Open the file for writing
    with open(file_name, "w") as f:
        # Write the updated values to the file, separated by commas
        f.write(",".join(str(value_list) for value_list in value_list))

def robotic_arm_calibration(letter):
    file_name = "calibration_coord_arm.txt"
    # Note: match statement supports only python 3.10. 
    match letter:
        case "a":
            num_point = 1
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

        case "b":
            num_point = 0
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

        case "d":
            num_point = 2
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

def RBG_box_coordinate(letter):
    file_name = "box_coordinate_RBG.txt"
    match letter:
        case "R":
            num_point = 0
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

        case "G":
            num_point = 1
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

        case "B":
            num_point = 2
            current_coor = device.get_pose().position
            storing_calibration_data(current_coor[0], current_coor[1], num_point, file_name)

def precision_testing():
    point_transformed_cam = convert_point_using_affine(point, src_points, dst_points_cam)
    point_transformed_arm = convert_point_using_affine(point_transformed_cam, dst_points_cam, dst_points_arm)
    print("Arm:", point_transformed_arm)
    device.jump_to(point_transformed_arm[0],point_transformed_arm[1], standard_z, 0)
    time.sleep(3)


def default_pos():
    device.jump_to (251, -67, 65, -51)
    time.sleep(0.5)


def read_calibration_from_file(file_name, type_of_list):
    # Open the file for reading
    with open(file_name, "r") as f:
        # Read the file contents
        contents = f.read()

    # Split the contents into a list of strings, each containing one value
    values = contents.split(",")

    # Convert each string to a float and store them in a list
    value = [float(value) for value in values]
    for i in range (3):
        for j in range (2):
            if type_of_list == "arm":
                dst_points_arm[i][j] = value[2*i+j]
            if type_of_list == "cam":
                dst_points_cam[i][j] = value[2*i+j]
            if type_of_list == "box":
                dst_points_box[i][j] = value[2*i+j]

def pick_and_drop_block(tar_x, tar_y, box_color):
    device.move_to(tar_x, tar_y, 20, 0)
    time.sleep(0.5)
    device.move_to(tar_x, tar_y, -45, 0)
    time.sleep(0.5)
    device.suck(enable=True)
    time.sleep(0.5)
    device.move_to(tar_x, tar_y, 20, 0)
    time.sleep(0.5)
    read_calibration_from_file("box_coordinate_RBG.txt", "box")
    match box_color:    
        case "RED":
            device.move_to(dst_points_box[0][0], dst_points_box[0][1], std_drop_height, 0)
            time.sleep(0.5)
            device.suck(enable=False)
            time.sleep(0.5)

        case "BLUE":
            device.move_to(dst_points_box[1][0], dst_points_box[1][1], std_drop_height, 0)
            time.sleep(0.5)
            device.suck(enable=False)
            time.sleep(0.5)

        case "GREEN":
            device.move_to(dst_points_box[2][0], dst_points_box[2][1], std_drop_height, 0)
            time.sleep(0.5)
            device.suck(enable=False)
            time.sleep(0.5)
            
###########################################################################
#                            OBJECT DECTATION                             #
###########################################################################
class ColorBlockLocator:
    def __init__(self):
        self.red_lower    = np.array(color_bound[0])
        self.red_upper    = np.array(color_bound[1])
        self.blue_lower   = np.array(color_bound[2])
        self.blue_upper   = np.array(color_bound[3])
        self.green_lower  = np.array(color_bound[4])
        self.green_upper  = np.array(color_bound[5])
        # self.yellow_lower = np.array(color_bound[6])
        # self.yellow_upper = np.array(color_bound[7])

    def locate_points(self, image):
        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Threshold the image to extract red, blue, and green regions
        red_mask = cv2.inRange(hsv, self.red_lower, self.red_upper)
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        # yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)

        # Find contours in the masks
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # yellow_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area and aspect ratio
        filtered_red_contours = self.filter_contours(red_contours)
        filtered_blue_contours = self.filter_contours(blue_contours)
        filtered_green_contours = self.filter_contours(green_contours)
        # filtered_yellow_contours = self.filter_contours(yellow_contours)

        # Sort contours from left to right
        filtered_red_contours = sorted(filtered_red_contours, key=lambda c: cv2.boundingRect(c)[0])
        filtered_blue_contours = sorted(filtered_blue_contours, key=lambda c: cv2.boundingRect(c)[0])
        filtered_green_contours = sorted(filtered_green_contours, key=lambda c: cv2.boundingRect(c)[0])
        # filtered_yellow_contours = sorted(filtered_yellow_contours, key=lambda c: cv2.boundingRect(c)[0])

        # Extract the cropped images and center points of each color block
        red_crops, red_center_points = self.extract_crops_and_center_points(image, filtered_red_contours)
        blue_crops, blue_center_points = self.extract_crops_and_center_points(image, filtered_blue_contours)
        green_crops, green_center_points = self.extract_crops_and_center_points(image, filtered_green_contours)
        # yellow_crops, yellow_center_points = self.extract_crops_and_center_points(image, filtered_yellow_contours)

        return red_crops, red_center_points, blue_crops, blue_center_points, green_crops, green_center_points

    def filter_contours(self, contours):
        filtered_contours = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            aspect_ratio = w / h
            if area > 100 and aspect_ratio < 1.5:
                filtered_contours.append(contour)
        return filtered_contours

    def extract_crops_and_center_points(self, img, contours):
        crops = []
        center_points = []
        for contour in contours:
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Crop the surface of the color block
            crop = img[y:y+h, x:x+w]
            crops.append(crop)

            # Calculate the center point of the color block
            center_x = x + w // 2
            center_y = y + h // 2
            center_points.append((center_x, center_y))

        return crops, center_points
###########################################################################

# Create empty lists to store the object points and image points
obj_points = []
img_points = []

# Generate the object points, which represent the coordinates of the corners in the real-world coordinate system
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)

# Initialize the camera
# Slot(0) is the webcam on Laptop, Slot(1) is the first external camera you connected
cap = cv2.VideoCapture(camera_model)
locator = ColorBlockLocator()

while True:
    device.clear_alarms()

     # Capture a frame from the camera
    ret, img = cap.read()

    if not ret:
        print("Failed to capture image from the camera. Make sure the camera is connected and working properly.")
        break

    cv2.imshow("Live Stream", img)
    
    if instruction == False:
        print("Please calibrate the coordinate of robotic arm and camera")
        print("by tapping '1', '2', '3', 'a', 'b', 'c', 'd'")
        instruction = True

    key = cv2.waitKey(1) & 0xFF

    if key == ord("1"):
        RBG_box_coordinate("R")
    
    if key == ord("2"):
        RBG_box_coordinate("G")

    if key == ord("3"):
        RBG_box_coordinate("B")

    if key == ord("a"):
        robotic_arm_calibration("a")
    
    if key == ord("b"):
        robotic_arm_calibration("b")

    if key == ord("d"):
        robotic_arm_calibration("d")

    if key == ord("q"):
        break
        
    if key == ord("c"):
        # Convert the captured image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the checkerboard corners in the image
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        # If the corners are found, add the object points and image points to the lists
        if ret == True:
            obj_points.append(objp)
            img_points.append(corners)
            draw_coordinates(img, corners)
            cv2.imshow("Captured Image", img)

        else:
            print("Failed to find checkerboard corners. Ensure the grid paper is properly placed and visible.")
        
        cv2.waitKey(0)
        cv2.destroyWindow("Captured Image")

        # Perform camera calibration using the object points and image points
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    
        # Print the camera matrix and distortion coefficients
        print("Camera Matrix:")
        print(mtx)
        print("\nDistortion Coefficients:")
        print(dist)

    if key == ord(" "):
        read_calibration_from_file("calibration_coord_arm.txt", "arm")
        read_calibration_from_file("calibration_coord_cam.txt", "cam")
        precision_testing()


        # device.jump_to(218.98680114746094, 160.3431854248047, 32.91130065917969, 0)
        default_pos()
        
        # print color coordinates
        while True:
            ret, frame = cap.read()
            image = cv2.resize(frame, (640, 480))

            red_crops, red_center_points, blue_crops, blue_center_points, green_crops, green_center_points = locator.locate_points(image)
            print(red_center_points, blue_center_points, green_center_points)

            print("RED: ")
            for c in red_center_points:
                cv2.circle(image, (int(c[0]), int(c[1])), 10, (  0,   0, 255), -1)
            
            print("BLUE: ")
            for c in blue_center_points:
                cv2.circle(image, (int(c[0]), int(c[1])), 10, (255, 255,   0), -1)
            
            print("GREEN: ")
            for c in green_center_points:
                cv2.circle(image, (int(c[0]), int(c[1])), 10, (  0, 255,   0), -1)

            cv2.imshow('Camera', image)
            
            # Break the loop and exit the program if the 'q' key is pressed
            if cv2.waitKey(500) & 0xFF == ord(' '):
                break
        
        for cor in red_center_points:
            red_center_points_arm = convert_point_using_affine(cor, dst_points_cam, dst_points_arm)
            pick_and_drop_block(red_center_points_arm[0], red_center_points_arm[1], "RED")
            print(cor, "RED:", red_center_points_arm)
        
        for cor in blue_center_points:
            blue_center_points_arm = convert_point_using_affine(cor, dst_points_cam, dst_points_arm)
            pick_and_drop_block(blue_center_points_arm[0], blue_center_points_arm[1], "BLUE")
            print(cor, "BLUE:", blue_center_points_arm)

        for cor in green_center_points:
            green_center_points_arm = convert_point_using_affine(cor, dst_points_cam, dst_points_arm)
            pick_and_drop_block(green_center_points_arm[0], green_center_points_arm[1], "GREEN")
            print(cor, "GREEN:", green_center_points_arm)

        # for cor in yellow_center_points:
        #     yellow_center_points_arm = convert_point_using_affine(cor, dst_points_cam, dst_points_arm)
        #     pick_and_drop_block(yellow_center_points_arm[0], yellow_center_points_arm[1], "YELLOW")
        #     print(cor, "YELLOW:", yellow_center_points_arm)

        device.suck(enable=False)
        # device.move_to(218.98680114746094, 160.3431854248047, 32.91130065917969, 0)
        default_pos()

        # Close screen and end operation
        cv2.waitKey(0)
        

        # Release the camera
        cap.release()

        # Destroy the window
        cv2.destroyAllWindows()
        