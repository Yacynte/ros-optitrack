#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import math
import sys
from djitellopy import tello
import threading
import signal
import time
import keyboard  # Import keyboard module for handling key events
from std_msgs.msg import String
# Define ANSI escape codes for colors
GREEN = "\033[1;32m"
RESET = "\033[0m"


class TelloControl:
    class Euler:
        def __init__(self, roll, pitch, yaw):
            self.x = math.degrees(roll)
            self.y = math.degrees(pitch)
            self.z = math.degrees(yaw)
    def __init__(self):
        self.vrpn_topic = rospy.get_param('~vrpn_topic', '/vrpn_client_node/tello/pose')
        self.waypoints_topic = rospy.get_param('~waypoints_topic', '/waypoints')

        self.message = None  # Variable to store received message

        # Subscribe to the topic 'string_message'
        rospy.Subscriber('string_message', String, self.message_callback)
        
        self.pose_sub = rospy.Subscriber(self.vrpn_topic, PoseStamped, self.pose_callback)
        rospy.sleep(1)  # Sleep for a second to ensure the publisher is connected
        self.waypoints_sub = rospy.Subscriber(self.waypoints_topic, Float64MultiArray, self.waypoints_callback)

        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.rate = rospy.Rate(10)  # 10 Hz
        self.stop_requested = threading.Event()
        signal.signal(signal.SIGINT, self.signal_handler)
        input_thread = threading.Thread(target=self.listen_for_exit)
        input_thread.start()
        self.velocity = ()
        self.displacement = ()
        self.tolerance = 0.1
        self.angle_tolerance = 8
        self.angle_to_waypoint = 0
        self.constant_speed = rospy.get_param('~constant_speed', 20)  # Desired constant speed (cm/s)
        self.rotation_speed = rospy.get_param('~rotation_speed', 20)

        rospy.loginfo("Tello controller initialized")

        self.drone = tello.Tello()
        # Modify the ports (if necessary and if the library allows it)
        #self.drone.STATE_UDP_PORT = 8891
        #self.drone.VIDEO_UDP_PORT = 11112

        
        self.drone.connect()
    def message_callback(self, data):
        # Callback function to handle incoming messages
        rospy.loginfo("Received message: %s", data.data)
        self.message = data.data  # Store the received message in a variable

    def pose_callback(self, data):
        current_position = data.pose
        # Multiply each coordinate of the position by 2
        current_position.position.x *= 2
        current_position.position.y *= 2
        current_position.position.z *= 2
        self.current_pose = current_position

    def waypoints_callback(self, data):
        self.waypoints = []
        for i in range(0, len(data.data), 4):
            self.waypoints.append({
                'x': data.data[i],
                'y': data.data[i+1],
                'z': data.data[i+2],
                'az': data.data[i+3]
            })
        # rospy.loginfo("Received waypoints: %s", self.waypoints)
        #self.current_waypoint_index = 0  # Reset to start from the first waypoint
    
    def quaternion_to_euler(self):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        euler = tf.transformations.euler_from_quaternion([
            self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w
        ])
        self.euler = self.Euler(euler[0], euler[1], euler[2])
        #print("euler", euler, self.euler.z)
        

        


    def at_waypoint(self, waypoint):
        if self.current_pose is None:
            return float('inf')
        self.quaternion_to_euler()
        dx = waypoint['x'] - self.current_pose.position.x #+ self.initial_pose.x
        dy = waypoint['y'] - self.current_pose.position.y #+ self.initial_pose.y
        dz = waypoint['z'] - self.current_pose.position.z #+ self.initial_pose.z
        daz = waypoint['az'] - self.euler.z #+ self.initial_pose.z
        self.distance = math.sqrt(dx**2 + dy**2 + dz**2)
        self.angle = daz
        self.displacement = (dx, dy, dz, self.angle_to_waypoint)
        return self.distance
        if self.distance < self.tolerance and self.angle < self.angle_tolerance:
            return True
        return  False
    

    def listen_for_exit(self):
        while not self.stop_requested.is_set():
            user_input = input()
            if user_input.strip().lower() == 'q':
                self.stop_requested.set()
                rospy.loginfo("Exit requested by user.")

    def signal_handler(self, sig, frame):
        self.stop_requested.set()
        rospy.loginfo("Exit requested by signal.")
        sys.exit(0)

    def control_loop(self):
        i0 = 0
        paused = False
        #self.initial_pose = self.current_pose.position
        while not rospy.is_shutdown():
            # Handle keyboard events
            if self.message == "pause":
                paused = not paused  # Toggle pause state
                if paused:
                    rospy.loginfo("Paused")
                else:
                    rospy.loginfo("Resumed")

            if self.message == "exit" :
                rospy.loginfo("Exiting control loop...")
                rospy.signal_shutdown("Task complete")
                break
                

            if not paused:
                #print("Here", self.current_pose, self.waypoints)
                if self.current_pose is not None and self.waypoints:
                    #print("Here1")
                    waypoint = self.waypoints[self.current_waypoint_index]
                    #waypoint['x'] = waypoint['x'] + self.initial_pose.x
                    #waypoint['y'] = waypoint['y'] + self.initial_pose.y
                    #waypoint['z'] = waypoint['z'] + self.initial_pose.z
                    #distance,  = self.distance_to_waypoint(waypoint)
                    #print("dist: ", distance)
                    #if self.at_waypoint(waypoint):
                    dist = self.at_waypoint(waypoint)
                    if dist < self.tolerance:
                        self.drone.send_rc_control(0, 0, 0, 0)
                        destination_angle = waypoint['az'] 
                        self.quaternion_to_euler()
                        j = 0
                        #target_angle = int(destination_angle - self.euler.z)
                        #self.rotate(target_angle, waypoint)
                        """
                        while abs(waypoint['az'] - self.euler.z) > self.angle_tolerance and self.message != "exit":
                            self.quaternion_to_euler()
                            self.drone.send_rc_control(0, 0, 0, self.rotation_speed)
                            
                            if j%250 == 0:
                                self.quaternion_to_euler()
                                position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, self.euler.z]
                                rospy.loginfo("Target waypoint %d: %s", self.current_waypoint_index+1, self.waypoints[self.current_waypoint_index])
                                rospy.loginfo("Drone's Position  %s", str(position))
                            j += 1
                            """
                        
                        self.quaternion_to_euler()
                        position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, self.euler.z]
                        rospy.loginfo("Target waypoint %d: %s", self.current_waypoint_index+1, self.waypoints[self.current_waypoint_index])
                        rospy.loginfo("Drone's Position  %s", str(position))
                        rospy.loginfo(f"{GREEN}Waypoint %d reached{RESET}", self.current_waypoint_index+1)
                        time.sleep(5)
                        #self.rate
                        if self.current_waypoint_index < len(self.waypoints) - 1:
                            self.current_waypoint_index += 1
                        else:
                            rospy.loginfo(f"{GREEN}All waypoints reached{RESET}")
                            break  # Exit the loop once all waypoints are reached
                    else: 
                        if i0%20 == 0:
                            self.quaternion_to_euler()
                            position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, self.euler.z]
                            rospy.loginfo("Target waypoint %d: %s", self.current_waypoint_index+1, self.waypoints[self.current_waypoint_index])
                            rospy.loginfo("Drone's Position  %s", str(position))
                        self.move_towards_waypoint(waypoint)
                        # self.move_towards_waypoint(waypoint)
                self.rate.sleep()
                i0 += 1

        if self.message != "exit":
            self.drone.flip_right()
        self.drone.land()
        rospy.on_shutdown(shutdown_hook)  # Register shutdown hook
        time.sleep(5)  # Adjust as needed
        self.drone.end()
        self.drone.__del__()

    def rotate(self, target_angle, waypoint):
        i2 = 0
        dx = waypoint['x'] - self.current_pose.position.x #+ self.initial_pose.x
        dy = waypoint['y'] - self.current_pose.position.y #+ self.initial_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        while abs(target_angle) > self.angle_tolerance and distance > self.tolerance:
            if self.message == "exit":
                rospy.signal_shutdown("Task complete")
                break
            #rospy.loginfo("Angle to waypoint %d: %d", self.current_waypoint_index, target_angle)
            self.drone.send_rc_control(0, 0, 0, self.rotation_speed)
            self.angle_between_current_point_and_waypoint = math.degrees(math.atan2(dy, dx))
            self.quaternion_to_euler()
            dx = waypoint['x'] - self.current_pose.position.x #+ self.initial_pose.x
            dy = waypoint['y'] - self.current_pose.position.y #+ self.initial_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            # print(self.angle_between_current_point_and_waypoint, self.euler.z, abs(target_angle))
            target_angle  = int(self.angle_between_current_point_and_waypoint - self.euler.z)
            if i2%250 == 0:
                self.quaternion_to_euler()
                position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, self.euler.z]
                rospy.loginfo("Target waypoint %d: %s", self.current_waypoint_index+1, self.waypoints[self.current_waypoint_index])
                rospy.loginfo("Drone's Position  %s", str(position))
            i2 += 1

    def move_towards_waypoint(self, waypoint):

        if self.current_pose is None:
            return
        self.quaternion_to_euler()
        dx = waypoint['x'] - self.current_pose.position.x #+ self.initial_pose.x
        dy = waypoint['y'] - self.current_pose.position.y #+ self.initial_pose.y
        dz = waypoint['z'] - self.current_pose.position.z #+ self.initial_pose.z
        daz = waypoint['az'] - self.euler.z
        # Normalize the direction vector
        distance = math.sqrt(dx**2 + dy**2)
        distance3 = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance == 0:
            return  # Already at the waypoint
        
        self.quaternion_to_euler()
        alpha = waypoint['az']
        self.angle_between_current_point_and_waypoint = math.degrees(math.atan2(dy, dx))
        self.angle_to_waypoint = int(self.angle_between_current_point_and_waypoint - self.euler.z)

        if distance > self.tolerance:
            self.rotate(self.angle_to_waypoint, waypoint)

        while abs(self.angle_to_waypoint) <= self.angle_tolerance and distance3 > self.tolerance:
            i3 = 0
            if self.message == "exit" :
                rospy.signal_shutdown("Task complete")
                break
            vz = int(self.constant_speed * (dz / abs(dz)) if abs(dz) > self.tolerance else 0)
            # rospy.loginfo("distance to waypoint %d: %s", self.current_waypoint_index, self.current_pose.position)
            if abs(dz) > self.tolerance and distance < self.tolerance:
                self.drone.send_rc_control(0, 0, vz, 0)
            else:
                self.drone.send_rc_control(0, self.constant_speed, vz, 0)
            self.angle_between_current_point_and_waypoint = math.degrees(math.atan2(dy, dx))
            self.quaternion_to_euler()
            self.angle_to_waypoint = self.angle_to_waypoint = int(self.angle_between_current_point_and_waypoint - self.euler.z)
            dx = waypoint['x'] - self.current_pose.position.x #+ self.initial_pose.x
            dy = waypoint['y'] - self.current_pose.position.y #+ self.initial_pose.y
            dz = waypoint['z'] - self.current_pose.position.z
            # Normalize the direction vector
            distance = math.sqrt(dx**2 + dy**2)
            distance3 = math.sqrt(dx**2 + dy**2 + dz**2)
            self.displacement = (dx, dy, dz, self.angle_to_waypoint)
            if i3%250 == 0:
                self.quaternion_to_euler()
                position = [self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z, self.euler.z]
                rospy.loginfo("Target waypoint %d: %s", self.current_waypoint_index+1, self.waypoints[self.current_waypoint_index])
                rospy.loginfo("Drone's Position  %s", str(position))
            i3 += 1

            #self.move_towards_waypoint(waypoint)

def shutdown_hook():
        rospy.loginfo("Shutting down node.")

def main():
    try:
        rospy.init_node('tello_controller', anonymous=True)
        controller = TelloControl()
        rospy.loginfo(f"Battery : {controller.drone.get_battery()}% ")
        time.sleep(10)
        controller.drone.takeoff()        
        # Delay to allow drone to takeoff
        time.sleep(5)  # Adjust as needed

        # Check if the drone is airborne
        if controller.drone.is_flying:
            print("Drone has successfully taken off!")
            controller.control_loop()
            rospy.loginfo(f"Battery : {controller.drone.get_battery()}% ")
            
        else:
            print("Failed to confirm takeoff.")
        

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting...")
        

if __name__ == '__main__':
    main()
    
