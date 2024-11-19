#!/usr/bin/env python
import time
import subprocess
import rospy

def run_sequential_nodes():
    # Initialize a ROS node (optional)
    rospy.init_node('sequential_launcher', anonymous=True)

    # Start the turtlesim node
    rospy.loginfo("Starting turtlesim node...")
    turtlesim_process = subprocess.Popen(['rosrun', 'turtlesim', 'turtlesim_node'])
    
    # Allow time for turtlesim to initialize
    rospy.loginfo("Waiting for turtlesim to initialize...")
    time.sleep(2)  # Adjust the delay if necessary

    # Call the /kill service to remove the default turtle
    rospy.loginfo("Removing the default turtle (turtle1)...")
    subprocess.call(['rosservice', 'call', '/kill', 'turtle1'])

    # Start the UI node
    rospy.loginfo("Starting the UI node...")
    ui_node_process = subprocess.Popen(['rosrun', 'assignment1_rt', 'UI_node'])

    # Wait for processes to complete (optional, you can adjust as needed)
    turtlesim_process.wait()
    ui_node_process.wait()

if __name__ == '__main__':
    try:
        run_sequential_nodes()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted before completion.")

