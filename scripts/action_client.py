#!/usr/bin/env python
import time
import rospy
import select
import actionlib
import sys
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import Custom_msg
from assignment_2_2024.msg import PlanningAction, PlanningGoal

def clbk_odom(msg):

    # Initialize a new message
    new_custom_msg = Custom_msg()
    
    # Retrieve the position and velocity from geometry_msgs and save the values
    new_custom_msg.x = msg.pose.pose.position.x            # x position coordinate
    new_custom_msg.y = msg.pose.pose.position.y            # y position coordinate
    new_custom_msg.vel_x = msg.twist.twist.linear.x        # linear velocity along x axis
    new_custom_msg.vel_z = msg.twist.twist.angular.z       # angular velocity around z axis
    
    # Publish new message on /robot_pos_vel topic 
    pub.publish(new_custom_msg)


def clbk_feedback(feedback):
    """
    Callback function that processes feedback from the client.
    
    :param feedback: Feedback from the target as "Target reached!" or "Target cancelled!".
    """
    if feedback.stat == "Target reached!":
        print(feedback)
        print("Target reached successfully!")
        print(f"Robot orientation (angular velocity around Z-axis): {feedback.vel_z} rad/s")
        print("Press 'Enter' to set a new goal\n")
    if feedback.stat == "Target cancelled!":
        print(feedback)


def action():
    """
    Action function that handles the goal coordinates from user input and sends the goal to the planner.
    While the robot is moving, the user can cancel the goal by pressing "c".
    """
    # Execution of client request to the server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    # Block the execution until communication with server is established
    client.wait_for_server()
    
    # While loop until the program finishes or is interrupted
    while not rospy.is_shutdown():
        time.sleep(0.5)
        # Get goal coordinates from user
        print("Set the goal coordinates!")
        try:
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            # No input validation now; accept any number as coordinates
            print(f"Goal coordinates set: (x={x}, y={y})")
        except ValueError:
            print("Invalid input. Please enter a valid number.")
            continue
        
        # Initialize an instance of PlanningGoal() to pass the goal coordinates
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Send the goal to the action server and set callbacks for:
        # done_cb = The action is done.
        # active_cb = The action becomes active.
        # feedback_cb = The action sends feedback.
        client.send_goal(goal, None, None, clbk_feedback)
        
        # Now the robot is reaching the goal. If we want to stop the robot we need
        # to cancel the goal by reading user input without blocking the execution.
        while not client.get_result():
            print("Robot is reaching the goal. Press 'c' to cancel the goal.")
            cancel = select.select([sys.stdin], [], [], 0.1)
            if cancel:
                user_input = sys.stdin.readline().strip()
                if user_input == 'c':
                    client.cancel_goal()
                    break


def main():
    """
    Main function where the ROS node is initialized and the publisher and subscriber are initialized.
    """
    global pub
    
    # Initialize the service node
    rospy.init_node('action_client')
    
    # Creating a ROS publisher to publish on /robot_pos_vel topic the position and velocity of Robot
    pub = rospy.Publisher('/robot_pos_vel', Custom_msg, queue_size=10)
    
    # Creating a ROS subscriber to listen to the /odom topic
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    # Run the action function
    action()


if __name__ == "__main__":
    main()


