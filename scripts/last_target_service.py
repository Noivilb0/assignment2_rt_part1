#!/usr/bin/env python
import rospy
from assignment_2_2024.msg import PlanningActionGoal
from assignment_2_2024.srv import Last_target, Last_targetResponse

# Global variables to store the last target coordinates
last_target_position = {'x': None, 'y': None}

def goal_callback(msg):
    """
    Callback function that updates the last target coordinates from the incoming goal message.
    
    :param msg: The incoming PlanningActionGoal message that contains the target position.
    """
    global last_target_position
    last_target_position['x'] = msg.goal.target_pose.pose.position.x
    last_target_position['y'] = msg.goal.target_pose.pose.position.y
    rospy.loginfo(f"Updated last target coordinates: x={last_target_position['x']}, y={last_target_position['y']}")


def service_callback(request):
    """
    Callback function that handles service requests and returns the last target coordinates.
    
    :param request: The incoming service request, which does not require any parameters.
    :return: A Last_targetResponse containing the last target coordinates.
    """
    if last_target_position['x'] is None or last_target_position['y'] is None:
        rospy.logwarn("No target set yet.")
        return Last_targetResponse(0.0, 0.0)  # Returning 0.0, 0.0 if no target has been set
    rospy.loginfo(f"Returning last target coordinates: x={last_target_position['x']}, y={last_target_position['y']}")
    return Last_targetResponse(last_target_position['x'], last_target_position['y'])


def initialize_node():
    """
    Initializes the ROS node, subscribes to topics, and advertises services.
    """
    rospy.init_node("last_target_service_node")
    
    # Create the service server
    rospy.Service("last_target", Last_target, service_callback)
    
    # Subscribe to the goal topic to track the last target coordinates
    rospy.Subscriber("/reaching_goal/goal", PlanningActionGoal, goal_callback)

    rospy.loginfo("Last target service is ready.")
    
    rospy.spin()  # Keep the node running


if __name__ == "__main__":
    initialize_node()

