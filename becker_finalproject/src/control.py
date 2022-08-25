#!/usr/bin/env python
"""
Main Driver program for the project.
Author: Kyle Becker
"""
import sys
import rospy
import actionlib
import process_image
from process_image import *
from projectserver.srv import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Constants for querying service
Color = [1, 2, 2, 0]
X =  [-2.3, 2.0, 0.0, 0.0]
Y = [2.5, 7.0, -3.0, 0.0]
i = 0


def Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony):
    """
    Gets the location of the next waypoint from the \getwaypoints server
    """
    rospy.wait_for_service('Final_ints')
    try:
        Final_ints = rospy.ServiceProxy('Final_ints', getwaypoint)
        
        resp1 = Final_ints(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
        return resp1
        
    except rospy.ServiceException as e:
        print("Service call failed:", e)


def movebase_client(x, y):
    """
    Handles the movement of the robot
    """
    try:
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = 1.5 if not y == -3.0 else -1.5
        goal.target_pose.pose.orientation.w = 1 if not y == -3.0 else 1
        
        print(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

def get_symbol_info():
    """
    Dummy function for getting waypoint locations and colors.
    """
    global i
    global X
    global Y
    global Color
    
    (color, x, y) = (Color[i], X[i], Y[i])
    i = (i+1)%4
    return (color, x, y)
    
          
def run():
    """
    Driver function for the program
    """
    print("="*16, "Run Called", "="*16)
    firstwp = True
    symboldetected = False
    symbolcolor = 1
    symbolpositionx = 0
    symbolpositiony = 0
    
    obj_finder = ProcessImage()
    # get initial points
    resp = Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
    goalx = resp.waypointx
    goaly = resp.waypointy
    firstwp = False
    result = False
    
    try:
    
        rospy.init_node('movebase_client')
            
        while True:
            # Move to goal
            result = movebase_client(goalx, goaly)
            
            obj_finder.color_to_find = symbolcolor
            
            if result:
                
                # Exit if excecution is done
                if (goalx, goaly) == (0,0):
                    print("Navigation over.")
                    return
                    
                print("+"*8, "Color to find:", symbolcolor, "+"*8)
                obj_finder.process_images = True
                
                # Check to make sure symbol detection is correct
                if obj_finder.color_found == symbolcolor:
                    symboldetected = True        
                else:
                    symboldetected = False
                print("Object found:", symboldetected)
                
                # Get next waypoint if symbol is detected
                if symboldetected:
                
                    obj_finder.process_images = False
                    obj_finder.color_found = None
                    (symbolcolor, symbolpositionx, symbolpositiony) = get_symbol_info()
                    resp = Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
                    goalx = resp.waypointx if resp is not None else 0
                    goaly = resp.waypointy if resp is not None else 0
                
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation over.")

           
if __name__ == '__main__':
    run()
