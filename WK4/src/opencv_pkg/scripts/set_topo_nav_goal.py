#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

from time import sleep
import rospy
import actionlib
from new_main import Tracker

from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

if __name__ == '__main__':
    rospy.init_node('topological_navigation_client')
    client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
    client.wait_for_server()
    tracker = Tracker()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint0"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint1"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    sleep(5)
    tracker.subs()
    # tracker.__init__()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint2"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    # sleep(10)
    # tracker.subs()
    # tracker.__init__()

    # send first goal
    goal = GotoNodeGoal()
    goal.target = "WayPoint3"
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    # sleep(10)
    # tracker.subs()
    # tracker.__init__()

    # send second goal
    goal.target = "WayPoint4"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
    # sleep(10)
    # tracker.subs()
    # tracker.__init__()

    goal.target = "WayPoint0"
    # Fill in the goal here
    client.send_goal(goal)
    status = client.wait_for_result() # wait until the action is complete
    result = client.get_result()
    rospy.loginfo("status is %s", status)
    rospy.loginfo("result is %s", result)
