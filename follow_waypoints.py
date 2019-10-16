#!/usr/bin/env python
#this code is modified Danial Snider's original code
import threading
import rospy
import actionlib
import collections
from math import *
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty



waypoints = []
class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        global waypoints
	global x_val;
	global y_val;
	global theta 
        # Execute waypoints each in sequence
	
	x_val = waypoints[0].pose.pose.position.x - waypoints[1].pose.pose.position.x
	y_val = waypoints[0].pose.pose.position.y - waypoints[1].pose.pose.position.y
	#this case is x1>x2, y1>y2

	index = int(x_val / 0.16)

	rospy.loginfo('x_val is : %s, y_val is : %s, index is : %s' %(x_val,y_val,index))
	theta = 3.1415 / 2;
        if waypoints == []:
            rospy.loginfo('The waypoint queue has been reset.')
            # Otherwise publish next waypoint as goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.pose.position = waypoints[0].pose.pose.position
        goal.target_pose.pose.orientation = waypoints[0].pose.pose.orientation
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                (waypoints[0].pose.pose.position.x, waypoints[0].pose.pose.position.y))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
	goal.target_pose.pose.orientation.z = -0.70711
	goal.target_pose.pose.orientation.w = 0.70711
        self.client.send_goal(goal)
        self.client.wait_for_result()
#starting point, looking right 
	#goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - y_val
	#self.client.send_goal(goal)
        #self.client.wait_for_result()
#right move	
	#goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - x_val
	#self.client.send_goal(goal)
        #self.client.wait_for_result()
#down move
	for i in range(index):
		rospy.loginfo('entered *for* loop index is : %s',i)
		if(((i+2)%2)==0):
			rospy.loginfo("entered *right* loop")
			goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - y_val
			goal.target_pose.pose.orientation.z = 1
			goal.target_pose.pose.orientation.w = 0
			self.client.send_goal(goal)
			self.client.wait_for_result()
			goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - 0.16
			goal.target_pose.pose.orientation.w = 0.70711#left
			goal.target_pose.pose.orientation.z = 0.70711
			self.client.send_goal(goal)
			self.client.wait_for_result()
			#move to right and looking down to move down, move down and looking left to move left
		else:
			rospy.loginfo("entered *left* loop")			
			goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + y_val
			goal.target_pose.pose.orientation.z = -1
			goal.target_pose.pose.orientation.w = 0
			self.client.send_goal(goal)
			self.client.wait_for_result()
			goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - 0.16
			goal.target_pose.pose.orientation.z = -0.70711
			goal.target_pose.pose.orientation.w = 0.70711
			self.client.send_goal(goal)
			self.client.wait_for_result()		       
			#move to left and looking down to move down, move down and looking right to move right
	return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = 'map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    rospy.loginfo(poses)
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher('/waypoints', PoseArray, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        topic = "/clicked_point"
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(pose)
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
