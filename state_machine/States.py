#!/usr/bin/env python3

from enum import Enum 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class State(Enum):
    INITIAL_POS = "initial_pos"
    PLANNING = "planning"
    LANE_FOLLOWING = "lane_following"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    TURNING_R = "turning_r"
    TURNING_L = "turning_l"
    TURNING_U = "turning_u"


class StateMachine: 
    def __init__(self): 
        # Initialize the ROS node
        rospy.init_node('state_machine_node')
        
        # Initialize current state
        self.current_state = State.INITIAL_POS
        
         # Create publishers for behavior activation
        self.state_pub = rospy.Publisher('/current_state', String, queue_size=10)
        self.lane_following_enable = rospy.Publisher('/lane_following/enable', Bool, queue_size=10)
        self.obstacle_avoidance_enable = rospy.Publisher('/obstacle_avoidance/enable', Bool, queue_size=10)
        self.turning_enable = rospy.Publisher('/turning/enable', Bool, queue_size=10)
            
        # Create subscribers
        rospy.Subscriber('/event', String, self.event_callback)
        rospy.Subscriber('/obstacle_detection', String, self.obstacle_callback)
        rospy.Subscriber('/april_tag_detection', String, self.april_tag_callback)

        # Subscribe to completion signals
        rospy.Subscriber('/lane_following/completed', Bool, self.lane_following_completed_callback)
        rospy.Subscriber('/obstacle_avoidance/completed', Bool, self.obstacle_avoidance_completed_callback)
        rospy.Subscriber('/turning/completed', Bool, self.turning_completed_callback)
        
        
        #State transition dictionary
        self.state_transitions = {
            State.INITIAL_POS: self.initial_pos_state,
            State.PLANNING: self.planning_state,
            State.LANE_FOLLOWING: self.lane_following_state,
            State.OBSTACLE_AVOIDANCE: self.obstacle_avoidance_state,
            State.TURNING_R: self.turning_state,
            State.TURNING_L: self.turning_state,
            State.TURNING_U: self.turning_state
        }
        
        
    def publish_state(self):
        """Publish current state to ROS topic"""
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_pub.publish(state_msg)
        rospy.loginfo(f"Current state: {self.current_state.value}")

    def event_callback(self, msg):
        """Handle general events"""
        self.handle_transition(msg.data)

    def obstacle_callback(self, msg):
        """Handle obstacle detection events"""
        if msg.data == "obstacle_detected":
            self.handle_transition("obstacle_observed")

    def april_tag_callback(self, msg):
        """Handle AprilTag detection events"""
        if msg.data in ["AprilTag_right", "AprilTag_left", "AprilTag_u"]:
            self.handle_transition(msg.data)

    def lane_following_completed_callback(self, msg):
        """Handle completion of lane following behavior"""
        if msg.data:
            self.handle_transition("lane_following_completed")

    def obstacle_avoidance_completed_callback(self, msg):
        """Handle completion of obstacle avoidance behavior"""
        if msg.data:
            self.handle_transition("obstacle_avoidance_completed")

    def turning_completed_callback(self, msg):
        """Handle completion of turning behavior"""
        if msg.data:
            self.handle_transition("turning_completed")
            

    def handle_transition(self, event):
        """Handle state transitions based on events"""
        if event in ['goal_given', 'trajectory_planned', 'obstacle_observed', 
                    'AprilTag_right', 'AprilTag_left', 'AprilTag_u', 'goal_reached']:
            new_state = self.state_transitions[self.current_state](event)
            if new_state != self.current_state:
                self.current_state = new_state
                self.publish_state()
                self.execute_state_action()
                
    def disable_all_behaviors(self):
        """Disable all behavior nodes"""
        disable_msg = Bool()
        disable_msg.data = False
        self.lane_following_enable.publish(disable_msg)
        self.obstacle_avoidance_enable.publish(disable_msg)
        self.turning_enable.publish(disable_msg) 
                        
    def activate_state_behavior(self):
        """Activate behavior node for current state"""
        enable_msg = Bool()
        enable_msg.data = True
        
        if self.current_state == State.LANE_FOLLOWING:
            self.lane_following_enable.publish(enable_msg)
        elif self.current_state == State.OBSTACLE_AVOIDANCE:
            self.obstacle_avoidance_enable.publish(enable_msg)
        elif self.current_state in [State.TURNING_R, State.TURNING_L, State.TURNING_U]:
            self.turning_enable.publish(enable_msg)

    def initial_pos_state(self, event):
        if event == "goal_reached":
            return State.PLANNING
        return self.current_state

    def planning_state(self, event):
        if event == "goal_given":
            return State.LANE_FOLLOWING
        return self.current_state

    def lane_following_state(self, event):
        if event == "obstacle_observed":
            return State.OBSTACLE_AVOIDANCE
        elif event == "AprilTag_right":
            return State.TURNING_R
        elif event == "AprilTag_left":
            return State.TURNING_L
        elif event == "AprilTag_u":
            return State.TURNING_U
        return self.current_state

    def obstacle_avoidance_state(self, event):
        if event == "obstacle_avoidance_completed":
            return State.LANE_FOLLOWING
        return self.current_state

    def turning_state(self, event):
        if event == "turning_completed":
            return State.LANE_FOLLOWING
        return self.current_state

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_state()
            rate.sleep()
            
if __name__ == '__main__':
    try:
        state_machine = StateMachine()
        state_machine.run()
    except rospy.ROSInterruptException:
        pass
    