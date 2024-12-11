#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from duckietown_msgs.msg import BoolStamped, WheelsCmdStamped
import smach
import enum 


class State(Enum): 
    PlanTrajectory = 0 
    LaneFollowing = 1 
    ObstacleAvoidance = 2 
    Stop = 3
    TurnRight = 4
    TurnLeft = 5
    TurnU = 6


class PlanTrajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['plan_complete'])
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: PLAN_TRAJECTORY")
        return 'plan_trajectory'
        
    def start_planning(self, vehicle_name): 
        pub = rospy.Publisher(f'/{vehicle_name}/planner_node/0', BoolStamped, queue_size=10)
        
        msg = BoolStamped()
        msg.header = Header()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = ''
        msg.data = False 
        
        rospy.loginfo("Start lane following")
        pub.publish(msg)


class LaneFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_observed', 'apriltag_detected'])
        self.vehicle_name = 'pebbles'
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: LANE_FOLLOWING")
        self.start_lane_following(self.vehicle_name)
        if self.obstacle_detected():
            return 'obstacle_observed'
        elif self.apriltag_detected():
            return 'aprilTag_detected'

    def start_lane_following(self, vehicle_name):
        pub = rospy.Publisher(f'/{vehicle_name}/lane_following_controller_node/1', BoolStamped, queue_size=10)
        
        msg = BoolStamped()
        msg.header = Header()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = ''
        msg.data = False 
        
        rospy.loginfo("Start lane following")
        pub.publish(msg)
 
 
class ObstacleAvoidance(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['obstacle_overtaken', 'overtaking_obstacle'])
        self.vehicle_name = 'pebbles'
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: OBSTACLE_AVOIDANCE")
        return 'obstacle_overtaken' 
    
        
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['nothing_changed','turn_apriltag','goal_apriltag','new_goal_given'])
        self.vehicle_name = 'pebbles'
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: STOP")
        self.stop_lane_following(self.vehicle_name)
        # TODO: Implement detection logic
        return 'nothing_changed'
    
    def stop_lane_following(self, vehicle_name):
        pub = rospy.Publisher(f'/{vehicle_name}/joy_mapper_node/joystick_override', BoolStamped, queue_size=10)
        
        msg = BoolStamped()
        msg.header = Header()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time(0)
        msg.header.frame_id = ''
        msg.data = True
        
        rospy.loginfo("Stop lane following")
        pub.publish(msg)

        pub_wheels_cmd = rospy.Publisher(f'/{vehicle_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header = Header()
        msg_wheels_cmd.header.seq = 0
        msg_wheels_cmd.header.stamp = rospy.Time(0)
        msg_wheels_cmd.header.frame_id = ''
        msg_wheels_cmd.vel_left = 0.0  
        msg_wheels_cmd.vel_right = 0.0 

        rospy.loginfo("Stopping wheels")
        pub_wheels_cmd.publish(msg_wheels_cmd)        


class TurningRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finish'])
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING")
        return 'turn_finish'
        
    # TODO: Add publishing message and subscribe to turning state    
        

class TurningLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finish'])
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING")
        return 'turn_finish'
        
    # TODO: Add publishing message and subscribe to turning state
        

class TurningU(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finish'])
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING")
        return 'turn_finish'
        
    # TODO: Add publishing message and subscribe to turning state
    
    

class StateMachineNode:
    def __init__(self):
        rospy.init_node("state_machine_node")

        #  define state machine
        self.sm = smach.StateMachine(outcomes=['Run'])
        with self.sm:
            smach.StateMachine.add('LANE_FOLLOWING', LaneFollowing(), transitions={'obstacle_observed': 'OBSTACLE_AVOIDANCE',
                                                                                   'apriltag_detected': 'STOP'})
            smach.StateMachine.add('STOP', Stop(), transitions={'nothing_changed': 'LANE_FOLLOWING', 
                                                                'turn_apriltag': 'TURNING_RIGHT_LEFT_U',
                                                                'goal_apriltag': 'STOP', 
                                                                'new_goal_given': 'PLAN_TRAJECTORY'})
            smach.StateMachine.add('TURNING', TurningRightLeftU(), transitions={'turn_finished': 'LANE_FOLLOWING'})
            smach.StateMachine.add('PLAN_TRAJECTORY', PlanTrajectory(), transitions={'plan_complete': 'LANE_FOLLOWING'})
            smach.StateMachine.add('OBSTACLE_AVOIDANCE', ObstacleAvoidance(), transitions={'obstacle_overtaken': 'LANE_FOLLOWING',
                                                                                           'overtaking_obstacle': 'OBSTACLE_AVOIDANCE'})

    def run(self):
        outcome = self.sm.execute()

if __name__ == "__main__":
    print('State Machine started')
    node = StateMachineNode(name="state_machine_node")
    node.run()