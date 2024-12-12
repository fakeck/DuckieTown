#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
from duckietown_msgs.msg import BoolStamped, WheelsCmdStamped
import smach


class PlanTrajectory(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['plan_complete'])
        self.vehicle_name = 'pebbles'
        self.plan_pub = rospy.Publisher(f'/{self.vehicle_name}/planner_node/0',BoolStamped,queue_size=10)
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: PLAN_TRAJECTORY")
        try:
            rospy.wait_for_message(f'/{self.vehicle_name}/planner_node/status', BoolStamped, timeout=1000)
            self.start_planning()
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for planner status")
        return 'plan_complete'
        
    def start_planning(self, vehicle_name): 
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.plan_pub.publish(msg)
        rospy.loginfo("Start lane following")


class LaneFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_observed', 'apriltag_detected'])
        self.vehicle_name = 'pebbles'
        self.lane_following_pub = rospy.Publisher(f'/{self.vehicle_name}/lane_following_controller_node/1',BoolStamped,queue_size=10)
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: LANE_FOLLOWING")
        self.start_lane_following()
        
        # Wait for obstacle detection message
        try: 
            obstacle_msg = rospy.wait_for_message(f'/{self.vehicle_name}/obstacle_detection/status', BoolStamped,timeout=1000)
            if obstacle_msg.data:
                    return 'obstacle_observed'
        except rospy.ROSException:
            rospy.logwarn("Timeout waiting for LaneFollowing status")       
        
        # Wait for apriltag detection message
        apriltag_msg = rospy.wait_for_message(f'/{self.vehicle_name}/apriltag_detection/status',BoolStamped,timeout=1000)
        if apriltag_msg.data:
                return 'apriltag_detected'
        return 'apriltag_detected' 

    def start_lane_following(self):
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.lane_following_pub.publish(msg)
             
class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['nothing_changed', 'turn_right', 'turn_left', 'turn_u', 'goal_apriltag', 'new_goal_given']
        )
        self.vehicle_name = 'pebbles'
        # Initialize both publishers in __init__
        self.override_pub = rospy.Publisher(
            f'/{self.vehicle_name}/joy_mapper_node/joystick_override',
            BoolStamped,
            queue_size=10
        )
        self.wheels_cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=10
        )
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: STOP")
        self.stop_vehicle()
        
        # Wait for apriltag type message
        try:
            tag_msg = rospy.wait_for_message(
                f'/{self.vehicle_name}/apriltag_type',
                BoolStamped,
                timeout=1000
            )
        except rospy.ROSException:
            rospy.logwarn("No apriltag type message received")
            
        return 'nothing_changed'
    
    def stop_vehicle(self):
        # Send override command
        override_msg = BoolStamped()
        override_msg.header.stamp = rospy.Time.now()
        override_msg.data = True
        self.override_pub.publish(override_msg)
        
        # Stop wheels
        wheels_msg = WheelsCmdStamped()
        wheels_msg.header.stamp = rospy.Time.now()
        wheels_msg.vel_left = 0.0
        wheels_msg.vel_right = 0.0
        self.wheels_cmd_pub.publish(wheels_msg)       


class ObstacleAvoidance(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle_overtaken', 'overtaking_obstacle'])
        self.vehicle_name = 'pebbles'
        self.avoidance_pub = rospy.Publisher(
            f'/{self.vehicle_name}/obstacle_avoidance_node/start',
            BoolStamped,
            queue_size=10
        )
        self.wheels_cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=10
        )
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: OBSTACLE_AVOIDANCE")
        self.start_avoidance()
        
        try:
            # Wait for avoidance status
            avoidance_msg = rospy.wait_for_message(
                f'/{self.vehicle_name}/obstacle_avoidance_node/status',
                BoolStamped,
                timeout=1000
            )
            if avoidance_msg.data:
                return 'obstacle_overtaken'
            else:
                return 'overtaking_obstacle'
        except rospy.ROSException:
            rospy.logwarn("No obstacle avoidance status received")
            return 'overtaking_obstacle'
            
    def start_avoidance(self):
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.avoidance_pub.publish(msg)

class TurningRight(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finished'])
        self.vehicle_name = 'pebbles'
        self.turn_pub = rospy.Publisher(
            f'/{self.vehicle_name}/turning_node/right',
            BoolStamped,
            queue_size=10
        )
        self.wheels_cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=10
        )
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING RIGHT")
        self.start_turn()
        
        try:
            # Wait for turn completion status
            turn_msg = rospy.wait_for_message(
                f'/{self.vehicle_name}/turning_node/status',
                BoolStamped,
                timeout=1000
            )
            if turn_msg.data:
                return 'turn_finished'
        except rospy.ROSException:
            rospy.logwarn("No turning status received")
            return 'turn_finished'
            
    def start_turn(self):
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.turn_pub.publish(msg)

class TurningLeft(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finished'])
        self.vehicle_name = 'pebbles'
        self.turn_pub = rospy.Publisher(
            f'/{self.vehicle_name}/turning_node/left',
            BoolStamped,
            queue_size=10
        )
        self.wheels_cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=10
        )
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING LEFT")
        self.start_turn()
        
        try:
            # Wait for turn completion status
            turn_msg = rospy.wait_for_message(
                f'/{self.vehicle_name}/turning_node/status',
                BoolStamped,
                timeout=1000
            )
            if turn_msg.data:
                return 'turn_finished'
        except rospy.ROSException:
            rospy.logwarn("No turning status received")
            return 'turn_finished'
            
    def start_turn(self):
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.turn_pub.publish(msg)

class TurningU(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turn_finished'])
        self.vehicle_name = 'pebbles'
        self.turn_pub = rospy.Publisher(
            f'/{self.vehicle_name}/turning_node/u_turn',
            BoolStamped,
            queue_size=10
        )
        self.wheels_cmd_pub = rospy.Publisher(
            f'/{self.vehicle_name}/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=10
        )
    
    def execute(self, userdata):
        rospy.loginfo("Executing state: TURNING U")
        self.start_turn()
        
        try:
            # Wait for turn completion status
            turn_msg = rospy.wait_for_message(
                f'/{self.vehicle_name}/turning_node/status',
                BoolStamped,
                timeout=1000
            )
            if turn_msg.data:
                return 'turn_finished'
        except rospy.ROSException:
            rospy.logwarn("No turning status received")
            return 'turn_finished'
            
    def start_turn(self):
        msg = BoolStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.turn_pub.publish(msg)
    
    

class StateMachineNode:
    def __init__(self):
        rospy.init_node("state_machine_node")

        #  define state machine
        self.sm = smach.StateMachine(outcomes=['Run'])
        with self.sm:
            smach.StateMachine.add('LANE_FOLLOWING', LaneFollowing(), transitions={'obstacle_observed': 'OBSTACLE_AVOIDANCE',
                                                                                   'apriltag_detected': 'STOP'})
            smach.StateMachine.add('STOP', Stop(), transitions={'nothing_changed': 'LANE_FOLLOWING', 
                                                                'turn_right': 'TURNING_RIGHT',
                                                                'turn_left': 'TURNING_LEFT',
                                                                'turn_u': 'TURNING_U',
                                                                'goal_apriltag': 'STOP', 
                                                                'new_goal_given': 'PLAN_TRAJECTORY'})
            smach.StateMachine.add('TURNING_RIGHT', TurningRight(), transitions={'turn_finished': 'LANE_FOLLOWING'})
            smach.StateMachine.add('TURNING_LEFT', TurningLeft(), transitions={'turn_finished': 'LANE_FOLLOWING'})
            smach.StateMachine.add('TURNING_U', TurningU(), transitions={'turn_finished': 'LANE_FOLLOWING'})
            smach.StateMachine.add('PLAN_TRAJECTORY', PlanTrajectory(), transitions={'plan_complete': 'LANE_FOLLOWING'})
            smach.StateMachine.add('OBSTACLE_AVOIDANCE', ObstacleAvoidance(), transitions={'obstacle_overtaken': 'LANE_FOLLOWING',
                                                                                           'overtaking_obstacle': 'OBSTACLE_AVOIDANCE'})

    def run(self):
        outcome = self.sm.execute()

if __name__ == "__main__":
    print('State Machine started')
    node = StateMachineNode()
    node.run()