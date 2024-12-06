#l/usr/bin/env python3 

import rospy 
from std_msgs.msg import String 

class MinimalRosNode: 
    def __init__(self) -> None:
        rospy.init_node("minimal_ros_node")
        
        self.rate = rospy.Rate(2) # in Hz 
        
        # Publisher 
        self.radio_pub = rospy.Publisher("radio", String, queue_size=10)
        
        # Subscriber 
        self.radio_sub = rospy.Subscriber("radio", String, self.radio_callback)
        
        # Data 
        self.counter = 0 
        
    def radio_callback(self, msg): 
        rospy.loginfo(f"Received: {msg.data}")
        
    def publish_radio_msg(self): 
        msg = String()
        msg.data = f"Hello, ROS {self.counter}"
        
    def run(self): 
        while not rospy.is_shutdown(): 
            self.publish_radio_msg()
            self.rate.sleep()
            self.counter += 1