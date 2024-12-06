from enum import Enum 

class StateMachine: 
    def __init__(self): 
        self.states = { 
            "intial_pos": self.initial_pos, 
            "planning": self.planning,
            "lane_following": self.lane_following, 
            "obstacle_avoidance": self.obstacle_avoidance, 
            "turning_r": self.turning_r,
            "turning_l": self.turning_l,
            "turning_u": self.turning_u,
            }
        
    def transition(self, event):
        if event in ['goal_given', 'trajectory_planned', 'obstacle_observerd', 'AprilTag_right', 'AprilTag_left', 'AprilTag_u', 'goal_reached']:
            self.current_state = self.states[self.current_state](event)
        else:
            print(f"Invalid event: {event}")
            
    def initial_pos_state(self, event): 
        if event == "goal_reached": 
            self.current_state = "planning"
        return self.current_state
    
    def planning_state(self, event): 
        if event == "goal_given":
            self.current_state = "lane_following"
        return self.current_state
    
    def lane_following_state(self, event): 
        if event == "trajectory_planned":
            self.current_state = "lane_following"
        return self.current_state

    def obstacle_avoidance_state(self, event): 
        if event == "obstacle_observed":
            self.current_state = "obstacle_avoidance"
        return self.current_state
        
    def turning_state(self, event): 
        if event == "AprilTag_right":
            self.current_state = "turning_r"
        elif event == "AprilTag_left":
            self.current_state = "turning_l"
        elif event == "AprilTag_u":
            self.current_state = "turning_u"
        return self.current_state
        
    