import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

# from strands_executive_msgs.msg import Task
# from task_executor.msg import *
# from topological_navigation.msg import GotoNodeAction, GotoNodeResult


class Checker(object):
    """ 
    Wraps up an action server in this checking object.
    """
    def __init__(self, check_action, action, action_clz):
        self.action = action
        self.action_clz = action_clz
        self.action_client = None        
        self.server = actionlib.SimpleActionServer(check_action, action_clz, self.do_check, False)
        self.server.start()

    def get_goal(self, goal):
        return goal

    def generate_report(self, state, result):
        pass

    def do_check(self, check_goal):
        if self.action_client == None:
            rospy.loginfo('waiting for %s server' % (self.action))
            self.action_client = actionlib.SimpleActionClient(self.action, self.action_clz)    
            self.action_client.wait_for_server()
            rospy.loginfo('connected to %s server' % (self.action))    

        goal = self.get_goal(check_goal)
        
        self.action_client.send_goal(goal)

        self.action_client.wait_for_result()

        result = self.action_client.get_result()
        state = self.action_client.get_state()

        self.generate_report(state, result)

        if state == GoalStatus.SUCCEEDED:
            self.server.set_succeeded(result)
        elif state == GoalStatus.PREEMPTED:
            self.server.set_preempted(result)
        else: 
            self.server.set_aborted(result)