#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import SearchAction, SearchGoal, SearchActionFeedback
from second_coursework.srv import YoloLastFrame, YoloLastFrameResponse

class Tester:
    def __init__(self) -> None:
        self.act_cli = actionlib.SimpleActionClient('SearchServer', SearchAction)
        self.feedback = rospy.Subscriber("/SearchServer/feedback", SearchActionFeedback, self.feedback)
        self.goal = SearchGoal()

    def exe(self):
        self.act_cli.wait_for_server()
        self.goal.room_to_search = 'A'
        self.act_cli.send_goal(self.goal)
        self.act_cli.wait_for_result()
        res = self.act_cli.get_result()
        rospy.loginfo(res)

    def feedback(self, msg):
        rospy.loginfo(msg)

    def test_yolo(self):
        rospy.wait_for_service('/detect_frame')
        get_coords_proxy = rospy.ServiceProxy('/detect_frame', YoloLastFrame)
        resp: YoloLastFrameResponse = get_coords_proxy()
        rospy.loginfo(resp)

if __name__ == '__main__':
    rospy.init_node('test_node')
    tester = Tester()
    tester.exe()
