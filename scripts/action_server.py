#!/usr/bin/env python3
''' this makes the server '''
from second_coursework.msg import SearchAction, SearchGoal, SearchFeedback, SearchResult
from second_coursework.srv import GetRoomCoord, GetRoomCoordRequest
import rospy
import actionlib

''' this makes the state machine '''
from smach_ros import ServiceState, SimpleActionState
from smach import CBState
import smach

'''first state machine '''
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

'''detection'''
from process_input_state import ProcessInput
from second_coursework.srv import YoloLastFrame

'''import the services that this action server depends on'''
from yolo_service import YOLOv4ROSITR

'''a class that implements the search action'''
class SearchServer:
    def __init__(self) -> None:
        self.server = actionlib.SimpleActionServer('SearchServer', SearchAction, self.execute, False)
        self.result = SearchResult()
        self.feedback = SearchFeedback()
        self.rate = rospy.Rate(5)
        self.flag = False
        self.server.start()

    def execute(self, goal: SearchGoal):        
        '''cuncurrence container'''
        cc = self.cuncurrence_container(goal)
        outcome = cc.execute()
        
        self.result.result_string_list = self.feedback.feedback_string_list
        self.result.result_int_list = self.feedback.feedback_int_list
        self.result.current_time = rospy.Time.now()
        self.feedback = None
        self.server.set_succeeded(self.result)

    def cuncurrence_container(self, goal):
        def child_term_cb(outcome_map):
            if outcome_map['detection'] == 'cake':
                self.flag = True
                return True
            return False
            
        movement_machine  = self.movement_state_machine(goal)
        detection_machine = self.detection_state_machine()
        feedback_machine = self.feedback_state_machine()

        cc = smach.Concurrence(outcomes=['cake', 'no_cake'],
                            default_outcome='cake',
                            child_termination_cb=child_term_cb,
                            outcome_map={'cake': {'detection' : 'cake'},
                                        'no_cake': {'detection' : 'no_cake',
                                                    'move': 'end',
                                                    'feedback': 'no_cake'}})

        with cc:
            smach.Concurrence.add('move', movement_machine)
            smach.Concurrence.add('detection', detection_machine)
            smach.Concurrence.add('feedback', feedback_machine)

        return cc

    def feedback_state_machine(self):
        sm = smach.StateMachine(outcomes=['no_cake'])

        with sm:
            @smach.cb_interface(outcomes=['succeeded', 'fails'])
            def feed_back_cb(ud):
                self.server.publish_feedback(self.feedback)
                self.rate.sleep()
                if self.flag:
                    return 'fails'
                return 'succeeded'

            smach.StateMachine.add('feedback', CBState(feed_back_cb), transitions={'succeeded': 'feedback', 'fails':'no_cake'})

        return sm

    '''detecting state machine using the yolo service'''
    def detection_state_machine(self):
        sm = smach.StateMachine(outcomes=['cake', 'no_cake'])

        with sm:
            def detection_cb(userdata, result):
                '''we do shit related to the feedback here, this is where we send feedback'''
                for i in range(len(result.detections)):
                    if result.detections[i].name in self.feedback.feedback_string_list:
                        index = self.feedback.feedback_string_list.index(result.detections[i].name)
                        self.feedback.feedback_int_list[index] += 1

                    else:
                        self.feedback.feedback_string_list.append(result.detections[i].name)
                        self.feedback.feedback_int_list.append(1)
                
                userdata.detections = result.detections
            
            smach.StateMachine.add('yolo_detect', ServiceState('/detect_frame', 
                                                            YoloLastFrame,
                                                            output_keys=['detections'],
                                                            response_cb=detection_cb),
                                    transitions={'succeeded': 'process', 'aborted':'no_cake', 'preempted':'no_cake'})
            
            smach.StateMachine.add('process', ProcessInput(),
                                    transitions={'succeeded': 'cake', 'no_cake':'yolo_detect'},
                                    remapping={'detections': 'detections'})
        return sm

    def movement_state_machine(self, goal: SearchGoal):
        sm = smach.StateMachine(outcomes=['end'])

        with sm:
            '''we process the request here'''
            def room_srv_cb(userdata, result):           
                userdata.x_out = result.point.x
                userdata.y_out = result.point.y

            smach.StateMachine.add('get_point', ServiceState('get_coord', 
                                                            GetRoomCoord,
                                                            response_cb=room_srv_cb,
                                                            output_keys=['x_out', 'y_out'],
                                                            request=GetRoomCoordRequest(goal.room_to_search)),
                                    transitions={'succeeded': 'not_at_point', 'aborted':'end', 'preempted':'end'})

            ''' we get the next coord from our room '''
            def goalCB(ud, goal):
                move_goal = MoveBaseGoal()
                move_goal.target_pose.header.stamp = rospy.get_rostime()
                move_goal.target_pose.header.frame_id = 'map'

                # position
                move_goal.target_pose.pose.position.x = ud.x_in
                move_goal.target_pose.pose.position.y = ud.y_in

                # orientation
                move_goal.target_pose.pose.orientation.w = 1

                return move_goal

            smach.StateMachine.add('not_at_point', SimpleActionState('move_base', 
                                                                    MoveBaseAction,
                                                                    goal_cb=goalCB,
                                                                    input_keys=['x_in', 'y_in']),
                                    transitions={'succeeded': 'get_point', 'aborted':'end', 'preempted':'end'},
                                    remapping={'x_in' : 'x_out',
                                               'y_in' : 'y_out'})
            
        return sm

if __name__ == '__main__':
    rospy.init_node('main_node')
    server = SearchServer()  
    yolo_detector = YOLOv4ROSITR()
    rospy.spin()