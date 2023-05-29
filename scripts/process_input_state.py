import smach

class ProcessInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'no_cake'], input_keys=['detections'])

    def execute(self, userdata):
        for i in range(len(userdata.detections)):
            if userdata.detections[i].name == 'cake':
                return 'succeeded'
        return 'no_cake'
