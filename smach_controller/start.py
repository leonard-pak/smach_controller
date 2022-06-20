import rclpy
import smach
import smach_ros
#from std_srvs.srv import Trigger

rclpy.init(args=None)
node = rclpy.create_node('smach_state_machine')
 
class FaceDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['worker','not_worker'])
        self.curent_face = ''
        #rospy.wait_for_service('face')
        #self.GetFace = rospy.ServiceProxy('face', Trigger)

    def execute(self, userdata):
        node.get_logger().info('Executing state Face Detection')
        #curent_face = self.GetFace()
        curent_face = ''
        if curent_face == 'wu_guo':
        # TODO
            return 'worker'
        else:
        # TODO
            return 'not_worker'

class ToolRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['num_1','not_tool'])

    def execute(self, userdata):
        node.get_logger().info('Executing state Tool Recognition')
        if 2 == 3:
        # TODO
            return 'num_1'
        else:
        # TODO
            return 'not_tool'
        
class GestureRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start','stop','not_gesture'])

    def execute(self, userdata):
        node.get_logger().info('Executing state Gesture Recognition')
        if self.counter < 3:
        # TODO
            return 'stop'
        else:
        # TODO
            return 'start'


class ExecuteTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finish', 'task_work_async', 'stop'])

    def execute(self, userdata):
        node.get_logger().info('Executing state Main Task')
        # TODO
        return 'async_task_start'
        
class Pause(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        node.get_logger().info('Executing state Pause Task')
        # TODO
        return 'stop_task'
        
class PrepareFinish(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        node.get_logger().info('Executing state Prepare Finish Task')
        # TODO
        return 'start_task'
        

def main():
    

    sm = smach.StateMachine(outcomes=['FINISH'])

    with sm:
        smach.StateMachine.add('FACE_DETECTION', FaceDetection(), 
                               transitions={'worker':'GESTURE_RECOGNITION', 
                                            'not_worker':'FACE_DETECTION'})
        smach.StateMachine.add('GESTURE_RECOGNITION', GestureRecognition(), 
                               transitions={'stop':'PREPARE_FINISH',
                                            'start':'TOOL_RECOGNITION',
                                            'not_gesture':'GESTURE_RECOGNITION'})
        smach.StateMachine.add('TOOL_RECOGNITION', ToolRecognition(), 
                               transitions={'num_1':'GO_TASK',
                                            'not_tool':'TOOL_RECOGNITION'})
        smach.StateMachine.add('GO_TASK', ExecuteTask(), 
                               transitions={'finish':'PREPARE_FINISH',
                                            'task_work_async':'GO_TASK',
                                            'stop':'PAUSE'})                                    
        smach.StateMachine.add('PAUSE', Pause(), 
                               transitions={'start':'GO_TASK'})
        smach.StateMachine.add('PREPARE_FINISH', PrepareFinish(), 
                               transitions={'success':'FINISH'})
                               
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
