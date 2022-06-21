import rclpy
import smach
import smach_ros

from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
 
class FaceDetection(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['worker','not_worker'])
        self.node = node

    def execute(self, userdata):
        node.get_logger().info('Executing state Face Detection')
        future = self.node.get_face.call_async(Trigger.Request())
        curent_face = ''
        while rclpy.ok():
	   rclpy.spin_once(node)
	   if self.future.done():
	       try:
                   response = self.future.result()
               except Exception as e:
                   node.get_logger().info(
                    'Service call failed %r' % (e,))
               else:
                   node.get_logger().info(
                    'Result of get_face: %s' % response.message)
                   curent_face = response.message
            break
        if curent_face == 'wu_guo':
            return 'worker'
        else:
            return 'not_worker'

class ToolRecognition(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['num_1','not_tool'])
        self.node = node

    def execute(self, userdata):
        node.get_logger().info('Executing state Tool Recognition')
        future = self.node.get_tool.call_async(Trigger.Request())
        curent_tool = ''
        while rclpy.ok():
	   rclpy.spin_once(node)
	   if self.future.done():
	       try:
                   response = self.future.result()
               except Exception as e:
                   node.get_logger().info(
                    'Service call failed %r' % (e,))
               else:
                   node.get_logger().info(
                    'Result of get_face: %s' % response.message)
                   curent_tool = response.message
            break
        if curent_tool == 'num_1':
            return 'num_1'
        else:
            return 'not_tool'
        
class GestureRecognition(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start','stop','not_gesture'])
        self.node = node

    def execute(self, userdata):
        node.get_logger().info('Executing state Gesture Recognition')
        future = self.node.get_gesture.call_async(Trigger.Request())
        curent_gesture = ''
        while rclpy.ok():
	   rclpy.spin_once(node)
	   if self.future.done():
	       try:
                   response = self.future.result()
               except Exception as e:
                   node.get_logger().info(
                    'Service call failed %r' % (e,))
               else:
                   node.get_logger().info(
                    'Result of get_face: %s' % response.message)
                   curent_gesture = response.message
               break
        if curent_gesture == 'stop':
            return 'stop'
        elif curent_gesture == 'start':
            return 'start'
        else:
            return 'not_gesture'


class ExecuteTask(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['finish', 'task_work_async', 'stop'])
        self.node = node

    def execute(self, userdata):
        node.get_logger().info('Executing state Main Task')
        self.StartAsyncTask()
        future = self.node.get_gesture.call_async(Trigger.Request())
        curent_gesture = ''
        while rclpy.ok():
	   rclpy.spin_once(node)
	   if self.future.done():
	       try:
                   response = self.future.result()
               except Exception as e:
                   node.get_logger().info(
                    'Service call failed %r' % (e,))
               else:
                   node.get_logger().info(
                    'Result of get_face: %s' % response.message)
                   curent_gesture = response.message
               break
        # TODO
        return 'async_task_start'
        
    def StartAsyncTask():
    # TODO
        pass
        
    def CheckTask():
    # TODO
        pass
        
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
        
        
class MainNode(Node):
    def __init__(self):
       super().__init__('smach_state_machine')
       
       self.subscription = self.create_subscription(
           Bool,
           'body_detection',
           self.IsHumanCallback,
           10)
       self.subscription  # prevent unused variable warning
       
       self.client_name = self.create_client(Trigger, 'get_name')
       self.client_gesture = self.create_client(Trigger, 'get_gesture')
       self.client_tool = self.create_client(Trigger, 'get_tool')
       
       while (not self.client_name.wait_for_service(timeout_sec=1.0)) or (not self.client_gesture.wait_for_service(timeout_sec=1.0)) or (not self.client_tool.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('services not available, waiting again...')
       
    
    def IsHumanCallback(self, msg):
        if msg.data:
            self.get_logger().info('Human here!')
            self.outcome = self.sm.execute()
        else:
            self.get_logger().info('Human go out!')
            # TODO
       
    def InitStateMachine():
        self.sm = smach.StateMachine(outcomes=['FINISH'])

        with self.sm:
            smach.StateMachine.add('FACE_DETECTION', FaceDetection(self), 
                                  transitions={'worker':'GESTURE_RECOGNITION', 
                                               'not_worker':'FACE_DETECTION'})
            smach.StateMachine.add('GESTURE_RECOGNITION', GestureRecognition(self), 
                                  transitions={'stop':'PREPARE_FINISH',
                                               'start':'TOOL_RECOGNITION',
                                               'not_gesture':'GESTURE_RECOGNITION'})
            smach.StateMachine.add('TOOL_RECOGNITION', ToolRecognition(self), 
                                  transitions={'num_1':'GO_TASK',
                                               'not_tool':'TOOL_RECOGNITION'})
            smach.StateMachine.add('GO_TASK', ExecuteTask(self), 
                                  transitions={'finish':'PREPARE_FINISH',
                                               'task_work_async':'GO_TASK',
                                               'stop':'PAUSE'})                                    
            smach.StateMachine.add('PAUSE', Pause(), 
                                  transitions={'start':'GO_TASK'})
            smach.StateMachine.add('PREPARE_FINISH', PrepareFinish(self), 
                                  transitions={'success':'FINISH'})
                                  
       sis = smach_ros.IntrospectionServer('smach_introspection_server', self.sm, '/SM_ROOT')
       sis.start()
                               
    

def main():
    
    rclpy.init(args=None)
    node = MainNode()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
