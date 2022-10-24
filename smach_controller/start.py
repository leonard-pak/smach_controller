import rclpy
import smach
import smach_ros
import time

from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

INSTRUMENT = 'screwdriver'
kuka_work = Bool()
kuka_work.data = False

class FaceDetection(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['worker', 'not_worker'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Face Detection')
        future = self.node.client_name.call_async(Trigger.Request())
        curent_face = ''
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(
                        'Result of get_face: %s' % response.message)
                    curent_face = response.message
                break
        if curent_face == 'Wu Guo':
            return 'worker'
        else:
            return 'not_worker'


class ToolRecognition(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=[INSTRUMENT, 'not_tool'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Tool Recognition')
        future = self.node.client_tool.call_async(Trigger.Request())
        curent_tool = ''
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(
                        'Result of get_face: %s' % response.message)
                    curent_tool = response.message
                break
        if curent_tool == INSTRUMENT:
            return INSTRUMENT
        else:
            return 'not_tool'


class GestureRecognition(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start', 'stop', 'not_gesture'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Gesture Recognition')
        future = self.node.client_cmd.call_async(Trigger.Request())
        curent_gesture = ''
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(
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
        smach.State.__init__(
            self, outcomes=['finish', 'task_work_async', 'stop'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Main Task')
        self.node.robot_simulation.StartAsyncTask()
        
        future = self.node.client_cmd.call_async(Trigger.Request())
        curent_gesture = ''
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                    kuka_work.data = True
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(
                        'Result of get_face: %s' % response.message)
                    curent_gesture = response.message
                break
        if self.node.robot_simulation.IsTaskFinish():
            return 'finish'
        elif curent_gesture == 'stop':
            self.start_time = None
            return 'stop'
        else:
            return 'task_work_async'


class Pause(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['start', 'not_gesture'])
        self.node = node

    def execute(self, userdata):
        self. node.get_logger().info('Executing state Pause Task')
        self.node.robot_simulation.PauseTask()      
        future = self.node.client_cmd.call_async(Trigger.Request())
        curent_gesture = ''
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                try:
                    response = future.result()
                    kuka_work.data = False
                except Exception as e:
                    self.node.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.node.get_logger().info(
                        'Result of get_face: %s' % response.message)
                    curent_gesture = response.message
                break
        if curent_gesture == 'start':
            return 'start'
        else:
            return 'not_gesture'


class PrepareFinish(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['success'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Prepare Finish Task')
        kuka_work.data = False
        time.sleep(5)
        return 'success'


class FakeRobotSimulation():
    def __init__(self):
        self.start_time = None
        self.pause_time = None

    def StartAsyncTask(self):
        if self.start_time == None:
            self.start_time = time.perf_counter()

    def IsTaskFinish(self):
        return (time.perf_counter() - self.start_time) > 15.0

    def PauseTask(self):
        self.pause_time = self.start_time
        self.start_time = None

class MainNode(Node):
    def __init__(self):
        super().__init__('smach_state_machine')

        # self.subscription = self.create_subscription(
        #     Bool,
        #     'body_detection',
        #     self.IsHumanCallback,
        #     10)
        # self.subscription  # prevent unused variable warning

        self.client_name = self.create_client(Trigger, 'get_name')
        self.client_cmd = self.create_client(Trigger, 'get_cmd')
        self.client_tool = self.create_client(Trigger, 'get_tool')
         
        self.pub_kuka = self.create_publisher(Bool, "/kuka_work", 1)
        self.timer = self.create_timer(0.5, self.kuka_work_callback)

        while (not self.client_name.wait_for_service(timeout_sec=1.0)) or (not self.client_cmd.wait_for_service(timeout_sec=1.0)) or (not self.client_tool.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('services not available, waiting again...')

        self.robot_simulation = FakeRobotSimulation()

        self.InitStateMachine()

        self.StartStateMachin()

    def kuka_work_callback(self):
        self.pub_kuka.publish(kuka_work)
        self.get_logger().info('Publishing kuka_work: "%s"' % kuka_work)
        
    def IsHumanCallback(self, msg):
        if msg.data:
            self.get_logger().info('Human here!')
            self.StartStateMachin()
        else:
            self.get_logger().info('Human go out!')
            self.StopStateMachin()

    def InitStateMachine(self):
        self.sm = smach.StateMachine(outcomes=['FINISH'])

        with self.sm:
            smach.StateMachine.add('FACE_DETECTION', FaceDetection(node=self),
                                   transitions={'worker': 'GESTURE_RECOGNITION',
                                                'not_worker': 'FACE_DETECTION'})
            smach.StateMachine.add('GESTURE_RECOGNITION', GestureRecognition(node=self),
                                   transitions={'stop': 'PREPARE_FINISH',
                                                'start': 'TOOL_RECOGNITION',
                                                'not_gesture': 'GESTURE_RECOGNITION'})
            smach.StateMachine.add('TOOL_RECOGNITION', ToolRecognition(node=self),
                                   transitions={INSTRUMENT: 'GO_TASK',
                                                'not_tool': 'TOOL_RECOGNITION'})
            smach.StateMachine.add('GO_TASK', ExecuteTask(node=self),
                                   transitions={'finish': 'PREPARE_FINISH',
                                                'task_work_async': 'GO_TASK',
                                                'stop': 'PAUSE'})
            smach.StateMachine.add('PAUSE', Pause(node=self),
                                   transitions={'start': 'GO_TASK',
                                                'not_gesture': 'PAUSE'})
            smach.StateMachine.add('PREPARE_FINISH', PrepareFinish(node=self),
                                   transitions={'success': 'FINISH'})

    def StartStateMachin(self):
        self.sis = smach_ros.IntrospectionServer(
            'smach_state_machine_viewer', self.sm, '/SM_ROOT')
        self.sis.start()
        self.outcome = self.sm.execute()

    def StopStateMachin(self):
        self.sis.stop()
        self.sm.close()


def main():

    rclpy.init(args=None)
    node = MainNode()
    
    rclpy.spin(node)


if __name__ == '__main__':
    main()
