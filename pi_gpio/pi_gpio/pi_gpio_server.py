import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pi_gpio_interface.action import GPIO as GPIO_Action
import RPi.GPIO as GPIO

class RaspberryPIGPIO():
    def __init__(self, pin_id, type):
        self.pin_id = pin_id
        self.type = type
        print (str(self.pin_id) + "-" + self.type)
        GPIO.setwarnings(False)
        #Use Broadcom pin-numbering scheme
        GPIO.setmode(GPIO.BCM) 
        if type == "in":
            GPIO.setup(pin_id, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #Set pin as input
        elif type == "out":
            GPIO.setup(pin_id, GPIO.OUT) #Set pin as output
    
    def set_pin(self, value):
        if value == 1:
            GPIO.output(self.pin_id, GPIO.HIGH) #Set pin High-1
        elif value == 0:
            GPIO.output(self.pin_id, GPIO.LOW) #Set pin Low-0

    def read_pins_from_file():
        f = open("src/pi_gpio/gpio_pins.txt", "r")
        pin_list = []
        for x in f:
            pin_list.append(x)
        f.close()
        
        return pin_list

class GPIOActionServer(Node):

    def __init__(self):
        super().__init__('pi_gpio_server')
              
        pin_list = RaspberryPIGPIO.read_pins_from_file()
        
        self.pin_dic = {}
        
        for pin in pin_list:
            pin_id, type = pin.split(',')
            self.pin_dic[pin_id] =  RaspberryPIGPIO(int(pin_id), type)

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        #Node, action_type, action_name, execute_callback
        self._action_server = ActionServer(
            self,
            GPIO_Action,
            'pi_gpio_server',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Executes the goal."""
        self.get_logger().info('Executing goal...')

        # Populate goal message
        goal_msg = goal_handle.request.gpio

        # Populate feedback message
        feedback_msg = GPIO_Action.Feedback()
        feedback_msg.feedback = 1

        # Populate result message
        result = GPIO_Action.Result()

        if not goal_handle.is_active:
            self.get_logger().info('Goal aborted')
            return GPIO_Action.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return GPIO_Action.Result()

        # Publish the feedback
        goal_handle.publish_feedback(feedback_msg)

        # get the pin ide and action type
        pin_id, action_type = goal_msg.split(',')   

        if action_type == "high":
            self.pin_dic[pin_id].set_pin(1)
            print("on")
            time.sleep(0.1)
            result.value = 3
       
        elif action_type == "low":
            time.sleep(2)
            self.pin_dic[pin_id].set_pin(0)
            print("off")
            time.sleep(0.1)
            result.value = 3

        elif action_type == "read":
            result.value = GPIO.input(int(pin_id))

        goal_handle.succeed()

        #Cleanup the GPIO pins, so other processes can use them
        GPIO.cleanup()

        return result

def main(args=None):
    rclpy.init(args=args)

    action_server = GPIOActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
