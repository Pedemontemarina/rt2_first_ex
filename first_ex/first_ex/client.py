import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.action import XPosition


class PositionClient(Node):

    def __init__(self):
        super().__init__('position_client') #nome del nodo
        self._action_client = ActionClient(self, XPosition,'xposition')
        # type of interface and name of the action (same of the server)
        self._goal_handle = None
        self._cancel_sent = False

        self._last_positions = []   # storico posizioni recenti
        self._stuck_threshold = 0.01  # se si muove meno di questo
        self._stuck_window = 10       # negli ultimi N feedback (~1 secondo)
    
    
    def send_goal(self, x_target): #metodo per inviare il goal al server
        goal_msg = XPosition.Goal()
        goal_msg.x_target = x_target

        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')

        # invio il goal in modo asincrono, restituisce future che è la risposta del server
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback = self.feedback_callback)
        # quando il server dice se accetta o no il goal allora chiamo la callback
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._goal_handle = goal_handle
        
        # voglio il risultato del goal!
        self._get_result_future = goal_handle.get_result_async()
        # quando è pronto chiamo la result callback
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_x= feedback.current_x
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_x))

        # cancelling the goal if the current_x is too near next.
        if self._cancel_sent or self._goal_handle is None:
            return

        # tieni solo gli ultimi N valori
        self._last_positions.append(current_x)
        if len(self._last_positions) > self._stuck_window:
            self._last_positions.pop(0)

        # se abbiamo abbastanza campioni, controlla se il robot è bloccato
        if len(self._last_positions) == self._stuck_window:
            movement = max(self._last_positions) - min(self._last_positions)
            if movement < self._stuck_threshold:
                self._cancel_sent = True
                self.get_logger().warn(f'Robot stuck! Movement in last {self._stuck_window} feedbacks: {movement:.4f} — cancelling')
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        self.get_logger().info('Cancel response received')

def main(args=None):
        rclpy.init(args=args)
        action_client = PositionClient()
        action_client.send_goal(10.2)
        rclpy.spin(action_client)
        
if __name__ == '__main__':
    main()
 