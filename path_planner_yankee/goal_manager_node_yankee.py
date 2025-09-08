import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import time
class GoalManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # Subscripción al topic donde se reciben los puntos
        self.subscription = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)

        # Subscripción al topic donde el tracker avisa que llegó
        self.reached_sub = self.create_subscription(Bool,'/goal_reached',self.reached_callback,10)
        # Subscripción al joystick
        self.joystick_sub = self.create_subscription(Joy,'/joy',self.joystick_callback,10)
        # Publicador de los waypoints hacia el tracker
        self.publisher = self.create_publisher(PoseStamped,'/goal_waypoint',10)

        self.waypoints = []
        self.current_index = 0
        self.waiting_for_reach = False
        self.start_sequence = False
        self.reset_sequence = False

        self.get_logger().info("Goal Manager Initialized")
    def goal_callback(self, msg: PoseStamped):
        """Guardar puntos recibidos"""
        if not self.start_sequence:
            self.waypoints.append(msg)
            self.get_logger().info(f"Punto recibido ({len(self.waypoints)})")
        elif self.reset_sequence:
            self.waypoints = []
            self.current_index = 0
            self.waiting_for_reach = False
            self.reset_sequence = False
            self.start_sequence = False
            self.get_logger().info("Secuencia reiniciada, puntos borrados")

            
    def reached_callback(self, msg: Bool):
        """Escuchar confirmación del tracker"""
        if msg.data and self.waiting_for_reach:
            self.get_logger().info("Goal alcanzado")
            self.waiting_for_reach = False
            self.current_index += 1

            # Publicar siguiente si hay más
            if len(self.waypoints) !=0 :
                self.publish_next_waypoint()
            else:
                self.get_logger().info("No hay más waypoints por publicar")
    def joystick_callback(self, msg: Joy):
        """ Iniciar (X) o reiniciar (O) la secuencia con el joystick """
        if msg.buttons[0] == 1 and self.current_index == 0 and not self.start_sequence and len(self.waypoints) != 0:
            self.start_sequence = True
            self.get_logger().info("Secuencia iniciada")
            self.publish_next_waypoint()
            self.current_index += 1
            time.sleep(0.5)  # Debounce
        if msg.buttons[1] == 1:
            self.start_sequence = False
            self.reset_sequence = True
            self.current_index = 0
    def publish_next_waypoint(self):
        """Publicar el siguiente punto"""
        if len(self.waypoints) != 0 and self.start_sequence:
            if self.current_index >= len(self.waypoints):
                self.get_logger().info("Todos los waypoints han sido alcanzados")
                self.current_index = 0
                self.start_sequence = False
                self.waiting_for_reach = False
                return
            next_goal = self.waypoints[self.current_index]
            self.publisher.publish(next_goal)
            self.get_logger().info(f"Publicando waypoint {self.current_index+1}/{len(self.waypoints)}")
            self.waiting_for_reach = True


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
