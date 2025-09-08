#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import time

class PathTopicHandle:
    def __init__(self, name, topic, timeout, priority, node):
        self.name = name
        self.topic = topic
        self.timeout = timeout
        self.priority = priority
        self.node = node
        self.last_msg = None
        self.last_time = 0.0

        self.sub = node.create_subscription(
            Path, topic, self.callback, 10
        )

    def callback(self, msg):
        self.last_msg = msg
        self.last_time = time.time()
        self.node.update_selected_path()

    def is_valid(self):
        """Chequea si el mensaje aún no ha expirado"""
        if self.last_msg is None:
            return False
        return (time.time() - self.last_time) <= self.timeout


class PathMuxNode(Node):
    def __init__(self):
        super().__init__("path_mux_node_yankee")

        # Declarar parámetro como lista vacía por defecto
        self.declare_parameter("topics", [])

        # Leer parámetro como lista de strings
        self.topics = self.get_parameter("topics").get_parameter_value().string_array_value

        if not self.topics:
            self.get_logger().warn("⚠️ No se configuraron tópicos en el parámetro 'topics'")
        else:
            self.get_logger().info(f"Tópicos configurados: {self.topics}")

        # Aquí iría tu lógica de mux (ej: suscribirte a varios y elegir uno)
        self.subscribers = []
        for topic in self.topics:
            sub = self.create_subscription(
                # Ejemplo con tipo genérico, ajusta al que uses (ej: Path, PoseStamped, etc.)
                msg_type=Path,  
                topic=topic,
                callback=self.callback_mux,
                qos_profile=10
            )
            self.subscribers.append(sub)

    def callback_mux(self, msg):
        # Aquí pones tu lógica de multiplexado de trayectorias
        self.get_logger().info(f"Recibido mensaje en el mux: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = PathMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
