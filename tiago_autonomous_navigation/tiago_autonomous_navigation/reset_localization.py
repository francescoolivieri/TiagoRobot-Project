import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

# NOTICE: at the moment UNUSED

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('reinitialize_global_localization_client')

        self.cli = self.create_client(
            Empty,
            'reinitialize_global_localization'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = Empty.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)

        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Call failed %r' % (e,)
                )
            else:
                minimal_client.get_logger().info('Call succeeded!')
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
