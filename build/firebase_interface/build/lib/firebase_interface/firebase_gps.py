import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import firebase_admin
from firebase_admin import credentials, db


class FirebaseGPSPublisher(Node):
    def __init__(self):
        super().__init__('firebase_gps_publisher')

        cred = credentials.Certificate('/home/trung/weather_app.json')  
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://map-1-b0eae-default-rtdb.asia-southeast1.firebasedatabase.app/'
        })

        self.publisher_ = self.create_publisher(Point, 'gps_data', 10)
        self.timer = self.create_timer(2.0, self.publish_data)

        self.get_logger().info('Firebase GPS publisher node started.')

    def publish_data(self):
        try:
            ref = db.reference('/Toa-do')
            data = ref.get()

            if data and 'lat' in data and 'lng' in data:
                point_msg = Point()
                point_msg.x = float(data['lat'])
                point_msg.y = float(data['lng'])
                point_msg.z = 0.0

                self.publisher_.publish(point_msg)
                self.get_logger().info(f'Published GPS: lat={point_msg.x}, lng={point_msg.y}')
            else:
                self.get_logger().warn('Toa-do data not found or missing lat/lng')

        except Exception as e:
            self.get_logger().error(f'Error accessing Firebase: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseGPSPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
