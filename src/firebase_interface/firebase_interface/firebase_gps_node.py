import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import firebase_admin
from firebase_admin import credentials, db
import numpy as np

class FirebaseGPSPublisher(Node):
    def __init__(self):
        super().__init__('firebase_gps_publisher')

        cred = credentials.Certificate('/home/trung/weather_app.json')  
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://map-1-b0eae-default-rtdb.asia-southeast1.firebasedatabase.app/'
        })

        self.lat_publisher = self.create_publisher(Float32, 'lat', 10)
        self.lng_publisher = self.create_publisher(Float32, 'lng', 10)
        self.timer = self.create_timer(2.0, self.publish_data)

        self.get_logger().info('Firebase GPS publisher node started.')

    def publish_data(self):
        try:
            ref = db.reference('/Toa-do')
            data = ref.get()

            if data and 'lat' in data and 'lng' in data:
                lat = np.float(data['lat'])
                lng = np.float(data['lng'])

                lat_msg = Float32()
                lng_msg = Float32()

                lat_msg.data = lat
                lng_msg.data = lng

                self.lat_publisher.publish(lat_msg)
                self.lng_publisher.publish(lng_msg)
                self.get_logger().info(f'GPS: lat={lat}, lng={lng}')
            else:
                self.get_logger().warn('Data not found or missing lat/lng')

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
