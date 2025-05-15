import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import firebase_admin
from firebase_admin import credentials, db


class FirebaseGPSPublisher(Node):
    def __init__(self):
        super().__init__('firebase_gps_publisher')

        cred = credentials.Certificate('/home/trung/weather_app.json')  
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://map-1-b0eae-default-rtdb.asia-southeast1.firebasedatabase.app/'
        })

        self.lat_pubs = [
            self.create_publisher(Float32, 'lat1', 10),
            self.create_publisher(Float32, 'lat2', 10),
            self.create_publisher(Float32, 'lat3', 10),
        ]
        self.lng_pubs = [
            self.create_publisher(Float32, 'lng1', 10),
            self.create_publisher(Float32, 'lng2', 10),
            self.create_publisher(Float32, 'lng3', 10),
        ]

        self.timer = self.create_timer(2.0, self.publish_data)
        self.get_logger().info('Firebase GPS publisher node started.')

    def publish_data(self):
        try:
            for i in range(1, 4):
                ref = db.reference(f'/Toa-do-{i}')
                data = ref.get()

                if data and 'lat' in data and 'lng' in data:
                    lat = float(data['lat'])
                    lng = float(data['lng'])

                    lat_msg = Float32()
                    lng_msg = Float32()

                    lat_msg.data = lat
                    lng_msg.data = lng

                    self.lat_pubs[i - 1].publish(lat_msg)
                    self.lng_pubs[i - 1].publish(lng_msg)

                    self.get_logger().info(f'Toa-do-{i} - lat={lat}, lng={lng}')
                else:
                    self.get_logger().warn(f'Toa-do-{i} missing lat/lng or not found')

                if i==3:
                    print("********\n")

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
