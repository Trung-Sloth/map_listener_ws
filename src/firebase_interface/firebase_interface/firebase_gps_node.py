import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import firebase_admin
from firebase_admin import credentials, db


class FirebaseUAVNode(Node):
    def __init__(self):
        super().__init__('firebase_uav_node')

        cred = credentials.Certificate('/home/trung/weather_app.json') 
        firebase_admin.initialize_app(cred, {
            'databaseURL': 'https://map-1-b0eae-default-rtdb.asia-southeast1.firebasedatabase.app/'
        })

        self.point_pubs = [
            self.create_publisher(Point, 'point1', 10),
            self.create_publisher(Point, 'point2', 10),
            self.create_publisher(Point, 'point3', 10),
        ]

        self.timer = self.create_timer(1.0, self.publish_from_firebase)

        self.subscription = self.create_subscription(
            Point,
            'uav_position',
            self.uav_callback,
            10
        )

    def publish_from_firebase(self):
        try:
            for i in range(1, 4):
                ref = db.reference(f'/Toa-do-{i}')
                data = ref.get()

                if data and 'lat' in data and 'lng' in data:
                    lat = float(data['lat'])
                    lng = float(data['lng'])

                    msg = Point()
                    msg.x = lat
                    msg.y = lng
                    msg.z = 0.0

                    self.point_pubs[i - 1].publish(msg)
                    self.get_logger().info(f'Toa-do-{i}: x={lat}, y={lng}')
                else:
                    self.get_logger().warn(f'No data: Toa-do-{i}')
            print("********\n")

        except Exception as e:
            self.get_logger().error(f'Error reading data from Firebase: {e}')

    def uav_callback(self, msg):
        lat_cur = msg.x
        lng_cur = msg.y
        #status  = msg.z
        try:
            db.reference('/Toa-do-hien-tai').set({
                'lat_cur': lat_cur,
                'lng_cur': lng_cur,
                #'status' : status
            })
            self.get_logger().info(f'Toa-do-hien-tai: lat_cur={lat_cur}, lng_cur={lng_cur}, status={status}')
        except Exception as e:
            self.get_logger().error(f'Error sending data to Firebase: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseUAVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
