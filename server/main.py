import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from std_msgs.msg import String  # Adjust based on the actual message type
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
import threading
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Add the origin of your frontend
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ROSNode(Node):
    def __init__(self):
        super().__init__('map_server')
        self.map_data = None
        self.location_data = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def map_callback(self, msg):
        self.map_data = msg
        print(msg.info.resolution)

    def odom_callback(self, msg):
        self.location_data = msg
        # print(msg.child_frame_id)

ros_node = None

def start_ros_node():
    global ros_node
    rclpy.init(args=None)
    ros_node = ROSNode()
    rclpy.spin(ros_node)
    rclpy.shutdown()

ros_thread = threading.Thread(target=start_ros_node)
ros_thread.start()

@app.get("/api/map")
async def get_map():
    if ros_node.map_data:
        map_info = {
            'info': {
                'width': ros_node.map_data.info.width,
                'height': ros_node.map_data.info.height,
                'origin': {
                    'position': {
                        'x': ros_node.map_data.info.origin.position.x,
                        'y': ros_node.map_data.info.origin.position.y,
                        'z': ros_node.map_data.info.origin.position.z
                    },
                    'orientation': {
                        'x': ros_node.map_data.info.origin.orientation.x,
                        'y': ros_node.map_data.info.origin.orientation.y,
                        'z': ros_node.map_data.info.origin.orientation.z,
                        'w': ros_node.map_data.info.origin.orientation.w
                    }
                },
                'resolution': ros_node.map_data.info.resolution,
            },
            'data': list(ros_node.map_data.data),
        }
        return map_info
    else:
        return {'error': 'No map data available'}

@app.get("/api/location")
async def get_location():
    return ros_node.location_data.child_frame_id

@app.get("/map/select")
async def map_select():
    return {"map selected"}

# Make sure to join the ROS thread when the FastAPI server shuts down
@app.on_event("shutdown")
def shutdown_event():
    ros_thread.join()
