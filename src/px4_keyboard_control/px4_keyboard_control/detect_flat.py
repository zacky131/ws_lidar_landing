import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('point_cloud_visualizer')
        
        # Subscribe to the /glim_ros/map topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/glim_ros/map',
            self.point_cloud_callback,
            10  # QoS Profile
        )
        self.get_logger().info("Subscribed to /glim_ros/map topic")

        # Initialize Matplotlib figure for 3D visualization
        self.fig = plt.figure(figsize=(10, 8))
        self.ax_3d = self.fig.add_subplot(111, projection='3d')

        plt.ion()  # Enable interactive mode

    def point_cloud_callback(self, msg: PointCloud2):
        """Process incoming PointCloud2 data and visualize it in 3D."""
        
        # Convert PointCloud2 to numpy array
        pc_data = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array(list(pc_data))

        # Check if we have valid points
        if points.shape[0] == 0:
            self.get_logger().warning("No valid points received from LiDAR.")
            return

        # Clear the previous plot
        self.ax_3d.clear()

        # Plot the points in 3D (x, y, z)
        self.ax_3d.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c=points[:, 2], cmap='viridis')

        # Set labels and title
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('3D Point Cloud Visualization')
        
        # Display the color bar
        self.fig.colorbar(self.ax_3d.collections[0], ax=self.ax_3d, label='Z Coordinate')

        # Redraw the plot
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    point_cloud_visualizer = PointCloudVisualizer()

    # Run the visualization loop
    rclpy.spin(point_cloud_visualizer)
    
    # Shutdown ROS when finished
    rclpy.shutdown()

if __name__ == '__main__':
    main()
