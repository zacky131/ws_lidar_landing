import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import pyvista as pv

class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('pointcloud_visualizer')
        
        # Subscription to the ROS2 PointCloud2 topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/glim_ros/map',  # Replace with your actual topic name
            self.listener_callback,
            10
        )
        
        # Create the PyVista plotter for visualization
        self.plotter = pv.Plotter()
        self.plotter.add_axes()
        self.plotter.show_grid()

        # Initialize an empty point cloud
        self.pcd = pv.PolyData()

        # Open the PyVista visualization window
        self.plotter.show()

    def listener_callback(self, msg):
        """Callback function to process incoming PointCloud2 messages."""
        
        # Convert PointCloud2 message to numpy array
        pc_data = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array(list(pc_data))

        # Check if we have valid points
        if points.shape[0] == 0:
            self.get_logger().warning("No valid points received from LiDAR.")
            return

        # Update the point cloud data for PyVista visualization
        self.pcd.points = points

        # Update the point cloud visualization
        self.update_point_cloud()

    def update_point_cloud(self):
        """Update the point cloud visualization in PyVista."""
        # Set the color of the points based on their Z-coordinate (height)
        self.pcd.point_arrays["z"] = self.pcd.points[:, 2]
        
        # Plot the point cloud
        self.plotter.update_coordinates(self.pcd.points)
        self.plotter.update_scalars(self.pcd.point_arrays["z"])
        self.plotter.render()

def main(args=None):
    """Main function to run the ROS2 node and visualization."""
    rclpy.init(args=args)
    point_cloud_visualizer = PointCloudVisualizer()

    try:
        # Run the ROS2 spin loop to process messages
        rclpy.spin(point_cloud_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        # Properly shutdown ROS2 and PyVista plotter
        point_cloud_visualizer.plotter.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
