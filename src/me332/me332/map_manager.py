#!/usr/bin/env python3
"""
Map Manager Module
Provides services for saving, loading, and managing SLAM maps
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
from slam_toolbox.srv import SaveMap, SerializePoseGraph, DeserializePoseGraph
import os
from ament_index_python.packages import get_package_share_directory


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        
        # Get package path
        self.me332_path = get_package_share_directory("me332")
        self.maps_dir = os.path.join(self.me332_path, 'maps')
        
        # Create maps directory if it doesn't exist
        os.makedirs(self.maps_dir, exist_ok=True)
        
        # Default map file path
        self.default_map_name = 'map'
        self.default_map_path = os.path.join(self.maps_dir, self.default_map_name)
        
        # Service clients for slam_toolbox
        self.save_map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.serialize_pose_graph_client = self.create_client(
            SerializePoseGraph, 
            '/slam_toolbox/serialize_map'
        )
        self.deserialize_pose_graph_client = self.create_client(
            DeserializePoseGraph,
            '/slam_toolbox/deserialize_map'
        )
        
        # Wait for services to be available
        self.get_logger().info('Waiting for slam_toolbox services...')
        if not self.save_map_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('slam_toolbox save_map service not available')
        if not self.serialize_pose_graph_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('slam_toolbox serialize_map service not available')
        
        # Service servers for map management
        self.save_map_service = self.create_service(
            Trigger,
            '/map_manager/save_map',
            self.save_map_callback
        )
        
        self.load_map_service = self.create_service(
            Trigger,
            '/map_manager/load_map',
            self.load_map_callback
        )
        
        # Parameter for map file name
        self.declare_parameter('map_name', self.default_map_name)
        
        self.get_logger().info('Map Manager initialized')
        self.get_logger().info(f'Maps directory: {self.maps_dir}')
    
    def save_map_callback(self, request, response):
        """Save the current map"""
        try:
            map_name = self.get_parameter('map_name').get_parameter_value().string_value
            if not map_name:
                map_name = self.default_map_name
            
            map_path = os.path.join(self.maps_dir, map_name)
            
            # Call slam_toolbox save_map service
            save_request = SaveMap.Request()
            save_request.name.data = map_path
            
            if not self.save_map_client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = 'slam_toolbox save_map service not available'
                self.get_logger().error(response.message)
                return response
            
            future = self.save_map_client.call_async(save_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                save_response = future.result()
                if save_response:
                    response.success = True
                    response.message = f'Map saved successfully to {map_path}.yaml'
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = 'Failed to save map: No response from slam_toolbox'
                    self.get_logger().error(response.message)
            else:
                response.success = False
                response.message = 'Failed to save map: Service call timeout'
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f'Error saving map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def load_map_callback(self, request, response):
        """Load a saved map"""
        try:
            map_name = self.get_parameter('map_name').get_parameter_value().string_value
            if not map_name:
                map_name = self.default_map_name
            
            map_path = os.path.join(self.maps_dir, map_name)
            pose_graph_path = f'{map_path}.posegraph'
            
            # Check if map files exist
            yaml_path = f'{map_path}.yaml'
            if not os.path.exists(yaml_path):
                response.success = False
                response.message = f'Map file not found: {yaml_path}'
                self.get_logger().error(response.message)
                return response
            
            # Call slam_toolbox deserialize_map service
            if not self.deserialize_pose_graph_client.wait_for_service(timeout_sec=2.0):
                response.success = False
                response.message = 'slam_toolbox deserialize_map service not available'
                self.get_logger().error(response.message)
                return response
            
            load_request = DeserializePoseGraph.Request()
            load_request.filename.data = pose_graph_path
            
            future = self.deserialize_pose_graph_client.call_async(load_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                load_response = future.result()
                if load_response:
                    response.success = True
                    response.message = f'Map loaded successfully from {yaml_path}'
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = 'Failed to load map: No response from slam_toolbox'
                    self.get_logger().error(response.message)
            else:
                response.success = False
                response.message = 'Failed to load map: Service call timeout'
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f'Error loading map: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def save_map_with_name(self, map_name):
        """Save map with a specific name"""
        from rclpy.parameter import Parameter
        old_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.set_parameters([Parameter('map_name', Parameter.Type.STRING, map_name)])
        
        request = Trigger.Request()
        response = self.save_map_callback(request, Trigger.Response())
        
        # Restore old name
        self.set_parameters([Parameter('map_name', Parameter.Type.STRING, old_name)])
        
        return response


def main(args=None):
    rclpy.init(args=args)
    map_manager = MapManager()
    
    try:
        rclpy.spin(map_manager)
    except KeyboardInterrupt:
        pass
    finally:
        map_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

