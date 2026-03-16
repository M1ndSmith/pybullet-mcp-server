"""Camera rendering functionality for vision simulation."""

from typing import List, Dict, Any, Optional, Tuple
import base64
import numpy as np
import pybullet as p

from .simulation_manager import SimulationManager


class CameraRenderer:
    """Handles camera rendering operations for vision simulation.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self, simulation_manager: SimulationManager):
        """Initialize the camera renderer.
        
        Args:
            simulation_manager: SimulationManager instance for accessing simulations.
        """
        self.simulation_manager = simulation_manager
    
    def compute_view_matrix(
        self,
        camera_eye_position: List[float],
        camera_target_position: List[float],
        camera_up_vector: List[float]
    ) -> List[float]:
        """Compute view matrix from camera pose.
        
        Args:
            camera_eye_position: Camera position [x, y, z].
            camera_target_position: Point camera looks at [x, y, z].
            camera_up_vector: Camera up direction [x, y, z].
            
        Returns:
            View matrix as list of 16 floats (4x4 matrix in row-major order).
            
        Raises:
            ValueError: If invalid parameters.
        """
        # Validate parameters
        if len(camera_eye_position) != 3:
            raise ValueError(f"camera_eye_position must have 3 coordinates, got {len(camera_eye_position)}")
        if len(camera_target_position) != 3:
            raise ValueError(f"camera_target_position must have 3 coordinates, got {len(camera_target_position)}")
        if len(camera_up_vector) != 3:
            raise ValueError(f"camera_up_vector must have 3 coordinates, got {len(camera_up_vector)}")
        
        # Compute view matrix
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_eye_position,
            cameraTargetPosition=camera_target_position,
            cameraUpVector=camera_up_vector
        )
        
        return list(view_matrix)
    
    def compute_projection_matrix(
        self,
        fov: float,
        aspect: float,
        near_plane: float,
        far_plane: float
    ) -> List[float]:
        """Compute projection matrix from camera parameters.
        
        Args:
            fov: Field of view in degrees.
            aspect: Aspect ratio (width / height).
            near_plane: Near clipping plane distance.
            far_plane: Far clipping plane distance.
            
        Returns:
            Projection matrix as list of 16 floats (4x4 matrix in row-major order).
            
        Raises:
            ValueError: If invalid parameters.
        """
        # Validate parameters
        if fov <= 0 or fov >= 180:
            raise ValueError(f"fov must be between 0 and 180 degrees, got {fov}")
        if aspect <= 0:
            raise ValueError(f"aspect must be positive, got {aspect}")
        if near_plane <= 0:
            raise ValueError(f"near_plane must be positive, got {near_plane}")
        if far_plane <= near_plane:
            raise ValueError(f"far_plane must be greater than near_plane, got {far_plane} <= {near_plane}")
        
        # Compute projection matrix
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=fov,
            aspect=aspect,
            nearVal=near_plane,
            farVal=far_plane
        )
        
        return list(projection_matrix)
    
    def get_camera_image(
        self,
        sim_id: str,
        width: int,
        height: int,
        view_matrix: List[float],
        projection_matrix: List[float],
        renderer: str = "ER_BULLET_HARDWARE_OPENGL"
    ) -> Dict[str, Any]:
        """Render RGB, depth, and segmentation images from camera.
        
        Args:
            sim_id: UUID string identifying the simulation.
            width: Image width in pixels.
            height: Image height in pixels.
            view_matrix: View matrix (16 floats from compute_view_matrix).
            projection_matrix: Projection matrix (16 floats from compute_projection_matrix).
            renderer: Renderer type. Options:
                - "ER_BULLET_HARDWARE_OPENGL" (default, fastest)
                - "ER_TINY_RENDERER" (software, slower but more compatible)
            
        Returns:
            Dictionary containing:
                - width: Image width
                - height: Image height
                - rgb: RGB image as base64-encoded PNG
                - depth: Depth buffer as list of floats (near=0, far=1)
                - segmentation: Segmentation mask as list of ints (object IDs)
                
        Raises:
            ValueError: If simulation not found or invalid parameters.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate parameters
        if width <= 0 or height <= 0:
            raise ValueError(f"width and height must be positive, got {width}x{height}")
        if len(view_matrix) != 16:
            raise ValueError(f"view_matrix must have 16 elements, got {len(view_matrix)}")
        if len(projection_matrix) != 16:
            raise ValueError(f"projection_matrix must have 16 elements, got {len(projection_matrix)}")
        
        # Map renderer string to constant
        renderer_map = {
            "ER_BULLET_HARDWARE_OPENGL": p.ER_BULLET_HARDWARE_OPENGL,
            "ER_TINY_RENDERER": p.ER_TINY_RENDERER
        }
        
        if renderer not in renderer_map:
            raise ValueError(
                f"Invalid renderer: {renderer}. "
                f"Must be one of: {list(renderer_map.keys())}"
            )
        
        # Render image
        _, _, rgb, depth, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=renderer_map[renderer],
            physicsClientId=sim.client_id
        )
        
        # Convert RGB to base64-encoded PNG
        rgb_array = np.array(rgb, dtype=np.uint8).reshape(height, width, 4)
        rgb_array = rgb_array[:, :, :3]  # Remove alpha channel
        
        # Convert to bytes for base64 encoding
        import io
        from PIL import Image
        
        img = Image.fromarray(rgb_array)
        buffer = io.BytesIO()
        img.save(buffer, format='PNG')
        rgb_base64 = base64.b64encode(buffer.getvalue()).decode('utf-8')
        
        # Convert depth and segmentation to lists
        depth_array = np.array(depth, dtype=np.float32).reshape(height, width)
        seg_array = np.array(seg, dtype=np.int32).reshape(height, width)
        
        depth_list = depth_array.flatten().tolist()
        seg_list = seg_array.flatten().tolist()
        
        return {
            "width": width,
            "height": height,
            "rgb": rgb_base64,
            "depth": depth_list,
            "segmentation": seg_list
        }
    
    def compute_view_matrix_from_yaw_pitch(
        self,
        distance: float,
        yaw: float,
        pitch: float,
        target_position: List[float],
        up_axis_index: int = 2
    ) -> List[float]:
        """Compute view matrix from spherical coordinates (easier for orbit cameras).
        
        Args:
            distance: Distance from camera to target.
            yaw: Yaw angle in degrees.
            pitch: Pitch angle in degrees.
            target_position: Point camera looks at [x, y, z].
            up_axis_index: Up axis (0=X, 1=Y, 2=Z). Default is 2 (Z-up).
            
        Returns:
            View matrix as list of 16 floats.
            
        Raises:
            ValueError: If invalid parameters.
        """
        # Validate parameters
        if distance <= 0:
            raise ValueError(f"distance must be positive, got {distance}")
        if len(target_position) != 3:
            raise ValueError(f"target_position must have 3 coordinates, got {len(target_position)}")
        if up_axis_index not in [0, 1, 2]:
            raise ValueError(f"up_axis_index must be 0, 1, or 2, got {up_axis_index}")
        
        # Compute view matrix using PyBullet's helper
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target_position,
            distance=distance,
            yaw=yaw,
            pitch=pitch,
            roll=0,
            upAxisIndex=up_axis_index
        )
        
        return list(view_matrix)
