"""Tests for camera rendering functionality."""

import pytest
import base64
import io
from PIL import Image
from src.simulation_manager import SimulationManager
from src.object_manager import ObjectManager
from src.camera_rendering import CameraRenderer


@pytest.fixture
def managers():
    """Create manager instances for testing."""
    sim_manager = SimulationManager()
    obj_manager = ObjectManager(sim_manager)
    camera_renderer = CameraRenderer(sim_manager)
    return sim_manager, obj_manager, camera_renderer


@pytest.fixture
def simulation(managers):
    """Create a test simulation."""
    sim_manager, _, _ = managers
    sim_id = sim_manager.create_simulation()
    yield sim_id
    # Cleanup
    if sim_manager.has_simulation(sim_id):
        sim_manager.destroy_simulation(sim_id)


class TestComputeViewMatrix:
    """Test view matrix computation."""
    
    def test_compute_view_matrix(self, managers):
        """Test computing view matrix from camera pose."""
        _, _, camera_renderer = managers
        
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[5, 5, 3],
            camera_target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1]
        )
        
        # Should return 16 floats (4x4 matrix)
        assert len(view_matrix) == 16
        assert all(isinstance(x, float) for x in view_matrix)
    
    def test_view_matrix_invalid_eye_position(self, managers):
        """Test that invalid eye position raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="camera_eye_position must have 3 coordinates"):
            camera_renderer.compute_view_matrix(
                camera_eye_position=[5, 5],  # Only 2 coordinates
                camera_target_position=[0, 0, 0],
                camera_up_vector=[0, 0, 1]
            )
    
    def test_view_matrix_invalid_target_position(self, managers):
        """Test that invalid target position raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="camera_target_position must have 3 coordinates"):
            camera_renderer.compute_view_matrix(
                camera_eye_position=[5, 5, 3],
                camera_target_position=[0, 0],  # Only 2 coordinates
                camera_up_vector=[0, 0, 1]
            )


class TestComputeProjectionMatrix:
    """Test projection matrix computation."""
    
    def test_compute_projection_matrix(self, managers):
        """Test computing projection matrix."""
        _, _, camera_renderer = managers
        
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60,
            aspect=640/480,
            near_plane=0.1,
            far_plane=100
        )
        
        # Should return 16 floats (4x4 matrix)
        assert len(proj_matrix) == 16
        assert all(isinstance(x, float) for x in proj_matrix)
    
    def test_projection_matrix_invalid_fov(self, managers):
        """Test that invalid FOV raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="fov must be between 0 and 180"):
            camera_renderer.compute_projection_matrix(
                fov=200,  # Too large
                aspect=1.33,
                near_plane=0.1,
                far_plane=100
            )
    
    def test_projection_matrix_invalid_aspect(self, managers):
        """Test that invalid aspect ratio raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="aspect must be positive"):
            camera_renderer.compute_projection_matrix(
                fov=60,
                aspect=-1.0,  # Negative
                near_plane=0.1,
                far_plane=100
            )
    
    def test_projection_matrix_invalid_planes(self, managers):
        """Test that invalid clipping planes raise error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="far_plane must be greater than near_plane"):
            camera_renderer.compute_projection_matrix(
                fov=60,
                aspect=1.33,
                near_plane=100,  # Far > near
                far_plane=0.1
            )


class TestComputeViewMatrixFromYawPitch:
    """Test orbit camera view matrix computation."""
    
    def test_compute_view_matrix_from_yaw_pitch(self, managers):
        """Test computing view matrix from spherical coordinates."""
        _, _, camera_renderer = managers
        
        view_matrix = camera_renderer.compute_view_matrix_from_yaw_pitch(
            distance=5,
            yaw=45,
            pitch=-30,
            target_position=[0, 0, 0]
        )
        
        # Should return 16 floats
        assert len(view_matrix) == 16
        assert all(isinstance(x, float) for x in view_matrix)
    
    def test_yaw_pitch_invalid_distance(self, managers):
        """Test that invalid distance raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="distance must be positive"):
            camera_renderer.compute_view_matrix_from_yaw_pitch(
                distance=-5,  # Negative
                yaw=45,
                pitch=-30,
                target_position=[0, 0, 0]
            )
    
    def test_yaw_pitch_invalid_up_axis(self, managers):
        """Test that invalid up axis raises error."""
        _, _, camera_renderer = managers
        
        with pytest.raises(ValueError, match="up_axis_index must be 0, 1, or 2"):
            camera_renderer.compute_view_matrix_from_yaw_pitch(
                distance=5,
                yaw=45,
                pitch=-30,
                target_position=[0, 0, 0],
                up_axis_index=3  # Invalid
            )


class TestGetCameraImage:
    """Test camera image rendering."""
    
    def test_render_camera_image(self, managers, simulation):
        """Test rendering RGB, depth, and segmentation images."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create object to render
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Setup camera
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[5, 5, 3],
            camera_target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60,
            aspect=640/480,
            near_plane=0.1,
            far_plane=100
        )
        
        # Render image
        result = camera_renderer.get_camera_image(
            sim_id=simulation,
            width=640,
            height=480,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )
        
        # Verify structure
        assert "width" in result
        assert "height" in result
        assert "rgb" in result
        assert "depth" in result
        assert "segmentation" in result
        
        # Verify dimensions
        assert result["width"] == 640
        assert result["height"] == 480
        
        # Verify RGB is base64 string
        assert isinstance(result["rgb"], str)
        assert len(result["rgb"]) > 0
        
        # Verify depth is list of floats
        assert isinstance(result["depth"], list)
        assert len(result["depth"]) == 640 * 480
        
        # Verify segmentation is list of ints
        assert isinstance(result["segmentation"], list)
        assert len(result["segmentation"]) == 640 * 480
    
    def test_decode_rgb_image(self, managers, simulation):
        """Test that RGB image can be decoded."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create colorful object
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[1.0],
            position=[0, 0, 1],
            mass=1.0,
            color=[1.0, 0.0, 0.0, 1.0]  # Red
        )
        
        # Setup camera
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[3, 0, 1],
            camera_target_position=[0, 0, 1],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60,
            aspect=1.0,
            near_plane=0.1,
            far_plane=100
        )
        
        # Render image
        result = camera_renderer.get_camera_image(
            sim_id=simulation,
            width=320,
            height=320,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )
        
        # Decode RGB image
        rgb_data = base64.b64decode(result["rgb"])
        img = Image.open(io.BytesIO(rgb_data))
        
        # Verify image properties
        assert img.size == (320, 320)
        assert img.mode == "RGB"
    
    def test_depth_buffer_values(self, managers, simulation):
        """Test that depth buffer contains valid values."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create object
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Setup camera
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[5, 0, 1],
            camera_target_position=[0, 0, 1],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60,
            aspect=1.0,
            near_plane=0.1,
            far_plane=100
        )
        
        # Render image
        result = camera_renderer.get_camera_image(
            sim_id=simulation,
            width=64,
            height=64,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )
        
        # Verify depth values are in valid range [0, 1]
        depth = result["depth"]
        assert all(0 <= d <= 1 for d in depth)
    
    def test_segmentation_mask(self, managers, simulation):
        """Test that segmentation mask contains object IDs."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create object
        obj_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Setup camera
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[3, 0, 1],
            camera_target_position=[0, 0, 1],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60,
            aspect=1.0,
            near_plane=0.1,
            far_plane=100
        )
        
        # Render image
        result = camera_renderer.get_camera_image(
            sim_id=simulation,
            width=64,
            height=64,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )
        
        # Segmentation should contain object ID
        seg = result["segmentation"]
        assert obj_id in seg
    
    def test_invalid_dimensions_raise_error(self, managers, simulation):
        """Test that invalid image dimensions raise error."""
        _, _, camera_renderer = managers
        
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[5, 5, 3],
            camera_target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60, aspect=1.0, near_plane=0.1, far_plane=100
        )
        
        with pytest.raises(ValueError, match="width and height must be positive"):
            camera_renderer.get_camera_image(
                sim_id=simulation,
                width=-640,  # Negative
                height=480,
                view_matrix=view_matrix,
                projection_matrix=proj_matrix
            )
    
    def test_invalid_view_matrix_raises_error(self, managers, simulation):
        """Test that invalid view matrix raises error."""
        _, _, camera_renderer = managers
        
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60, aspect=1.0, near_plane=0.1, far_plane=100
        )
        
        with pytest.raises(ValueError, match="view_matrix must have 16 elements"):
            camera_renderer.get_camera_image(
                sim_id=simulation,
                width=640,
                height=480,
                view_matrix=[1, 2, 3],  # Only 3 elements
                projection_matrix=proj_matrix
            )


class TestCameraWorkflows:
    """Test complete camera workflows."""
    
    def test_vision_based_control(self, managers, simulation):
        """Test using camera for vision-based control."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create target object
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[2, 0, 1],
            mass=1.0,
            color=[1.0, 0.0, 0.0, 1.0]  # Red target
        )
        
        # Setup camera on robot
        view_matrix = camera_renderer.compute_view_matrix(
            camera_eye_position=[0, 0, 1],
            camera_target_position=[2, 0, 1],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = camera_renderer.compute_projection_matrix(
            fov=60, aspect=1.0, near_plane=0.1, far_plane=100
        )
        
        # Capture image
        result = camera_renderer.get_camera_image(
            sim_id=simulation,
            width=320,
            height=320,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )
        
        # Verify we got an image
        assert result["width"] == 320
        assert result["height"] == 320
        assert len(result["rgb"]) > 0
    
    def test_orbit_camera(self, managers, simulation):
        """Test orbit camera around object."""
        sim_manager, obj_manager, camera_renderer = managers
        
        # Create object to orbit
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Orbit camera at different angles
        for yaw in [0, 90, 180, 270]:
            view_matrix = camera_renderer.compute_view_matrix_from_yaw_pitch(
                distance=5,
                yaw=yaw,
                pitch=-30,
                target_position=[0, 0, 1]
            )
            proj_matrix = camera_renderer.compute_projection_matrix(
                fov=60, aspect=1.0, near_plane=0.1, far_plane=100
            )
            
            result = camera_renderer.get_camera_image(
                sim_id=simulation,
                width=128,
                height=128,
                view_matrix=view_matrix,
                projection_matrix=proj_matrix
            )
            
            # Should get valid image at each angle
            assert len(result["rgb"]) > 0
