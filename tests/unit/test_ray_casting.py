"""Tests for ray casting functionality."""

import pytest
import math
from src.simulation_manager import SimulationManager
from src.object_manager import ObjectManager
from src.ray_casting import RayCastingHandler


@pytest.fixture
def managers():
    """Create manager instances for testing."""
    sim_manager = SimulationManager()
    obj_manager = ObjectManager(sim_manager)
    ray_handler = RayCastingHandler(sim_manager)
    return sim_manager, obj_manager, ray_handler


@pytest.fixture
def simulation(managers):
    """Create a test simulation."""
    sim_manager, _, _ = managers
    sim_id = sim_manager.create_simulation()
    yield sim_id
    # Cleanup
    if sim_manager.has_simulation(sim_id):
        sim_manager.destroy_simulation(sim_id)


class TestRayTest:
    """Test single ray casting."""
    
    def test_ray_hits_object(self, managers, simulation):
        """Test that ray detects an object in its path."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object at position [5, 0, 1]
        obj_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Cast ray from [0, 0, 1] to [10, 0, 1] (should hit object)
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Verify hit
        assert result["hit"] is True
        assert result["object_id"] == obj_id
        assert 0 < result["hit_fraction"] < 1
        assert len(result["hit_position"]) == 3
        assert len(result["hit_normal"]) == 3
    
    def test_ray_misses_object(self, managers, simulation):
        """Test that ray returns no hit when missing objects."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object at position [5, 5, 1] (off to the side)
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[5, 5, 1],
            mass=1.0
        )
        
        # Cast ray from [0, 0, 1] to [10, 0, 1] (should miss)
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Verify no hit
        assert result["hit"] is False
        assert result["object_id"] == -1
    
    def test_ray_hit_fraction(self, managers, simulation):
        """Test that hit_fraction correctly indicates distance."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object at position [5, 0, 1]
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Cast ray from [0, 0, 1] to [10, 0, 1]
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Hit fraction should be around 0.5 (object at midpoint)
        assert result["hit"] is True
        assert 0.4 < result["hit_fraction"] < 0.6
    
    def test_ray_hit_position(self, managers, simulation):
        """Test that hit_position is accurate."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Cast ray
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Hit position should be near [5, 0, 1]
        assert result["hit"] is True
        hit_pos = result["hit_position"]
        assert 4 < hit_pos[0] < 6  # x near 5
        assert -1 < hit_pos[1] < 1  # y near 0
        assert 0 < hit_pos[2] < 2  # z near 1
    
    def test_ray_invalid_coordinates_raises_error(self, managers, simulation):
        """Test that invalid ray coordinates raise error."""
        _, _, ray_handler = managers
        
        # Try with wrong number of coordinates
        with pytest.raises(ValueError, match="ray_from must have 3 coordinates"):
            ray_handler.ray_test(
                sim_id=simulation,
                ray_from=[0, 0],  # Only 2 coordinates
                ray_to=[10, 0, 1]
            )
        
        with pytest.raises(ValueError, match="ray_to must have 3 coordinates"):
            ray_handler.ray_test(
                sim_id=simulation,
                ray_from=[0, 0, 1],
                ray_to=[10, 0]  # Only 2 coordinates
            )


class TestRayTestBatch:
    """Test batch ray casting."""
    
    def test_batch_multiple_rays(self, managers, simulation):
        """Test casting multiple rays at once."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Cast 3 rays
        rays_from = [
            [0, 0, 1],
            [0, 0, 1],
            [0, 0, 1]
        ]
        rays_to = [
            [10, 0, 1],  # Should hit
            [10, 5, 1],  # Should miss
            [10, 0, 5]   # Should miss
        ]
        
        results = ray_handler.ray_test_batch(
            sim_id=simulation,
            rays_from=rays_from,
            rays_to=rays_to
        )
        
        # Verify results
        assert len(results) == 3
        assert results[0]["hit"] is True  # First ray hits
        assert results[1]["hit"] is False  # Second ray misses
        assert results[2]["hit"] is False  # Third ray misses
    
    def test_batch_lidar_simulation(self, managers, simulation):
        """Test simulating a 360-degree lidar."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object in front
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Simulate 36-ray lidar (10 degree increments)
        num_rays = 36
        rays_from = [[0, 0, 1]] * num_rays
        rays_to = []
        
        for i in range(num_rays):
            angle = i * (2 * math.pi / num_rays)
            rays_to.append([
                10 * math.cos(angle),
                10 * math.sin(angle),
                1
            ])
        
        results = ray_handler.ray_test_batch(
            sim_id=simulation,
            rays_from=rays_from,
            rays_to=rays_to
        )
        
        # Verify we got results for all rays
        assert len(results) == num_rays
        
        # At least one ray should hit the object
        hits = sum(1 for r in results if r["hit"])
        assert hits > 0
    
    def test_batch_empty_arrays(self, managers, simulation):
        """Test batch with empty arrays."""
        _, _, ray_handler = managers
        
        results = ray_handler.ray_test_batch(
            sim_id=simulation,
            rays_from=[],
            rays_to=[]
        )
        
        assert results == []
    
    def test_batch_mismatched_lengths_raises_error(self, managers, simulation):
        """Test that mismatched array lengths raise error."""
        _, _, ray_handler = managers
        
        with pytest.raises(ValueError, match="rays_from and rays_to must have same length"):
            ray_handler.ray_test_batch(
                sim_id=simulation,
                rays_from=[[0, 0, 1], [0, 0, 1]],
                rays_to=[[10, 0, 1]]  # Only 1 element
            )
    
    def test_batch_invalid_coordinates_raises_error(self, managers, simulation):
        """Test that invalid coordinates in batch raise error."""
        _, _, ray_handler = managers
        
        with pytest.raises(ValueError, match="rays_from\\[0\\] must have 3 coordinates"):
            ray_handler.ray_test_batch(
                sim_id=simulation,
                rays_from=[[0, 0]],  # Only 2 coordinates
                rays_to=[[10, 0, 1]]
            )


class TestRayCastingWorkflows:
    """Test complete workflows with ray casting."""
    
    def test_proximity_sensor(self, managers, simulation):
        """Test using ray casting as a proximity sensor."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create wall
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[5.0, 0.1, 2.0],
            position=[3, 0, 1],
            mass=0.0  # Static
        )
        
        # Cast ray forward
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Calculate distance
        if result["hit"]:
            distance = result["hit_fraction"] * 10
            assert distance < 5  # Wall is within 5 meters
    
    def test_line_of_sight_check(self, managers, simulation):
        """Test checking if two points have line of sight."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create obstacle between two points
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 1.0],
            position=[5, 0, 1],
            mass=1.0
        )
        
        # Check line of sight from [0,0,1] to [10,0,1]
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 0, 1]
        )
        
        # Line of sight is blocked
        assert result["hit"] is True
        
        # Check line of sight from [0,0,1] to [10,5,1] (should be clear)
        result2 = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[10, 5, 1]
        )
        
        # Line of sight is clear
        assert result2["hit"] is False
    
    def test_distance_measurement(self, managers, simulation):
        """Test measuring distance to nearest obstacle."""
        sim_manager, obj_manager, ray_handler = managers
        
        # Create object at known distance
        obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[7, 0, 1],
            mass=1.0
        )
        
        # Measure distance
        result = ray_handler.ray_test(
            sim_id=simulation,
            ray_from=[0, 0, 1],
            ray_to=[20, 0, 1]
        )
        
        if result["hit"]:
            distance = result["hit_fraction"] * 20
            # Object is at 7m, sphere radius is 0.5m, so hit should be around 6.5m
            assert 6 < distance < 8
