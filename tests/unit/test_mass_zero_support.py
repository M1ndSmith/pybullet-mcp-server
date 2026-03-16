"""Tests for mass=0 support (static objects)."""

import pytest
import pybullet as p
from src.simulation_manager import SimulationManager
from src.object_manager import ObjectManager


@pytest.fixture
def managers():
    """Create manager instances for testing."""
    sim_manager = SimulationManager()
    obj_manager = ObjectManager(sim_manager)
    return sim_manager, obj_manager


@pytest.fixture
def simulation(managers):
    """Create a test simulation."""
    sim_manager, _ = managers
    sim_id = sim_manager.create_simulation()
    yield sim_id
    # Cleanup
    if sim_manager.has_simulation(sim_id):
        sim_manager.destroy_simulation(sim_id)


class TestMassZeroSupport:
    """Test mass=0 support for creating static objects."""
    
    def test_create_box_with_mass_zero(self, managers, simulation):
        """Test creating a box with mass=0 (static object)."""
        sim_manager, obj_manager = managers
        
        # Create box with mass=0
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[5.0, 5.0, 0.1],
            position=[0, 0, 0],
            mass=0.0  # Static object
        )
        
        assert object_id >= 0
        
        # Verify object exists
        sim = sim_manager.get_simulation(simulation)
        assert object_id in sim.objects
        assert sim.objects[object_id]["mass"] == 0.0
    
    def test_create_sphere_with_mass_zero(self, managers, simulation):
        """Test creating a sphere with mass=0."""
        sim_manager, obj_manager = managers
        
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=0.0
        )
        
        assert object_id >= 0
        sim = sim_manager.get_simulation(simulation)
        assert sim.objects[object_id]["mass"] == 0.0
    
    def test_create_cylinder_with_mass_zero(self, managers, simulation):
        """Test creating a cylinder with mass=0."""
        sim_manager, obj_manager = managers
        
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="cylinder",
            dimensions=[0.5, 1.0],
            position=[0, 0, 1],
            mass=0.0
        )
        
        assert object_id >= 0
        sim = sim_manager.get_simulation(simulation)
        assert sim.objects[object_id]["mass"] == 0.0
    
    def test_create_capsule_with_mass_zero(self, managers, simulation):
        """Test creating a capsule with mass=0."""
        sim_manager, obj_manager = managers
        
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="capsule",
            dimensions=[0.3, 0.8],
            position=[0, 0, 1],
            mass=0.0
        )
        
        assert object_id >= 0
        sim = sim_manager.get_simulation(simulation)
        assert sim.objects[object_id]["mass"] == 0.0
    
    def test_static_object_does_not_move(self, managers, simulation):
        """Test that mass=0 objects don't move under gravity."""
        sim_manager, obj_manager = managers
        
        # Create static ground plane
        ground_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[5.0, 5.0, 0.1],
            position=[0, 0, 0],
            mass=0.0  # Static
        )
        
        # Get initial position
        sim = sim_manager.get_simulation(simulation)
        initial_pos, _ = p.getBasePositionAndOrientation(ground_id, physicsClientId=sim.client_id)
        
        # Step simulation
        for _ in range(100):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final position
        final_pos, _ = p.getBasePositionAndOrientation(ground_id, physicsClientId=sim.client_id)
        
        # Static object should not move
        assert initial_pos[0] == pytest.approx(final_pos[0], abs=1e-6)
        assert initial_pos[1] == pytest.approx(final_pos[1], abs=1e-6)
        assert initial_pos[2] == pytest.approx(final_pos[2], abs=1e-6)
    
    def test_dynamic_object_falls_on_static_ground(self, managers, simulation):
        """Test that dynamic objects fall and collide with static ground."""
        sim_manager, obj_manager = managers
        
        # Create static ground
        ground_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[5.0, 5.0, 0.1],
            position=[0, 0, 0],
            mass=0.0  # Static
        )
        
        # Create dynamic box above ground
        box_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 2],
            mass=1.0  # Dynamic
        )
        
        sim = sim_manager.get_simulation(simulation)
        
        # Get initial position of dynamic box
        initial_pos, _ = p.getBasePositionAndOrientation(box_id, physicsClientId=sim.client_id)
        
        # Step simulation
        for _ in range(200):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final position
        final_pos, _ = p.getBasePositionAndOrientation(box_id, physicsClientId=sim.client_id)
        
        # Dynamic box should have fallen
        assert final_pos[2] < initial_pos[2]
        
        # Box should be resting on ground (z ≈ 0.5, which is ground + half box height)
        assert final_pos[2] == pytest.approx(0.6, abs=0.2)  # Allow some tolerance
    
    def test_negative_mass_raises_error(self, managers, simulation):
        """Test that negative mass raises ValueError."""
        _, obj_manager = managers
        
        with pytest.raises(ValueError, match="Mass must be non-negative"):
            obj_manager.create_primitive(
                sim_id=simulation,
                shape="box",
                dimensions=[1.0, 1.0, 1.0],
                position=[0, 0, 1],
                mass=-1.0  # Invalid
            )
    
    def test_mass_zero_with_all_parameters(self, managers, simulation):
        """Test mass=0 with color, friction, and restitution."""
        sim_manager, obj_manager = managers
        
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[1.0, 1.0, 0.1],
            position=[0, 0, 0],
            mass=0.0,
            color=[0.5, 0.5, 0.5, 1.0],
            friction=0.8,
            restitution=0.2
        )
        
        assert object_id >= 0
        sim = sim_manager.get_simulation(simulation)
        metadata = sim.objects[object_id]
        
        assert metadata["mass"] == 0.0
        assert metadata["friction"] == 0.8
        assert metadata["restitution"] == 0.2
        assert metadata["color"] == [0.5, 0.5, 0.5, 1.0]
