"""Tests for object velocity control."""

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


class TestVelocityControl:
    """Test object velocity control functionality."""
    
    def test_set_linear_velocity(self, managers, simulation):
        """Test setting linear velocity on an object."""
        sim_manager, obj_manager = managers
        
        # Create a box
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set linear velocity
        target_velocity = [5.0, 0.0, 0.0]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=target_velocity
        )
        
        # Verify velocity was set
        sim = sim_manager.get_simulation(simulation)
        lin_vel, _ = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        assert lin_vel[0] == pytest.approx(target_velocity[0], abs=1e-6)
        assert lin_vel[1] == pytest.approx(target_velocity[1], abs=1e-6)
        assert lin_vel[2] == pytest.approx(target_velocity[2], abs=1e-6)
    
    def test_set_angular_velocity(self, managers, simulation):
        """Test setting angular velocity on an object."""
        sim_manager, obj_manager = managers
        
        # Create a sphere
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set angular velocity
        target_velocity = [0.0, 0.0, 3.14]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            angular_velocity=target_velocity
        )
        
        # Verify velocity was set
        sim = sim_manager.get_simulation(simulation)
        _, ang_vel = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        assert ang_vel[0] == pytest.approx(target_velocity[0], abs=1e-6)
        assert ang_vel[1] == pytest.approx(target_velocity[1], abs=1e-6)
        assert ang_vel[2] == pytest.approx(target_velocity[2], abs=1e-6)
    
    def test_set_both_velocities(self, managers, simulation):
        """Test setting both linear and angular velocities simultaneously."""
        sim_manager, obj_manager = managers
        
        # Create a cylinder
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="cylinder",
            dimensions=[0.3, 1.0],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set both velocities
        target_lin_vel = [2.0, 3.0, 1.0]
        target_ang_vel = [0.5, 0.5, 1.0]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=target_lin_vel,
            angular_velocity=target_ang_vel
        )
        
        # Verify both velocities were set
        sim = sim_manager.get_simulation(simulation)
        lin_vel, ang_vel = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        for i in range(3):
            assert lin_vel[i] == pytest.approx(target_lin_vel[i], abs=1e-6)
            assert ang_vel[i] == pytest.approx(target_ang_vel[i], abs=1e-6)
    
    def test_stop_object(self, managers, simulation):
        """Test stopping a moving object by setting velocities to zero."""
        sim_manager, obj_manager = managers
        
        # Create a box with initial velocity
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Give it some velocity
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=[10.0, 5.0, 2.0],
            angular_velocity=[1.0, 2.0, 3.0]
        )
        
        # Stop it
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=[0.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.0]
        )
        
        # Verify it stopped
        sim = sim_manager.get_simulation(simulation)
        lin_vel, ang_vel = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        for i in range(3):
            assert lin_vel[i] == pytest.approx(0.0, abs=1e-6)
            assert ang_vel[i] == pytest.approx(0.0, abs=1e-6)
    
    def test_object_moves_with_velocity(self, managers, simulation):
        """Test that object actually moves after setting velocity."""
        sim_manager, obj_manager = managers
        
        # Create a box
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Get initial position
        sim = sim_manager.get_simulation(simulation)
        initial_pos, _ = p.getBasePositionAndOrientation(object_id, physicsClientId=sim.client_id)
        
        # Set velocity
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=[5.0, 0.0, 0.0]
        )
        
        # Step simulation
        for _ in range(100):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final position
        final_pos, _ = p.getBasePositionAndOrientation(object_id, physicsClientId=sim.client_id)
        
        # Object should have moved in x direction
        assert final_pos[0] > initial_pos[0]
        # Should have moved significantly (at least 1 meter)
        assert (final_pos[0] - initial_pos[0]) > 1.0
    
    def test_set_only_linear_velocity_preserves_angular(self, managers, simulation):
        """Test that setting only linear velocity preserves angular velocity."""
        sim_manager, obj_manager = managers
        
        # Create a sphere
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set initial angular velocity
        initial_ang_vel = [1.0, 2.0, 3.0]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            angular_velocity=initial_ang_vel
        )
        
        # Now set only linear velocity
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=[5.0, 0.0, 0.0]
        )
        
        # Verify angular velocity is preserved
        sim = sim_manager.get_simulation(simulation)
        _, ang_vel = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        for i in range(3):
            assert ang_vel[i] == pytest.approx(initial_ang_vel[i], abs=1e-6)
    
    def test_set_only_angular_velocity_preserves_linear(self, managers, simulation):
        """Test that setting only angular velocity preserves linear velocity."""
        sim_manager, obj_manager = managers
        
        # Create a box
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set initial linear velocity
        initial_lin_vel = [3.0, 2.0, 1.0]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=initial_lin_vel
        )
        
        # Now set only angular velocity
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            angular_velocity=[0.0, 0.0, 5.0]
        )
        
        # Verify linear velocity is preserved
        sim = sim_manager.get_simulation(simulation)
        lin_vel, _ = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        for i in range(3):
            assert lin_vel[i] == pytest.approx(initial_lin_vel[i], abs=1e-6)
    
    def test_projectile_launch(self, managers, simulation):
        """Test launching a projectile with initial velocity."""
        sim_manager, obj_manager = managers
        
        # Create ground
        ground_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[10.0, 10.0, 0.1],
            position=[0, 0, 0],
            mass=0.0  # Static
        )
        
        # Create projectile
        projectile_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.2],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Launch projectile at 45 degrees
        launch_speed = 10.0
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=projectile_id,
            linear_velocity=[launch_speed * 0.707, 0.0, launch_speed * 0.707]
        )
        
        sim = sim_manager.get_simulation(simulation)
        
        # Step simulation
        max_height = 0
        for _ in range(200):
            p.stepSimulation(physicsClientId=sim.client_id)
            pos, _ = p.getBasePositionAndOrientation(projectile_id, physicsClientId=sim.client_id)
            max_height = max(max_height, pos[2])
        
        # Projectile should have reached significant height
        assert max_height > 2.0
    
    def test_invalid_object_raises_error(self, managers, simulation):
        """Test that invalid object ID raises ValueError."""
        _, obj_manager = managers
        
        with pytest.raises(ValueError, match="Object 999 not found"):
            obj_manager.set_object_velocity(
                sim_id=simulation,
                object_id=999,
                linear_velocity=[1.0, 0.0, 0.0]
            )
    
    def test_no_velocities_raises_error(self, managers, simulation):
        """Test that providing no velocities raises ValueError."""
        sim_manager, obj_manager = managers
        
        # Create a box
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        with pytest.raises(ValueError, match="At least one of linear_velocity or angular_velocity must be provided"):
            obj_manager.set_object_velocity(
                sim_id=simulation,
                object_id=object_id
            )
    
    def test_negative_velocities(self, managers, simulation):
        """Test that negative velocities work correctly."""
        sim_manager, obj_manager = managers
        
        # Create a box
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Set negative velocities
        target_lin_vel = [-5.0, -3.0, -2.0]
        target_ang_vel = [-1.0, -2.0, -3.0]
        obj_manager.set_object_velocity(
            sim_id=simulation,
            object_id=object_id,
            linear_velocity=target_lin_vel,
            angular_velocity=target_ang_vel
        )
        
        # Verify velocities were set
        sim = sim_manager.get_simulation(simulation)
        lin_vel, ang_vel = p.getBaseVelocity(object_id, physicsClientId=sim.client_id)
        
        for i in range(3):
            assert lin_vel[i] == pytest.approx(target_lin_vel[i], abs=1e-6)
            assert ang_vel[i] == pytest.approx(target_ang_vel[i], abs=1e-6)
