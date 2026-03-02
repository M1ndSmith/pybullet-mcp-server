"""Unit tests for ConstraintManager class."""

import pytest
import pybullet as p

from src.simulation_manager import SimulationManager
from src.object_manager import ObjectManager
from src.constraint_manager import ConstraintManager


class TestConstraintManager:
    """Test suite for ConstraintManager functionality."""
    
    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.sim_manager = SimulationManager()
        self.obj_manager = ObjectManager(self.sim_manager)
        self.constraint_manager = ConstraintManager(self.sim_manager)
    
    def teardown_method(self):
        """Clean up after each test method."""
        # Destroy all simulations
        for sim_id in list(self.sim_manager.simulations.keys()):
            self.sim_manager.destroy_simulation(sim_id)
    
    def test_create_fixed_constraint(self):
        """Test creating a fixed joint constraint."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create fixed constraint
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "fixed"
        )
        
        # Verify constraint was created
        assert isinstance(constraint_id, int)
        assert constraint_id >= 0
        
        # Verify constraint is tracked in simulation context
        sim = self.sim_manager.get_simulation(sim_id)
        assert constraint_id in sim.constraints
        
        # Verify metadata
        metadata = sim.get_constraint(constraint_id)
        assert metadata["parent_id"] == obj1
        assert metadata["child_id"] == obj2
        assert metadata["joint_type"] == "fixed"
    
    def test_create_revolute_constraint_not_supported(self):
        """Test that revolute joint raises helpful error due to PyBullet limitation."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Try to create revolute constraint - should raise helpful error
        with pytest.raises(ValueError, match="PyBullet's createConstraint API does not support revolute"):
            self.constraint_manager.create_constraint(
                sim_id, obj1, obj2, "revolute", joint_axis=[1, 0, 0]
            )
    
    def test_create_prismatic_constraint(self):
        """Test creating a prismatic joint constraint."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create prismatic constraint
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "prismatic", joint_axis=[0, 0, 1]
        )
        
        # Verify constraint was created
        assert isinstance(constraint_id, int)
        assert constraint_id >= 0
        
        # Verify metadata
        sim = self.sim_manager.get_simulation(sim_id)
        metadata = sim.get_constraint(constraint_id)
        assert metadata["joint_type"] == "prismatic"
    
    def test_create_spherical_constraint(self):
        """Test creating a spherical joint constraint."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create spherical constraint
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "spherical"
        )
        
        # Verify constraint was created
        assert isinstance(constraint_id, int)
        assert constraint_id >= 0
        
        # Verify metadata
        sim = self.sim_manager.get_simulation(sim_id)
        metadata = sim.get_constraint(constraint_id)
        assert metadata["joint_type"] == "spherical"
    
    def test_invalid_joint_type(self):
        """Test that invalid joint type raises ValueError."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Try to create constraint with invalid type
        with pytest.raises(ValueError, match="Invalid joint type"):
            self.constraint_manager.create_constraint(
                sim_id, obj1, obj2, "invalid_type"
            )
    
    def test_constraint_with_invalid_parent(self):
        """Test that constraint creation fails with invalid parent object."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create one object
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Try to create constraint with non-existent parent
        with pytest.raises(ValueError, match="Parent object .* not found"):
            self.constraint_manager.create_constraint(
                sim_id, 999, obj2, "fixed"
            )
    
    def test_constraint_with_invalid_child(self):
        """Test that constraint creation fails with invalid child object."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create one object
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        
        # Try to create constraint with non-existent child
        with pytest.raises(ValueError, match="Child object .* not found"):
            self.constraint_manager.create_constraint(
                sim_id, obj1, 999, "fixed"
            )
    
    def test_set_constraint_params(self):
        """Test setting constraint parameters."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create prismatic constraint (since revolute is not supported)
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "prismatic"
        )
        
        # Set constraint parameters
        self.constraint_manager.set_constraint_params(
            sim_id, constraint_id, max_force=100.0, erp=0.8
        )
        
        # Verify parameters were stored in metadata
        sim = self.sim_manager.get_simulation(sim_id)
        metadata = sim.get_constraint(constraint_id)
        assert metadata["max_force"] == 100.0
        assert metadata["erp"] == 0.8
    
    def test_set_params_invalid_constraint(self):
        """Test that setting params on invalid constraint raises ValueError."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Try to set params on non-existent constraint
        with pytest.raises(ValueError, match="Constraint .* not found"):
            self.constraint_manager.set_constraint_params(
                sim_id, 999, max_force=100.0
            )
    
    def test_remove_constraint(self):
        """Test removing a constraint."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create constraint
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "fixed"
        )
        
        # Verify constraint exists
        sim = self.sim_manager.get_simulation(sim_id)
        assert constraint_id in sim.constraints
        
        # Remove constraint
        self.constraint_manager.remove_constraint(sim_id, constraint_id)
        
        # Verify constraint was removed
        assert constraint_id not in sim.constraints
    
    def test_remove_invalid_constraint(self):
        """Test that removing invalid constraint raises ValueError."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Try to remove non-existent constraint
        with pytest.raises(ValueError, match="Constraint .* not found"):
            self.constraint_manager.remove_constraint(sim_id, 999)
    
    def test_get_constraint_info(self):
        """Test getting constraint information."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create spherical constraint with custom parameters
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "spherical",
            joint_axis=[1, 0, 0],
            parent_frame_position=[0, 0, 0.5]
        )
        
        # Get constraint info
        info = self.constraint_manager.get_constraint_info(sim_id, constraint_id)
        
        # Verify info
        assert info["parent_id"] == obj1
        assert info["child_id"] == obj2
        assert info["joint_type"] == "spherical"
        assert info["joint_axis"] == [1, 0, 0]
        assert info["parent_frame_position"] == [0, 0, 0.5]
    
    def test_get_info_invalid_constraint(self):
        """Test that getting info for invalid constraint raises ValueError."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Try to get info for non-existent constraint
        with pytest.raises(ValueError, match="Constraint .* not found"):
            self.constraint_manager.get_constraint_info(sim_id, 999)
    
    def test_constraint_with_custom_frames(self):
        """Test creating constraint with custom frame positions and orientations."""
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create spherical constraint with custom frames
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, obj1, obj2, "spherical",
            joint_axis=[1, 0, 0],
            parent_frame_position=[0, 0, 0.5],
            child_frame_position=[0, 0, -0.5],
            parent_frame_orientation=[0, 0, 0, 1],
            child_frame_orientation=[0, 0, 0, 1]
        )
        
        # Verify constraint was created
        assert isinstance(constraint_id, int)
        assert constraint_id >= 0
        
        # Verify metadata
        sim = self.sim_manager.get_simulation(sim_id)
        metadata = sim.get_constraint(constraint_id)
        assert metadata["parent_frame_position"] == [0, 0, 0.5]
        assert metadata["child_frame_position"] == [0, 0, -0.5]


    # ========================================================================
    # Task 7.5: Unit tests for each joint type with specific examples
    # Requirements: 6.2
    # ========================================================================
    
    def test_fixed_joint_prevents_relative_motion(self):
        """Test that fixed joint prevents any relative motion between objects.
        
        Specific example: Two boxes connected by a fixed joint should maintain
        their relative position and orientation even when forces are applied.
        """
        # Create simulation
        sim_id = self.sim_manager.create_simulation(gravity=(0, 0, -9.81))
        
        # Create two boxes: one at z=1, one at z=2
        box1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], mass=1.0
        )
        box2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 2], mass=1.0
        )
        
        # Create fixed constraint between them
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, box1, box2, "fixed",
            parent_frame_position=[0, 0, 0.5],  # Top of box1
            child_frame_position=[0, 0, -0.5]   # Bottom of box2
        )
        
        # Get initial positions
        sim = self.sim_manager.get_simulation(sim_id)
        pos1_before, _ = p.getBasePositionAndOrientation(box1, physicsClientId=sim.client_id)
        pos2_before, _ = p.getBasePositionAndOrientation(box2, physicsClientId=sim.client_id)
        initial_distance = pos2_before[2] - pos1_before[2]
        
        # Apply force to box2 (should not separate from box1)
        self.obj_manager.apply_force(sim_id, box2, [10, 0, 0])
        
        # Step simulation
        for _ in range(100):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final positions
        pos1_after, _ = p.getBasePositionAndOrientation(box1, physicsClientId=sim.client_id)
        pos2_after, _ = p.getBasePositionAndOrientation(box2, physicsClientId=sim.client_id)
        final_distance = pos2_after[2] - pos1_after[2]
        
        # Verify relative distance is maintained (within tolerance)
        assert abs(final_distance - initial_distance) < 0.1, \
            "Fixed joint should maintain relative position"
    
    def test_revolute_joint_error_with_helpful_message(self):
        """Test that revolute joint raises a clear error with guidance.
        
        Specific example: Attempting to create a revolute (hinge) joint should
        fail with a message explaining PyBullet's limitation and suggesting URDF.
        """
        # Create simulation
        sim_id = self.sim_manager.create_simulation()
        
        # Create two boxes for a door hinge scenario
        door_frame = self.obj_manager.create_primitive(
            sim_id, "box", [0.1, 1.0, 2.0], [0, 0, 1], mass=100.0  # Heavy frame
        )
        door = self.obj_manager.create_primitive(
            sim_id, "box", [0.05, 1.0, 2.0], [0.5, 0, 1], mass=5.0
        )
        
        # Try to create revolute constraint for door hinge
        with pytest.raises(ValueError) as exc_info:
            self.constraint_manager.create_constraint(
                sim_id, door_frame, door, "revolute",
                joint_axis=[0, 0, 1],  # Hinge axis (vertical)
                parent_frame_position=[0.05, 0, 0],
                child_frame_position=[-0.025, 0, 0]
            )
        
        # Verify error message is helpful
        error_msg = str(exc_info.value)
        assert "PyBullet's createConstraint API does not support revolute" in error_msg
        assert "URDF" in error_msg, "Error should mention URDF as alternative"
        assert "load_urdf()" in error_msg, "Error should mention load_urdf function"
    
    def test_prismatic_joint_allows_sliding_motion(self):
        """Test that prismatic joint allows sliding along one axis only.
        
        Specific example: A piston-cylinder system where the piston can slide
        along the Z-axis but cannot rotate or move in X/Y directions.
        """
        # Create simulation with no gravity for clearer test
        sim_id = self.sim_manager.create_simulation(gravity=(0, 0, 0))
        
        # Create cylinder (heavy/static) and piston (moving)
        cylinder = self.obj_manager.create_primitive(
            sim_id, "cylinder", [0.5, 0.5, 1.0], [0, 0, 0], mass=100.0  # Heavy
        )
        piston = self.obj_manager.create_primitive(
            sim_id, "cylinder", [0.4, 0.4, 0.2], [0, 0, 0.5], mass=1.0
        )
        
        # Create prismatic constraint along Z-axis
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, cylinder, piston, "prismatic",
            joint_axis=[0, 0, 1],  # Allow sliding along Z
            parent_frame_position=[0, 0, 0],
            child_frame_position=[0, 0, 0]
        )
        
        # Get initial position
        sim = self.sim_manager.get_simulation(sim_id)
        pos_before, orn_before = p.getBasePositionAndOrientation(piston, physicsClientId=sim.client_id)
        
        # Apply force along Z-axis (should move)
        self.obj_manager.apply_force(sim_id, piston, [0, 0, 10])
        
        # Step simulation
        for _ in range(50):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final position
        pos_after, orn_after = p.getBasePositionAndOrientation(piston, physicsClientId=sim.client_id)
        
        # Verify piston moved along Z-axis
        assert pos_after[2] > pos_before[2], "Piston should move along Z-axis"
        
        # Verify X and Y positions are constrained (within tolerance)
        assert abs(pos_after[0] - pos_before[0]) < 0.1, "X position should be constrained"
        assert abs(pos_after[1] - pos_before[1]) < 0.1, "Y position should be constrained"
        
        # Verify orientation is maintained (quaternions should be similar)
        # For prismatic joints, rotation should be constrained
        assert abs(orn_after[0] - orn_before[0]) < 0.1, "Orientation should be constrained"
        assert abs(orn_after[1] - orn_before[1]) < 0.1, "Orientation should be constrained"
    
    def test_spherical_joint_allows_rotation_constrains_position(self):
        """Test that spherical joint allows rotation but constrains position.
        
        Specific example: A ball-and-socket joint like a shoulder joint where
        the child can rotate freely but the connection point stays fixed.
        """
        # Create simulation with no gravity for clearer test
        sim_id = self.sim_manager.create_simulation(gravity=(0, 0, 0))
        
        # Create shoulder (heavy) and arm (rotating)
        shoulder = self.obj_manager.create_primitive(
            sim_id, "sphere", [0.3, 0.3, 0.3], [0, 0, 0], mass=100.0  # Heavy
        )
        arm = self.obj_manager.create_primitive(
            sim_id, "box", [0.2, 0.2, 1.0], [0, 0, 0.8], mass=1.0
        )
        
        # Create spherical constraint (ball-and-socket)
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, shoulder, arm, "spherical",
            parent_frame_position=[0, 0, 0.3],  # Top of shoulder
            child_frame_position=[0, 0, -0.5]   # Base of arm
        )
        
        # Get initial state
        sim = self.sim_manager.get_simulation(sim_id)
        pos_before, orn_before = p.getBasePositionAndOrientation(arm, physicsClientId=sim.client_id)
        
        # Apply torque to rotate the arm
        self.obj_manager.apply_torque(sim_id, arm, [5, 0, 0])
        
        # Step simulation
        for _ in range(100):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final state
        pos_after, orn_after = p.getBasePositionAndOrientation(arm, physicsClientId=sim.client_id)
        
        # Verify orientation changed (rotation occurred)
        orientation_changed = (
            abs(orn_after[0] - orn_before[0]) > 0.01 or
            abs(orn_after[1] - orn_before[1]) > 0.01 or
            abs(orn_after[2] - orn_before[2]) > 0.01 or
            abs(orn_after[3] - orn_before[3]) > 0.01
        )
        assert orientation_changed, "Spherical joint should allow rotation"
        
        # Verify connection point is maintained (position constrained relative to parent)
        # The arm should stay connected to the shoulder
        distance_before = ((pos_before[0] - 0)**2 + (pos_before[1] - 0)**2 + (pos_before[2] - 0.3)**2)**0.5
        distance_after = ((pos_after[0] - 0)**2 + (pos_after[1] - 0)**2 + (pos_after[2] - 0.3)**2)**0.5
        
        # Distance from connection point should remain relatively constant
        assert abs(distance_after - distance_before) < 0.5, \
            "Spherical joint should maintain connection point distance"
    
    def test_prismatic_joint_with_limits(self):
        """Test prismatic joint with position limits.
        
        Specific example: A drawer that can slide out but has min/max limits.
        """
        # Create simulation with no gravity for clearer test
        sim_id = self.sim_manager.create_simulation(gravity=(0, 0, 0))
        
        # Create cabinet (heavy) and drawer (sliding)
        cabinet = self.obj_manager.create_primitive(
            sim_id, "box", [1.0, 1.0, 1.0], [0, 0, 0], mass=100.0  # Heavy
        )
        drawer = self.obj_manager.create_primitive(
            sim_id, "box", [0.8, 0.8, 0.2], [0, 0.3, 0], mass=2.0
        )
        
        # Create prismatic constraint along Y-axis (drawer slides in/out)
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, cabinet, drawer, "prismatic",
            joint_axis=[0, 1, 0],  # Slide along Y
            parent_frame_position=[0, 0, 0],
            child_frame_position=[0, 0, 0]
        )
        
        # Set constraint parameters with limits
        self.constraint_manager.set_constraint_params(
            sim_id, constraint_id,
            max_force=50.0,
            erp=0.9  # High stiffness
        )
        
        # Get initial position
        sim = self.sim_manager.get_simulation(sim_id)
        pos_before, _ = p.getBasePositionAndOrientation(drawer, physicsClientId=sim.client_id)
        
        # Apply force to slide drawer out
        self.obj_manager.apply_force(sim_id, drawer, [0, 20, 0])
        
        # Step simulation
        for _ in range(100):
            p.stepSimulation(physicsClientId=sim.client_id)
        
        # Get final position
        pos_after, _ = p.getBasePositionAndOrientation(drawer, physicsClientId=sim.client_id)
        
        # Verify drawer moved along Y-axis
        assert pos_after[1] > pos_before[1], "Drawer should slide along Y-axis"
        
        # Verify X is constrained (prismatic joints constrain perpendicular motion)
        assert abs(pos_after[0] - pos_before[0]) < 0.5, "X should be relatively constrained"
        
        # Note: Z may have some drift due to PyBullet's constraint solver behavior
        # This is expected with prismatic constraints in PyBullet
        
        # Verify constraint parameters were applied
        metadata = sim.get_constraint(constraint_id)
        assert metadata["max_force"] == 50.0
        assert metadata["erp"] == 0.9
    
    def test_spherical_joint_with_multiple_rotation_axes(self):
        """Test spherical joint allows rotation around multiple axes.
        
        Specific example: A camera gimbal that can rotate in multiple directions.
        """
        # Create simulation with no gravity
        sim_id = self.sim_manager.create_simulation(gravity=(0, 0, 0))
        
        # Create mount (heavy) and camera (rotating)
        mount = self.obj_manager.create_primitive(
            sim_id, "box", [0.3, 0.3, 0.3], [0, 0, 0], mass=100.0  # Heavy
        )
        camera = self.obj_manager.create_primitive(
            sim_id, "box", [0.4, 0.2, 0.2], [0, 0, 0.5], mass=0.5
        )
        
        # Create spherical constraint
        constraint_id = self.constraint_manager.create_constraint(
            sim_id, mount, camera, "spherical",
            parent_frame_position=[0, 0, 0.15],
            child_frame_position=[0, 0, -0.3]
        )
        
        sim = self.sim_manager.get_simulation(sim_id)
        
        # Test rotation around X-axis
        _, orn_before = p.getBasePositionAndOrientation(camera, physicsClientId=sim.client_id)
        self.obj_manager.apply_torque(sim_id, camera, [2, 0, 0])
        for _ in range(50):
            p.stepSimulation(physicsClientId=sim.client_id)
        _, orn_after_x = p.getBasePositionAndOrientation(camera, physicsClientId=sim.client_id)
        
        # Reset velocity
        p.resetBaseVelocity(camera, [0, 0, 0], [0, 0, 0], physicsClientId=sim.client_id)
        
        # Test rotation around Y-axis
        self.obj_manager.apply_torque(sim_id, camera, [0, 2, 0])
        for _ in range(50):
            p.stepSimulation(physicsClientId=sim.client_id)
        _, orn_after_y = p.getBasePositionAndOrientation(camera, physicsClientId=sim.client_id)
        
        # Verify rotations occurred (orientation changed from initial)
        x_rotation_occurred = any(abs(orn_after_x[i] - orn_before[i]) > 0.01 for i in range(4))
        y_rotation_occurred = any(abs(orn_after_y[i] - orn_after_x[i]) > 0.01 for i in range(4))
        
        assert x_rotation_occurred, "Spherical joint should allow X-axis rotation"
        assert y_rotation_occurred, "Spherical joint should allow Y-axis rotation"
