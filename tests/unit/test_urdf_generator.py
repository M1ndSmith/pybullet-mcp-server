"""Tests for URDF generator (revolute joints)."""

import pytest
import os
import tempfile
import pybullet as p
from src.urdf_generator import (
    calculate_box_inertia,
    calculate_sphere_inertia,
    calculate_cylinder_inertia,
    generate_revolute_joint_urdf
)
from src.simulation_manager import SimulationManager
from src.object_manager import ObjectManager


class TestInertiaCalculations:
    """Test inertia tensor calculations."""
    
    def test_box_inertia(self):
        """Test box inertia calculation."""
        mass = 1.0
        dimensions = [0.5, 0.5, 0.5]  # Half extents
        
        ixx, iyy, izz = calculate_box_inertia(mass, dimensions)
        
        # For a cube, all inertias should be equal
        assert ixx == pytest.approx(iyy, abs=1e-6)
        assert iyy == pytest.approx(izz, abs=1e-6)
        
        # Check formula: I = (1/12) * m * (h² + d²) for cube
        # Full dimensions: 1x1x1
        expected = (1.0 / 12.0) * mass * (1.0 * 1.0 + 1.0 * 1.0)
        assert ixx == pytest.approx(expected, abs=1e-6)
    
    def test_sphere_inertia(self):
        """Test sphere inertia calculation."""
        mass = 1.0
        radius = 0.5
        
        ixx, iyy, izz = calculate_sphere_inertia(mass, radius)
        
        # For a sphere, all inertias should be equal
        assert ixx == pytest.approx(iyy, abs=1e-6)
        assert iyy == pytest.approx(izz, abs=1e-6)
        
        # Check formula: I = (2/5) * m * r²
        expected = (2.0 / 5.0) * mass * radius * radius
        assert ixx == pytest.approx(expected, abs=1e-6)
    
    def test_cylinder_inertia(self):
        """Test cylinder inertia calculation."""
        mass = 1.0
        radius = 0.5
        height = 1.0
        
        ixx, iyy, izz = calculate_cylinder_inertia(mass, radius, height)
        
        # For cylinder, ixx == iyy (symmetry around z-axis)
        assert ixx == pytest.approx(iyy, abs=1e-6)
        
        # Check formulas
        expected_ixx = (1.0 / 12.0) * mass * (3 * radius * radius + height * height)
        expected_izz = 0.5 * mass * radius * radius
        
        assert ixx == pytest.approx(expected_ixx, abs=1e-6)
        assert izz == pytest.approx(expected_izz, abs=1e-6)


class TestURDFGeneration:
    """Test URDF file generation."""
    
    def test_generate_box_box_revolute_joint(self):
        """Test generating URDF with two boxes connected by revolute joint."""
        urdf_path = generate_revolute_joint_urdf(
            parent_shape="box",
            child_shape="box",
            parent_dimensions=[0.5, 0.5, 0.5],
            child_dimensions=[0.3, 0.3, 0.3],
            parent_mass=10.0,
            child_mass=1.0,
            joint_axis=[0, 0, 1],
            joint_origin=[0.5, 0, 0],
            joint_limits=(-1.57, 1.57)
        )
        
        # Verify file was created
        assert os.path.exists(urdf_path)
        
        # Read and verify content
        with open(urdf_path, 'r') as f:
            content = f.read()
        
        assert '<?xml version="1.0"?>' in content
        assert '<robot name="revolute_joint_model">' in content
        assert '<joint name="revolute_joint" type="revolute">' in content
        assert '<parent link="parent_link"/>' in content
        assert '<child link="child_link"/>' in content
        assert '<axis xyz="0 0 1"/>' in content
        assert 'lower="-1.57"' in content
        assert 'upper="1.57"' in content
        
        # Cleanup
        os.remove(urdf_path)
    
    def test_generate_sphere_cylinder_revolute_joint(self):
        """Test generating URDF with sphere and cylinder."""
        urdf_path = generate_revolute_joint_urdf(
            parent_shape="sphere",
            child_shape="cylinder",
            parent_dimensions=[0.5],
            child_dimensions=[0.2, 1.0],
            parent_mass=5.0,
            child_mass=2.0,
            joint_axis=[1, 0, 0]
        )
        
        assert os.path.exists(urdf_path)
        
        with open(urdf_path, 'r') as f:
            content = f.read()
        
        assert '<sphere radius="0.5"/>' in content
        assert '<cylinder radius="0.2" length="1.0"/>' in content
        assert '<axis xyz="1 0 0"/>' in content
        
        os.remove(urdf_path)
    
    def test_generate_with_custom_output_path(self):
        """Test generating URDF with custom output path."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = os.path.join(tmpdir, "custom_revolute.urdf")
            
            urdf_path = generate_revolute_joint_urdf(
                parent_shape="box",
                child_shape="box",
                parent_dimensions=[1.0, 1.0, 1.0],
                child_dimensions=[0.5, 0.5, 0.5],
                parent_mass=10.0,
                child_mass=1.0,
                joint_axis=[0, 1, 0],
                output_path=output_path
            )
            
            assert urdf_path == output_path
            assert os.path.exists(output_path)
    
    def test_invalid_parent_shape_raises_error(self):
        """Test that invalid parent shape raises ValueError."""
        with pytest.raises(ValueError, match="Invalid parent_shape"):
            generate_revolute_joint_urdf(
                parent_shape="invalid",
                child_shape="box",
                parent_dimensions=[1.0, 1.0, 1.0],
                child_dimensions=[0.5, 0.5, 0.5],
                parent_mass=10.0,
                child_mass=1.0,
                joint_axis=[0, 0, 1]
            )
    
    def test_invalid_child_shape_raises_error(self):
        """Test that invalid child shape raises ValueError."""
        with pytest.raises(ValueError, match="Invalid child_shape"):
            generate_revolute_joint_urdf(
                parent_shape="box",
                child_shape="invalid",
                parent_dimensions=[1.0, 1.0, 1.0],
                child_dimensions=[0.5, 0.5, 0.5],
                parent_mass=10.0,
                child_mass=1.0,
                joint_axis=[0, 0, 1]
            )
    
    def test_load_generated_urdf_in_pybullet(self):
        """Test that generated URDF can be loaded in PyBullet."""
        # Generate URDF
        urdf_path = generate_revolute_joint_urdf(
            parent_shape="box",
            child_shape="box",
            parent_dimensions=[0.5, 0.5, 0.5],
            child_dimensions=[0.3, 0.3, 0.3],
            parent_mass=10.0,
            child_mass=1.0,
            joint_axis=[0, 0, 1],
            joint_origin=[0.6, 0, 0]
        )
        
        # Create simulation and load URDF (disable strict path validation for tests)
        sim_manager = SimulationManager()
        obj_manager = ObjectManager(sim_manager, strict_path_validation=False)
        
        sim_id = sim_manager.create_simulation()
        sim = sim_manager.get_simulation(sim_id)
        
        try:
            # Load URDF
            object_id = obj_manager.load_urdf(
                sim_id=sim_id,
                file_path=urdf_path,
                position=[0, 0, 1]
            )
            
            assert object_id >= 0
            
            # Verify it has joints
            num_joints = p.getNumJoints(object_id, physicsClientId=sim.client_id)
            assert num_joints == 1  # Should have 1 revolute joint
            
            # Get joint info
            joint_info = p.getJointInfo(object_id, 0, physicsClientId=sim.client_id)
            joint_type = joint_info[2]
            
            # Verify it's a revolute joint (type 0)
            assert joint_type == p.JOINT_REVOLUTE
            
        finally:
            sim_manager.destroy_simulation(sim_id)
            os.remove(urdf_path)
    
    def test_revolute_joint_can_rotate(self):
        """Test that revolute joint actually rotates."""
        # Generate URDF
        urdf_path = generate_revolute_joint_urdf(
            parent_shape="box",
            child_shape="box",
            parent_dimensions=[0.5, 0.5, 0.5],
            child_dimensions=[0.3, 0.3, 0.3],
            parent_mass=100.0,  # Heavy parent (anchor)
            child_mass=1.0,
            joint_axis=[0, 0, 1],  # Rotate around z-axis
            joint_origin=[0.6, 0, 0],
            joint_limits=(-3.14, 3.14)
        )
        
        sim_manager = SimulationManager()
        obj_manager = ObjectManager(sim_manager, strict_path_validation=False)
        
        sim_id = sim_manager.create_simulation()
        sim = sim_manager.get_simulation(sim_id)
        
        try:
            # Load URDF
            object_id = obj_manager.load_urdf(
                sim_id=sim_id,
                file_path=urdf_path,
                position=[0, 0, 1]
            )
            
            # Get initial joint position
            joint_state = p.getJointState(object_id, 0, physicsClientId=sim.client_id)
            initial_position = joint_state[0]
            
            # Apply motor control to rotate joint
            p.setJointMotorControl2(
                bodyUniqueId=object_id,
                jointIndex=0,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=2.0,
                force=10.0,
                physicsClientId=sim.client_id
            )
            
            # Step simulation
            for _ in range(100):
                p.stepSimulation(physicsClientId=sim.client_id)
            
            # Get final joint position
            joint_state = p.getJointState(object_id, 0, physicsClientId=sim.client_id)
            final_position = joint_state[0]
            
            # Joint should have rotated
            assert abs(final_position - initial_position) > 0.1
            
        finally:
            sim_manager.destroy_simulation(sim_id)
            os.remove(urdf_path)
    
    def test_joint_limits_are_enforced(self):
        """Test that joint limits are properly set in URDF."""
        urdf_path = generate_revolute_joint_urdf(
            parent_shape="box",
            child_shape="box",
            parent_dimensions=[0.5, 0.5, 0.5],
            child_dimensions=[0.3, 0.3, 0.3],
            parent_mass=10.0,
            child_mass=1.0,
            joint_axis=[0, 0, 1],
            joint_limits=(-0.5, 0.5),  # Limited range
            max_effort=50.0,
            max_velocity=5.0
        )
        
        with open(urdf_path, 'r') as f:
            content = f.read()
        
        assert 'lower="-0.5"' in content
        assert 'upper="0.5"' in content
        assert 'effort="50.0"' in content
        assert 'velocity="5.0"' in content
        
        os.remove(urdf_path)
