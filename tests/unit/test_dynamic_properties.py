"""Tests for dynamic property modification."""

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


class TestChangeDynamics:
    """Test changing object dynamics at runtime."""
    
    def test_change_mass(self, managers, simulation):
        """Test changing object mass."""
        sim_manager, obj_manager = managers
        
        # Create object with initial mass
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Change mass
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            mass=5.0
        )
        
        # Verify mass changed
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["mass"] == pytest.approx(5.0, abs=1e-6)
    
    def test_change_friction(self, managers, simulation):
        """Test changing lateral friction."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0,
            friction=0.5
        )
        
        # Change friction
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            lateral_friction=0.1
        )
        
        # Verify friction changed
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["lateral_friction"] == pytest.approx(0.1, abs=1e-6)
    
    def test_change_restitution(self, managers, simulation):
        """Test changing restitution (bounciness)."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0,
            restitution=0.5
        )
        
        # Change restitution
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            restitution=0.9
        )
        
        # Verify restitution changed
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["restitution"] == pytest.approx(0.9, abs=1e-6)
    
    def test_change_multiple_properties(self, managers, simulation):
        """Test changing multiple properties at once."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Change multiple properties
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            mass=3.0,
            lateral_friction=0.2,
            restitution=0.8
        )
        
        # Verify all changed
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["mass"] == pytest.approx(3.0, abs=1e-6)
        assert info["lateral_friction"] == pytest.approx(0.2, abs=1e-6)
        assert info["restitution"] == pytest.approx(0.8, abs=1e-6)
    
    def test_change_rolling_and_spinning_friction(self, managers, simulation):
        """Test changing rolling and spinning friction."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Change friction types
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            rolling_friction=0.05,
            spinning_friction=0.03
        )
        
        # Verify changed
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["rolling_friction"] == pytest.approx(0.05, abs=1e-6)
        assert info["spinning_friction"] == pytest.approx(0.03, abs=1e-6)
    
    def test_no_properties_raises_error(self, managers, simulation):
        """Test that providing no properties raises error."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Try to change with no properties
        with pytest.raises(ValueError, match="At least one property must be specified"):
            obj_manager.change_dynamics(
                sim_id=simulation,
                object_id=object_id
            )
    
    def test_negative_mass_raises_error(self, managers, simulation):
        """Test that negative mass raises error."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Try to set negative mass
        with pytest.raises(ValueError, match="Mass must be non-negative"):
            obj_manager.change_dynamics(
                sim_id=simulation,
                object_id=object_id,
                mass=-1.0
            )
    
    def test_invalid_object_raises_error(self, managers, simulation):
        """Test that invalid object ID raises error."""
        _, obj_manager = managers
        
        with pytest.raises(ValueError, match="Object 999 not found"):
            obj_manager.change_dynamics(
                sim_id=simulation,
                object_id=999,
                mass=5.0
            )


class TestGetDynamicsInfo:
    """Test querying object dynamics."""
    
    def test_get_dynamics_info(self, managers, simulation):
        """Test getting dynamics info for an object."""
        sim_manager, obj_manager = managers
        
        # Create object with known properties
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=2.5,
            friction=0.7,
            restitution=0.3
        )
        
        # Get dynamics info
        info = obj_manager.get_dynamics_info(simulation, object_id)
        
        # Verify structure
        assert "mass" in info
        assert "lateral_friction" in info
        assert "restitution" in info
        assert "local_inertia_diagonal" in info
        assert "rolling_friction" in info
        assert "spinning_friction" in info
        
        # Verify values
        assert info["mass"] == pytest.approx(2.5, abs=1e-6)
        assert info["lateral_friction"] == pytest.approx(0.7, abs=1e-6)
        assert info["restitution"] == pytest.approx(0.3, abs=1e-6)
    
    def test_get_dynamics_info_returns_all_fields(self, managers, simulation):
        """Test that all expected fields are returned."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        
        # Get dynamics info
        info = obj_manager.get_dynamics_info(simulation, object_id)
        
        # Verify all expected fields exist
        expected_fields = [
            "mass", "lateral_friction", "local_inertia_diagonal",
            "local_inertia_pos", "local_inertia_orn", "restitution",
            "rolling_friction", "spinning_friction", "contact_damping",
            "contact_stiffness", "body_type", "collision_margin"
        ]
        
        for field in expected_fields:
            assert field in info, f"Missing field: {field}"
    
    def test_get_dynamics_info_invalid_object(self, managers, simulation):
        """Test that invalid object ID raises error."""
        _, obj_manager = managers
        
        with pytest.raises(ValueError, match="Object 999 not found"):
            obj_manager.get_dynamics_info(simulation, 999)
    
    def test_dynamics_persist_after_query(self, managers, simulation):
        """Test that querying doesn't change dynamics."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=3.0
        )
        
        # Get info twice
        info1 = obj_manager.get_dynamics_info(simulation, object_id)
        info2 = obj_manager.get_dynamics_info(simulation, object_id)
        
        # Should be identical
        assert info1["mass"] == info2["mass"]
        assert info1["lateral_friction"] == info2["lateral_friction"]
        assert info1["restitution"] == info2["restitution"]


class TestDynamicsWorkflow:
    """Test complete workflows with dynamics modification."""
    
    def test_make_object_slippery(self, managers, simulation):
        """Test making an object slippery by reducing friction."""
        sim_manager, obj_manager = managers
        
        # Create object with normal friction
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0,
            friction=0.5
        )
        
        # Make it slippery
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            lateral_friction=0.05
        )
        
        # Verify it's slippery
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["lateral_friction"] < 0.1
    
    def test_make_object_bouncy(self, managers, simulation):
        """Test making an object bouncy by increasing restitution."""
        sim_manager, obj_manager = managers
        
        # Create object with low bounce
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="sphere",
            dimensions=[0.5],
            position=[0, 0, 1],
            mass=1.0,
            restitution=0.1
        )
        
        # Make it bouncy
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            restitution=0.95
        )
        
        # Verify it's bouncy
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["restitution"] > 0.9
    
    def test_simulate_damage_by_reducing_mass(self, managers, simulation):
        """Test simulating damage by reducing mass."""
        sim_manager, obj_manager = managers
        
        # Create object
        object_id = obj_manager.create_primitive(
            sim_id=simulation,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=10.0
        )
        
        # Simulate damage (reduce mass)
        obj_manager.change_dynamics(
            sim_id=simulation,
            object_id=object_id,
            mass=5.0
        )
        
        # Verify mass reduced
        info = obj_manager.get_dynamics_info(simulation, object_id)
        assert info["mass"] == pytest.approx(5.0, abs=1e-6)
