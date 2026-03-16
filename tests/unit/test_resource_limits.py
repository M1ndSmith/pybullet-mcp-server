"""Tests for resource limits (DoS prevention)."""

import pytest
from src.simulation_manager import (
    SimulationManager,
    MAX_SIMULATIONS,
    MAX_OBJECTS_PER_SIMULATION,
    MAX_CONSTRAINTS_PER_SIMULATION
)
from src.object_manager import ObjectManager
from src.constraint_manager import ConstraintManager


@pytest.fixture
def managers():
    """Create manager instances for testing."""
    sim_manager = SimulationManager()
    obj_manager = ObjectManager(sim_manager)
    constraint_manager = ConstraintManager(sim_manager)
    return sim_manager, obj_manager, constraint_manager


class TestSimulationLimits:
    """Test maximum simulation limits."""
    
    def test_max_simulations_limit_enforced(self, managers):
        """Test that creating more than MAX_SIMULATIONS raises error."""
        sim_manager, _, _ = managers
        
        # Create MAX_SIMULATIONS simulations
        sim_ids = []
        for i in range(MAX_SIMULATIONS):
            sim_id = sim_manager.create_simulation()
            sim_ids.append(sim_id)
        
        # Verify we have MAX_SIMULATIONS
        assert len(sim_manager.list_simulations()) == MAX_SIMULATIONS
        
        # Try to create one more - should fail
        with pytest.raises(ValueError, match=f"Maximum number of simulations \\({MAX_SIMULATIONS}\\) reached"):
            sim_manager.create_simulation()
        
        # Cleanup
        for sim_id in sim_ids:
            sim_manager.destroy_simulation(sim_id)
    
    def test_can_create_after_destroying(self, managers):
        """Test that destroying a simulation allows creating a new one."""
        sim_manager, _, _ = managers
        
        # Create MAX_SIMULATIONS simulations
        sim_ids = []
        for i in range(MAX_SIMULATIONS):
            sim_id = sim_manager.create_simulation()
            sim_ids.append(sim_id)
        
        # Destroy one simulation
        sim_manager.destroy_simulation(sim_ids[0])
        
        # Should be able to create a new one now
        new_sim_id = sim_manager.create_simulation()
        assert new_sim_id is not None
        
        # Cleanup
        sim_manager.destroy_simulation(new_sim_id)
        for sim_id in sim_ids[1:]:
            sim_manager.destroy_simulation(sim_id)
    
    def test_error_message_is_descriptive(self, managers):
        """Test that error message provides helpful information."""
        sim_manager, _, _ = managers
        
        # Create MAX_SIMULATIONS simulations
        sim_ids = []
        for i in range(MAX_SIMULATIONS):
            sim_id = sim_manager.create_simulation()
            sim_ids.append(sim_id)
        
        # Try to create one more
        with pytest.raises(ValueError) as exc_info:
            sim_manager.create_simulation()
        
        error_msg = str(exc_info.value)
        assert "Maximum number of simulations" in error_msg
        assert str(MAX_SIMULATIONS) in error_msg
        assert "Destroy existing simulations" in error_msg
        
        # Cleanup
        for sim_id in sim_ids:
            sim_manager.destroy_simulation(sim_id)


class TestObjectLimits:
    """Test maximum object limits per simulation."""
    
    def test_max_objects_limit_enforced(self, managers):
        """Test that adding more than MAX_OBJECTS_PER_SIMULATION raises error."""
        sim_manager, obj_manager, _ = managers
        
        # Create a simulation
        sim_id = sim_manager.create_simulation()
        
        # Add MAX_OBJECTS_PER_SIMULATION objects
        for i in range(MAX_OBJECTS_PER_SIMULATION):
            obj_manager.create_primitive(
                sim_id=sim_id,
                shape="sphere",
                dimensions=[0.1],
                position=[i * 0.3, 0, 1],
                mass=1.0
            )
        
        # Verify we have MAX_OBJECTS_PER_SIMULATION objects
        sim = sim_manager.get_simulation(sim_id)
        assert len(sim.objects) == MAX_OBJECTS_PER_SIMULATION
        
        # Try to add one more - should fail
        with pytest.raises(ValueError, match=f"Maximum number of objects \\({MAX_OBJECTS_PER_SIMULATION}\\) reached"):
            obj_manager.create_primitive(
                sim_id=sim_id,
                shape="sphere",
                dimensions=[0.1],
                position=[0, 0, 1],
                mass=1.0
            )
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id)
    
    def test_object_limit_per_simulation_independent(self, managers):
        """Test that object limits are per-simulation, not global."""
        sim_manager, obj_manager, _ = managers
        
        # Create two simulations
        sim_id1 = sim_manager.create_simulation()
        sim_id2 = sim_manager.create_simulation()
        
        # Add 5 objects to each simulation
        for i in range(5):
            obj_manager.create_primitive(
                sim_id=sim_id1,
                shape="sphere",
                dimensions=[0.1],
                position=[i * 0.3, 0, 1],
                mass=1.0
            )
            obj_manager.create_primitive(
                sim_id=sim_id2,
                shape="sphere",
                dimensions=[0.1],
                position=[i * 0.3, 0, 1],
                mass=1.0
            )
        
        # Verify each simulation has 5 objects
        sim1 = sim_manager.get_simulation(sim_id1)
        sim2 = sim_manager.get_simulation(sim_id2)
        assert len(sim1.objects) == 5
        assert len(sim2.objects) == 5
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id1)
        sim_manager.destroy_simulation(sim_id2)
    
    def test_error_message_is_descriptive_for_objects(self, managers):
        """Test that object limit error message is helpful."""
        sim_manager, obj_manager, _ = managers
        
        # Create a simulation
        sim_id = sim_manager.create_simulation()
        
        # Add MAX_OBJECTS_PER_SIMULATION objects
        for i in range(MAX_OBJECTS_PER_SIMULATION):
            obj_manager.create_primitive(
                sim_id=sim_id,
                shape="sphere",
                dimensions=[0.1],
                position=[i * 0.3, 0, 1],
                mass=1.0
            )
        
        # Try to add one more
        with pytest.raises(ValueError) as exc_info:
            obj_manager.create_primitive(
                sim_id=sim_id,
                shape="sphere",
                dimensions=[0.1],
                position=[0, 0, 1],
                mass=1.0
            )
        
        error_msg = str(exc_info.value)
        assert "Maximum number of objects" in error_msg
        assert str(MAX_OBJECTS_PER_SIMULATION) in error_msg
        assert "Remove objects" in error_msg
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id)


class TestConstraintLimits:
    """Test maximum constraint limits per simulation."""
    
    def test_max_constraints_limit_enforced(self, managers):
        """Test that adding more than MAX_CONSTRAINTS_PER_SIMULATION raises error."""
        sim_manager, obj_manager, constraint_manager = managers
        
        # Create a simulation
        sim_id = sim_manager.create_simulation()
        
        # Create enough objects for constraints (need 2 per constraint)
        # We'll reuse objects to avoid hitting object limit
        obj1 = obj_manager.create_primitive(
            sim_id=sim_id,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        obj2 = obj_manager.create_primitive(
            sim_id=sim_id,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[1, 0, 1],
            mass=1.0
        )
        
        # Add MAX_CONSTRAINTS_PER_SIMULATION constraints
        for i in range(MAX_CONSTRAINTS_PER_SIMULATION):
            constraint_manager.create_constraint(
                sim_id=sim_id,
                parent_id=obj1,
                child_id=obj2,
                joint_type="fixed"
            )
        
        # Verify we have MAX_CONSTRAINTS_PER_SIMULATION constraints
        sim = sim_manager.get_simulation(sim_id)
        assert len(sim.constraints) == MAX_CONSTRAINTS_PER_SIMULATION
        
        # Try to add one more - should fail
        with pytest.raises(ValueError, match=f"Maximum number of constraints \\({MAX_CONSTRAINTS_PER_SIMULATION}\\) reached"):
            constraint_manager.create_constraint(
                sim_id=sim_id,
                parent_id=obj1,
                child_id=obj2,
                joint_type="fixed"
            )
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id)
    
    def test_constraint_limit_per_simulation_independent(self, managers):
        """Test that constraint limits are per-simulation, not global."""
        sim_manager, obj_manager, constraint_manager = managers
        
        # Create two simulations
        sim_id1 = sim_manager.create_simulation()
        sim_id2 = sim_manager.create_simulation()
        
        # Create objects in each simulation
        obj1_sim1 = obj_manager.create_primitive(
            sim_id=sim_id1,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        obj2_sim1 = obj_manager.create_primitive(
            sim_id=sim_id1,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[1, 0, 1],
            mass=1.0
        )
        
        obj1_sim2 = obj_manager.create_primitive(
            sim_id=sim_id2,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        obj2_sim2 = obj_manager.create_primitive(
            sim_id=sim_id2,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[1, 0, 1],
            mass=1.0
        )
        
        # Add 3 constraints to each simulation
        for i in range(3):
            constraint_manager.create_constraint(
                sim_id=sim_id1,
                parent_id=obj1_sim1,
                child_id=obj2_sim1,
                joint_type="fixed"
            )
            constraint_manager.create_constraint(
                sim_id=sim_id2,
                parent_id=obj1_sim2,
                child_id=obj2_sim2,
                joint_type="fixed"
            )
        
        # Verify each simulation has 3 constraints
        sim1 = sim_manager.get_simulation(sim_id1)
        sim2 = sim_manager.get_simulation(sim_id2)
        assert len(sim1.constraints) == 3
        assert len(sim2.constraints) == 3
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id1)
        sim_manager.destroy_simulation(sim_id2)
    
    def test_error_message_is_descriptive_for_constraints(self, managers):
        """Test that constraint limit error message is helpful."""
        sim_manager, obj_manager, constraint_manager = managers
        
        # Create a simulation
        sim_id = sim_manager.create_simulation()
        
        # Create objects
        obj1 = obj_manager.create_primitive(
            sim_id=sim_id,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[0, 0, 1],
            mass=1.0
        )
        obj2 = obj_manager.create_primitive(
            sim_id=sim_id,
            shape="box",
            dimensions=[0.5, 0.5, 0.5],
            position=[1, 0, 1],
            mass=1.0
        )
        
        # Add MAX_CONSTRAINTS_PER_SIMULATION constraints
        for i in range(MAX_CONSTRAINTS_PER_SIMULATION):
            constraint_manager.create_constraint(
                sim_id=sim_id,
                parent_id=obj1,
                child_id=obj2,
                joint_type="fixed"
            )
        
        # Try to add one more
        with pytest.raises(ValueError) as exc_info:
            constraint_manager.create_constraint(
                sim_id=sim_id,
                parent_id=obj1,
                child_id=obj2,
                joint_type="fixed"
            )
        
        error_msg = str(exc_info.value)
        assert "Maximum number of constraints" in error_msg
        assert str(MAX_CONSTRAINTS_PER_SIMULATION) in error_msg
        assert "Remove constraints" in error_msg
        
        # Cleanup
        sim_manager.destroy_simulation(sim_id)


class TestResourceLimitConfiguration:
    """Test that resource limits are properly configured."""
    
    def test_limits_are_reasonable(self):
        """Test that default limits are reasonable for production use."""
        # These should be high enough for legitimate use but low enough to prevent DoS
        assert MAX_SIMULATIONS >= 1, "Must allow at least 1 simulation"
        assert MAX_SIMULATIONS <= 100, "Too many simulations could cause DoS"
        
        assert MAX_OBJECTS_PER_SIMULATION >= 10, "Must allow reasonable number of objects"
        assert MAX_OBJECTS_PER_SIMULATION <= 10000, "Too many objects could cause DoS"
        
        assert MAX_CONSTRAINTS_PER_SIMULATION >= 10, "Must allow reasonable number of constraints"
        assert MAX_CONSTRAINTS_PER_SIMULATION <= 5000, "Too many constraints could cause DoS"
    
    def test_limits_are_documented(self):
        """Test that limits are defined as module-level constants."""
        from src.simulation_manager import (
            MAX_SIMULATIONS,
            MAX_OBJECTS_PER_SIMULATION,
            MAX_CONSTRAINTS_PER_SIMULATION
        )
        
        # Verify they're integers
        assert isinstance(MAX_SIMULATIONS, int)
        assert isinstance(MAX_OBJECTS_PER_SIMULATION, int)
        assert isinstance(MAX_CONSTRAINTS_PER_SIMULATION, int)
        
        # Verify they're positive
        assert MAX_SIMULATIONS > 0
        assert MAX_OBJECTS_PER_SIMULATION > 0
        assert MAX_CONSTRAINTS_PER_SIMULATION > 0
