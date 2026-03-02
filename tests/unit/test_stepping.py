"""Unit tests for simulation stepping methods."""

import pytest
from src import SimulationManager, ObjectManager


class TestSimulationStepping:
    """Test suite for simulation stepping functionality."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.manager = SimulationManager()
        self.object_manager = ObjectManager(self.manager)
    
    def teardown_method(self):
        """Clean up after tests."""
        # Destroy all simulations to free resources
        for sim_id in list(self.manager.simulations.keys()):
            try:
                self.manager.destroy_simulation(sim_id)
            except:
                pass
    
    def test_step_simulation_advances_time(self):
        """Test that step_simulation advances simulation time by one timestep."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        initial_time = sim.simulation_time
        initial_timestep = sim.timestep
        
        self.manager.step_simulation(sim_id)
        
        assert sim.simulation_time == initial_time + initial_timestep
    
    def test_step_simulation_raises_on_invalid_id(self):
        """Test that step_simulation raises ValueError for invalid simulation ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.manager.step_simulation("invalid-uuid")
    
    def test_step_multiple_advances_time_correctly(self):
        """Test that step_multiple advances simulation time by steps * timestep."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        initial_time = sim.simulation_time
        initial_timestep = sim.timestep
        steps = 10
        
        self.manager.step_multiple(sim_id, steps)
        
        expected_time = initial_time + (initial_timestep * steps)
        assert abs(sim.simulation_time - expected_time) < 1e-9  # Account for floating point precision
    
    def test_step_multiple_with_zero_steps(self):
        """Test that step_multiple with 0 steps doesn't change time."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        initial_time = sim.simulation_time
        
        self.manager.step_multiple(sim_id, 0)
        
        assert sim.simulation_time == initial_time
    
    def test_step_multiple_raises_on_negative_steps(self):
        """Test that step_multiple raises ValueError for negative steps."""
        sim_id = self.manager.create_simulation()
        
        with pytest.raises(ValueError, match="Steps must be non-negative"):
            self.manager.step_multiple(sim_id, -5)
    
    def test_step_multiple_raises_on_invalid_id(self):
        """Test that step_multiple raises ValueError for invalid simulation ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.manager.step_multiple("invalid-uuid", 10)
    
    def test_set_timestep_updates_timestep(self):
        """Test that set_timestep updates the simulation timestep."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        new_timestep = 0.02
        self.manager.set_timestep(sim_id, new_timestep)
        
        assert sim.timestep == new_timestep
    
    def test_set_timestep_affects_subsequent_steps(self):
        """Test that set_timestep affects subsequent simulation steps."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        new_timestep = 0.05
        self.manager.set_timestep(sim_id, new_timestep)
        
        initial_time = sim.simulation_time
        self.manager.step_simulation(sim_id)
        
        assert sim.simulation_time == initial_time + new_timestep
    
    def test_set_timestep_raises_on_zero(self):
        """Test that set_timestep raises ValueError for zero timestep."""
        sim_id = self.manager.create_simulation()
        
        with pytest.raises(ValueError, match="Timestep must be positive"):
            self.manager.set_timestep(sim_id, 0.0)
    
    def test_set_timestep_raises_on_negative(self):
        """Test that set_timestep raises ValueError for negative timestep."""
        sim_id = self.manager.create_simulation()
        
        with pytest.raises(ValueError, match="Timestep must be positive"):
            self.manager.set_timestep(sim_id, -0.01)
    
    def test_set_timestep_raises_on_invalid_id(self):
        """Test that set_timestep raises ValueError for invalid simulation ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.manager.set_timestep("invalid-uuid", 0.01)

    def test_physics_state_updates_during_stepping(self):
        """Test that objects with gravity fall when simulation steps.
        
        Validates: Requirements 4.4
        """
        # Create simulation with Earth gravity
        sim_id = self.manager.create_simulation(gravity=(0.0, 0.0, -9.81))
        
        # Create a sphere object at height 5.0 meters
        initial_position = [0.0, 0.0, 5.0]
        object_id = self.object_manager.create_primitive(
            sim_id=sim_id,
            shape="sphere",
            dimensions=[0.5],  # radius
            position=initial_position,
            mass=1.0
        )
        
        # Get initial state
        initial_state = self.object_manager.get_object_state(sim_id, object_id)
        initial_z = initial_state["position"][2]
        
        # Step the simulation multiple times to allow gravity to take effect
        self.manager.step_multiple(sim_id, 100)
        
        # Get state after stepping
        final_state = self.object_manager.get_object_state(sim_id, object_id)
        final_z = final_state["position"][2]
        
        # Verify that the object has fallen (z position decreased)
        assert final_z < initial_z, (
            f"Object should have fallen under gravity. "
            f"Initial z: {initial_z}, Final z: {final_z}"
        )
        
        # Verify that the object has downward velocity
        final_velocity_z = final_state["linear_velocity"][2]
        assert final_velocity_z < 0, (
            f"Object should have negative z velocity (falling). "
            f"Got velocity_z: {final_velocity_z}"
        )
        
        # Verify that the object has moved a significant distance
        # With gravity -9.81 m/s^2 and 100 steps at default timestep (1/240 s)
        # the object should fall several meters
        distance_fallen = initial_z - final_z
        assert distance_fallen > 0.5, (
            f"Object should have fallen at least 0.5 meters. "
            f"Actual distance: {distance_fallen}"
        )
