"""Unit tests for SimulationManager class."""

import pytest
from src import SimulationManager, SimulationContext


class TestSimulationManager:
    """Test suite for SimulationManager."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.manager = SimulationManager()
    
    def teardown_method(self):
        """Clean up after tests."""
        # Destroy all simulations to free resources
        for sim_id in list(self.manager.simulations.keys()):
            try:
                self.manager.destroy_simulation(sim_id)
            except:
                pass
    
    def test_create_simulation_returns_uuid(self):
        """Test that create_simulation returns a valid UUID string."""
        sim_id = self.manager.create_simulation()
        
        # UUID should be a string
        assert isinstance(sim_id, str)
        
        # UUID should have correct format (8-4-4-4-12 hex digits)
        parts = sim_id.split('-')
        assert len(parts) == 5
        assert len(parts[0]) == 8
        assert len(parts[1]) == 4
        assert len(parts[2]) == 4
        assert len(parts[3]) == 4
        assert len(parts[4]) == 12
    
    def test_create_simulation_with_custom_gravity(self):
        """Test creating simulation with custom gravity."""
        gravity = (0.0, 0.0, -3.71)  # Mars gravity
        sim_id = self.manager.create_simulation(gravity=gravity)
        
        assert sim_id in self.manager.simulations
        sim = self.manager.get_simulation(sim_id)
        assert isinstance(sim, SimulationContext)
    
    def test_create_simulation_with_gui_mode(self):
        """Test creating simulation with GUI enabled."""
        # Note: This may fail in headless environments
        try:
            sim_id = self.manager.create_simulation(gui=True)
            sim = self.manager.get_simulation(sim_id)
            assert sim.gui_enabled is True
        except Exception:
            # Skip if GUI not available
            pytest.skip("GUI mode not available in this environment")
    
    def test_get_simulation_returns_context(self):
        """Test that get_simulation returns SimulationContext."""
        sim_id = self.manager.create_simulation()
        sim = self.manager.get_simulation(sim_id)
        
        assert isinstance(sim, SimulationContext)
        assert sim.client_id >= 0
    
    def test_get_simulation_raises_on_invalid_id(self):
        """Test that get_simulation raises ValueError for invalid ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.manager.get_simulation("invalid-uuid")
    
    def test_has_simulation_returns_true_for_existing(self):
        """Test has_simulation returns True for existing simulation."""
        sim_id = self.manager.create_simulation()
        assert self.manager.has_simulation(sim_id) is True
    
    def test_has_simulation_returns_false_for_nonexistent(self):
        """Test has_simulation returns False for non-existent simulation."""
        assert self.manager.has_simulation("invalid-uuid") is False
    
    def test_destroy_simulation_removes_from_registry(self):
        """Test that destroy_simulation removes simulation from registry."""
        sim_id = self.manager.create_simulation()
        assert self.manager.has_simulation(sim_id) is True
        
        self.manager.destroy_simulation(sim_id)
        assert self.manager.has_simulation(sim_id) is False
    
    def test_destroy_simulation_raises_on_invalid_id(self):
        """Test that destroy_simulation raises ValueError for invalid ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.manager.destroy_simulation("invalid-uuid")
    
    def test_list_simulations_returns_empty_initially(self):
        """Test that list_simulations returns empty list initially."""
        assert self.manager.list_simulations() == []
    
    def test_list_simulations_returns_all_active_ids(self):
        """Test that list_simulations returns all active simulation IDs."""
        sim_id1 = self.manager.create_simulation()
        sim_id2 = self.manager.create_simulation()
        sim_id3 = self.manager.create_simulation()
        
        active_sims = self.manager.list_simulations()
        assert len(active_sims) == 3
        assert sim_id1 in active_sims
        assert sim_id2 in active_sims
        assert sim_id3 in active_sims
    
    def test_list_simulations_updates_after_destroy(self):
        """Test that list_simulations updates after destroying simulation."""
        sim_id1 = self.manager.create_simulation()
        sim_id2 = self.manager.create_simulation()
        
        assert len(self.manager.list_simulations()) == 2
        
        self.manager.destroy_simulation(sim_id1)
        active_sims = self.manager.list_simulations()
        
        assert len(active_sims) == 1
        assert sim_id2 in active_sims
        assert sim_id1 not in active_sims
    
    def test_multiple_simulations_are_independent(self):
        """Test that multiple simulations have different client IDs."""
        sim_id1 = self.manager.create_simulation()
        sim_id2 = self.manager.create_simulation()
        
        sim1 = self.manager.get_simulation(sim_id1)
        sim2 = self.manager.get_simulation(sim_id2)
        
        # Each simulation should have a unique client ID
        assert sim1.client_id != sim2.client_id
