"""Unit tests for PersistenceHandler class."""

import pytest
import json
import os
import tempfile
from src import SimulationManager, ObjectManager
from src.persistence import PersistenceHandler


class TestPersistenceHandler:
    """Test suite for PersistenceHandler."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.sim_manager = SimulationManager()
        self.obj_manager = ObjectManager(self.sim_manager, strict_path_validation=False)
        self.persistence = PersistenceHandler(self.sim_manager, self.obj_manager, strict_path_validation=False)
    
    def teardown_method(self):
        """Clean up after tests."""
        # Destroy all simulations to free resources
        for sim_id in list(self.sim_manager.simulations.keys()):
            try:
                self.sim_manager.destroy_simulation(sim_id)
            except:
                pass
    
    def test_serialize_empty_simulation(self):
        """Test serializing an empty simulation."""
        sim_id = self.sim_manager.create_simulation()
        
        state = self.persistence.serialize_simulation(sim_id)
        
        assert "gravity" in state
        assert "timestep" in state
        assert "objects" in state
        assert "constraints" in state
        assert isinstance(state["objects"], list)
        assert isinstance(state["constraints"], list)
        assert len(state["objects"]) == 0
        assert len(state["constraints"]) == 0
    
    def test_serialize_simulation_with_gravity(self):
        """Test that serialization captures gravity correctly."""
        gravity = (0.0, 0.0, -3.71)  # Mars gravity
        sim_id = self.sim_manager.create_simulation(gravity=gravity)
        
        state = self.persistence.serialize_simulation(sim_id)
        
        assert state["gravity"] == list(gravity)
    
    def test_serialize_simulation_with_objects(self):
        """Test serializing simulation with objects."""
        sim_id = self.sim_manager.create_simulation()
        
        # Add a box
        obj_id = self.obj_manager.create_primitive(
            sim_id,
            "box",
            [0.5, 0.5, 0.5],
            [0.0, 0.0, 1.0],
            mass=2.0,
            color=[1.0, 0.0, 0.0, 1.0]
        )
        
        state = self.persistence.serialize_simulation(sim_id)
        
        assert len(state["objects"]) == 1
        obj_data = state["objects"][0]
        assert obj_data["type"] == "primitive"
        assert obj_data["shape"] == "box"
        assert obj_data["mass"] == 2.0
        assert obj_data["dimensions"] == [0.5, 0.5, 0.5]
        assert "position" in obj_data
        assert "orientation" in obj_data
        assert "linear_velocity" in obj_data
        assert "angular_velocity" in obj_data
    
    def test_deserialize_empty_simulation(self):
        """Test deserializing an empty simulation."""
        state = {
            "gravity": [0.0, 0.0, -9.81],
            "timestep": 0.01,
            "objects": [],
            "constraints": []
        }
        
        new_sim_id = self.persistence.deserialize_simulation(state)
        
        assert self.sim_manager.has_simulation(new_sim_id)
        sim = self.sim_manager.get_simulation(new_sim_id)
        assert sim.timestep == 0.01
    
    def test_deserialize_raises_on_missing_fields(self):
        """Test that deserialize raises ValueError for missing required fields."""
        # Missing gravity
        state = {"timestep": 0.01, "objects": [], "constraints": []}
        with pytest.raises(ValueError, match="State missing required field: gravity"):
            self.persistence.deserialize_simulation(state)
        
        # Missing timestep
        state = {"gravity": [0, 0, -9.81], "objects": [], "constraints": []}
        with pytest.raises(ValueError, match="State missing required field: timestep"):
            self.persistence.deserialize_simulation(state)
        
        # Missing objects
        state = {"gravity": [0, 0, -9.81], "timestep": 0.01, "constraints": []}
        with pytest.raises(ValueError, match="State missing required field: objects"):
            self.persistence.deserialize_simulation(state)
        
        # Missing constraints
        state = {"gravity": [0, 0, -9.81], "timestep": 0.01, "objects": []}
        with pytest.raises(ValueError, match="State missing required field: constraints"):
            self.persistence.deserialize_simulation(state)
    
    def test_save_state_creates_file(self):
        """Test that save_state creates a file."""
        sim_id = self.sim_manager.create_simulation()
        
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.json') as f:
            temp_path = f.name
        
        try:
            self.persistence.save_state(sim_id, temp_path)
            
            assert os.path.exists(temp_path)
            
            # Verify file contains valid JSON
            with open(temp_path, 'r') as f:
                data = json.load(f)
                assert "gravity" in data
                assert "timestep" in data
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
    
    def test_save_state_raises_on_invalid_simulation(self):
        """Test that save_state raises ValueError for invalid simulation."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.json') as f:
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError, match="Simulation .* not found"):
                self.persistence.save_state("invalid-uuid", temp_path)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
    
    def test_load_state_raises_on_missing_file(self):
        """Test that load_state raises IOError for missing file."""
        with pytest.raises(IOError, match="File not found"):
            self.persistence.load_state("/nonexistent/path/file.json")
    
    def test_load_state_raises_on_invalid_json(self):
        """Test that load_state raises ValueError for invalid JSON."""
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.json') as f:
            f.write("{ invalid json }")
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError, match="Invalid JSON"):
                self.persistence.load_state(temp_path)
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
    
    def test_round_trip_empty_simulation(self):
        """Test save and load round-trip for empty simulation."""
        # Create and save simulation
        sim_id = self.sim_manager.create_simulation(gravity=(0.0, 0.0, -5.0))
        self.sim_manager.set_timestep(sim_id, 0.02)
        
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.json') as f:
            temp_path = f.name
        
        try:
            self.persistence.save_state(sim_id, temp_path)
            
            # Load simulation
            new_sim_id = self.persistence.load_state(temp_path)
            
            # Verify loaded simulation matches original
            new_sim = self.sim_manager.get_simulation(new_sim_id)
            assert new_sim.timestep == 0.02
            assert len(new_sim.objects) == 0
            assert len(new_sim.constraints) == 0
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
    
    def test_round_trip_simulation_with_objects(self):
        """Test save and load round-trip for simulation with objects."""
        # Create simulation with objects
        sim_id = self.sim_manager.create_simulation()
        
        # Add multiple objects
        box_id = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0.0, 0.0, 1.0],
            mass=2.0, color=[1.0, 0.0, 0.0, 1.0]
        )
        sphere_id = self.obj_manager.create_primitive(
            sim_id, "sphere", [0.3], [1.0, 0.0, 1.0],
            mass=1.5, color=[0.0, 1.0, 0.0, 1.0]
        )
        
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.json') as f:
            temp_path = f.name
        
        try:
            self.persistence.save_state(sim_id, temp_path)
            
            # Load simulation
            new_sim_id = self.persistence.load_state(temp_path)
            
            # Verify loaded simulation has same number of objects
            new_sim = self.sim_manager.get_simulation(new_sim_id)
            assert len(new_sim.objects) == 2
            
            # Verify object types
            obj_types = [obj["type"] for obj in new_sim.objects.values()]
            assert "primitive" in obj_types
        finally:
            if os.path.exists(temp_path):
                os.unlink(temp_path)
