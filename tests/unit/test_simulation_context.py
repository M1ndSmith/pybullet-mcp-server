"""Unit tests for SimulationContext data class."""

import pytest
import pybullet as p
from src.simulation_context import SimulationContext


class TestSimulationContext:
    """Test suite for SimulationContext class."""
    
    def test_initialization_with_defaults(self):
        """Test that SimulationContext initializes with default values."""
        # Create a physics client
        client_id = p.connect(p.DIRECT)
        
        try:
            # Create context
            context = SimulationContext(client_id=client_id, gui_enabled=False)
            
            # Verify attributes
            assert context.client_id == client_id
            assert context.gui_enabled is False
            assert context.timestep == 1.0 / 240.0  # Default timestep
            assert context.objects == {}
            assert context.constraints == {}
        finally:
            # Cleanup
            context.cleanup()
    
    def test_initialization_with_custom_timestep(self):
        """Test that SimulationContext can be initialized with custom timestep."""
        client_id = p.connect(p.DIRECT)
        
        try:
            custom_timestep = 0.01
            context = SimulationContext(
                client_id=client_id,
                gui_enabled=False,
                timestep=custom_timestep
            )
            
            assert context.timestep == custom_timestep
        finally:
            context.cleanup()
    
    def test_cleanup_disconnects_client(self):
        """Test that cleanup properly disconnects the physics client."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        # Cleanup should disconnect without error
        context.cleanup()
        
        # Attempting to use the client after cleanup should fail
        with pytest.raises(p.error):
            p.getNumBodies(physicsClientId=client_id)
    
    def test_add_and_get_object(self):
        """Test adding and retrieving object metadata."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        try:
            # Add object metadata
            object_id = 1
            metadata = {
                "type": "primitive",
                "shape": "box",
                "mass": 1.0,
                "position": [0, 0, 1]
            }
            context.add_object(object_id, metadata)
            
            # Retrieve and verify
            retrieved = context.get_object(object_id)
            assert retrieved == metadata
            assert object_id in context.objects
        finally:
            context.cleanup()
    
    def test_remove_object(self):
        """Test removing object metadata."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        try:
            # Add object
            object_id = 1
            metadata = {"type": "primitive"}
            context.add_object(object_id, metadata)
            
            # Remove object
            result = context.remove_object(object_id)
            assert result is True
            assert object_id not in context.objects
            assert context.get_object(object_id) is None
            
            # Try removing non-existent object
            result = context.remove_object(999)
            assert result is False
        finally:
            context.cleanup()
    
    def test_add_and_get_constraint(self):
        """Test adding and retrieving constraint metadata."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        try:
            # Add constraint metadata
            constraint_id = 1
            metadata = {
                "type": "revolute",
                "parent_id": 1,
                "child_id": 2,
                "joint_axis": [0, 0, 1]
            }
            context.add_constraint(constraint_id, metadata)
            
            # Retrieve and verify
            retrieved = context.get_constraint(constraint_id)
            assert retrieved == metadata
            assert constraint_id in context.constraints
        finally:
            context.cleanup()
    
    def test_remove_constraint(self):
        """Test removing constraint metadata."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        try:
            # Add constraint
            constraint_id = 1
            metadata = {"type": "fixed"}
            context.add_constraint(constraint_id, metadata)
            
            # Remove constraint
            result = context.remove_constraint(constraint_id)
            assert result is True
            assert constraint_id not in context.constraints
            assert context.get_constraint(constraint_id) is None
            
            # Try removing non-existent constraint
            result = context.remove_constraint(999)
            assert result is False
        finally:
            context.cleanup()
    
    def test_set_timestep(self):
        """Test updating the simulation timestep."""
        client_id = p.connect(p.DIRECT)
        context = SimulationContext(client_id=client_id, gui_enabled=False)
        
        try:
            # Set new timestep
            new_timestep = 0.02
            context.set_timestep(new_timestep)
            
            # Verify it was updated
            assert context.timestep == new_timestep
        finally:
            context.cleanup()
    
    def test_gui_enabled_flag(self):
        """Test that gui_enabled flag is properly stored."""
        client_id = p.connect(p.DIRECT)
        
        try:
            # Test with GUI disabled
            context_no_gui = SimulationContext(client_id=client_id, gui_enabled=False)
            assert context_no_gui.gui_enabled is False
            context_no_gui.cleanup()
            
            # Test with GUI enabled (still using DIRECT mode for testing)
            client_id2 = p.connect(p.DIRECT)
            context_gui = SimulationContext(client_id=client_id2, gui_enabled=True)
            assert context_gui.gui_enabled is True
            context_gui.cleanup()
        except:
            # Ensure cleanup even if test fails
            try:
                p.disconnect(physicsClientId=client_id)
            except:
                pass
