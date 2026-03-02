"""Unit tests for CollisionQueryHandler class."""

import pytest
import pybullet as p
from src import SimulationManager, CollisionQueryHandler, ObjectManager


class TestCollisionQueryHandler:
    """Test suite for CollisionQueryHandler."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.sim_manager = SimulationManager()
        self.obj_manager = ObjectManager(self.sim_manager)
        self.collision_handler = CollisionQueryHandler(self.sim_manager)
    
    def teardown_method(self):
        """Clean up after tests."""
        # Destroy all simulations to free resources
        for sim_id in list(self.sim_manager.simulations.keys()):
            try:
                self.sim_manager.destroy_simulation(sim_id)
            except:
                pass
    
    def test_get_all_contacts_returns_empty_list_when_no_collisions(self):
        """Test that get_all_contacts returns empty list when no objects collide."""
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects far apart
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 5], 1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [10, 0, 5], 1.0
        )
        
        # Step simulation
        self.sim_manager.step_simulation(sim_id)
        
        # Query contacts
        contacts = self.collision_handler.get_all_contacts(sim_id)
        
        # Should be empty since objects are far apart
        assert isinstance(contacts, list)
        assert len(contacts) == 0
    
    def test_get_all_contacts_detects_collisions(self):
        """Test that get_all_contacts detects colliding objects."""
        sim_id = self.sim_manager.create_simulation()
        
        # Create two overlapping boxes
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], 1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1.5], 1.0
        )
        
        # Step simulation to process collisions
        self.sim_manager.step_simulation(sim_id)
        
        # Query contacts
        contacts = self.collision_handler.get_all_contacts(sim_id)
        
        # Should detect collision
        assert isinstance(contacts, list)
        assert len(contacts) > 0
        
        # Verify contact structure
        contact = contacts[0]
        assert "object_a" in contact
        assert "object_b" in contact
        assert "position_on_a" in contact
        assert "position_on_b" in contact
        assert "contact_normal" in contact
        assert "contact_distance" in contact
        assert "normal_force" in contact
    
    def test_get_contacts_for_pair_returns_empty_when_not_colliding(self):
        """Test that get_contacts_for_pair returns empty list for non-colliding pair."""
        sim_id = self.sim_manager.create_simulation()
        
        # Create two objects far apart
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 5], 1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [10, 0, 5], 1.0
        )
        
        # Step simulation
        self.sim_manager.step_simulation(sim_id)
        
        # Query contacts for specific pair
        contacts = self.collision_handler.get_contacts_for_pair(sim_id, obj1, obj2)
        
        # Should be empty
        assert isinstance(contacts, list)
        assert len(contacts) == 0
    
    def test_get_contacts_for_pair_detects_specific_collision(self):
        """Test that get_contacts_for_pair detects collision between specific objects."""
        sim_id = self.sim_manager.create_simulation()
        
        # Create three objects: two colliding, one separate
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], 1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1.5], 1.0
        )
        obj3 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [10, 0, 5], 1.0
        )
        
        # Step simulation
        self.sim_manager.step_simulation(sim_id)
        
        # Query contacts for colliding pair
        contacts_12 = self.collision_handler.get_contacts_for_pair(sim_id, obj1, obj2)
        
        # Should detect collision between obj1 and obj2
        assert len(contacts_12) > 0
        
        # Query contacts for non-colliding pair
        contacts_13 = self.collision_handler.get_contacts_for_pair(sim_id, obj1, obj3)
        
        # Should not detect collision between obj1 and obj3
        assert len(contacts_13) == 0
    
    def test_get_all_contacts_raises_on_invalid_sim_id(self):
        """Test that get_all_contacts raises ValueError for invalid simulation ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.collision_handler.get_all_contacts("invalid-uuid")
    
    def test_get_contacts_for_pair_raises_on_invalid_sim_id(self):
        """Test that get_contacts_for_pair raises ValueError for invalid simulation ID."""
        with pytest.raises(ValueError, match="Simulation .* not found"):
            self.collision_handler.get_contacts_for_pair("invalid-uuid", 1, 2)
    
    def test_format_contact_info_structures_data_correctly(self):
        """Test that format_contact_info properly structures contact data."""
        # Create a mock contact point tuple (as returned by PyBullet)
        mock_contact = (
            0,  # contactFlag
            1,  # bodyUniqueIdA
            2,  # bodyUniqueIdB
            -1,  # linkIndexA
            -1,  # linkIndexB
            (0.0, 0.0, 1.0),  # positionOnA
            (0.0, 0.0, 1.5),  # positionOnB
            (0.0, 0.0, 1.0),  # contactNormalOnB
            -0.1,  # contactDistance
            9.81  # normalForce
        )
        
        formatted = self.collision_handler.format_contact_info(mock_contact)
        
        # Verify structure
        assert formatted["object_a"] == 1
        assert formatted["object_b"] == 2
        assert formatted["position_on_a"] == [0.0, 0.0, 1.0]
        assert formatted["position_on_b"] == [0.0, 0.0, 1.5]
        assert formatted["contact_normal"] == [0.0, 0.0, 1.0]
        assert formatted["contact_distance"] == -0.1
        assert formatted["normal_force"] == 9.81
    
    def test_contact_info_contains_all_required_fields(self):
        """Test that formatted contact info contains all required fields."""
        sim_id = self.sim_manager.create_simulation()
        
        # Create two overlapping boxes
        obj1 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1], 1.0
        )
        obj2 = self.obj_manager.create_primitive(
            sim_id, "box", [0.5, 0.5, 0.5], [0, 0, 1.5], 1.0
        )
        
        # Step simulation
        self.sim_manager.step_simulation(sim_id)
        
        # Query contacts
        contacts = self.collision_handler.get_all_contacts(sim_id)
        
        if len(contacts) > 0:
            contact = contacts[0]
            
            # Verify all required fields exist
            required_fields = [
                "object_a", "object_b", "position_on_a", "position_on_b",
                "contact_normal", "contact_distance", "normal_force"
            ]
            for field in required_fields:
                assert field in contact, f"Missing required field: {field}"
            
            # Verify field types
            assert isinstance(contact["object_a"], int)
            assert isinstance(contact["object_b"], int)
            assert isinstance(contact["position_on_a"], list)
            assert isinstance(contact["position_on_b"], list)
            assert isinstance(contact["contact_normal"], list)
            assert isinstance(contact["contact_distance"], (int, float))
            assert isinstance(contact["normal_force"], (int, float))
