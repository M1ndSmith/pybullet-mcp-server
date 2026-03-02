"""CollisionQueryHandler class for querying collision information in PyBullet simulations."""

from typing import List, Dict, Any
import pybullet as p

from .simulation_manager import SimulationManager


class CollisionQueryHandler:
    """Handles collision detection queries for PyBullet simulations.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self, simulation_manager: SimulationManager):
        """Initialize the collision query handler.
        
        Args:
            simulation_manager: SimulationManager instance to access simulations.
        """
        self.simulation_manager = simulation_manager
    
    def get_all_contacts(self, sim_id: str) -> List[Dict[str, Any]]:
        """Query all contact points in the simulation.
        
        Args:
            sim_id: UUID string identifying the simulation.
            
        Returns:
            List of contact point dictionaries with positions, normals, and forces.
            Returns empty list if no collisions exist.
            
        Raises:
            ValueError: If simulation ID is not found.
        """
        # Get simulation context (raises ValueError if not found)
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Query all contact points from PyBullet
        contact_points = p.getContactPoints(physicsClientId=sim.client_id)
        
        # Format and return contact information
        return [self.format_contact_info(cp) for cp in contact_points]
    
    def get_contacts_for_pair(
        self, 
        sim_id: str, 
        obj_a: int, 
        obj_b: int
    ) -> List[Dict[str, Any]]:
        """Query contact points between a specific pair of objects.
        
        Args:
            sim_id: UUID string identifying the simulation.
            obj_a: First object ID.
            obj_b: Second object ID.
            
        Returns:
            List of contact point dictionaries for the specified pair.
            Returns empty list if objects are not in contact.
            
        Raises:
            ValueError: If simulation ID is not found.
        """
        # Get simulation context (raises ValueError if not found)
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Query contact points for specific object pair
        contact_points = p.getContactPoints(
            bodyA=obj_a,
            bodyB=obj_b,
            physicsClientId=sim.client_id
        )
        
        # Format and return contact information
        return [self.format_contact_info(cp) for cp in contact_points]
    
    def format_contact_info(self, contact_point: tuple) -> Dict[str, Any]:
        """Structure contact data into a dictionary format.
        
        PyBullet getContactPoints returns tuples with the following structure:
        [0] contactFlag
        [1] bodyUniqueIdA
        [2] bodyUniqueIdB
        [3] linkIndexA
        [4] linkIndexB
        [5] positionOnA (x, y, z)
        [6] positionOnB (x, y, z)
        [7] contactNormalOnB (x, y, z)
        [8] contactDistance
        [9] normalForce
        
        Args:
            contact_point: Raw contact point tuple from PyBullet.
            
        Returns:
            Dictionary with structured contact information including:
            - object_a: ID of first object
            - object_b: ID of second object
            - position_on_a: Contact position on first object [x, y, z]
            - position_on_b: Contact position on second object [x, y, z]
            - contact_normal: Normal vector at contact point [x, y, z]
            - contact_distance: Distance between objects (negative = penetration)
            - normal_force: Force magnitude along contact normal
        """
        return {
            "object_a": contact_point[1],
            "object_b": contact_point[2],
            "position_on_a": list(contact_point[5]),
            "position_on_b": list(contact_point[6]),
            "contact_normal": list(contact_point[7]),
            "contact_distance": contact_point[8],
            "normal_force": contact_point[9]
        }
