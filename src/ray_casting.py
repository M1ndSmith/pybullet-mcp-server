"""Ray casting functionality for sensor simulation."""

from typing import List, Dict, Any
import pybullet as p

from .simulation_manager import SimulationManager


class RayCastingHandler:
    """Handles ray casting operations for sensor simulation.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self, simulation_manager: SimulationManager):
        """Initialize the ray casting handler.
        
        Args:
            simulation_manager: SimulationManager instance for accessing simulations.
        """
        self.simulation_manager = simulation_manager
    
    def ray_test(
        self,
        sim_id: str,
        ray_from: List[float],
        ray_to: List[float]
    ) -> Dict[str, Any]:
        """Cast a single ray and return intersection information.
        
        Args:
            sim_id: UUID string identifying the simulation.
            ray_from: Starting position [x, y, z] of the ray.
            ray_to: Ending position [x, y, z] of the ray.
            
        Returns:
            Dictionary containing:
                - hit: Boolean indicating if ray hit an object
                - object_id: ID of hit object (-1 if no hit)
                - link_index: Link index of hit (-1 if base or no hit)
                - hit_fraction: Fraction along ray where hit occurred (0-1)
                - hit_position: Position [x, y, z] where ray hit
                - hit_normal: Surface normal [x, y, z] at hit point
                
        Raises:
            ValueError: If simulation not found or invalid ray parameters.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate ray parameters
        if len(ray_from) != 3:
            raise ValueError(f"ray_from must have 3 coordinates, got {len(ray_from)}")
        if len(ray_to) != 3:
            raise ValueError(f"ray_to must have 3 coordinates, got {len(ray_to)}")
        
        # Cast ray
        result = p.rayTest(
            ray_from,
            ray_to,
            physicsClientId=sim.client_id
        )[0]  # rayTest returns a list with one result
        
        # Parse result
        object_id = result[0]
        link_index = result[1]
        hit_fraction = result[2]
        hit_position = result[3]
        hit_normal = result[4]
        
        return {
            "hit": object_id >= 0,
            "object_id": object_id,
            "link_index": link_index,
            "hit_fraction": hit_fraction,
            "hit_position": list(hit_position),
            "hit_normal": list(hit_normal)
        }
    
    def ray_test_batch(
        self,
        sim_id: str,
        rays_from: List[List[float]],
        rays_to: List[List[float]]
    ) -> List[Dict[str, Any]]:
        """Cast multiple rays efficiently (for lidar/sensor simulation).
        
        Args:
            sim_id: UUID string identifying the simulation.
            rays_from: List of starting positions [[x, y, z], ...].
            rays_to: List of ending positions [[x, y, z], ...].
            
        Returns:
            List of dictionaries, one per ray, each containing:
                - hit: Boolean indicating if ray hit an object
                - object_id: ID of hit object (-1 if no hit)
                - link_index: Link index of hit (-1 if base or no hit)
                - hit_fraction: Fraction along ray where hit occurred (0-1)
                - hit_position: Position [x, y, z] where ray hit
                - hit_normal: Surface normal [x, y, z] at hit point
                
        Raises:
            ValueError: If simulation not found or mismatched array lengths.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate parameters
        if len(rays_from) != len(rays_to):
            raise ValueError(
                f"rays_from and rays_to must have same length, "
                f"got {len(rays_from)} and {len(rays_to)}"
            )
        
        if not rays_from:
            return []
        
        # Validate each ray
        for i, (ray_from, ray_to) in enumerate(zip(rays_from, rays_to)):
            if len(ray_from) != 3:
                raise ValueError(f"rays_from[{i}] must have 3 coordinates, got {len(ray_from)}")
            if len(ray_to) != 3:
                raise ValueError(f"rays_to[{i}] must have 3 coordinates, got {len(ray_to)}")
        
        # Cast all rays at once (efficient)
        results = p.rayTestBatch(
            rays_from,
            rays_to,
            physicsClientId=sim.client_id
        )
        
        # Parse results
        parsed_results = []
        for result in results:
            object_id = result[0]
            link_index = result[1]
            hit_fraction = result[2]
            hit_position = result[3]
            hit_normal = result[4]
            
            parsed_results.append({
                "hit": object_id >= 0,
                "object_id": object_id,
                "link_index": link_index,
                "hit_fraction": hit_fraction,
                "hit_position": list(hit_position),
                "hit_normal": list(hit_normal)
            })
        
        return parsed_results
