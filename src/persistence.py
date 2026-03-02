"""PersistenceHandler class for saving and loading PyBullet simulation states."""

import json
import os
from typing import Dict, Any, List
import pybullet as p

from .simulation_manager import SimulationManager
from .object_manager import ObjectManager


class PersistenceHandler:
    """Handles serialization and deserialization of simulation states.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, IOError, etc.) which
    MCP tools will convert to ToolError.
    """
    
    def __init__(
        self,
        simulation_manager: SimulationManager,
        object_manager: ObjectManager
    ):
        """Initialize the persistence handler.
        
        Args:
            simulation_manager: SimulationManager instance for accessing simulations.
            object_manager: ObjectManager instance for creating objects.
        """
        self.simulation_manager = simulation_manager
        self.object_manager = object_manager
    
    def serialize_simulation(self, sim_id: str) -> Dict[str, Any]:
        """Convert SimulationContext to JSON-serializable dictionary.
        
        Args:
            sim_id: UUID string identifying the simulation.
            
        Returns:
            Dictionary containing complete simulation state including:
                - gravity: [x, y, z]
                - timestep: float
                - objects: list of object data
                - constraints: list of constraint data
            
        Raises:
            ValueError: If simulation not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Serialize all objects
        objects = []
        for object_id in sim.objects.keys():
            obj_data = self._serialize_object(sim, object_id)
            objects.append(obj_data)
        
        # Serialize all constraints
        constraints = []
        for constraint_id in sim.constraints.keys():
            const_data = self._serialize_constraint(sim, constraint_id)
            constraints.append(const_data)
        
        return {
            "gravity": list(sim.gravity),
            "timestep": sim.timestep,
            "objects": objects,
            "constraints": constraints
        }
    
    def deserialize_simulation(
        self,
        state: Dict[str, Any],
        gui: bool = False
    ) -> str:
        """Recreate SimulationContext from dictionary.
        
        Args:
            state: Dictionary containing simulation state (from serialize_simulation).
            gui: Whether to enable GUI visualization for the new simulation.
            
        Returns:
            UUID string identifying the newly created simulation.
            
        Raises:
            ValueError: If state data is invalid or missing required fields.
        """
        # Validate required fields
        if "gravity" not in state:
            raise ValueError("State missing required field: gravity")
        if "timestep" not in state:
            raise ValueError("State missing required field: timestep")
        if "objects" not in state:
            raise ValueError("State missing required field: objects")
        if "constraints" not in state:
            raise ValueError("State missing required field: constraints")
        
        # Create new simulation with specified gravity
        sim_id = self.simulation_manager.create_simulation(
            tuple(state["gravity"]),
            gui
        )
        
        # Set timestep
        self.simulation_manager.set_timestep(sim_id, state["timestep"])
        
        # Recreate objects and track ID mapping (old ID -> new ID)
        object_id_map = {}
        for obj_data in state["objects"]:
            new_id = self._deserialize_object(sim_id, obj_data)
            object_id_map[obj_data["object_id"]] = new_id
        
        # Recreate constraints with remapped object IDs
        for const_data in state["constraints"]:
            self._deserialize_constraint(sim_id, const_data, object_id_map)
        
        return sim_id
    
    def save_state(self, sim_id: str, file_path: str) -> None:
        """Write simulation state to file.
        
        Args:
            sim_id: UUID string identifying the simulation.
            file_path: Path where the state file should be written.
            
        Raises:
            ValueError: If simulation not found.
            IOError: If file cannot be written (permission denied, disk full, etc.).
        """
        # Serialize simulation to dictionary
        state = self.serialize_simulation(sim_id)
        
        # Write to file with error handling
        try:
            with open(file_path, 'w') as f:
                json.dump(state, f, indent=2)
        except PermissionError:
            raise IOError(f"Permission denied writing to {file_path}")
        except OSError as e:
            raise IOError(f"Failed to write to {file_path}: {str(e)}")
    
    def load_state(self, file_path: str, gui: bool = False) -> str:
        """Read simulation from file and create new simulation.
        
        Args:
            file_path: Path to the state file to load.
            gui: Whether to enable GUI visualization for the loaded simulation.
            
        Returns:
            UUID string identifying the newly created simulation.
            
        Raises:
            IOError: If file cannot be read or doesn't exist.
            ValueError: If file contains invalid JSON or missing required fields.
        """
        # Validate file exists
        if not os.path.exists(file_path):
            raise IOError(f"File not found: {file_path}")
        
        # Read and parse file with error handling
        try:
            with open(file_path, 'r') as f:
                state = json.load(f)
        except PermissionError:
            raise IOError(f"Permission denied reading from {file_path}")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in {file_path}: {str(e)}")
        except OSError as e:
            raise IOError(f"Failed to read from {file_path}: {str(e)}")
        
        # Deserialize and create new simulation
        return self.deserialize_simulation(state, gui)
    
    def _serialize_object(
        self,
        sim,
        object_id: int
    ) -> Dict[str, Any]:
        """Convert object to JSON-serializable dictionary.
        
        Args:
            sim: SimulationContext instance.
            object_id: PyBullet object ID.
            
        Returns:
            Dictionary containing object state and metadata.
        """
        # Get object metadata from simulation context
        metadata = sim.objects[object_id]
        
        # Get current position and orientation
        pos, orn = p.getBasePositionAndOrientation(
            object_id,
            physicsClientId=sim.client_id
        )
        
        # Get current velocities
        lin_vel, ang_vel = p.getBaseVelocity(
            object_id,
            physicsClientId=sim.client_id
        )
        
        # Build serialized object data
        obj_data = {
            "object_id": object_id,
            "type": metadata["type"],
            "shape": metadata["shape"],
            "position": list(pos),
            "orientation": list(orn),
            "linear_velocity": list(lin_vel),
            "angular_velocity": list(ang_vel)
        }
        
        # Add type-specific metadata
        if metadata["type"] == "primitive":
            obj_data.update({
                "dimensions": metadata["dimensions"],
                "mass": metadata["mass"],
                "color": metadata["color"],
                "friction": metadata["friction"],
                "restitution": metadata["restitution"]
            })
        elif metadata["type"] == "urdf":
            obj_data["file_path"] = metadata["file_path"]
        
        return obj_data
    
    def _deserialize_object(
        self,
        sim_id: str,
        obj_data: Dict[str, Any]
    ) -> int:
        """Recreate object from serialized data.
        
        Args:
            sim_id: UUID string identifying the simulation.
            obj_data: Dictionary containing object state and metadata.
            
        Returns:
            New PyBullet object ID.
            
        Raises:
            ValueError: If object data is invalid.
        """
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Create object based on type
        if obj_data["type"] == "primitive":
            # Create primitive shape
            new_id = self.object_manager.create_primitive(
                sim_id,
                obj_data["shape"],
                obj_data["dimensions"],
                obj_data["position"],
                obj_data["mass"],
                obj_data["color"],
                obj_data["friction"],
                obj_data["restitution"]
            )
        elif obj_data["type"] == "urdf":
            # Load URDF model
            new_id = self.object_manager.load_urdf(
                sim_id,
                obj_data["file_path"],
                obj_data["position"],
                obj_data["orientation"]
            )
        else:
            raise ValueError(f"Unknown object type: {obj_data['type']}")
        
        # Restore orientation (for primitives, since URDF already has it)
        if obj_data["type"] == "primitive":
            p.resetBasePositionAndOrientation(
                new_id,
                obj_data["position"],
                obj_data["orientation"],
                physicsClientId=sim.client_id
            )
        
        # Restore velocities
        p.resetBaseVelocity(
            new_id,
            linearVelocity=obj_data["linear_velocity"],
            angularVelocity=obj_data["angular_velocity"],
            physicsClientId=sim.client_id
        )
        
        return new_id
    
    def _serialize_constraint(
        self,
        sim,
        constraint_id: int
    ) -> Dict[str, Any]:
        """Convert constraint to JSON-serializable dictionary.
        
        Args:
            sim: SimulationContext instance.
            constraint_id: PyBullet constraint ID.
            
        Returns:
            Dictionary containing constraint metadata.
        """
        # Get constraint metadata from simulation context
        metadata = sim.constraints[constraint_id]
        
        return {
            "constraint_id": constraint_id,
            **metadata
        }
    
    def _deserialize_constraint(
        self,
        sim_id: str,
        const_data: Dict[str, Any],
        object_id_map: Dict[int, int]
    ) -> int:
        """Recreate constraint from serialized data.
        
        Args:
            sim_id: UUID string identifying the simulation.
            const_data: Dictionary containing constraint metadata.
            object_id_map: Mapping from old object IDs to new object IDs.
            
        Returns:
            New PyBullet constraint ID.
            
        Raises:
            ValueError: If constraint data is invalid or references missing objects.
        """
        # This is a placeholder for constraint recreation
        # Actual implementation would depend on ConstraintManager
        # For now, we just track the metadata without creating the constraint
        # This will be implemented when ConstraintManager is available
        pass
