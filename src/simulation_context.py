"""SimulationContext data class for managing PyBullet simulation state."""

from dataclasses import dataclass, field
from typing import Dict, Any, Optional
import pybullet as p


@dataclass
class SimulationContext:
    """Encapsulates the state of a single PyBullet physics simulation.

    Attributes:
        client_id: PyBullet physics client ID
        gravity: Gravity vector (x, y, z) in m/s^2
        objects: Dictionary mapping object IDs to metadata
        constraints: Dictionary mapping constraint IDs to metadata
        timestep: Current simulation timestep duration in seconds
        gui_enabled: Whether GUI visualization is active
        simulation_time: Total elapsed simulation time in seconds
        debug_contact_points: Whether to visualize contact points
        debug_frames: Whether to visualize coordinate frames
    """

    client_id: int
    gui_enabled: bool
    gravity: tuple = (0.0, 0.0, -9.81)
    timestep: float = 1.0 / 240.0  # Default PyBullet timestep
    simulation_time: float = 0.0
    objects: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    constraints: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    debug_contact_points: bool = False
    debug_frames: bool = False

    def __post_init__(self):
        """Initialize the simulation context after dataclass initialization."""
        # Set the timestep for the physics client
        p.setTimeStep(self.timestep, physicsClientId=self.client_id)

    def cleanup(self) -> None:
        """Clean up simulation resources and disconnect the physics client."""
        try:
            # Disconnect the physics client to free resources
            p.disconnect(physicsClientId=self.client_id)
        except p.error as e:
            # Handle case where client is already disconnected
            pass

    def add_object(self, object_id: int, metadata: Dict[str, Any]) -> None:
        """Add an object to the simulation context.

        Args:
            object_id: Unique identifier for the object in PyBullet
            metadata: Dictionary containing object properties and state
        """
        self.objects[object_id] = metadata

    def remove_object(self, object_id: int) -> bool:
        """Remove an object from the simulation context.

        Args:
            object_id: Unique identifier for the object

        Returns:
            True if object was removed, False if it didn't exist
        """
        if object_id in self.objects:
            del self.objects[object_id]
            return True
        return False

    def get_object(self, object_id: int) -> Optional[Dict[str, Any]]:
        """Get object metadata by ID.

        Args:
            object_id: Unique identifier for the object

        Returns:
            Object metadata dictionary or None if not found
        """
        return self.objects.get(object_id)

    def add_constraint(self, constraint_id: int, metadata: Dict[str, Any]) -> None:
        """Add a constraint to the simulation context.

        Args:
            constraint_id: Unique identifier for the constraint in PyBullet
            metadata: Dictionary containing constraint properties
        """
        self.constraints[constraint_id] = metadata

    def remove_constraint(self, constraint_id: int) -> bool:
        """Remove a constraint from the simulation context.

        Args:
            constraint_id: Unique identifier for the constraint

        Returns:
            True if constraint was removed, False if it didn't exist
        """
        if constraint_id in self.constraints:
            del self.constraints[constraint_id]
            return True
        return False

    def get_constraint(self, constraint_id: int) -> Optional[Dict[str, Any]]:
        """Get constraint metadata by ID.

        Args:
            constraint_id: Unique identifier for the constraint

        Returns:
            Constraint metadata dictionary or None if not found
        """
        return self.constraints.get(constraint_id)

    def set_timestep(self, timestep: float) -> None:
        """Update the simulation timestep duration.

        Args:
            timestep: New timestep duration in seconds
        """
        self.timestep = timestep
        p.setTimeStep(timestep, physicsClientId=self.client_id)
