"""ConstraintManager class for managing constraints in PyBullet simulations."""

from typing import Dict, Any, List, Optional
import pybullet as p

from .simulation_manager import SimulationManager


# Joint type mapping for PyBullet
# Note: PyBullet's createConstraint API has limited joint type support compared to URDF joints.
# The following constraint types are supported:
# - "fixed": JOINT_FIXED (4) - Fixed constraint with 0 DOF
# - "prismatic": JOINT_PRISMATIC (1) - Sliding constraint with 1 DOF translation
# - "spherical": JOINT_POINT2POINT (5) - Ball/spherical joint with 3 DOF rotation
#
# IMPORTANT LIMITATION: PyBullet's createConstraint does NOT support revolute (hinge) joints.
# JOINT_REVOLUTE (0) is only for URDF joint definitions, not runtime constraints.
# For revolute joints, users must define them in URDF files and load the model.
#
# We raise a clear error for "revolute" to inform users of this limitation.
JOINT_TYPE_MAP = {
    "fixed": p.JOINT_FIXED,
    "prismatic": p.JOINT_PRISMATIC,
    "spherical": p.JOINT_POINT2POINT,  # Ball joint with 3 DOF rotation
}


class ConstraintManager:
    """Manages constraints (joints) in PyBullet physics simulations.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self, simulation_manager: SimulationManager):
        """Initialize the constraint manager.
        
        Args:
            simulation_manager: SimulationManager instance for accessing simulations.
        """
        self.simulation_manager = simulation_manager
    
    def create_constraint(
        self,
        sim_id: str,
        parent_id: int,
        child_id: int,
        joint_type: str,
        joint_axis: Optional[List[float]] = None,
        parent_frame_position: Optional[List[float]] = None,
        child_frame_position: Optional[List[float]] = None,
        parent_frame_orientation: Optional[List[float]] = None,
        child_frame_orientation: Optional[List[float]] = None
    ) -> int:
        """Create a constraint (joint) between two objects.
        
        Args:
            sim_id: UUID string identifying the simulation.
            parent_id: PyBullet object ID of the parent body.
            child_id: PyBullet object ID of the child body.
            joint_type: Type of joint - "fixed", "revolute", "prismatic", or "spherical".
            joint_axis: Axis of rotation/translation [x, y, z]. Default is [0, 0, 1].
            parent_frame_position: Position in parent frame [x, y, z]. Default is [0, 0, 0].
            child_frame_position: Position in child frame [x, y, z]. Default is [0, 0, 0].
            parent_frame_orientation: Orientation in parent frame as quaternion [x, y, z, w].
                                     Default is [0, 0, 0, 1].
            child_frame_orientation: Orientation in child frame as quaternion [x, y, z, w].
                                    Default is [0, 0, 0, 1].
            
        Returns:
            PyBullet constraint ID (integer).
            
        Raises:
            ValueError: If simulation not found, invalid joint type, or objects don't exist.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Special handling for revolute joints - PyBullet limitation
        if joint_type == "revolute":
            raise ValueError(
                "PyBullet's createConstraint API does not support revolute (hinge) joints. "
                "Revolute joints can only be defined in URDF files. "
                "For runtime constraints, use 'fixed', 'prismatic', or 'spherical' joint types. "
                "Alternatively, create a URDF model with revolute joints and load it using load_urdf()."
            )
        
        # Validate joint type
        if joint_type not in JOINT_TYPE_MAP:
            raise ValueError(
                f"Invalid joint type: {joint_type}. Must be one of: {list(JOINT_TYPE_MAP.keys())}"
            )
        
        # Validate objects exist in simulation
        if parent_id not in sim.objects:
            raise ValueError(f"Parent object {parent_id} not found in simulation {sim_id}")
        if child_id not in sim.objects:
            raise ValueError(f"Child object {child_id} not found in simulation {sim_id}")
        
        # Set defaults
        if joint_axis is None:
            joint_axis = [0.0, 0.0, 1.0]
        if parent_frame_position is None:
            parent_frame_position = [0.0, 0.0, 0.0]
        if child_frame_position is None:
            child_frame_position = [0.0, 0.0, 0.0]
        if parent_frame_orientation is None:
            parent_frame_orientation = [0.0, 0.0, 0.0, 1.0]
        if child_frame_orientation is None:
            child_frame_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Create constraint
        constraint_id = p.createConstraint(
            parentBodyUniqueId=parent_id,
            parentLinkIndex=-1,  # -1 for base link
            childBodyUniqueId=child_id,
            childLinkIndex=-1,  # -1 for base link
            jointType=JOINT_TYPE_MAP[joint_type],
            jointAxis=joint_axis,
            parentFramePosition=parent_frame_position,
            childFramePosition=child_frame_position,
            parentFrameOrientation=parent_frame_orientation,
            childFrameOrientation=child_frame_orientation,
            physicsClientId=sim.client_id
        )
        
        # Track constraint metadata in simulation context
        metadata = {
            "parent_id": parent_id,
            "child_id": child_id,
            "joint_type": joint_type,
            "joint_axis": joint_axis,
            "parent_frame_position": parent_frame_position,
            "child_frame_position": child_frame_position,
            "parent_frame_orientation": parent_frame_orientation,
            "child_frame_orientation": child_frame_orientation
        }
        sim.add_constraint(constraint_id, metadata)
        
        return constraint_id
    
    def set_constraint_params(
        self,
        sim_id: str,
        constraint_id: int,
        max_force: Optional[float] = None,
        gear_ratio: Optional[float] = None,
        gear_aux_link: Optional[int] = None,
        relative_position_target: Optional[float] = None,
        erp: Optional[float] = None
    ) -> None:
        """Configure constraint parameters such as limits, damping, and friction.
        
        Args:
            sim_id: UUID string identifying the simulation.
            constraint_id: PyBullet constraint ID.
            max_force: Maximum force the constraint can apply. Default is None (no change).
            gear_ratio: Gear ratio for the constraint. Default is None (no change).
            gear_aux_link: Auxiliary link for gear constraint. Default is None (no change).
            relative_position_target: Target position for the constraint. Default is None (no change).
            erp: Error reduction parameter (constraint stiffness). Default is None (no change).
            
        Raises:
            ValueError: If simulation or constraint not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate constraint exists in simulation
        if constraint_id not in sim.constraints:
            raise ValueError(f"Constraint {constraint_id} not found in simulation {sim_id}")
        
        # Build kwargs for changeConstraint - only include non-None parameters
        kwargs = {"physicsClientId": sim.client_id}
        
        if max_force is not None:
            kwargs["maxForce"] = max_force
        if gear_ratio is not None:
            kwargs["gearRatio"] = gear_ratio
        if gear_aux_link is not None:
            kwargs["gearAuxLink"] = gear_aux_link
        if relative_position_target is not None:
            kwargs["relativePositionTarget"] = relative_position_target
        if erp is not None:
            kwargs["erp"] = erp
        
        # Apply constraint parameters
        p.changeConstraint(constraint_id, **kwargs)
        
        # Update metadata with new parameters
        metadata = sim.get_constraint(constraint_id)
        if metadata:
            if max_force is not None:
                metadata["max_force"] = max_force
            if gear_ratio is not None:
                metadata["gear_ratio"] = gear_ratio
            if gear_aux_link is not None:
                metadata["gear_aux_link"] = gear_aux_link
            if relative_position_target is not None:
                metadata["relative_position_target"] = relative_position_target
            if erp is not None:
                metadata["erp"] = erp
    
    def remove_constraint(
        self,
        sim_id: str,
        constraint_id: int
    ) -> None:
        """Remove a constraint from the simulation.
        
        Args:
            sim_id: UUID string identifying the simulation.
            constraint_id: PyBullet constraint ID.
            
        Raises:
            ValueError: If simulation or constraint not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate constraint exists in simulation
        if constraint_id not in sim.constraints:
            raise ValueError(f"Constraint {constraint_id} not found in simulation {sim_id}")
        
        # Remove constraint from PyBullet
        p.removeConstraint(constraint_id, physicsClientId=sim.client_id)
        
        # Remove from simulation context
        sim.remove_constraint(constraint_id)
    
    def get_constraint_info(
        self,
        sim_id: str,
        constraint_id: int
    ) -> Dict[str, Any]:
        """Get constraint metadata and current state.
        
        Args:
            sim_id: UUID string identifying the simulation.
            constraint_id: PyBullet constraint ID.
            
        Returns:
            Dictionary containing constraint metadata.
            
        Raises:
            ValueError: If simulation or constraint not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate constraint exists in simulation
        if constraint_id not in sim.constraints:
            raise ValueError(f"Constraint {constraint_id} not found in simulation {sim_id}")
        
        # Get metadata from simulation context
        metadata = sim.get_constraint(constraint_id)
        
        # Return a copy to prevent external modification
        return dict(metadata) if metadata else {}
