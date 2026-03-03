"""ObjectManager class for managing objects in PyBullet simulations."""

from typing import List, Dict, Any, Optional, Tuple
import os
import pybullet as p

from .simulation_manager import SimulationManager


# Shape type mapping for PyBullet
SHAPE_MAP = {
    "box": p.GEOM_BOX,
    "sphere": p.GEOM_SPHERE,
    "cylinder": p.GEOM_CYLINDER,
    "capsule": p.GEOM_CAPSULE
}


def validate_file_path(file_path: str, allowed_base: str = None, strict: bool = True) -> str:
    """Validate and normalize file path to prevent path traversal attacks.
    
    Args:
        file_path: Path to validate.
        allowed_base: Base directory to restrict access to. If None, uses current directory.
        strict: If True, enforce path restrictions. If False, only normalize path.
        
    Returns:
        Absolute normalized path.
        
    Raises:
        ValueError: If strict=True and path is outside allowed directory or contains suspicious patterns.
    """
    # Resolve to absolute path
    abs_path = os.path.abspath(file_path)
    
    # If not strict mode, just return normalized path
    if not strict:
        return abs_path
    
    # Default to current working directory if no base specified
    if allowed_base is None:
        allowed_base = os.getcwd()
    
    # Resolve allowed base to absolute path
    allowed_abs = os.path.abspath(allowed_base)
    
    # Check if path is within allowed directory
    if not abs_path.startswith(allowed_abs):
        raise ValueError(
            f"Access denied: File path '{file_path}' is outside allowed directory. "
            f"Only files within '{allowed_abs}' are accessible."
        )
    
    return abs_path


class ObjectManager:
    """Manages objects in PyBullet physics simulations.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self, simulation_manager: SimulationManager, strict_path_validation: bool = True):
        """Initialize the object manager.
        
        Args:
            simulation_manager: SimulationManager instance for accessing simulations.
            strict_path_validation: If True, enforce path restrictions. If False, allow any path.
        """
        self.simulation_manager = simulation_manager
        self.strict_path_validation = strict_path_validation
    
    def create_primitive(
        self,
        sim_id: str,
        shape: str,
        dimensions: List[float],
        position: List[float],
        mass: float = 1.0,
        color: Optional[List[float]] = None,
        friction: float = 0.5,
        restitution: float = 0.5
    ) -> int:
        """Create a primitive shape in the simulation.
        
        Args:
            sim_id: UUID string identifying the simulation.
            shape: Shape type - "box", "sphere", "cylinder", or "capsule".
            dimensions: Shape dimensions. For box: [half_x, half_y, half_z],
                       for sphere: [radius], for cylinder/capsule: [radius, height].
            position: Initial position [x, y, z].
            mass: Object mass in kg. Default is 1.0.
            color: RGBA color [r, g, b, a]. Default is white [1, 1, 1, 1].
            friction: Friction coefficient. Default is 0.5.
            restitution: Restitution (bounciness) coefficient. Default is 0.5.
            
        Returns:
            PyBullet object ID (integer).
            
        Raises:
            ValueError: If simulation not found or invalid shape type.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate shape type
        if shape not in SHAPE_MAP:
            raise ValueError(
                f"Invalid shape: {shape}. Must be one of: {list(SHAPE_MAP.keys())}"
            )
        
        # Validate mass is positive
        if mass <= 0:
            raise ValueError(f"Mass must be positive, got {mass}")
        
        # Validate dimensions are positive
        if not dimensions:
            raise ValueError("Dimensions cannot be empty")
        
        for i, dim in enumerate(dimensions):
            if dim <= 0:
                raise ValueError(f"All dimensions must be positive, got {dim} at index {i}")
        
        # Default color to white if not specified
        if color is None:
            color = [1.0, 1.0, 1.0, 1.0]
        
        # Create collision shape based on type
        shape_type = SHAPE_MAP[shape]
        
        if shape == "box":
            # Box requires halfExtents [half_x, half_y, half_z]
            collision_shape = p.createCollisionShape(
                shapeType=shape_type,
                halfExtents=dimensions,
                physicsClientId=sim.client_id
            )
            visual_shape = p.createVisualShape(
                shapeType=shape_type,
                halfExtents=dimensions,
                rgbaColor=color,
                physicsClientId=sim.client_id
            )
        elif shape == "sphere":
            # Sphere requires radius
            radius = dimensions[0] if dimensions else 0.5
            collision_shape = p.createCollisionShape(
                shapeType=shape_type,
                radius=radius,
                physicsClientId=sim.client_id
            )
            visual_shape = p.createVisualShape(
                shapeType=shape_type,
                radius=radius,
                rgbaColor=color,
                physicsClientId=sim.client_id
            )
        elif shape in ["cylinder", "capsule"]:
            # Cylinder and capsule require radius and height
            radius = dimensions[0] if len(dimensions) > 0 else 0.5
            height = dimensions[1] if len(dimensions) > 1 else 1.0
            collision_shape = p.createCollisionShape(
                shapeType=shape_type,
                radius=radius,
                height=height,
                physicsClientId=sim.client_id
            )
            visual_shape = p.createVisualShape(
                shapeType=shape_type,
                radius=radius,
                length=height,
                rgbaColor=color,
                physicsClientId=sim.client_id
            )
        
        # Create multi-body with collision and visual shapes
        object_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
            physicsClientId=sim.client_id
        )
        
        # Set friction and restitution
        p.changeDynamics(
            object_id,
            -1,  # -1 for base link
            lateralFriction=friction,
            restitution=restitution,
            physicsClientId=sim.client_id
        )
        
        # Track object metadata in simulation context
        metadata = {
            "type": "primitive",
            "shape": shape,
            "dimensions": dimensions,
            "mass": mass,
            "color": color,
            "friction": friction,
            "restitution": restitution
        }
        sim.add_object(object_id, metadata)
        
        return object_id
    
    def load_urdf(
        self,
        sim_id: str,
        file_path: str,
        position: List[float],
        orientation: Optional[List[float]] = None
    ) -> int:
        """Load a URDF model into the simulation.
        
        Args:
            sim_id: UUID string identifying the simulation.
            file_path: Path to the URDF file.
            position: Initial position [x, y, z].
            orientation: Initial orientation as quaternion [x, y, z, w].
                        Default is [0, 0, 0, 1] (no rotation).
            
        Returns:
            PyBullet object ID (integer).
            
        Raises:
            ValueError: If simulation not found or path validation fails.
            FileNotFoundError: If URDF file doesn't exist.
            RuntimeError: If URDF loading fails.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate file path to prevent path traversal
        try:
            validated_path = validate_file_path(file_path, strict=self.strict_path_validation)
        except ValueError as e:
            raise ValueError(f"Invalid file path: {str(e)}")
        
        # Validate file exists
        if not os.path.exists(validated_path):
            raise ValueError(f"URDF file not found: {file_path}")
        
        # Default orientation to identity quaternion
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Load URDF file
        try:
            object_id = p.loadURDF(
                validated_path,
                basePosition=position,
                baseOrientation=orientation,
                physicsClientId=sim.client_id
            )
        except p.error as e:
            raise RuntimeError(f"Failed to load URDF from {file_path}: {str(e)}")
        
        # Track object metadata in simulation context
        metadata = {
            "type": "urdf",
            "file_path": validated_path,
            "shape": "urdf"
        }
        sim.add_object(object_id, metadata)
        
        return object_id
    
    def set_object_pose(
        self,
        sim_id: str,
        object_id: int,
        position: List[float],
        orientation: List[float]
    ) -> None:
        """Update an object's position and orientation.
        
        Args:
            sim_id: UUID string identifying the simulation.
            object_id: PyBullet object ID.
            position: New position [x, y, z].
            orientation: New orientation as quaternion [x, y, z, w].
            
        Raises:
            ValueError: If simulation not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate object exists in simulation
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Update object pose
        p.resetBasePositionAndOrientation(
            object_id,
            position,
            orientation,
            physicsClientId=sim.client_id
        )
    
    def apply_force(
        self,
        sim_id: str,
        object_id: int,
        force: List[float],
        position: Optional[List[float]] = None
    ) -> None:
        """Apply a force vector to an object.
        
        Args:
            sim_id: UUID string identifying the simulation.
            object_id: PyBullet object ID.
            force: Force vector [fx, fy, fz] in Newtons.
            position: Position to apply force [x, y, z] in world coordinates.
                     If None, force is applied at the object's center of mass.
            
        Raises:
            ValueError: If simulation not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate object exists in simulation
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        if position is not None:
            # Apply force at specific world position
            p.applyExternalForce(
                object_id,
                -1,  # -1 for base link
                force,
                position,
                p.WORLD_FRAME,
                physicsClientId=sim.client_id
            )
        else:
            # Apply force at center of mass in link frame
            p.applyExternalForce(
                object_id,
                -1,  # -1 for base link
                force,
                [0.0, 0.0, 0.0],
                p.LINK_FRAME,
                physicsClientId=sim.client_id
            )
    
    def apply_torque(
        self,
        sim_id: str,
        object_id: int,
        torque: List[float]
    ) -> None:
        """Apply a torque vector to an object.
        
        Args:
            sim_id: UUID string identifying the simulation.
            object_id: PyBullet object ID.
            torque: Torque vector [tx, ty, tz] in Newton-meters.
            
        Raises:
            ValueError: If simulation not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate object exists in simulation
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Apply torque in link frame
        p.applyExternalTorque(
            object_id,
            -1,  # -1 for base link
            torque,
            p.LINK_FRAME,
            physicsClientId=sim.client_id
        )
    
    def get_object_state(
        self,
        sim_id: str,
        object_id: int
    ) -> Dict[str, Any]:
        """Query complete object state including position, orientation, and velocities.
        
        Args:
            sim_id: UUID string identifying the simulation.
            object_id: PyBullet object ID.
            
        Returns:
            Dictionary containing:
                - position: [x, y, z]
                - orientation: [x, y, z, w] quaternion
                - linear_velocity: [vx, vy, vz]
                - angular_velocity: [wx, wy, wz]
            
        Raises:
            ValueError: If simulation not found.
        """
        # Get simulation context
        sim = self.simulation_manager.get_simulation(sim_id)
        
        # Validate object exists in simulation
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Get position and orientation
        pos, orn = p.getBasePositionAndOrientation(
            object_id,
            physicsClientId=sim.client_id
        )
        
        # Get velocities
        lin_vel, ang_vel = p.getBaseVelocity(
            object_id,
            physicsClientId=sim.client_id
        )
        
        return {
            "position": list(pos),
            "orientation": list(orn),
            "linear_velocity": list(lin_vel),
            "angular_velocity": list(ang_vel)
        }
