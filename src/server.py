"""PyBullet MCP Server - FastMCP server exposing PyBullet physics simulation tools."""

from typing import Optional
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError

from .simulation_manager import SimulationManager
from .object_manager import ObjectManager
from .constraint_manager import ConstraintManager
from .collision_detection import CollisionQueryHandler
from .persistence import PersistenceHandler


# Create FastMCP server instance
mcp = FastMCP("PyBullet Server")

# Initialize manager instances
simulation_manager = SimulationManager()
object_manager = ObjectManager(simulation_manager)
constraint_manager = ConstraintManager(simulation_manager)
collision_handler = CollisionQueryHandler(simulation_manager)
persistence_handler = PersistenceHandler(simulation_manager, object_manager, constraint_manager)


# ============================================================================
# Simulation Management Tools
# ============================================================================

@mcp.tool
def create_simulation(
    gravity: list[float] = [0.0, 0.0, -9.81],
    gui: bool = False
) -> dict:
    """Create a new physics simulation.
    
    Args:
        gravity: Gravity vector [x, y, z] in m/s^2. Default is Earth gravity [0, 0, -9.81].
        gui: Whether to enable GUI visualization window. Default is False (headless mode).
    
    Returns:
        Dictionary containing:
            - simulation_id: UUID string identifying the new simulation
            - gravity: Applied gravity vector
            - gui_enabled: Whether GUI is active
    
    Example:
        create_simulation(gravity=[0, 0, -9.81], gui=True)
    """
    try:
        sim_id = simulation_manager.create_simulation(tuple(gravity), gui)
        return {
            "simulation_id": sim_id,
            "gravity": gravity,
            "gui_enabled": gui
        }
    except Exception as e:
        raise ToolError(f"Failed to create simulation: {str(e)}")


@mcp.tool
def list_simulations() -> list:
    """List all active simulation IDs.
    
    Returns:
        List of UUID strings for all active simulations.
    
    Example:
        list_simulations()
    """
    return simulation_manager.list_simulations()


@mcp.tool
def destroy_simulation(sim_id: str) -> str:
    """Terminate a simulation and clean up resources.
    
    Args:
        sim_id: UUID string identifying the simulation to destroy.
    
    Returns:
        Confirmation message.
    
    Example:
        destroy_simulation(sim_id="abc-123-def")
    """
    try:
        simulation_manager.destroy_simulation(sim_id)
        return f"Simulation {sim_id} destroyed successfully"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to destroy simulation: {str(e)}")


@mcp.tool
def step_simulation(sim_id: str, steps: int = 1) -> dict:
    """Advance the simulation by one or more timesteps.
    
    Args:
        sim_id: UUID string identifying the simulation.
        steps: Number of timesteps to execute. Default is 1.
    
    Returns:
        Dictionary containing:
            - steps_executed: Number of timesteps executed
            - simulation_time: Total elapsed simulation time in seconds
    
    Example:
        step_simulation(sim_id="abc-123", steps=10)
    """
    try:
        if steps == 1:
            simulation_manager.step_simulation(sim_id)
        else:
            simulation_manager.step_multiple(sim_id, steps)
        
        sim = simulation_manager.get_simulation(sim_id)
        return {
            "steps_executed": steps,
            "simulation_time": sim.simulation_time
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to step simulation: {str(e)}")


@mcp.tool
def set_timestep(sim_id: str, timestep: float) -> str:
    """Configure the timestep duration for a simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        timestep: New timestep duration in seconds. Must be positive.
    
    Returns:
        Confirmation message.
    
    Example:
        set_timestep(sim_id="abc-123", timestep=0.01)
    """
    try:
        simulation_manager.set_timestep(sim_id, timestep)
        return f"Timestep set to {timestep} seconds for simulation {sim_id}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to set timestep: {str(e)}")


# ============================================================================
# Object Management Tools - Primitive Shapes
# ============================================================================

@mcp.tool
def add_box(
    sim_id: str,
    dimensions: list[float],
    position: list[float],
    mass: float = 1.0,
    color: Optional[list[float]] = None
) -> dict:
    """Add a box (rectangular prism) to the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        dimensions: Box half-extents [half_x, half_y, half_z] in meters.
        position: Initial position [x, y, z] in meters.
        mass: Object mass in kg. Default is 1.0.
        color: RGBA color [r, g, b, a] where values are 0-1. Default is white.
    
    Returns:
        Dictionary containing:
            - object_id: PyBullet object ID (integer)
            - shape: "box"
            - position: Initial position
    
    Example:
        add_box(sim_id="abc-123", dimensions=[0.5, 0.5, 0.5], position=[0, 0, 1], mass=1.0)
    """
    try:
        object_id = object_manager.create_primitive(
            sim_id, "box", dimensions, position, mass, color
        )
        return {
            "object_id": object_id,
            "shape": "box",
            "position": position
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to create box: {str(e)}")


@mcp.tool
def add_sphere(
    sim_id: str,
    radius: float,
    position: list[float],
    mass: float = 1.0,
    color: Optional[list[float]] = None
) -> dict:
    """Add a sphere to the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        radius: Sphere radius in meters.
        position: Initial position [x, y, z] in meters.
        mass: Object mass in kg. Default is 1.0.
        color: RGBA color [r, g, b, a] where values are 0-1. Default is white.
    
    Returns:
        Dictionary containing:
            - object_id: PyBullet object ID (integer)
            - shape: "sphere"
            - position: Initial position
    
    Example:
        add_sphere(sim_id="abc-123", radius=0.5, position=[0, 0, 1], mass=1.0)
    """
    try:
        object_id = object_manager.create_primitive(
            sim_id, "sphere", [radius], position, mass, color
        )
        return {
            "object_id": object_id,
            "shape": "sphere",
            "position": position
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to create sphere: {str(e)}")


@mcp.tool
def add_cylinder(
    sim_id: str,
    radius: float,
    height: float,
    position: list[float],
    mass: float = 1.0,
    color: Optional[list[float]] = None
) -> dict:
    """Add a cylinder to the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        radius: Cylinder radius in meters.
        height: Cylinder height in meters.
        position: Initial position [x, y, z] in meters.
        mass: Object mass in kg. Default is 1.0.
        color: RGBA color [r, g, b, a] where values are 0-1. Default is white.
    
    Returns:
        Dictionary containing:
            - object_id: PyBullet object ID (integer)
            - shape: "cylinder"
            - position: Initial position
    
    Example:
        add_cylinder(sim_id="abc-123", radius=0.3, height=1.0, position=[0, 0, 1], mass=1.0)
    """
    try:
        object_id = object_manager.create_primitive(
            sim_id, "cylinder", [radius, height], position, mass, color
        )
        return {
            "object_id": object_id,
            "shape": "cylinder",
            "position": position
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to create cylinder: {str(e)}")


@mcp.tool
def add_capsule(
    sim_id: str,
    radius: float,
    height: float,
    position: list[float],
    mass: float = 1.0,
    color: Optional[list[float]] = None
) -> dict:
    """Add a capsule (cylinder with hemispherical ends) to the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        radius: Capsule radius in meters.
        height: Capsule height (cylindrical section) in meters.
        position: Initial position [x, y, z] in meters.
        mass: Object mass in kg. Default is 1.0.
        color: RGBA color [r, g, b, a] where values are 0-1. Default is white.
    
    Returns:
        Dictionary containing:
            - object_id: PyBullet object ID (integer)
            - shape: "capsule"
            - position: Initial position
    
    Example:
        add_capsule(sim_id="abc-123", radius=0.3, height=1.0, position=[0, 0, 1], mass=1.0)
    """
    try:
        object_id = object_manager.create_primitive(
            sim_id, "capsule", [radius, height], position, mass, color
        )
        return {
            "object_id": object_id,
            "shape": "capsule",
            "position": position
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to create capsule: {str(e)}")


# ============================================================================
# Object Management Tools - URDF Loading
# ============================================================================

@mcp.tool
def load_urdf(
    sim_id: str,
    file_path: str,
    position: list[float],
    orientation: Optional[list[float]] = None
) -> dict:
    """Load a URDF model into the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        file_path: Path to the URDF file.
        position: Initial position [x, y, z] in meters.
        orientation: Initial orientation as quaternion [x, y, z, w]. Default is [0, 0, 0, 1].
    
    Returns:
        Dictionary containing:
            - object_id: PyBullet object ID (integer)
            - file_path: Path to loaded URDF file
            - position: Initial position
    
    Example:
        load_urdf(sim_id="abc-123", file_path="robot.urdf", position=[0, 0, 0])
    """
    try:
        object_id = object_manager.load_urdf(sim_id, file_path, position, orientation)
        return {
            "object_id": object_id,
            "file_path": file_path,
            "position": position
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to load URDF: {str(e)}")


# ============================================================================
# Object State Manipulation Tools
# ============================================================================

@mcp.tool
def set_object_pose(
    sim_id: str,
    object_id: int,
    position: list[float],
    orientation: list[float]
) -> str:
    """Update an object's position and orientation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        position: New position [x, y, z] in meters.
        orientation: New orientation as quaternion [x, y, z, w].
    
    Returns:
        Confirmation message.
    
    Example:
        set_object_pose(sim_id="abc-123", object_id=1, position=[1, 0, 0], orientation=[0, 0, 0, 1])
    """
    try:
        object_manager.set_object_pose(sim_id, object_id, position, orientation)
        return f"Object {object_id} pose updated successfully"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to set object pose: {str(e)}")


@mcp.tool
def get_object_state(sim_id: str, object_id: int) -> dict:
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
    
    Example:
        get_object_state(sim_id="abc-123", object_id=1)
    """
    try:
        return object_manager.get_object_state(sim_id, object_id)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get object state: {str(e)}")


@mcp.tool
def apply_force(
    sim_id: str,
    object_id: int,
    force: list[float],
    position: Optional[list[float]] = None
) -> str:
    """Apply a force vector to an object.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        force: Force vector [fx, fy, fz] in Newtons.
        position: Position to apply force [x, y, z] in world coordinates.
                 If None, force is applied at the object's center of mass.
    
    Returns:
        Confirmation message.
    
    Example:
        apply_force(sim_id="abc-123", object_id=1, force=[10, 0, 0])
    """
    try:
        object_manager.apply_force(sim_id, object_id, force, position)
        return f"Force applied to object {object_id}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to apply force: {str(e)}")


@mcp.tool
def apply_torque(
    sim_id: str,
    object_id: int,
    torque: list[float]
) -> str:
    """Apply a torque vector to an object.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        torque: Torque vector [tx, ty, tz] in Newton-meters.
    
    Returns:
        Confirmation message.
    
    Example:
        apply_torque(sim_id="abc-123", object_id=1, torque=[0, 0, 5])
    """
    try:
        object_manager.apply_torque(sim_id, object_id, torque)
        return f"Torque applied to object {object_id}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to apply torque: {str(e)}")


# ============================================================================
# Constraint Management Tools
# ============================================================================

@mcp.tool
def create_constraint(
    sim_id: str,
    parent_id: int,
    child_id: int,
    joint_type: str,
    joint_axis: Optional[list[float]] = None,
    parent_frame_position: Optional[list[float]] = None,
    child_frame_position: Optional[list[float]] = None,
    parent_frame_orientation: Optional[list[float]] = None,
    child_frame_orientation: Optional[list[float]] = None
) -> dict:
    """Create a constraint (joint) between two objects.
    
    Args:
        sim_id: UUID string identifying the simulation.
        parent_id: PyBullet object ID of the parent body.
        child_id: PyBullet object ID of the child body.
        joint_type: Type of joint - "fixed", "prismatic", or "spherical".
                   Note: "revolute" joints are not supported by PyBullet's createConstraint API.
        joint_axis: Axis of rotation/translation [x, y, z]. Default is [0, 0, 1].
        parent_frame_position: Position in parent frame [x, y, z]. Default is [0, 0, 0].
        child_frame_position: Position in child frame [x, y, z]. Default is [0, 0, 0].
        parent_frame_orientation: Orientation in parent frame as quaternion [x, y, z, w].
        child_frame_orientation: Orientation in child frame as quaternion [x, y, z, w].
    
    Returns:
        Dictionary containing:
            - constraint_id: PyBullet constraint ID (integer)
            - joint_type: Type of joint created
    
    Example:
        create_constraint(sim_id="abc-123", parent_id=1, child_id=2, joint_type="fixed")
    """
    try:
        constraint_id = constraint_manager.create_constraint(
            sim_id, parent_id, child_id, joint_type,
            joint_axis, parent_frame_position, child_frame_position,
            parent_frame_orientation, child_frame_orientation
        )
        return {
            "constraint_id": constraint_id,
            "joint_type": joint_type
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to create constraint: {str(e)}")


@mcp.tool
def remove_constraint(sim_id: str, constraint_id: int) -> str:
    """Remove a constraint from the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
        constraint_id: PyBullet constraint ID.
    
    Returns:
        Confirmation message.
    
    Example:
        remove_constraint(sim_id="abc-123", constraint_id=1)
    """
    try:
        constraint_manager.remove_constraint(sim_id, constraint_id)
        return f"Constraint {constraint_id} removed successfully"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to remove constraint: {str(e)}")


# ============================================================================
# Collision Detection Tools
# ============================================================================

@mcp.tool
def get_all_collisions(sim_id: str) -> list:
    """Query all contact points in the simulation.
    
    Args:
        sim_id: UUID string identifying the simulation.
    
    Returns:
        List of contact point dictionaries, each containing:
            - object_a: ID of first object
            - object_b: ID of second object
            - position_on_a: Contact position on first object [x, y, z]
            - position_on_b: Contact position on second object [x, y, z]
            - contact_normal: Normal vector at contact point [x, y, z]
            - contact_distance: Distance between objects (negative = penetration)
            - normal_force: Force magnitude along contact normal
        Returns empty list if no collisions exist.
    
    Example:
        get_all_collisions(sim_id="abc-123")
    """
    try:
        return collision_handler.get_all_contacts(sim_id)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get collisions: {str(e)}")


@mcp.tool
def get_collisions_for_pair(sim_id: str, obj_a: int, obj_b: int) -> list:
    """Query contact points between a specific pair of objects.
    
    Args:
        sim_id: UUID string identifying the simulation.
        obj_a: First object ID.
        obj_b: Second object ID.
    
    Returns:
        List of contact point dictionaries for the specified pair.
        Returns empty list if objects are not in contact.
    
    Example:
        get_collisions_for_pair(sim_id="abc-123", obj_a=1, obj_b=2)
    """
    try:
        return collision_handler.get_contacts_for_pair(sim_id, obj_a, obj_b)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get collisions for pair: {str(e)}")


# ============================================================================
# Visualization and Debug Tools
# ============================================================================

@mcp.tool
def enable_debug_visualization(
    sim_id: str,
    show_contact_points: bool = True,
    show_frames: bool = False
) -> str:
    """Enable debug visualization options for a simulation.

    Args:
        sim_id: UUID string identifying the simulation.
        show_contact_points: Whether to visualize contact points. Default is True.
        show_frames: Whether to visualize coordinate frames for objects. Default is False.

    Returns:
        Confirmation message.

    Example:
        enable_debug_visualization(sim_id="abc-123", show_contact_points=True, show_frames=True)
    """
    try:
        simulation_manager.enable_debug_visualization(sim_id, show_contact_points, show_frames)
        return f"Debug visualization enabled for simulation {sim_id}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to enable debug visualization: {str(e)}")


@mcp.tool
def set_camera(
    sim_id: str,
    distance: float,
    yaw: float,
    pitch: float,
    target_position: list[float]
) -> str:
    """Set camera position for GUI visualization.

    Args:
        sim_id: UUID string identifying the simulation.
        distance: Distance from camera to target in meters.
        yaw: Camera yaw angle in degrees.
        pitch: Camera pitch angle in degrees.
        target_position: Position [x, y, z] that camera looks at.

    Returns:
        Confirmation message.

    Example:
        set_camera(sim_id="abc-123", distance=5.0, yaw=45, pitch=-30, target_position=[0, 0, 0])
    """
    try:
        simulation_manager.set_camera(sim_id, distance, yaw, pitch, target_position)
        return f"Camera position set for simulation {sim_id}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to set camera: {str(e)}")



# ============================================================================
# Persistence Tools
# ============================================================================

@mcp.tool
def save_simulation(sim_id: str, file_path: str) -> str:
    """Save simulation state to a file.
    
    Args:
        sim_id: UUID string identifying the simulation.
        file_path: Path where the state file should be written.
    
    Returns:
        Confirmation message with file path.
    
    Example:
        save_simulation(sim_id="abc-123", file_path="simulation_state.json")
    """
    try:
        persistence_handler.save_state(sim_id, file_path)
        return f"Simulation saved to {file_path}"
    except ValueError as e:
        raise ToolError(str(e))
    except IOError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to save simulation: {str(e)}")


@mcp.tool
def load_simulation(file_path: str, gui: bool = False) -> dict:
    """Load simulation state from a file.
    
    Args:
        file_path: Path to the state file to load.
        gui: Whether to enable GUI visualization for the loaded simulation. Default is False.
    
    Returns:
        Dictionary containing:
            - simulation_id: UUID string identifying the newly created simulation
            - file_path: Path to loaded file
    
    Example:
        load_simulation(file_path="simulation_state.json", gui=True)
    """
    try:
        sim_id = persistence_handler.load_state(file_path, gui)
        return {
            "simulation_id": sim_id,
            "file_path": file_path
        }
    except IOError as e:
        raise ToolError(str(e))
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to load simulation: {str(e)}")


# ============================================================================
# Joint Control Tools (Robot Manipulation)
# ============================================================================

@mcp.tool
def get_num_joints(sim_id: str, object_id: int) -> int:
    """Get the number of joints in a URDF model.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID (must be a URDF model with joints).
    
    Returns:
        Number of joints in the model (integer).
    
    Example:
        get_num_joints(sim_id="abc-123", object_id=0)
    """
    try:
        import pybullet as p
        sim = simulation_manager.get_simulation(sim_id)
        
        # Validate object exists
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        num_joints = p.getNumJoints(object_id, physicsClientId=sim.client_id)
        return num_joints
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get number of joints: {str(e)}")


@mcp.tool
def get_joint_info(sim_id: str, object_id: int, joint_index: int) -> dict:
    """Get detailed information about a specific joint.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        joint_index: Index of the joint (0 to num_joints-1).
    
    Returns:
        Dictionary containing:
            - joint_index: Index of the joint
            - joint_name: Name of the joint
            - joint_type: Type of joint (0=REVOLUTE, 1=PRISMATIC, 4=FIXED, etc.)
            - joint_lower_limit: Lower position limit
            - joint_upper_limit: Upper position limit
            - joint_max_force: Maximum force the joint can apply
            - joint_max_velocity: Maximum velocity of the joint
            - joint_axis: Axis of rotation/translation [x, y, z]
    
    Example:
        get_joint_info(sim_id="abc-123", object_id=0, joint_index=0)
    """
    try:
        import pybullet as p
        sim = simulation_manager.get_simulation(sim_id)
        
        # Validate object exists
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Get joint info
        info = p.getJointInfo(object_id, joint_index, physicsClientId=sim.client_id)
        
        return {
            "joint_index": info[0],
            "joint_name": info[1].decode('utf-8'),
            "joint_type": info[2],
            "joint_lower_limit": info[8],
            "joint_upper_limit": info[9],
            "joint_max_force": info[10],
            "joint_max_velocity": info[11],
            "joint_axis": list(info[13])
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get joint info: {str(e)}")


@mcp.tool
def get_joint_state(sim_id: str, object_id: int, joint_index: int) -> dict:
    """Get the current state of a joint (position, velocity, forces).
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        joint_index: Index of the joint.
    
    Returns:
        Dictionary containing:
            - joint_position: Current position of the joint
            - joint_velocity: Current velocity of the joint
            - joint_reaction_forces: Reaction forces at the joint [Fx, Fy, Fz, Mx, My, Mz]
            - applied_motor_torque: Torque applied by the motor
    
    Example:
        get_joint_state(sim_id="abc-123", object_id=0, joint_index=0)
    """
    try:
        import pybullet as p
        sim = simulation_manager.get_simulation(sim_id)
        
        # Validate object exists
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Get joint state
        state = p.getJointState(object_id, joint_index, physicsClientId=sim.client_id)
        
        return {
            "joint_position": state[0],
            "joint_velocity": state[1],
            "joint_reaction_forces": list(state[2]),
            "applied_motor_torque": state[3]
        }
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get joint state: {str(e)}")


@mcp.tool
def set_joint_motor_control(
    sim_id: str,
    object_id: int,
    joint_index: int,
    control_mode: str,
    target_position: Optional[float] = None,
    target_velocity: Optional[float] = None,
    force: Optional[float] = None,
    position_gain: float = 0.1,
    velocity_gain: float = 1.0
) -> str:
    """Control a robot joint using position, velocity, or torque control.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        joint_index: Index of the joint to control.
        control_mode: Control mode - "POSITION_CONTROL", "VELOCITY_CONTROL", or "TORQUE_CONTROL".
        target_position: Target position for position control (radians or meters).
        target_velocity: Target velocity for velocity control (rad/s or m/s).
        force: Maximum force/torque to apply (Newtons or Newton-meters).
        position_gain: Position gain (Kp) for position control. Default is 0.1.
        velocity_gain: Velocity gain (Kd) for position/velocity control. Default is 1.0.
    
    Returns:
        Confirmation message.
    
    Example:
        set_joint_motor_control(sim_id="abc-123", object_id=0, joint_index=0, 
                               control_mode="POSITION_CONTROL", target_position=1.57, force=100)
    """
    try:
        import pybullet as p
        sim = simulation_manager.get_simulation(sim_id)
        
        # Validate object exists
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Map control mode string to PyBullet constant
        mode_map = {
            "POSITION_CONTROL": p.POSITION_CONTROL,
            "VELOCITY_CONTROL": p.VELOCITY_CONTROL,
            "TORQUE_CONTROL": p.TORQUE_CONTROL
        }
        
        if control_mode not in mode_map:
            raise ValueError(
                f"Invalid control mode: {control_mode}. "
                f"Must be one of: {list(mode_map.keys())}"
            )
        
        # Build kwargs for setJointMotorControl2
        kwargs = {
            "bodyUniqueId": object_id,
            "jointIndex": joint_index,
            "controlMode": mode_map[control_mode],
            "physicsClientId": sim.client_id
        }
        
        if target_position is not None:
            kwargs["targetPosition"] = target_position
        if target_velocity is not None:
            kwargs["targetVelocity"] = target_velocity
        if force is not None:
            kwargs["force"] = force
        if control_mode == "POSITION_CONTROL":
            kwargs["positionGain"] = position_gain
            kwargs["velocityGain"] = velocity_gain
        elif control_mode == "VELOCITY_CONTROL":
            kwargs["velocityGain"] = velocity_gain
        
        # Apply motor control
        p.setJointMotorControl2(**kwargs)
        
        return f"Joint {joint_index} motor control set to {control_mode}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to set joint motor control: {str(e)}")


@mcp.tool
def calculate_inverse_kinematics(
    sim_id: str,
    object_id: int,
    end_effector_link_index: int,
    target_position: list[float],
    target_orientation: Optional[list[float]] = None,
    lower_limits: Optional[list[float]] = None,
    upper_limits: Optional[list[float]] = None,
    joint_ranges: Optional[list[float]] = None,
    rest_poses: Optional[list[float]] = None
) -> list[float]:
    """Calculate inverse kinematics to reach a target end-effector pose.
    
    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID (robot with joints).
        end_effector_link_index: Index of the end-effector link.
        target_position: Target position [x, y, z] for the end-effector.
        target_orientation: Target orientation as quaternion [x, y, z, w]. Optional.
        lower_limits: Lower joint limits. Optional.
        upper_limits: Upper joint limits. Optional.
        joint_ranges: Range of motion for each joint. Optional.
        rest_poses: Rest poses for null space. Optional.
    
    Returns:
        List of joint positions (angles/distances) to reach the target pose.
    
    Example:
        calculate_inverse_kinematics(sim_id="abc-123", object_id=0, 
                                     end_effector_link_index=6, 
                                     target_position=[0.5, 0.0, 0.5])
    """
    try:
        import pybullet as p
        sim = simulation_manager.get_simulation(sim_id)
        
        # Validate object exists
        if object_id not in sim.objects:
            raise ValueError(f"Object {object_id} not found in simulation {sim_id}")
        
        # Build kwargs for calculateInverseKinematics
        kwargs = {
            "bodyUniqueId": object_id,
            "endEffectorLinkIndex": end_effector_link_index,
            "targetPosition": target_position,
            "physicsClientId": sim.client_id
        }
        
        if target_orientation is not None:
            kwargs["targetOrientation"] = target_orientation
        if lower_limits is not None:
            kwargs["lowerLimits"] = lower_limits
        if upper_limits is not None:
            kwargs["upperLimits"] = upper_limits
        if joint_ranges is not None:
            kwargs["jointRanges"] = joint_ranges
        if rest_poses is not None:
            kwargs["restPoses"] = rest_poses
        
        # Calculate IK
        joint_positions = p.calculateInverseKinematics(**kwargs)
        
        return list(joint_positions)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to calculate inverse kinematics: {str(e)}")


# ============================================================================
# Server Entry Point
# ============================================================================

if __name__ == "__main__":
    mcp.run(transport="http", port=8000)
