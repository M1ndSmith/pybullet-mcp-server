"""PyBullet MCP Server - FastMCP server exposing PyBullet physics simulation tools."""

from typing import Optional
from fastmcp import FastMCP
from fastmcp.exceptions import ToolError

from .simulation_manager import SimulationManager
from .object_manager import ObjectManager
from .constraint_manager import ConstraintManager
from .collision_detection import CollisionQueryHandler
from .persistence import PersistenceHandler
from .urdf_generator import generate_revolute_joint_urdf
from .ray_casting import RayCastingHandler
from .camera_rendering import CameraRenderer


# Create FastMCP server instance
mcp = FastMCP("PyBullet Server")

# Initialize manager instances
simulation_manager = SimulationManager()
object_manager = ObjectManager(simulation_manager)
constraint_manager = ConstraintManager(simulation_manager)
collision_handler = CollisionQueryHandler(simulation_manager)
persistence_handler = PersistenceHandler(simulation_manager, object_manager, constraint_manager)
ray_casting_handler = RayCastingHandler(simulation_manager)
camera_renderer = CameraRenderer(simulation_manager)


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
@mcp.tool
def set_object_velocity(
    sim_id: str,
    object_id: int,
    linear_velocity: Optional[list[float]] = None,
    angular_velocity: Optional[list[float]] = None
) -> str:
    """Set an object's linear and/or angular velocity directly.

    This instantly changes an object's velocity without applying forces.
    Useful for launching projectiles, resetting velocities, or teleporting
    objects with momentum.

    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        linear_velocity: Linear velocity [vx, vy, vz] in m/s. If None, keeps current velocity.
        angular_velocity: Angular velocity [wx, wy, wz] in rad/s. If None, keeps current velocity.

    Returns:
        Confirmation message.

    Example:
        # Launch a projectile horizontally
        set_object_velocity(sim_id="abc-123", object_id=1, linear_velocity=[10, 0, 0])

        # Stop an object completely
        set_object_velocity(sim_id="abc-123", object_id=1,
                          linear_velocity=[0, 0, 0], angular_velocity=[0, 0, 0])

        # Make object spin without moving
        set_object_velocity(sim_id="abc-123", object_id=1, angular_velocity=[0, 0, 5])

    Note:
        At least one of linear_velocity or angular_velocity must be provided.
    """
    try:
        object_manager.set_object_velocity(sim_id, object_id, linear_velocity, angular_velocity)

        # Build response message
        parts = []
        if linear_velocity is not None:
            parts.append(f"linear velocity set to {linear_velocity}")
        if angular_velocity is not None:
            parts.append(f"angular velocity set to {angular_velocity}")

        return f"Object {object_id} {' and '.join(parts)}"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to set object velocity: {str(e)}")
@mcp.tool
def change_dynamics(
    sim_id: str,
    object_id: int,
    link_index: int = -1,
    mass: Optional[float] = None,
    lateral_friction: Optional[float] = None,
    spinning_friction: Optional[float] = None,
    rolling_friction: Optional[float] = None,
    restitution: Optional[float] = None,
    linear_damping: Optional[float] = None,
    angular_damping: Optional[float] = None,
    contact_stiffness: Optional[float] = None,
    contact_damping: Optional[float] = None
) -> str:
    """Modify object physics properties at runtime.

    Allows changing physical properties after object creation. Useful for testing
    failure scenarios, simulating wear/damage, or creating dynamic environments.

    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        link_index: Link index (-1 for base). Default is -1.
        mass: New mass in kg. None to keep current.
        lateral_friction: Friction coefficient. None to keep current.
        spinning_friction: Spinning friction coefficient. None to keep current.
        rolling_friction: Rolling friction coefficient. None to keep current.
        restitution: Restitution (bounciness) coefficient. None to keep current.
        linear_damping: Linear damping coefficient. None to keep current.
        angular_damping: Angular damping coefficient. None to keep current.
        contact_stiffness: Contact stiffness. None to keep current.
        contact_damping: Contact damping. None to keep current.

    Returns:
        Confirmation message.

    Example:
        # Make object slippery
        change_dynamics(sim_id="abc-123", object_id=1, lateral_friction=0.1)

        # Change mass and restitution
        change_dynamics(sim_id="abc-123", object_id=1, mass=5.0, restitution=0.9)

    Note:
        At least one property must be specified.
    """
    try:
        object_manager.change_dynamics(
            sim_id, object_id, link_index, mass, lateral_friction,
            spinning_friction, rolling_friction, restitution,
            linear_damping, angular_damping, contact_stiffness, contact_damping
        )

        # Build response message
        changes = []
        if mass is not None:
            changes.append(f"mass={mass}")
        if lateral_friction is not None:
            changes.append(f"friction={lateral_friction}")
        if restitution is not None:
            changes.append(f"restitution={restitution}")

        if changes:
            return f"Object {object_id} dynamics updated: {', '.join(changes[:3])}"
        else:
            return f"Object {object_id} dynamics updated"
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to change dynamics: {str(e)}")


@mcp.tool
def get_dynamics_info(sim_id: str, object_id: int, link_index: int = -1) -> dict:
    """Query current dynamic properties of an object.

    Returns all physical properties including mass, friction, restitution, damping, etc.

    Args:
        sim_id: UUID string identifying the simulation.
        object_id: PyBullet object ID.
        link_index: Link index (-1 for base). Default is -1.

    Returns:
        Dictionary containing:
            - mass: Object mass in kg
            - lateral_friction: Lateral friction coefficient
            - local_inertia_diagonal: Inertia tensor diagonal [Ixx, Iyy, Izz]
            - restitution: Restitution coefficient
            - rolling_friction: Rolling friction coefficient
            - spinning_friction: Spinning friction coefficient
            - contact_damping: Contact damping
            - contact_stiffness: Contact stiffness
            - body_type: Body type (1=dynamic, 2=multibody, 3=soft body)
            - collision_margin: Collision margin

    Example:
        get_dynamics_info(sim_id="abc-123", object_id=1)
    """
    try:
        return object_manager.get_dynamics_info(sim_id, object_id, link_index)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to get dynamics info: {str(e)}")
@mcp.tool
def ray_test(
    sim_id: str,
    ray_from: list[float],
    ray_to: list[float]
) -> dict:
    """Cast a single ray to detect obstacles and measure distances.

    Useful for proximity sensors, line-of-sight checks, and simple distance measurements.

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

    Example:
        # Check if path is clear
        result = ray_test(sim_id="abc-123", ray_from=[0, 0, 1], ray_to=[10, 0, 1])
        if result["hit"]:
            print(f"Obstacle at distance: {result['hit_fraction'] * 10}")
    """
    try:
        return ray_casting_handler.ray_test(sim_id, ray_from, ray_to)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to cast ray: {str(e)}")


@mcp.tool
def ray_test_batch(
    sim_id: str,
    rays_from: list[list[float]],
    rays_to: list[list[float]]
) -> list[dict]:
    """Cast multiple rays efficiently for lidar/sensor simulation.

    Much more efficient than calling ray_test multiple times. Perfect for
    simulating lidar, sonar arrays, or multi-beam sensors.

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

    Example:
        # Simulate 360-degree lidar with 36 rays
        import math
        rays_from = [[0, 0, 1]] * 36
        rays_to = []
        for i in range(36):
            angle = i * (2 * math.pi / 36)
            rays_to.append([10 * math.cos(angle), 10 * math.sin(angle), 1])

        results = ray_test_batch(sim_id="abc-123", rays_from=rays_from, rays_to=rays_to)
        distances = [r["hit_fraction"] * 10 if r["hit"] else 10 for r in results]
    """
    try:
        return ray_casting_handler.ray_test_batch(sim_id, rays_from, rays_to)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to cast rays: {str(e)}")
@mcp.tool
def compute_view_matrix(
    camera_eye_position: list[float],
    camera_target_position: list[float],
    camera_up_vector: list[float]
) -> list[float]:
    """Compute view matrix from camera pose.

    Helper function to create view matrix for get_camera_image().

    Args:
        camera_eye_position: Camera position [x, y, z].
        camera_target_position: Point camera looks at [x, y, z].
        camera_up_vector: Camera up direction [x, y, z]. Usually [0, 0, 1] for Z-up.

    Returns:
        View matrix as list of 16 floats (4x4 matrix).

    Example:
        # Camera at [5, 5, 3] looking at origin
        view_matrix = compute_view_matrix(
            camera_eye_position=[5, 5, 3],
            camera_target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1]
        )
    """
    try:
        return camera_renderer.compute_view_matrix(
            camera_eye_position, camera_target_position, camera_up_vector
        )
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to compute view matrix: {str(e)}")


@mcp.tool
def compute_projection_matrix(
    fov: float,
    aspect: float,
    near_plane: float,
    far_plane: float
) -> list[float]:
    """Compute projection matrix from camera parameters.

    Helper function to create projection matrix for get_camera_image().

    Args:
        fov: Field of view in degrees (e.g., 60).
        aspect: Aspect ratio (width / height, e.g., 1.33 for 640x480).
        near_plane: Near clipping plane distance (e.g., 0.1).
        far_plane: Far clipping plane distance (e.g., 100).

    Returns:
        Projection matrix as list of 16 floats (4x4 matrix).

    Example:
        # Standard camera for 640x480 image
        proj_matrix = compute_projection_matrix(
            fov=60,
            aspect=640/480,
            near_plane=0.1,
            far_plane=100
        )
    """
    try:
        return camera_renderer.compute_projection_matrix(fov, aspect, near_plane, far_plane)
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to compute projection matrix: {str(e)}")


@mcp.tool
def get_camera_image(
    sim_id: str,
    width: int,
    height: int,
    view_matrix: list[float],
    projection_matrix: list[float],
    renderer: str = "ER_BULLET_HARDWARE_OPENGL"
) -> dict:
    """Render RGB, depth, and segmentation images from camera.

    Captures images from a virtual camera for vision-based control, computer vision
    testing, or ML training data generation.

    Args:
        sim_id: UUID string identifying the simulation.
        width: Image width in pixels (e.g., 640).
        height: Image height in pixels (e.g., 480).
        view_matrix: View matrix from compute_view_matrix().
        projection_matrix: Projection matrix from compute_projection_matrix().
        renderer: Renderer type. Options:
            - "ER_BULLET_HARDWARE_OPENGL" (default, fastest)
            - "ER_TINY_RENDERER" (software, slower but more compatible)

    Returns:
        Dictionary containing:
            - width: Image width
            - height: Image height
            - rgb: RGB image as base64-encoded PNG string
            - depth: Depth buffer as list of floats (0=near, 1=far)
            - segmentation: Segmentation mask as list of ints (object IDs)

    Example:
        # Setup camera
        view_matrix = compute_view_matrix(
            camera_eye_position=[5, 5, 3],
            camera_target_position=[0, 0, 0],
            camera_up_vector=[0, 0, 1]
        )
        proj_matrix = compute_projection_matrix(
            fov=60, aspect=640/480, near_plane=0.1, far_plane=100
        )

        # Render image
        image = get_camera_image(
            sim_id="abc-123",
            width=640,
            height=480,
            view_matrix=view_matrix,
            projection_matrix=proj_matrix
        )

        # Decode RGB image
        import base64
        from PIL import Image
        import io
        rgb_data = base64.b64decode(image["rgb"])
        img = Image.open(io.BytesIO(rgb_data))
    """
    try:
        return camera_renderer.get_camera_image(
            sim_id, width, height, view_matrix, projection_matrix, renderer
        )
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to render camera image: {str(e)}")


@mcp.tool
def compute_view_matrix_from_yaw_pitch(
    distance: float,
    yaw: float,
    pitch: float,
    target_position: list[float],
    up_axis_index: int = 2
) -> list[float]:
    """Compute view matrix from spherical coordinates (orbit camera).

    Easier alternative to compute_view_matrix() for orbit-style cameras.

    Args:
        distance: Distance from camera to target.
        yaw: Yaw angle in degrees (rotation around up axis).
        pitch: Pitch angle in degrees (up/down tilt).
        target_position: Point camera looks at [x, y, z].
        up_axis_index: Up axis (0=X, 1=Y, 2=Z). Default is 2 (Z-up).

    Returns:
        View matrix as list of 16 floats (4x4 matrix).

    Example:
        # Orbit camera 5 meters away, 45 degrees yaw, -30 degrees pitch
        view_matrix = compute_view_matrix_from_yaw_pitch(
            distance=5,
            yaw=45,
            pitch=-30,
            target_position=[0, 0, 0]
        )
    """
    try:
        return camera_renderer.compute_view_matrix_from_yaw_pitch(
            distance, yaw, pitch, target_position, up_axis_index
        )
    except ValueError as e:
        raise ToolError(str(e))
    except Exception as e:
        raise ToolError(f"Failed to compute view matrix: {str(e)}")


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
# URDF Generation Tools
# ============================================================================

@mcp.tool
def generate_revolute_joint(
    parent_shape: str,
    child_shape: str,
    parent_dimensions: list[float],
    child_dimensions: list[float],
    parent_mass: float,
    child_mass: float,
    joint_axis: list[float],
    joint_origin: list[float] = None,
    joint_lower_limit: float = -3.14159,
    joint_upper_limit: float = 3.14159,
    max_effort: float = 100.0,
    max_velocity: float = 10.0,
    output_path: str = None
) -> dict:
    """Generate a URDF file with a revolute (hinge) joint between two shapes.
    
    PyBullet's createConstraint API does not support revolute joints at runtime.
    This tool generates a URDF file that can be loaded with load_urdf() to create
    objects connected by a revolute joint.
    
    Args:
        parent_shape: Shape type for parent - "box", "sphere", or "cylinder"
        child_shape: Shape type for child - "box", "sphere", or "cylinder"
        parent_dimensions: Dimensions for parent shape:
            - box: [half_x, half_y, half_z]
            - sphere: [radius]
            - cylinder: [radius, height]
        child_dimensions: Dimensions for child shape (same format as parent)
        parent_mass: Mass of parent link in kg
        child_mass: Mass of child link in kg
        joint_axis: Axis of rotation [x, y, z] (e.g., [0, 0, 1] for z-axis)
        joint_origin: Joint position relative to parent [x, y, z]. Default [0, 0, 0]
        joint_lower_limit: Lower joint limit in radians. Default -π
        joint_upper_limit: Upper joint limit in radians. Default π
        max_effort: Maximum joint effort in N·m. Default 100.0
        max_velocity: Maximum joint velocity in rad/s. Default 10.0
        output_path: Path to save URDF file. If None, creates temp file.
    
    Returns:
        Dictionary with:
            - urdf_path: Path to generated URDF file
            - parent_shape: Parent shape type
            - child_shape: Child shape type
            - joint_type: "revolute"
    
    Example:
        # Create a door with hinge
        result = generate_revolute_joint(
            parent_shape="box",
            child_shape="box",
            parent_dimensions=[0.05, 1.0, 1.5],  # Wall (thin, wide, tall)
            child_dimensions=[0.025, 0.8, 1.4],  # Door (thin, narrower, slightly shorter)
            parent_mass=100.0,  # Heavy wall
            child_mass=10.0,    # Lighter door
            joint_axis=[0, 0, 1],  # Rotate around z-axis
            joint_origin=[0, 0.9, 0],  # Hinge at edge of wall
            joint_lower_limit=-1.57,  # -90 degrees
            joint_upper_limit=1.57    # +90 degrees
        )
        
        # Load the generated URDF
        load_urdf(sim_id="...", file_path=result["urdf_path"], position=[0, 0, 0])
    
    Raises:
        ToolError: If invalid shape type or generation fails
    """
    try:
        # Set defaults
        if joint_origin is None:
            joint_origin = [0.0, 0.0, 0.0]
        
        # Generate URDF file
        urdf_path = generate_revolute_joint_urdf(
            parent_shape=parent_shape,
            child_shape=child_shape,
            parent_dimensions=parent_dimensions,
            child_dimensions=child_dimensions,
            parent_mass=parent_mass,
            child_mass=child_mass,
            joint_axis=joint_axis,
            joint_origin=joint_origin,
            joint_limits=(joint_lower_limit, joint_upper_limit),
            max_effort=max_effort,
            max_velocity=max_velocity,
            output_path=output_path
        )
        
        return {
            "urdf_path": urdf_path,
            "parent_shape": parent_shape,
            "child_shape": child_shape,
            "joint_type": "revolute",
            "joint_axis": joint_axis,
            "joint_limits": [joint_lower_limit, joint_upper_limit]
        }
        
    except ValueError as e:
        raise ToolError(f"Failed to generate revolute joint URDF: {e}")
    except Exception as e:
        raise ToolError(f"Unexpected error generating URDF: {e}")


# ============================================================================
# Server Entry Point
# ============================================================================

if __name__ == "__main__":
    mcp.run(transport="http", port=8000)
