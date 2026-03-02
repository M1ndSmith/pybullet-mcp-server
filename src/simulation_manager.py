"""SimulationManager class for managing multiple PyBullet simulations."""

import uuid
from typing import Dict, Tuple, List
import pybullet as p

from .simulation_context import SimulationContext


class SimulationManager:
    """Manages multiple PyBullet physics simulations.
    
    This is a helper class called BY MCP tools, not an MCP tool itself.
    Methods raise standard Python exceptions (ValueError, etc.) which MCP tools
    will convert to ToolError.
    """
    
    def __init__(self):
        """Initialize the simulation manager with an empty simulation registry."""
        self.simulations: Dict[str, SimulationContext] = {}
    
    def create_simulation(
        self, 
        gravity: Tuple[float, float, float] = (0.0, 0.0, -9.81),
        gui: bool = False
    ) -> str:
        """Create a new PyBullet physics simulation.
        
        Args:
            gravity: Gravity vector (x, y, z) in m/s^2. Default is Earth gravity.
            gui: Whether to enable GUI visualization window.
            
        Returns:
            UUID string identifying the new simulation.
            
        Raises:
            RuntimeError: If PyBullet client connection fails.
        """
        # Connect to PyBullet physics engine
        mode = p.GUI if gui else p.DIRECT
        client_id = p.connect(mode)
        
        if client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet physics engine")
        
        # Configure gravity for this simulation
        p.setGravity(gravity[0], gravity[1], gravity[2], physicsClientId=client_id)
        
        # Generate unique simulation ID
        sim_id = str(uuid.uuid4())
        
        # Create and store simulation context
        self.simulations[sim_id] = SimulationContext(
            client_id=client_id,
            gui_enabled=gui,
            gravity=gravity
        )
        
        return sim_id
    
    def get_simulation(self, sim_id: str) -> SimulationContext:
        """Retrieve a simulation context by ID.
        
        Args:
            sim_id: UUID string identifying the simulation.
            
        Returns:
            SimulationContext for the requested simulation.
            
        Raises:
            ValueError: If simulation ID is not found.
        """
        if sim_id not in self.simulations:
            raise ValueError(f"Simulation {sim_id} not found")
        return self.simulations[sim_id]
    
    def has_simulation(self, sim_id: str) -> bool:
        """Check if a simulation exists.
        
        Args:
            sim_id: UUID string identifying the simulation.
            
        Returns:
            True if simulation exists, False otherwise.
        """
        return sim_id in self.simulations
    
    def destroy_simulation(self, sim_id: str) -> None:
        """Clean up a simulation and remove it from the registry.
        
        Args:
            sim_id: UUID string identifying the simulation.
            
        Raises:
            ValueError: If simulation ID is not found.
        """
        if sim_id not in self.simulations:
            raise ValueError(f"Simulation {sim_id} not found")
        
        # Get simulation context and clean up resources
        sim = self.simulations[sim_id]
        sim.cleanup()
        
        # Remove from registry
        del self.simulations[sim_id]
    
    def list_simulations(self) -> List[str]:
        """Return all active simulation IDs.
        
        Returns:
            List of UUID strings for all active simulations.
        """
        return list(self.simulations.keys())
    def step_simulation(self, sim_id: str) -> None:
        """Advance the simulation by one timestep.

        Args:
            sim_id: UUID string identifying the simulation.

        Raises:
            ValueError: If simulation ID is not found.
        """
        sim = self.get_simulation(sim_id)
        p.stepSimulation(physicsClientId=sim.client_id)
        sim.simulation_time += sim.timestep

    def step_multiple(self, sim_id: str, steps: int) -> None:
        """Advance the simulation by multiple timesteps.

        Args:
            sim_id: UUID string identifying the simulation.
            steps: Number of timesteps to execute.

        Raises:
            ValueError: If simulation ID is not found or steps is negative.
        """
        if steps < 0:
            raise ValueError(f"Steps must be non-negative, got {steps}")

        sim = self.get_simulation(sim_id)
        for _ in range(steps):
            p.stepSimulation(physicsClientId=sim.client_id)
        sim.simulation_time += sim.timestep * steps

    def set_timestep(self, sim_id: str, timestep: float) -> None:
        """Configure the timestep duration for a simulation.

        Args:
            sim_id: UUID string identifying the simulation.
            timestep: New timestep duration in seconds.

        Raises:
            ValueError: If simulation ID is not found or timestep is non-positive.
        """
        if timestep <= 0:
            raise ValueError(f"Timestep must be positive, got {timestep}")

        sim = self.get_simulation(sim_id)
        sim.set_timestep(timestep)

    def enable_debug_visualization(self, sim_id: str, show_contact_points: bool = True,
                                   show_frames: bool = False) -> None:
        """Enable debug visualization options for a simulation.

        Args:
            sim_id: UUID string identifying the simulation.
            show_contact_points: Whether to visualize contact points.
            show_frames: Whether to visualize coordinate frames for objects.

        Raises:
            ValueError: If simulation ID is not found.
        """
        sim = self.get_simulation(sim_id)

        # Configure debug visualization options
        if show_contact_points:
            p.configureDebugVisualizer(
                p.COV_ENABLE_RENDERING, 1,
                physicsClientId=sim.client_id
            )

        # Store visualization settings in simulation context
        sim.debug_contact_points = show_contact_points
        sim.debug_frames = show_frames

    def set_camera(self, sim_id: str, distance: float, yaw: float, pitch: float,
                   target_position: list[float]) -> None:
        """Set camera position for GUI visualization.

        Args:
            sim_id: UUID string identifying the simulation.
            distance: Distance from camera to target in meters.
            yaw: Camera yaw angle in degrees.
            pitch: Camera pitch angle in degrees.
            target_position: Position [x, y, z] that camera looks at.

        Raises:
            ValueError: If simulation ID is not found or GUI is not enabled.
        """
        sim = self.get_simulation(sim_id)

        if not sim.gui_enabled:
            raise ValueError(f"Cannot set camera for simulation {sim_id}: GUI mode is not enabled")

        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=target_position,
            physicsClientId=sim.client_id
        )

