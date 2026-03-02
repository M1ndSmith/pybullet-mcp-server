# PyBullet MCP Server

Version 1.0.0

A Model Context Protocol (MCP) server that enables AI assistants to interact with PyBullet physics simulations. Build physics-based projects through natural language interactions with AI agents.

## Features

- **20 MCP Tools**: Comprehensive API for physics simulation control
- **Simulation Management**: Create and manage multiple independent physics simulations with configurable gravity
- **Object Manipulation**: Add primitive shapes (box, sphere, cylinder, capsule) and URDF models with full property control
- **Physics Control**: Apply forces, torques, and step through simulations with configurable timesteps
- **State Persistence**: Save and load complete simulation states to/from JSON files
- **Constraints**: Create joints between objects (fixed, revolute, prismatic, spherical)
- **Collision Detection**: Query contact points with detailed collision information
- **Visualization**: Optional GUI mode with debug visualization and camera control
- **Error Handling**: Comprehensive validation with descriptive error messages

## Installation

### Prerequisites

- Python 3.9 or higher
- pip package manager
- Virtual environment (recommended)

### Install Dependencies

1. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install FastMCP and PyBullet:
```bash
pip install fastmcp pybullet
```

3. For development (includes testing tools):
```bash
pip install fastmcp pybullet pytest hypothesis pytest-cov
```

### Verify Installation

Check that the required packages are installed:
```bash
python -c "import pybullet; import mcp; print('Installation successful!')"
```

## Quick Start

### Running the Server

See [QUICKSTART.md](QUICKSTART.md) for detailed instructions on:
- Starting the server
- Configuring Claude Desktop
- Example prompts to explore all features
- Common workflows and use cases

**Quick command:**
```bash
source venv/bin/activate  # On Windows: venv\Scripts\activate
python -m src.server
```

The server will start and listen for MCP protocol connections from AI assistants.

### Example Usage with MCP Client

Once connected to an MCP client (like Claude Desktop), you can interact through natural language:

**1. Create a simulation:**
```
Create a new physics simulation with Earth gravity
```
This calls `create_simulation` with default gravity [0, 0, -9.81].

**2. Add objects:**
```
Add a red box at position (0, 0, 1) with dimensions 0.5x0.5x0.5 and mass 1.0
```
This calls `add_box` to create a box object.

```
Add a sphere at (2, 0, 1) with radius 0.3
```
This calls `add_sphere` to create a sphere.

**3. Run the simulation:**
```
Step the simulation forward 100 times
```
This calls `step_simulation` with steps=100.

**4. Query object state:**
```
What is the position and velocity of object 0?
```
This calls `get_object_state` to retrieve position, orientation, and velocities.

**5. Apply forces:**
```
Apply a force of [10, 0, 0] to object 0
```
This calls `apply_force` to push the object.

**6. Save the simulation:**
```
Save the current simulation state to simulation.json
```
This calls `save_simulation` to persist the state.

**7. Load a simulation:**
```
Load the simulation from simulation.json
```
This calls `load_simulation` to restore the saved state.

## MCP Client Configuration

### Claude Desktop Setup

To use this server with Claude Desktop, add the following to your MCP configuration file:

**macOS**: `~/Library/Application Support/Claude/claude_desktop_config.json`
**Windows**: `%APPDATA%\Claude\claude_desktop_config.json`

```json
{
  "mcpServers": {
    "pybullet": {
      "command": "python",
      "args": ["-m", "src.server"],
      "cwd": "/absolute/path/to/pybullet-mcp-server",
      "env": {
        "PYTHONPATH": "/absolute/path/to/pybullet-mcp-server"
      }
    }
  }
}
```

**Important**: 
- Replace `/absolute/path/to/pybullet-mcp-server` with the actual path to your project directory
- Ensure the virtual environment is activated or use the full path to the Python interpreter in the venv
- Restart Claude Desktop after updating the configuration

### Alternative: Using Virtual Environment Python

If you're using a virtual environment, specify the venv Python directly:

```json
{
  "mcpServers": {
    "pybullet": {
      "command": "/absolute/path/to/pybullet-mcp-server/venv/bin/python",
      "args": ["-m", "src.server"],
      "cwd": "/absolute/path/to/pybullet-mcp-server"
    }
  }
}
```

On Windows, use: `"command": "C:\\path\\to\\pybullet-mcp-server\\venv\\Scripts\\python.exe"`

## Available Tools

The server exposes 20 tools through the MCP protocol:

### Simulation Management (5 tools)
- **`create_simulation`**: Initialize a new physics simulation with configurable gravity and optional GUI
  - Parameters: `gravity` (list[float], default: [0, 0, -9.81]), `gui` (bool, default: false)
  - Returns: simulation_id, gravity, gui_enabled
  
- **`list_simulations`**: Get all active simulation IDs
  - Returns: list of simulation IDs
  
- **`destroy_simulation`**: Clean up and remove a simulation
  - Parameters: `sim_id` (str)
  - Returns: confirmation message
  
- **`step_simulation`**: Advance simulation by one or more timesteps
  - Parameters: `sim_id` (str), `steps` (int, default: 1)
  - Returns: simulation_id, steps_taken, current_time
  
- **`set_timestep`**: Configure the timestep duration for a simulation
  - Parameters: `sim_id` (str), `timestep` (float)
  - Returns: confirmation message

### Object Operations (9 tools)
- **`add_box`**: Add a box shape to the simulation
  - Parameters: `sim_id`, `dimensions` (list[float]), `position` (list[float]), `mass` (float, default: 1.0), `color` (list[float], optional)
  - Returns: object_id, shape, position
  
- **`add_sphere`**: Add a sphere shape to the simulation
  - Parameters: `sim_id`, `radius` (float), `position` (list[float]), `mass` (float, default: 1.0), `color` (list[float], optional)
  - Returns: object_id, shape, position
  
- **`add_cylinder`**: Add a cylinder shape to the simulation
  - Parameters: `sim_id`, `radius` (float), `height` (float), `position` (list[float]), `mass` (float, default: 1.0), `color` (list[float], optional)
  - Returns: object_id, shape, position
  
- **`add_capsule`**: Add a capsule shape to the simulation
  - Parameters: `sim_id`, `radius` (float), `height` (float), `position` (list[float]), `mass` (float, default: 1.0), `color` (list[float], optional)
  - Returns: object_id, shape, position
  
- **`load_urdf`**: Load a robot model from a URDF file
  - Parameters: `sim_id`, `file_path` (str), `position` (list[float]), `orientation` (list[float], optional)
  - Returns: object_id, file_path, position
  
- **`set_object_pose`**: Update object position and orientation
  - Parameters: `sim_id`, `object_id` (int), `position` (list[float]), `orientation` (list[float])
  - Returns: confirmation message
  
- **`get_object_state`**: Query complete object state
  - Parameters: `sim_id`, `object_id` (int)
  - Returns: position, orientation, linear_velocity, angular_velocity
  
- **`apply_force`**: Apply a force vector to an object
  - Parameters: `sim_id`, `object_id` (int), `force` (list[float]), `position` (list[float], optional)
  - Returns: confirmation message
  
- **`apply_torque`**: Apply rotational force to an object
  - Parameters: `sim_id`, `object_id` (int), `torque` (list[float])
  - Returns: confirmation message

### Constraint Management (2 tools)
- **`create_constraint`**: Create a joint between two objects
  - Parameters: `sim_id`, `parent_id` (int), `child_id` (int), `joint_type` (str), `joint_axis` (list[float], optional), `parent_frame_position` (list[float], optional), `child_frame_position` (list[float], optional)
  - Joint types: "fixed", "revolute", "prismatic", "spherical"
  - Returns: constraint_id, joint_type
  
- **`remove_constraint`**: Remove a constraint from the simulation
  - Parameters: `sim_id`, `constraint_id` (int)
  - Returns: confirmation message

### Collision Detection (2 tools)
- **`get_all_collisions`**: Query all contact points in the simulation
  - Parameters: `sim_id`
  - Returns: list of contact points with positions, normals, forces
  
- **`get_collisions_for_pair`**: Query contact points between specific objects
  - Parameters: `sim_id`, `obj_a` (int), `obj_b` (int)
  - Returns: list of contact points for the pair

### Visualization (2 tools)
- **`enable_debug_visualization`**: Enable debug rendering of contact points and frames
  - Parameters: `sim_id`, `show_contacts` (bool, default: true), `show_frames` (bool, default: false)
  - Returns: confirmation message
  
- **`set_camera`**: Configure camera position and target for GUI mode
  - Parameters: `sim_id`, `distance` (float), `yaw` (float), `pitch` (float), `target` (list[float])
  - Returns: confirmation message

### Persistence (2 tools)
- **`save_simulation`**: Save simulation state to a JSON file
  - Parameters: `sim_id`, `file_path` (str)
  - Returns: confirmation with file path
  
- **`load_simulation`**: Load simulation state from a JSON file
  - Parameters: `file_path` (str), `gui` (bool, default: false)
  - Returns: new simulation_id, file_path

## Example Workflows

### Basic Falling Box

Create a simple simulation with a box falling under gravity:

```python
# Through MCP client (natural language):
"Create a simulation with Earth gravity"
"Add a box at position (0, 0, 5) with dimensions 1x1x1"
"Step the simulation 200 times"
"What is the position of object 0?"
```

### Stacked Objects

Create a stack of objects:

```python
"Create a simulation"
"Add a box at (0, 0, 0.5) with dimensions 10x10x1 and mass 0"  # Ground
"Add a box at (0, 0, 1.5) with dimensions 1x1x1"
"Add a sphere at (0, 0, 3) with radius 0.5"
"Step the simulation 300 times"
"Get all collisions"
```

### Constrained Objects

Create objects connected by a joint:

```python
"Create a simulation"
"Add a box at (0, 0, 2) with dimensions 1x1x1"  # Object 0
"Add a sphere at (2, 0, 2) with radius 0.5"     # Object 1
"Create a revolute constraint between object 0 and object 1"
"Apply a torque of [0, 0, 10] to object 1"
"Step the simulation 200 times"
```

### Save and Load

Persist a simulation:

```python
"Create a simulation"
"Add a box at (0, 0, 1)"
"Add a sphere at (1, 0, 1)"
"Step the simulation 50 times"
"Save the simulation to my_sim.json"

# Later...
"Load the simulation from my_sim.json"
"Step the simulation 50 more times"
```

### Robot Simulation

Load a URDF robot model:

```python
"Create a simulation with GUI enabled"
"Load URDF from /path/to/robot.urdf at position (0, 0, 1)"
"Step the simulation 100 times"
"Get the state of object 0"
```

## Development

### Project Structure

```
pybullet-mcp-server/
├── src/                          # Source code
│   ├── __init__.py
│   ├── server.py                 # FastMCP server with 20 MCP tools
│   ├── simulation_manager.py    # Simulation lifecycle management
│   ├── object_manager.py         # Object creation and manipulation
│   ├── constraint_manager.py    # Joint and constraint handling
│   ├── persistence.py            # Save/load functionality
│   └── collision_detection.py   # Collision query handling
├── tests/                        # Test suite
│   ├── unit/                     # Unit tests for each component
│   ├── property/                 # Property-based tests (Hypothesis)
│   └── integration/              # End-to-end workflow tests
├── venv/                         # Virtual environment (created by you)
├── pyproject.toml                # Project configuration
└── README.md                     # This file
```

### Running Tests

Activate the virtual environment first:
```bash
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

Run all tests:
```bash
pytest
```

Run specific test categories:
```bash
pytest tests/unit/              # Unit tests only
pytest tests/property/          # Property-based tests only
pytest tests/integration/       # Integration tests only
```

Run with verbose output:
```bash
pytest -v
```

Run with coverage report:
```bash
pytest --cov=src --cov-report=html
open htmlcov/index.html  # View coverage report
```

### Code Quality

The project uses modern Python tooling for code quality:

**Format code with Black:**
```bash
black src tests
```

**Lint with Ruff:**
```bash
ruff check src tests
```

**Type check with mypy:**
```bash
mypy src
```

### Testing Strategy

The project uses three types of tests:

1. **Unit Tests**: Test individual components in isolation
2. **Property-Based Tests**: Use Hypothesis to test properties across random inputs (100+ iterations per test)
3. **Integration Tests**: Test complete workflows end-to-end

All 21 correctness properties from the design document are validated with property-based tests.

## Persistence File Format

Simulation states are saved as JSON files with the following structure:

```json
{
  "gravity": [0.0, 0.0, -9.81],
  "timestep": 0.01,
  "objects": [
    {
      "object_id": 0,
      "type": "primitive",
      "shape": "box",
      "dimensions": [0.5, 0.5, 0.5],
      "position": [0.0, 0.0, 1.0],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "linear_velocity": [0.0, 0.0, -0.98],
      "angular_velocity": [0.0, 0.0, 0.0],
      "mass": 1.0,
      "color": [1.0, 0.0, 0.0, 1.0]
    },
    {
      "object_id": 1,
      "type": "urdf",
      "urdf_path": "/path/to/model.urdf",
      "position": [2.0, 0.0, 0.5],
      "orientation": [0.0, 0.0, 0.0, 1.0],
      "linear_velocity": [0.0, 0.0, 0.0],
      "angular_velocity": [0.0, 0.0, 0.0]
    }
  ],
  "constraints": [
    {
      "constraint_id": 0,
      "parent_id": 0,
      "child_id": 1,
      "joint_type": "revolute",
      "parent_frame_position": [0.0, 0.0, 0.0],
      "child_frame_position": [0.0, 0.0, 0.0],
      "joint_axis": [0.0, 0.0, 1.0]
    }
  ]
}
```

### Compatibility Notes

- **Object IDs**: Object IDs are reassigned when loading (may differ from saved IDs)
- **URDF Files**: URDF file paths must be valid when loading
- **Simulation ID**: A new simulation ID is generated when loading
- **Format Version**: Current format is compatible with PyBullet 3.2.5+

## Compatibility

- **Python**: 3.9, 3.10, 3.11, 3.12
- **PyBullet**: 3.2.5 or higher
- **FastMCP**: 0.3.5 or higher (latest recommended)
- **Operating Systems**: Linux, macOS, Windows
- **MCP Clients**: Claude Desktop, any MCP-compatible client

### Technical Notes

- Only one GUI simulation can be active at a time (PyBullet limitation)
- URDF files must be accessible from the server's working directory
- Large simulations (1000+ objects) may impact performance
- GUI mode not available in headless environments (Docker, SSH without X11)

### Future Enhancements

Planned features for upcoming versions:

- Robot joint control and manipulation
- Inverse kinematics calculations
- Joint state queries and monitoring
- Advanced velocity control
- Dynamic property modification
- Ray casting for sensor simulation
- Camera rendering capabilities

## Troubleshooting

### Common Issues

**1. Import errors when starting the server**
```
ModuleNotFoundError: No module named 'mcp' or 'pybullet'
```
**Solution**: Ensure your virtual environment is activated and dependencies are installed:
```bash
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install fastmcp pybullet
```

**2. Python version errors**
```
SyntaxError or version compatibility issues
```
**Solution**: Check Python version (must be 3.9+):
```bash
python --version
```
If needed, create a venv with a specific Python version:
```bash
python3.9 -m venv venv
```

**3. PyBullet GUI not showing**
```
GUI window doesn't appear when gui=true
```
**Solution**: 
- GUI mode must be explicitly enabled: `create_simulation(gui=true)`
- Some systems (servers, Docker) don't support GUI mode - use headless mode instead
- On Linux, ensure X11 is available: `echo $DISPLAY`

**4. File permission errors**
```
PermissionError when saving/loading simulations
```
**Solution**: 
- Ensure write permissions in the target directory
- Use absolute paths: `/full/path/to/simulation.json`
- Check disk space: `df -h`

**5. MCP client can't connect to server**
```
Claude Desktop shows "Server not responding"
```
**Solution**:
- Verify the `cwd` path in the MCP config is correct
- Use absolute paths, not relative paths
- Check that Python can find the src module: `python -c "import src.server"`
- Restart Claude Desktop after config changes
- Check Claude Desktop logs for error messages

**6. Simulation behaves unexpectedly**
```
Objects fall through the ground or constraints don't work
```
**Solution**:
- Ensure you're stepping the simulation: `step_simulation(sim_id, steps=100)`
- Check timestep value (default 0.01 is usually good)
- Verify object masses are positive
- For ground planes, use a box with large dimensions and mass=0

**7. URDF loading fails**
```
ToolError: Failed to load URDF
```
**Solution**:
- Verify the URDF file path is correct and accessible
- Use absolute paths for URDF files
- Check that mesh files referenced in URDF are also accessible
- Validate URDF syntax with PyBullet directly

### Debug Mode

To see detailed error messages, check the server output in your terminal. The server logs all operations and errors.

### Getting Help

If you encounter issues not covered here:
1. Check the server logs for detailed error messages
2. Verify your MCP client configuration
3. Test the server directly with Python to isolate MCP vs. PyBullet issues
4. Review the PyBullet documentation for physics-specific questions

## Architecture

The server follows a layered architecture:

```
MCP Client (Claude Desktop)
         ↓
   MCP Protocol
         ↓
FastMCP Server (20 tools)
         ↓
Manager Classes (helpers)
         ↓
PyBullet Physics Engine
```

**Key Components:**
- **FastMCP Server** (`src/server.py`): Exposes 20 MCP tools using `@mcp.tool` decorators
- **SimulationManager**: Manages PyBullet physics clients and simulation lifecycle
- **ObjectManager**: Handles object creation, manipulation, and state queries
- **ConstraintManager**: Creates and manages joints between objects
- **PersistenceHandler**: Serializes/deserializes simulation state to JSON
- **CollisionQueryHandler**: Queries contact points and collision information

Each MCP tool validates inputs, calls the appropriate manager, and returns Python objects (dict/list/str/int) directly - FastMCP handles the MCP protocol conversion automatically.

## Contributing

Contributions are welcome! Please ensure:
- All tests pass: `pytest`
- Code is formatted: `black src tests`
- Code passes linting: `ruff check src tests`
- Type hints are correct: `mypy src`
- New features include tests (unit + property-based)
- Documentation is updated



## Acknowledgments

- Built with [FastMCP](https://github.com/jlowin/fastmcp) by Jeremiah Lowin
- Physics simulation powered by [PyBullet](https://pybullet.org/)
- Property-based testing with [Hypothesis](https://hypothesis.readthedocs.io/)

## Support

For issues, questions, or contributions:
- Review this README and troubleshooting section
- Check the design document in `.kiro/specs/pybullet-mcp-server/design.md`
- Test the server directly with Python to isolate issues
- Review PyBullet documentation for physics-specific questions
