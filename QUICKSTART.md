# PyBullet MCP Server - Quick Start Guide

Version 1.0.0

This guide will help you start the server and explore all available tools through example prompts.

## Table of Contents

1. [Starting the Server](#starting-the-server)
2. [Configuring Claude Desktop](#configuring-claude-desktop)
3. [Example Prompts by Category](#example-prompts-by-category)
4. [Complete Exploration Workflow](#complete-exploration-workflow)
5. [Troubleshooting](#troubleshooting)

---

## Starting the Server

### Step 1: Activate Virtual Environment

```bash
# On Linux/macOS
source venv/bin/activate

# On Windows
venv\Scripts\activate
```

### Step 2: Start the MCP Server

```bash
python -m src.server
```

You should see output indicating the server is running and listening for MCP connections.

### Step 3: Keep the Terminal Open

Leave this terminal window open while using the server. The server will log all operations.

---

## Configuring Claude Desktop

### macOS Configuration

Edit: `~/Library/Application Support/Claude/claude_desktop_config.json`

```json
{
  "mcpServers": {
    "pybullet": {
      "command": "/absolute/path/to/pybullet_mcp/venv/bin/python",
      "args": ["-m", "src.server"],
      "cwd": "/absolute/path/to/pybullet_mcp"
    }
  }
}
```

### Windows Configuration

Edit: `%APPDATA%\Claude\claude_desktop_config.json`

```json
{
  "mcpServers": {
    "pybullet": {
      "command": "C:\\path\\to\\pybullet_mcp\\venv\\Scripts\\python.exe",
      "args": ["-m", "src.server"],
      "cwd": "C:\\path\\to\\pybullet_mcp"
    }
  }
}
```

### Linux Configuration

Similar to macOS, adjust paths accordingly.

**Important**: Replace `/absolute/path/to/pybullet_mcp` with your actual project directory path.

### Restart Claude Desktop

After saving the configuration, restart Claude Desktop for changes to take effect.

---

## Example Prompts by Category

### 1. Simulation Management (5 tools)

#### Create a Simulation
```
Create a new physics simulation with Earth gravity
```

#### Create Custom Gravity Simulation
```
Create a simulation with gravity [0, 0, -5] and GUI disabled
```

#### List Active Simulations
```
List all active simulations
```

#### Step the Simulation
```
Step the simulation forward 100 times
```

#### Configure Timestep
```
Set the timestep to 0.001 seconds for more accurate physics
```

#### Destroy Simulation
```
Destroy the simulation and clean up resources
```

---

### 2. Object Operations (9 tools)

#### Add Primitive Shapes
```
Add a red box at position (0, 0, 1) with dimensions 0.5x0.5x0.5 and mass 1.0
```

```
Add a blue sphere at position (2, 0, 1) with radius 0.3 and mass 0.5
```

```
Add a green cylinder at position (-1, 0, 1) with radius 0.2, height 1.0, and mass 2.0
```

```
Add a yellow capsule at position (1, 1, 1) with radius 0.15, height 0.8, and mass 1.5
```

#### Load URDF Model
```
Load the URDF file from /absolute/path/to/robot.urdf at position (0, 0, 0)
```

**Note**: For testing, you can use PyBullet's built-in URDFs:
```
Load URDF from /path/to/venv/lib/python3.x/site-packages/pybullet_data/husky/husky.urdf at position (0, 0, 2)
```

#### Query Object State
```
What is the position and velocity of object 0?
```

```
Get the complete state of object 2
```

#### Manipulate Objects
```
Move object 1 to position (1, 0, 0) with orientation (0, 0, 0, 1)
```

```
Apply a force of [10, 0, 0] to object 0
```

```
Apply a force of [0, 0, 50] at position [0.5, 0, 0] to object 1
```

```
Apply a torque of [0, 0, 5] to object 2
```

---

### 3. Constraint Management (2 tools)

#### Create Constraints
```
Create a fixed constraint between object 0 and object 1
```

```
Create a prismatic constraint between object 1 and object 2 with joint axis [0, 0, 1]
```

```
Create a spherical constraint between object 2 and object 3
```

#### Remove Constraints
```
Remove constraint 1 from the simulation
```

---

### 4. Collision Detection (2 tools)

#### Query All Collisions
```
Get all collisions in the simulation
```

```
Show me all contact points with their forces
```

#### Query Specific Pair
```
Check if object 0 and object 1 are colliding
```

```
Get collision information between object 2 and object 3
```

---

### 5. Visualization (2 tools)

#### Enable Debug Visualization
```
Enable debug visualization with contact points visible
```

```
Enable debug visualization showing both contact points and coordinate frames
```

#### Control Camera
```
Set the camera to distance 5, yaw 45, pitch -30, looking at position (0, 0, 0)
```

```
Move the camera to view from above: distance 10, yaw 0, pitch -89, target (0, 0, 0)
```

---

### 6. Persistence (2 tools)

#### Save Simulation
```
Save the current simulation state to simulation_checkpoint.json
```

```
Save the simulation to /tmp/my_simulation.json
```

#### Load Simulation
```
Load the simulation from simulation_checkpoint.json
```

```
Load the simulation from /tmp/my_simulation.json with GUI enabled
```

---

### 7. Robot Control (5 tools)

#### Query Joint Information
```
How many joints does object 0 have?
```

```
Get information about joint 0 of object 0
```

```
Get detailed information for all joints of object 0 (joints 0 through 5)
```

#### Query Joint State
```
What is the current state of joint 0 on object 0?
```

```
Get the position and velocity of joint 2
```

#### Control Joints
```
Set joint 0 to position 1.57 using position control with force 100
```

```
Control joint 1 with velocity 2.0 using velocity control
```

```
Apply torque control to joint 2 with force 10
```

```
Set joint 3 to position 0.5 with position gain 0.1 and velocity gain 0.05
```

#### Inverse Kinematics
```
Calculate inverse kinematics for object 0 end-effector link 6 to reach position [0.5, 0, 0.5]
```

```
Calculate IK for end-effector at position [0.3, 0.2, 0.4] with orientation [0, 0, 0, 1]
```

```
Calculate IK with joint limits: lower [-3.14, -1.57, 0], upper [3.14, 1.57, 3.14]
```

---

## Complete Exploration Workflow

Here's a complete sequence of prompts that exercises all 27 tools:

### Phase 1: Setup (Simulation Management)
```
1. Create a new physics simulation with Earth gravity
2. List all active simulations
3. Set the timestep to 0.01 seconds
```

### Phase 2: Build Scene (Object Operations)
```
4. Add a large box at position (0, 0, 0) with dimensions 5x5x0.1 and mass 1000 (ground plane)
5. Add a red box at position (0, 0, 2) with dimensions 0.5x0.5x0.5 and mass 1.0
6. Add a blue sphere at position (1, 0, 3) with radius 0.3 and mass 0.5
7. Add a green cylinder at position (-1, 0, 2.5) with radius 0.2, height 1.0, and mass 2.0
8. Add a yellow capsule at position (2, 0, 2) with radius 0.15, height 0.8, and mass 1.5
```

### Phase 3: Create Connections (Constraints)
```
9. Create a fixed constraint between object 1 (red box) and object 2 (blue sphere)
10. Create a prismatic constraint between object 3 (cylinder) and object 4 (capsule) with joint axis [1, 0, 0]
```

### Phase 4: Apply Forces (Object Manipulation)
```
11. Apply a force of [20, 0, 0] to object 1
12. Apply a torque of [0, 0, 10] to object 3
13. Apply a force of [0, 0, 50] at position [0.5, 0, 0] to object 4
```

### Phase 5: Run Simulation (Simulation Management)
```
14. Step the simulation forward 100 times
```

### Phase 6: Query State (Object Operations & Collision Detection)
```
15. What is the position and velocity of object 1?
16. Get the state of object 2
17. Get all collisions in the simulation
18. Check if object 1 and object 0 (ground) are colliding
```

### Phase 7: Modify and Continue
```
19. Move object 4 to position (3, 0, 5) with orientation (0, 0, 0, 1)
20. Step the simulation forward 50 more times
21. Get all collisions again
```

### Phase 8: Persistence
```
22. Save the current simulation state to test_simulation.json
23. Destroy the simulation
24. Load the simulation from test_simulation.json
25. Step the simulation forward 50 times to verify it loaded correctly
```

### Phase 9: Visualization (if GUI enabled)
```
26. Create a new simulation with GUI enabled
27. Add a box and sphere
28. Enable debug visualization with contact points
29. Set camera to distance 5, yaw 45, pitch -30, target (0, 0, 0)
30. Step the simulation 200 times and observe
```

### Phase 10: Robot Control (with URDF)
```
31. Create a new simulation
32. Load URDF from /path/to/robot.urdf at position (0, 0, 0)
33. How many joints does object 0 have?
34. Get information about joint 0
35. Get the current state of joint 0
36. Set joint 0 to position 1.57 using position control
37. Step the simulation 100 times
38. Get the state of joint 0 again
39. Calculate inverse kinematics for end-effector link 6 at position [0.5, 0, 0.5]
40. Apply the IK solution to the joints
41. Step the simulation 50 times
```

### Phase 11: Cleanup
```
42. List all active simulations
43. Destroy all simulations
```

---

## Advanced Examples

### Example 1: Falling Stack
```
Create a simulation
Add a ground plane: box at (0,0,0) with dimensions 10x10x0.1 and mass 1000
Add box 1 at (0,0,1) with dimensions 1x1x1 and mass 1
Add box 2 at (0,0,2.5) with dimensions 1x1x1 and mass 1
Add box 3 at (0,0,4) with dimensions 1x1x1 and mass 1
Step the simulation 300 times
Get all collisions
What's the position of object 3?
```

### Example 2: Pendulum
```
Create a simulation
Add a heavy anchor: sphere at (0,0,3) with radius 0.1 and mass 100
Add a pendulum bob: sphere at (0,0,1) with radius 0.3 and mass 2
Create a fixed constraint between object 0 and object 1
Apply a force of [50, 0, 0] to object 1
Step the simulation 500 times
Get the state of object 1
```

### Example 3: Sliding Objects
```
Create a simulation
Add a ramp: box at (0,0,0.5) with dimensions 5x5x0.1 and mass 1000
Move the ramp to create an angle (you'll need to calculate quaternion)
Add a sliding box at (0,0,3) with dimensions 0.5x0.5x0.5 and mass 1
Step the simulation 400 times
Track the position of the sliding box every 100 steps
```

### Example 4: Robot Arm Control (URDF)
```
Create a simulation with GUI
Load URDF from /path/to/robot_arm.urdf at position (0,0,0)
How many joints does object 0 have?
Get information about joint 0
Get the current state of joint 0
Set joint 0 to position 1.57 using position control with force 100
Set joint 1 to position -0.5 using position control
Step the simulation 200 times
Get the state of joint 0
Calculate inverse kinematics for end-effector link 6 at position [0.4, 0.2, 0.3]
```

### Example 5: Robot Manipulation with IK
```
Create a simulation
Load URDF from /path/to/robot_arm.urdf at position (0,0,0)
Add a target sphere at (0.5, 0, 0.5) with radius 0.05
Calculate IK for end-effector link 6 to reach position [0.5, 0, 0.5]
Apply the IK solution to all joints using position control
Step the simulation 300 times
Check if the end-effector reached the target
```

---

## Troubleshooting

### Server Not Starting

**Problem**: `ModuleNotFoundError: No module named 'mcp'`

**Solution**:
```bash
source venv/bin/activate
pip install fastmcp pybullet
```

### Claude Desktop Can't Connect

**Problem**: "Server not responding" in Claude Desktop

**Solutions**:
1. Check that paths in `claude_desktop_config.json` are absolute, not relative
2. Verify the Python path points to your venv Python
3. Restart Claude Desktop after config changes
4. Check Claude Desktop logs for error messages

### GUI Not Showing

**Problem**: GUI window doesn't appear when `gui=true`

**Solutions**:
1. Only ONE GUI simulation allowed per process
2. Destroy existing GUI simulation before creating a new one
3. Some systems (servers, Docker) don't support GUI - use headless mode
4. On Linux, ensure X11 is available: `echo $DISPLAY`

### URDF Loading Fails

**Problem**: `ToolError: File not found: robot.urdf`

**Solutions**:
1. Use absolute paths for URDF files
2. Verify the file exists: `ls -la /path/to/robot.urdf`
3. Check that mesh files referenced in URDF are also accessible
4. Test with PyBullet's built-in URDFs first

### Objects Fall Through Ground

**Problem**: Objects pass through the ground plane

**Solutions**:
1. Ensure you're stepping the simulation: `step_simulation(sim_id, steps=100)`
2. Check timestep value (default 0.01 is usually good)
3. Verify ground plane has large dimensions and high mass
4. Check that objects have positive mass

### Simulation Runs Slowly

**Problem**: Simulation takes a long time to step

**Solutions**:
1. Use headless mode (GUI adds overhead)
2. Increase timestep for faster (less accurate) simulation
3. Reduce number of objects
4. Use primitive shapes instead of complex URDF meshes

---

## Tips for Effective Use

### 1. Start Simple
Begin with basic shapes and simple scenarios before moving to complex URDF models.

### 2. Step Incrementally
Step the simulation in small increments (10-100 steps) and check state frequently.

### 3. Use Descriptive Names
When saving simulations, use descriptive filenames: `pendulum_test.json`, `robot_arm_grasp.json`

### 4. Monitor Collisions
Check collisions regularly to understand object interactions.

### 5. Save Checkpoints
Save simulation state before making major changes so you can reload if needed.

### 6. Use Headless Mode
For batch testing or automated workflows, use headless mode (GUI disabled) for better performance.

### 7. Verify Object IDs
After loading URDF or creating objects, note the object IDs returned for later reference.

---

## Next Steps

After exploring the tools:

1. **Read the full documentation**: See `README.md` for detailed API reference
2. **Check the architecture**: See `docs/ARCHITECTURE.md` for system design
3. **Review examples**: See `docs/TOOLS.md` for usage patterns
4. **Run tests**: Execute `pytest` to see property-based testing in action
5. **Explore PyBullet**: Learn more at https://pybullet.org/

---

## Future Enhancements

Planned features for future versions:

- Robot joint control and manipulation
- Inverse kinematics calculations
- Joint state queries and monitoring
- Advanced velocity control
- Dynamic property modification
- Ray casting for sensor simulation
- Camera rendering capabilities

---

## Getting Help

If you encounter issues:
1. Check the server logs in your terminal
2. Review this quickstart guide
3. Read the troubleshooting section
4. Check `README.md` for detailed documentation
5. Verify your MCP client configuration

For bugs or feature requests, please refer to the project repository.
