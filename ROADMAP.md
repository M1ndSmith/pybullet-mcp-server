# PyBullet MCP Server - Development Roadmap

Version 1.0.0 - Current Release
Last Updated: 2025-03-02

## Overview

This document outlines planned improvements, bug fixes, and feature additions for future versions of the PyBullet MCP Server.

---

## Priority 1: Critical Fixes (v1.1.0)

### Security Issues

#### 1. File Path Validation
**Status**: Not Implemented  
**Priority**: CRITICAL  
**Estimated Effort**: 1-2 hours

**Problem**: Path traversal vulnerability in `load_urdf()` and `load_simulation()` functions.

**Implementation**:
```python
# In persistence.py and object_manager.py
import os

def _validate_file_path(file_path: str, allowed_base: str) -> str:
    """Validate and normalize file path to prevent path traversal."""
    abs_path = os.path.abspath(file_path)
    allowed_abs = os.path.abspath(allowed_base)
    
    if not abs_path.startswith(allowed_abs):
        raise ValueError(f"File path outside allowed directory: {file_path}")
    
    return abs_path
```

**Files to Modify**:
- `src/persistence.py` - Add validation to `load_simulation()`
- `src/object_manager.py` - Add validation to `load_urdf()`
- Add configuration for allowed directories

**Testing**:
- Test with `../../../etc/passwd` paths
- Test with absolute paths outside allowed directories
- Test with symlinks
- Verify legitimate paths still work

---

#### 2. Resource Limits
**Status**: Not Implemented  
**Priority**: CRITICAL  
**Estimated Effort**: 2-3 hours

**Problem**: No limits on simulations, objects, or constraints - DoS vulnerability.

**Implementation**:
```python
# In src/simulation_manager.py
MAX_SIMULATIONS = 10
MAX_OBJECTS_PER_SIMULATION = 1000
MAX_CONSTRAINTS_PER_SIMULATION = 500

def create_simulation(self, ...):
    if len(self.simulations) >= MAX_SIMULATIONS:
        raise ValueError(f"Maximum simulations ({MAX_SIMULATIONS}) reached")
    # ... rest of code

# In src/simulation_context.py
def add_object(self, ...):
    if len(self.objects) >= MAX_OBJECTS_PER_SIMULATION:
        raise ValueError(f"Maximum objects ({MAX_OBJECTS_PER_SIMULATION}) reached")
    # ... rest of code
```

**Files to Modify**:
- `src/simulation_manager.py` - Add simulation limit
- `src/simulation_context.py` - Add object and constraint limits
- Add configuration file for limits
- Add environment variable overrides

**Testing**:
- Test creating MAX_SIMULATIONS + 1 simulations
- Test adding MAX_OBJECTS + 1 objects
- Test adding MAX_CONSTRAINTS + 1 constraints
- Verify error messages are clear

---

### Functional Bugs

#### 3. Constraint Deserialization
**Status**: Not Implemented (stub exists)  
**Priority**: HIGH  
**Estimated Effort**: 2-3 hours

**Problem**: `_deserialize_constraint()` in `persistence.py` is a placeholder. Constraints are not restored when loading simulations.

**Implementation**:
```python
# In src/persistence.py
def _deserialize_constraint(self, constraint_data: dict, context: SimulationContext):
    """Recreate constraint from saved data."""
    constraint_id = self.constraint_manager.create_constraint(
        sim_id=context.sim_id,
        parent_id=constraint_data["parent_id"],
        child_id=constraint_data["child_id"],
        joint_type=constraint_data["joint_type"],
        joint_axis=constraint_data.get("joint_axis", [0, 0, 1]),
        parent_frame_position=constraint_data.get("parent_frame_position", [0, 0, 0]),
        child_frame_position=constraint_data.get("child_frame_position", [0, 0, 0]),
        parent_frame_orientation=constraint_data.get("parent_frame_orientation"),
        child_frame_orientation=constraint_data.get("child_frame_orientation")
    )
    return constraint_id
```

**Files to Modify**:
- `src/persistence.py` - Implement `_deserialize_constraint()`
- Add constraint ID mapping (saved ID → new ID)

**Testing**:
- Create simulation with constraints
- Save to JSON
- Load from JSON
- Verify constraints are recreated
- Verify constraint behavior is preserved
- Test with all joint types (fixed, prismatic, spherical)

---

## Priority 2: Robot Control Features (v2.0.0)

### Essential for AI-Controlled Robots

#### 4. Joint Control
**Status**: Not Implemented  
**Priority**: BLOCKING for robotics use case  
**Estimated Effort**: 3-4 hours

**Implementation**: Add 4 new MCP tools

**Tool 1: get_num_joints**
```python
@mcp.tool()
def get_num_joints(sim_id: str, object_id: int) -> int:
    """Get number of joints in a URDF model."""
    context = simulation_manager.get_context(sim_id)
    return p.getNumJoints(object_id, physicsClientId=context.client_id)
```

**Tool 2: get_joint_info**
```python
@mcp.tool()
def get_joint_info(sim_id: str, object_id: int, joint_index: int) -> dict:
    """Get joint properties (type, limits, axis, name)."""
    context = simulation_manager.get_context(sim_id)
    info = p.getJointInfo(object_id, joint_index, physicsClientId=context.client_id)
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
```

**Tool 3: get_joint_state**
```python
@mcp.tool()
def get_joint_state(sim_id: str, object_id: int, joint_index: int) -> dict:
    """Get current joint position, velocity, forces."""
    context = simulation_manager.get_context(sim_id)
    state = p.getJointState(object_id, joint_index, physicsClientId=context.client_id)
    return {
        "joint_position": state[0],
        "joint_velocity": state[1],
        "joint_reaction_forces": list(state[2]),
        "applied_motor_torque": state[3]
    }
```

**Tool 4: set_joint_motor_control**
```python
@mcp.tool()
def set_joint_motor_control(
    sim_id: str,
    object_id: int,
    joint_index: int,
    control_mode: str,  # "POSITION_CONTROL", "VELOCITY_CONTROL", "TORQUE_CONTROL"
    target_position: float = None,
    target_velocity: float = None,
    force: float = None,
    position_gain: float = 0.1,
    velocity_gain: float = 1.0
) -> str:
    """Control a robot joint."""
    context = simulation_manager.get_context(sim_id)
    
    mode_map = {
        "POSITION_CONTROL": p.POSITION_CONTROL,
        "VELOCITY_CONTROL": p.VELOCITY_CONTROL,
        "TORQUE_CONTROL": p.TORQUE_CONTROL
    }
    
    p.setJointMotorControl2(
        bodyUniqueId=object_id,
        jointIndex=joint_index,
        controlMode=mode_map[control_mode],
        targetPosition=target_position,
        targetVelocity=target_velocity,
        force=force,
        positionGain=position_gain,
        velocityGain=velocity_gain,
        physicsClientId=context.client_id
    )
    return f"Joint {joint_index} control set"
```

**Files to Modify**:
- `src/server.py` - Add 4 new @mcp.tool functions
- Create `src/joint_manager.py` - New manager class for joint operations
- Update `QUICKSTART.md` - Add joint control examples
- Update `README.md` - Document new tools

**Testing**:
- Load URDF with multiple joints
- Query joint count and properties
- Set joint positions and verify movement
- Test all control modes (position, velocity, torque)
- Test joint limits enforcement

---

#### 5. Inverse Kinematics
**Status**: Not Implemented  
**Priority**: HIGH  
**Estimated Effort**: 2-3 hours

**Implementation**: Add 1 new MCP tool

```python
@mcp.tool()
def calculate_inverse_kinematics(
    sim_id: str,
    object_id: int,
    end_effector_link_index: int,
    target_position: list[float],
    target_orientation: list[float] = None,
    lower_limits: list[float] = None,
    upper_limits: list[float] = None,
    joint_ranges: list[float] = None,
    rest_poses: list[float] = None
) -> list[float]:
    """Calculate joint angles to reach target end-effector pose."""
    context = simulation_manager.get_context(sim_id)
    
    kwargs = {
        "bodyUniqueId": object_id,
        "endEffectorLinkIndex": end_effector_link_index,
        "targetPosition": target_position,
        "physicsClientId": context.client_id
    }
    
    if target_orientation:
        kwargs["targetOrientation"] = target_orientation
    if lower_limits:
        kwargs["lowerLimits"] = lower_limits
    if upper_limits:
        kwargs["upperLimits"] = upper_limits
    if joint_ranges:
        kwargs["jointRanges"] = joint_ranges
    if rest_poses:
        kwargs["restPoses"] = rest_poses
    
    joint_positions = p.calculateInverseKinematics(**kwargs)
    return list(joint_positions)
```

**Files to Modify**:
- `src/server.py` or `src/joint_manager.py` - Add IK tool
- Update documentation

**Testing**:
- Load robot arm URDF
- Calculate IK for reachable positions
- Verify joint angles reach target
- Test with joint limits
- Test unreachable positions (should handle gracefully)

---

## Priority 3: Quality Improvements (v1.2.0)

### Code Quality

#### 6. Consistent Parameter Validation
**Status**: Inconsistent  
**Priority**: MEDIUM  
**Estimated Effort**: 2-3 hours

**Problem**: Some tools accept shorthand arrays `[z]`, others require full `[x, y, z]`.

**Decision Needed**: 
- Option A: Support shorthands everywhere
- Option B: Remove shorthands, require full arrays

**Recommendation**: Option B (remove shorthands for consistency)

**Files to Modify**:
- `src/object_manager.py` - Standardize position/force/torque parameters
- Update all tool docstrings
- Update QUICKSTART.md examples

**Testing**:
- Test all tools with full arrays
- Verify error messages for incorrect array lengths
- Update all example prompts

---

#### 7. Mass=0 Support for Static Objects
**Status**: Rejected (validation too strict)  
**Priority**: LOW  
**Estimated Effort**: 1 hour

**Problem**: `add_box(mass=0)` fails, but PyBullet supports mass=0 for static objects.

**Implementation**:
```python
# In src/object_manager.py
def add_box(self, ..., mass: float = 1.0, is_static: bool = False):
    if is_static:
        mass = 0
    elif mass <= 0:
        raise ValueError(f"Mass must be positive for dynamic objects, got {mass}")
    # ... rest of code
```

**Alternative**: Just allow mass=0 without new parameter.

**Files to Modify**:
- `src/object_manager.py` - Update validation in all add_* methods

**Testing**:
- Create static ground plane with mass=0
- Verify it doesn't move under forces
- Verify dynamic objects collide with it correctly

---

#### 8. Revolute Joint URDF Helper
**Status**: Not Implemented  
**Priority**: LOW  
**Estimated Effort**: 2-3 hours

**Problem**: PyBullet's `createConstraint` doesn't support revolute joints. Users must write URDF XML manually.

**Implementation**: Add helper function to generate simple URDF files

```python
@mcp.tool()
def generate_revolute_joint_urdf(
    parent_shape: str,
    child_shape: str,
    joint_axis: list[float],
    output_path: str,
    parent_dimensions: list[float] = None,
    child_dimensions: list[float] = None
) -> str:
    """Generate a URDF file with a revolute joint between two shapes."""
    urdf_xml = f"""<?xml version="1.0"?>
<robot name="simple_revolute">
  <link name="parent">
    <visual>
      <geometry>
        <{parent_shape} size="{' '.join(map(str, parent_dimensions or [0.5, 0.5, 0.5]))}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <{parent_shape} size="{' '.join(map(str, parent_dimensions or [0.5, 0.5, 0.5]))}"/>
      </geometry>
    </collision>
  </link>
  
  <link name="child">
    <visual>
      <geometry>
        <{child_shape} size="{' '.join(map(str, child_dimensions or [0.5, 0.5, 0.5]))}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <{child_shape} size="{' '.join(map(str, child_dimensions or [0.5, 0.5, 0.5]))}"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="revolute_joint" type="revolute">
    <parent link="parent"/>
    <child link="child"/>
    <axis xyz="{joint_axis[0]} {joint_axis[1]} {joint_axis[2]}"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
</robot>
"""
    
    with open(output_path, 'w') as f:
        f.write(urdf_xml)
    
    return output_path
```

**Files to Modify**:
- `src/server.py` - Add new tool
- Create `src/urdf_generator.py` - URDF generation utilities

**Testing**:
- Generate URDF for box-box revolute joint
- Load generated URDF
- Verify joint works correctly
- Test with different shapes and axes

---

## Priority 4: Advanced Features (v2.1.0+)

### Red Teaming & Testing Capabilities

#### 9. Velocity Control
**Status**: Not Implemented  
**Priority**: MEDIUM  
**Estimated Effort**: 1 hour

**Implementation**: Add 1 new MCP tool

```python
@mcp.tool()
def reset_base_velocity(
    sim_id: str,
    object_id: int,
    linear_velocity: list[float],
    angular_velocity: list[float]
) -> str:
    """Directly set object velocity (for testing edge cases)."""
    context = simulation_manager.get_context(sim_id)
    p.resetBaseVelocity(
        object_id,
        linearVelocity=linear_velocity,
        angularVelocity=angular_velocity,
        physicsClientId=context.client_id
    )
    return f"Velocity set for object {object_id}"
```

**Use Case**: Inject unexpected velocities for adversarial testing.

---

#### 10. Dynamic Property Modification
**Status**: Not Implemented  
**Priority**: MEDIUM  
**Estimated Effort**: 2 hours

**Implementation**: Add 2 new MCP tools

```python
@mcp.tool()
def change_dynamics(
    sim_id: str,
    object_id: int,
    link_index: int = -1,
    mass: float = None,
    lateral_friction: float = None,
    spinning_friction: float = None,
    rolling_friction: float = None,
    restitution: float = None,
    linear_damping: float = None,
    angular_damping: float = None
) -> str:
    """Modify object physics properties at runtime."""
    context = simulation_manager.get_context(sim_id)
    
    kwargs = {"physicsClientId": context.client_id}
    if mass is not None:
        kwargs["mass"] = mass
    if lateral_friction is not None:
        kwargs["lateralFriction"] = lateral_friction
    # ... add other parameters
    
    p.changeDynamics(object_id, link_index, **kwargs)
    return f"Dynamics updated for object {object_id}"

@mcp.tool()
def get_dynamics_info(sim_id: str, object_id: int, link_index: int = -1) -> dict:
    """Query current dynamic properties."""
    context = simulation_manager.get_context(sim_id)
    info = p.getDynamicsInfo(object_id, link_index, physicsClientId=context.client_id)
    return {
        "mass": info[0],
        "lateral_friction": info[1],
        "local_inertia_diagonal": list(info[2]),
        "restitution": info[5],
        "rolling_friction": info[6],
        "spinning_friction": info[7],
        "contact_damping": info[8],
        "contact_stiffness": info[9]
    }
```

**Use Case**: Test failure scenarios (slippery gripper, unexpected mass changes).

---

#### 11. Ray Casting
**Status**: Not Implemented  
**Priority**: MEDIUM  
**Estimated Effort**: 2-3 hours

**Implementation**: Add 2 new MCP tools

```python
@mcp.tool()
def ray_test(
    sim_id: str,
    ray_from_position: list[float],
    ray_to_position: list[float]
) -> dict:
    """Cast a ray and find intersections (for lidar/vision simulation)."""
    context = simulation_manager.get_context(sim_id)
    result = p.rayTest(
        ray_from_position,
        ray_to_position,
        physicsClientId=context.client_id
    )[0]
    
    return {
        "object_id": result[0],
        "link_index": result[1],
        "hit_fraction": result[2],
        "hit_position": list(result[3]),
        "hit_normal": list(result[4])
    }

@mcp.tool()
def ray_test_batch(
    sim_id: str,
    ray_from_positions: list[list[float]],
    ray_to_positions: list[list[float]]
) -> list[dict]:
    """Cast multiple rays efficiently (for lidar simulation)."""
    context = simulation_manager.get_context(sim_id)
    results = p.rayTestBatch(
        ray_from_positions,
        ray_to_positions,
        physicsClientId=context.client_id
    )
    
    return [
        {
            "object_id": r[0],
            "link_index": r[1],
            "hit_fraction": r[2],
            "hit_position": list(r[3]),
            "hit_normal": list(r[4])
        }
        for r in results
    ]
```

**Use Case**: Simulate lidar, sonar, vision sensors for robotics.

---

#### 12. Camera Rendering
**Status**: Not Implemented  
**Priority**: LOW  
**Estimated Effort**: 4-5 hours

**Implementation**: Add 3 new MCP tools

```python
@mcp.tool()
def get_camera_image(
    sim_id: str,
    width: int,
    height: int,
    view_matrix: list[float],
    projection_matrix: list[float]
) -> dict:
    """Render RGB, depth, and segmentation images."""
    context = simulation_manager.get_context(sim_id)
    
    _, _, rgb, depth, seg = p.getCameraImage(
        width, height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        physicsClientId=context.client_id
    )
    
    import base64
    return {
        "rgb": base64.b64encode(rgb).decode('utf-8'),
        "depth": base64.b64encode(depth).decode('utf-8'),
        "segmentation": base64.b64encode(seg).decode('utf-8')
    }

@mcp.tool()
def compute_view_matrix(
    camera_eye_position: list[float],
    camera_target_position: list[float],
    camera_up_vector: list[float]
) -> list[float]:
    """Helper to create view matrix from camera pose."""
    return list(p.computeViewMatrix(
        camera_eye_position,
        camera_target_position,
        camera_up_vector
    ))

@mcp.tool()
def compute_projection_matrix(
    fov: float,
    aspect: float,
    near_plane: float,
    far_plane: float
) -> list[float]:
    """Helper to create projection matrix."""
    return list(p.computeProjectionMatrixFOV(fov, aspect, near_plane, far_plane))
```

**Use Case**: Vision-based control, computer vision testing.

---

## Priority 5: Documentation & Polish

#### 13. Architecture Documentation
**Status**: Partially complete  
**Priority**: LOW  
**Estimated Effort**: 2-3 hours

**Tasks**:
- Create `docs/` folder
- Move architecture diagrams to separate file
- Add sequence diagrams for common workflows
- Document manager class responsibilities
- Add contribution guidelines

---

#### 14. Example Gallery
**Status**: Not Implemented  
**Priority**: LOW  
**Estimated Effort**: 3-4 hours

**Tasks**:
- Create `examples/` folder
- Add Python scripts for common scenarios:
  - Falling stack
  - Pendulum
  - Robot arm manipulation
  - Collision detection demo
  - Save/load workflow
- Add screenshots/videos of examples

---

#### 15. Performance Benchmarks
**Status**: Not Implemented  
**Priority**: LOW  
**Estimated Effort**: 2-3 hours

**Tasks**:
- Create `benchmarks/` folder
- Benchmark simulation step performance
- Benchmark object creation performance
- Benchmark collision detection performance
- Document performance characteristics
- Add optimization recommendations

---

## Summary by Version

### v1.1.0 (Critical Fixes)
- File path validation (security)
- Resource limits (security)
- Constraint deserialization (bug fix)

**Estimated Total Effort**: 5-8 hours

---

### v2.0.0 (Robot Control)
- Joint control (4 tools)
- Inverse kinematics (1 tool)

**Estimated Total Effort**: 5-7 hours

---

### v1.2.0 (Quality)
- Consistent parameter validation
- Mass=0 support
- Revolute joint URDF helper

**Estimated Total Effort**: 5-7 hours

---

### v2.1.0+ (Advanced Features)
- Velocity control
- Dynamic property modification
- Ray casting
- Camera rendering

**Estimated Total Effort**: 9-14 hours

---

## Implementation Order Recommendation

1. **Week 1**: v1.1.0 critical fixes (security + bug fix)
2. **Week 2**: v2.0.0 robot control (joint control + IK)
3. **Week 3**: v1.2.0 quality improvements
4. **Week 4+**: v2.1.0 advanced features (as needed)

---

## Notes

- All estimates assume familiarity with codebase
- Testing time included in estimates
- Documentation updates included in estimates
- Breaking changes should be avoided when possible
- Maintain backward compatibility for v1.x releases
- v2.0.0 can introduce breaking changes if needed

---

## Tracking

Use GitHub Issues to track implementation:
- Label issues with version milestone (v1.1.0, v2.0.0, etc.)
- Label issues with priority (critical, high, medium, low)
- Label issues with type (security, bug, feature, quality)
- Link issues to this roadmap document

---

Last Updated: 2025-03-02  
Maintainer: M1ndSmith
