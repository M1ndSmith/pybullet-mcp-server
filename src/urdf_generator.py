"""URDF generator for creating revolute joints between objects."""

from typing import List, Optional, Tuple
import os
import tempfile


def calculate_box_inertia(mass: float, dimensions: List[float]) -> Tuple[float, float, float]:
    """Calculate inertia tensor for a box.
    
    Args:
        mass: Mass in kg
        dimensions: [half_x, half_y, half_z] in meters
        
    Returns:
        Tuple of (ixx, iyy, izz) inertia values
        
    Formula: For a box with full dimensions (w, h, d):
        ixx = (1/12) * m * (h² + d²)
        iyy = (1/12) * m * (w² + d²)
        izz = (1/12) * m * (w² + h²)
    """
    # Convert half extents to full dimensions
    w = dimensions[0] * 2  # width (x)
    h = dimensions[1] * 2  # height (y)
    d = dimensions[2] * 2  # depth (z)
    
    ixx = (1.0 / 12.0) * mass * (h * h + d * d)
    iyy = (1.0 / 12.0) * mass * (w * w + d * d)
    izz = (1.0 / 12.0) * mass * (w * w + h * h)
    
    return (ixx, iyy, izz)


def calculate_sphere_inertia(mass: float, radius: float) -> Tuple[float, float, float]:
    """Calculate inertia tensor for a sphere.
    
    Args:
        mass: Mass in kg
        radius: Radius in meters
        
    Returns:
        Tuple of (ixx, iyy, izz) inertia values (all equal for sphere)
        
    Formula: I = (2/5) * m * r²
    """
    inertia = (2.0 / 5.0) * mass * radius * radius
    return (inertia, inertia, inertia)


def calculate_cylinder_inertia(mass: float, radius: float, height: float) -> Tuple[float, float, float]:
    """Calculate inertia tensor for a cylinder (axis along z).
    
    Args:
        mass: Mass in kg
        radius: Radius in meters
        height: Height in meters
        
    Returns:
        Tuple of (ixx, iyy, izz) inertia values
        
    Formula:
        ixx = iyy = (1/12) * m * (3r² + h²)
        izz = (1/2) * m * r²
    """
    ixx = (1.0 / 12.0) * mass * (3 * radius * radius + height * height)
    iyy = ixx
    izz = 0.5 * mass * radius * radius
    
    return (ixx, iyy, izz)


def generate_revolute_joint_urdf(
    parent_shape: str,
    child_shape: str,
    parent_dimensions: List[float],
    child_dimensions: List[float],
    parent_mass: float,
    child_mass: float,
    joint_axis: List[float],
    joint_origin: Optional[List[float]] = None,
    joint_limits: Optional[Tuple[float, float]] = None,
    max_effort: float = 100.0,
    max_velocity: float = 10.0,
    output_path: Optional[str] = None
) -> str:
    """Generate a URDF file with a revolute joint between two shapes.
    
    Args:
        parent_shape: Shape type - "box", "sphere", or "cylinder"
        child_shape: Shape type - "box", "sphere", or "cylinder"
        parent_dimensions: Dimensions for parent shape
        child_dimensions: Dimensions for child shape
        parent_mass: Mass of parent link in kg
        child_mass: Mass of child link in kg
        joint_axis: Axis of rotation [x, y, z]
        joint_origin: Joint position relative to parent [x, y, z]. Default [0, 0, 0]
        joint_limits: (lower, upper) limits in radians. Default (-3.14, 3.14)
        max_effort: Maximum joint effort in N·m. Default 100.0
        max_velocity: Maximum joint velocity in rad/s. Default 10.0
        output_path: Path to save URDF file. If None, creates temp file.
        
    Returns:
        Path to generated URDF file
        
    Raises:
        ValueError: If invalid shape type or dimensions
    """
    # Validate shapes
    valid_shapes = ["box", "sphere", "cylinder"]
    if parent_shape not in valid_shapes:
        raise ValueError(f"Invalid parent_shape: {parent_shape}. Must be one of {valid_shapes}")
    if child_shape not in valid_shapes:
        raise ValueError(f"Invalid child_shape: {child_shape}. Must be one of {valid_shapes}")
    
    # Set defaults
    if joint_origin is None:
        joint_origin = [0.0, 0.0, 0.0]
    if joint_limits is None:
        joint_limits = (-3.14159, 3.14159)
    
    # Calculate inertias
    if parent_shape == "box":
        parent_inertia = calculate_box_inertia(parent_mass, parent_dimensions)
    elif parent_shape == "sphere":
        parent_inertia = calculate_sphere_inertia(parent_mass, parent_dimensions[0])
    elif parent_shape == "cylinder":
        parent_inertia = calculate_cylinder_inertia(parent_mass, parent_dimensions[0], parent_dimensions[1])
    
    if child_shape == "box":
        child_inertia = calculate_box_inertia(child_mass, child_dimensions)
    elif child_shape == "sphere":
        child_inertia = calculate_sphere_inertia(child_mass, child_dimensions[0])
    elif child_shape == "cylinder":
        child_inertia = calculate_cylinder_inertia(child_mass, child_dimensions[0], child_dimensions[1])
    
    # Generate geometry XML
    def get_geometry_xml(shape: str, dimensions: List[float]) -> str:
        if shape == "box":
            # Box uses full size, not half extents
            size = [d * 2 for d in dimensions]
            return f'<box size="{size[0]} {size[1]} {size[2]}"/>'
        elif shape == "sphere":
            return f'<sphere radius="{dimensions[0]}"/>'
        elif shape == "cylinder":
            return f'<cylinder radius="{dimensions[0]}" length="{dimensions[1]}"/>'
    
    parent_geom = get_geometry_xml(parent_shape, parent_dimensions)
    child_geom = get_geometry_xml(child_shape, child_dimensions)
    
    # Generate URDF XML
    urdf_content = f'''<?xml version="1.0"?>
<robot name="revolute_joint_model">
  <!-- Parent Link -->
  <link name="parent_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{parent_mass}"/>
      <inertia ixx="{parent_inertia[0]}" ixy="0.0" ixz="0.0" 
               iyy="{parent_inertia[1]}" iyz="0.0" izz="{parent_inertia[2]}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        {parent_geom}
      </geometry>
      <material name="parent_color">
        <color rgba="0.8 0.8 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        {parent_geom}
      </geometry>
    </collision>
  </link>

  <!-- Revolute Joint -->
  <joint name="revolute_joint" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="{joint_origin[0]} {joint_origin[1]} {joint_origin[2]}" rpy="0 0 0"/>
    <axis xyz="{joint_axis[0]} {joint_axis[1]} {joint_axis[2]}"/>
    <limit lower="{joint_limits[0]}" upper="{joint_limits[1]}" 
           effort="{max_effort}" velocity="{max_velocity}"/>
    <dynamics damping="0.1" friction="0.1"/>
  </joint>

  <!-- Child Link -->
  <link name="child_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{child_mass}"/>
      <inertia ixx="{child_inertia[0]}" ixy="0.0" ixz="0.0" 
               iyy="{child_inertia[1]}" iyz="0.0" izz="{child_inertia[2]}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        {child_geom}
      </geometry>
      <material name="child_color">
        <color rgba="0.2 0.6 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        {child_geom}
      </geometry>
    </collision>
  </link>
</robot>
'''
    
    # Write to file
    if output_path is None:
        # Create temporary file
        fd, output_path = tempfile.mkstemp(suffix='.urdf', prefix='revolute_joint_')
        os.close(fd)
    
    with open(output_path, 'w') as f:
        f.write(urdf_content)
    
    return output_path
