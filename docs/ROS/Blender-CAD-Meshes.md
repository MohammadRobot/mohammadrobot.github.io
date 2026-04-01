# Blender Workflow for Cleaning CAD Meshes for ROS 2 / Gazebo

Use this workflow to convert raw CAD exports into clean simulation meshes that render correctly and run faster.

By the end, you should have:

- one clean visual mesh (`.dae` or `.obj`)
- one simplified collision mesh (`.stl`)
- a URDF-ready mesh setup with fewer simulation issues

## Quick Start

1. Import mesh into Blender.
2. Clean geometry (`By Distance`, normals, triangulation).
3. Decimate for visual mesh and apply transforms.
4. Create a heavily simplified collision copy.
5. Export visual and collision meshes, then reference both in URDF.

Expected result:

- no missing faces in Gazebo
- consistent scale and orientation
- faster simulation and stable collisions

!!! tip
    Keep your visual and collision meshes separate. This gives better performance and more stable physics than reusing one heavy mesh for both.

## Prerequisites

| Item | Requirement |
| --- | --- |
| Source model | CAD export (`.obj`, `.stl`, or `.dae`) |
| Blender | Any recent version with Decimate modifier |
| Target stack | ROS 2 + URDF + Gazebo |
| Goal | visual quality and simulation stability |

## Workflow Overview

Typical pipeline:

```text
CAD (Onshape / SolidWorks)
        v
Export STEP / OBJ
        v
Blender cleanup
        v
Visual mesh (.dae / .obj)
Collision mesh (.stl)
        v
URDF
        v
ROS 2 + Gazebo
```

## Step-by-Step

### 1) Import and identify the mesh

1. Start Blender.
2. Go to `File -> Import`.
3. Import your mesh file (`.dae`, `.stl`, `.obj`).
4. In the Outliner, select the object with the triangle icon.

Example Outliner:

```text
Collection
 └ robot_part
```

Expected result:

- correct mesh object is selected
- object is visible in the viewport

### 2) Enter edit mode and select all geometry

With the mesh selected:

```text
Tab
A
```

Or use:

```text
Object Mode -> Edit Mode
```

Expected result:

- all vertices/edges/faces are selected
- you can edit the full mesh at once

### 3) Remove duplicate vertices

CAD exports often contain overlapping vertices.

Use:

```text
M -> By Distance
```

Example message:

```text
Removed 12000 vertices
```

Expected result:

- cleaner topology
- fewer non-manifold artifacts later

### 4) Fix normals and orientation

Recalculate normals:

```text
Shift + N
```

Enable face orientation overlay:

```text
Viewport Overlays -> Face Orientation
```

Interpret colors:

- Blue = correct outward face
- Red = flipped face

If needed, flip selected incorrect faces:

```text
Alt + N -> Flip
```

Expected result:

- outer surfaces are mostly blue
- no random face disappearance in Gazebo

!!! warning
    Do not skip normal checks. Flipped normals are a common cause of missing surfaces in simulation.

### 5) Triangulate and reduce polygon count

Triangulate:

```text
A
Ctrl + T
```

Apply decimation for the visual mesh:

```text
Add Modifier -> Decimate
Ratio: 0.2 - 0.4
```

Example:

```text
1,200,000 faces -> 40,000 faces
```

Expected result:

- all faces are triangles
- visual mesh is lighter but still acceptable

### 6) Apply object transforms

Return to object mode and apply transforms:

```text
Tab
Ctrl + A -> Rotation & Scale
```

Expected result:

- scale and orientation are normalized
- URDF and Gazebo import behavior is consistent

### 7) Export the visual mesh

Export:

```text
File -> Export -> Collada (.dae)
```

Recommended export options:

- Apply Modifiers
- Selected Objects
- Triangulate

Example output:

```text
robot_visual.dae
```

### 8) Create and export the collision mesh

Duplicate the object:

```text
Shift + D
```

Rename:

```text
robot_collision
```

Apply heavier simplification:

```text
Decimate Ratio = 0.05
```

Export:

```text
File -> Export -> STL
```

Example output:

```text
robot_collision.stl
```

Expected result:

- collision mesh is much lower poly than visual mesh
- collision checks run faster in simulation

## URDF Integration

Use separate visual and collision references:

```xml
<link name="base_link">

  <visual>
    <geometry>
      <mesh filename="package://robot_description/meshes/robot_visual.dae"/>
    </geometry>
  </visual>

  <collision>
    <geometry>
      <mesh filename="package://robot_description/meshes/robot_collision.stl"/>
    </geometry>
  </collision>

</link>
```

## Mesh Budgets

| Mesh Type | Recommended Faces |
| --- | --- |
| Visual | `< 50k` |
| Collision | `< 5k` |

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| Missing faces in Gazebo | flipped normals | recalculate normals and check face orientation |
| Wrong robot size in simulation | unapplied scale | apply `Rotation & Scale` before export |
| Slow simulation | collision mesh too detailed | lower collision decimate ratio |
| Unstable collisions | tiny CAD details in collision mesh | remove screws/threads/fillets from collision mesh |
| Mesh looks broken after export | duplicate vertices / bad topology | rerun `By Distance`, then triangulate |

## Best Practices

- Remove decorative CAD details that do not affect physics.
- Keep collision geometry simple and convex where possible.
- Keep mesh names and file names explicit (`*_visual`, `*_collision`).
- Test meshes in RViz and Gazebo before integrating into a full stack.

## Next Steps

- validate TF and inertia after mesh import in URDF
- test with your robot controllers in Gazebo
- tune collision quality only as much as needed for reliable contact

This workflow helps meshes render correctly, load faster, and simulate more reliably in ROS 2 projects.
