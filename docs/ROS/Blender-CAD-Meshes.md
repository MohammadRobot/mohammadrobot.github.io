# Blender Workflow for Cleaning CAD Meshes for ROS 2 / Gazebo

This guide explains how to fix common mesh problems when importing CAD models, for example from Onshape, into Blender before using them in ROS 2 and Gazebo.

Typical problems this workflow fixes:

- Missing faces in Gazebo
- Flipped normals
- Extremely large mesh sizes
- Broken CAD mesh geometry

The goal is to produce a clean **visual mesh** (`.dae` or `.obj`) and a simple **collision mesh** (`.stl`) for use in URDF.

## 1. Open the mesh in Blender

1. Start Blender.
2. Go to `File -> Import`.
3. Import your mesh file:

- Collada (`.dae`)
- STL (`.stl`)
- OBJ (`.obj`)

If importing from CAD:

- OBJ is usually the most stable format.

## 2. Select the mesh object

Look at the **Outliner** in the top-right panel.

Select the object with the **triangle icon**. That means it is a mesh.

Example:

```text
Collection
 └ robot_part
```

## 3. Enter Edit Mode

With the mesh selected, press:

```text
Tab
```

Or use:

```text
Object Mode -> Edit Mode
```

Now you should see vertices, edges, and faces.

## 4. Select all geometry

Press:

```text
A
```

Everything should now be selected.

## 5. Remove duplicate vertices

CAD exports often contain duplicated vertices.

Press:

```text
M
```

Then choose:

```text
By Distance
```

Blender will merge overlapping vertices.

Example message:

```text
Removed 12000 vertices
```

## 6. Fix face normals

Incorrect normals cause missing faces in Gazebo.

Press:

```text
Shift + N
```

This recalculates all normals to point outward.

## 7. Check face orientation

Enable the overlay:

```text
Viewport Overlays -> Face Orientation
```

Colors mean:

- Blue -> Correct orientation
- Red -> Flipped faces

Goal:

```text
Outer surfaces should be BLUE
```

## 8. Flip incorrect faces if necessary

If some faces remain red:

1. Select those faces.
2. Press:

```text
Alt + N
```

3. Choose:

```text
Flip
```

## 9. Triangulate the mesh

Gazebo works best with triangular faces.

Press:

```text
A
Ctrl + T
```

Now all faces are triangles.

## 10. Reduce mesh size

CAD models often contain millions of polygons.

1. Go to the **Modifiers** panel.
2. Click:

```text
Add Modifier -> Decimate
```

Recommended settings:

```text
Ratio: 0.2 - 0.4
```

Apply the modifier.

Example result:

```text
1,200,000 faces -> 40,000 faces
```

## 11. Apply object transforms

Return to Object Mode:

```text
Tab
```

Then press:

```text
Ctrl + A
```

Choose:

```text
Rotation & Scale
```

This prevents scaling issues in Gazebo.

## 12. Export the visual mesh

Export the cleaned mesh:

```text
File -> Export -> Collada (.dae)
```

Recommended options:

- Apply Modifiers
- Selected Objects
- Triangulate

Example file:

```text
robot_visual.dae
```

## 13. Create a collision mesh

Duplicate the object:

```text
Shift + D
```

Rename it:

```text
robot_collision
```

Apply heavy simplification:

```text
Decimate Ratio = 0.05
```

Export as:

```text
File -> Export -> STL
```

Example file:

```text
robot_collision.stl
```

## 14. Use in URDF

Example URDF link:

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

## Recommended mesh sizes

| Mesh Type | Recommended Faces |
| --- | --- |
| Visual | `< 50k` |
| Collision | `< 5k` |

## Best practices for robot simulation

Avoid including small CAD details.

Remove:

- Screws
- Threads
- Fillets
- Decorative logos

These increase simulation cost without improving physics.

## Typical robotics pipeline

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

## Summary

This workflow helps robot meshes:

- Render correctly in Gazebo
- Load faster
- Simulate more efficiently
- Avoid flipped normals and missing faces
