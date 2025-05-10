# How to Load Mesh Files in MuJoCo

This tutorial will first overview how to load mesh files in MJCF, then in URDF

## MJCF

1. Use the `<mesh>` tag inside `<asset>`

    This declares a mesh asset named link1_mesh, and tells MuJoCo to load it from the file path `"base.stl"` relative to the path in `<compiler meshdir="..."/>`.

2. Reference the mesh from `<geom>` (in `<body>`)

    This uses the asset defined earlier by name (mesh="link1_mesh") in the simulation geometry.

3. Ensure `<compiler meshdir="..."/>` is set

    This makes all file="..." in the `<mesh>` tags resolve relative to that path.

```xml
<mujoco>
    <compiler meshdir="relative/from/mjcf/to/mesh/locations/"/>
    ...
    <asset>
    <mesh name="mesh_name" file="your_mesh_file.stl"/> <!--File is relative to conmpiler meshdir location-->
    </asset>
    ...
    <worldbody>
        ...
        <body name="body_name">
            <geom type="mesh" mesh="mesh_name"/>
        </body>
        ...
    </worldbody>
    ...
</mujoco>
```

## URDF

1. Use the `<mujoco>` tag inside `<robot>`.

    This allows you to specify MuJoCo-specific settings, such as the `meshdir` attribute, which defines the directory for mesh files.

2. Define the `<link>` and `<visual>` tags.

    Use the `<geometry>` tag inside `<visual>` to reference the mesh file. Ensure the `filename` attribute points to the correct file path relative to `meshdir`.

3. Ensure `<compiler meshdir="..."/>` is set.

    This ensures all mesh file paths are resolved relative to the specified directory.

See MuJoCo [URDF extensions](https://mujoco.readthedocs.io/en/stable/modeling.html#urdf-extensions) and [compiler](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjscompiler) tags section for more information.

```xml
<robot>
    <mujoco>
        <compiler meshdir="path/to/mesh/files/"/>
    </mujoco>
    ...
    <link name="link_name">
        <collision>
            <geometry>
                <mesh filename="your_mesh_file.stl"/> <!--MuJoCo Supports .obj and .stl mesh files ONLY-->
            </geometry>
        </collision>
    </link>
    ...
</robot>
```

<!-- 
### Alternatively

Currently Under Experimentation

```xml
<mujoco>
    <compiler meshdir="relative/from/mjcf/to/mesh/locations/" balanceinertia="true" discardvisual="false"/>
    ...
    <robot>
        ...
    </robot>
    ...
</mujoco>    
``` -->