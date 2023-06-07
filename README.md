# InfinyToolkit

## Description
This repository gather all open code from InfinyTech3D that could be of any use with the https://www.sofa-framework.org/
Either as components or script. This repository should be sued as a external SOFA plugin.


### Features
Several components are still work in progress or just benchmarks. Here are the components that can be highlighted:
- **MiddleForceField:** basically compute a force field to a barycentric coordinate. Can be useful to fake a simple breathing or beating motion.
- **RotationEngine:** allows to apply a succession of rotation to an object.
- **Triangle2RefinedTriangleTopologicalMapping:** define a topological mapping between a coarse triangulation to a refined triangulation.
- **NearestTexcoordsMap:** define a mapping for texture coordinates from a surface mesh (obj) to the surface of a volume mesh.
- **AdvancedCarvingManager:** another version of the SOFA carving manager with several options using performer classes:
    - SimpleCarvingPerformer: similar to SOFA carving manager
    - SurfaceCarvingPerformer: will push the surface without removing element to fake a progressive carving
    - BurningPerformer: will change the texture coordinates of the mesh before carving
    - RefineCarvingPerformer: will refine the volume mesh before carving (using [MeshRefinement plugin](https://github.com/InfinyTech3D/MeshRefinement))
    - CuttingPerformer: will define a fine cut or incision in the mesh (using [MeshRefinement plugin](https://github.com/InfinyTech3D/MeshRefinement))

    
### Architecture
- **examples:** with several examples of the components and some benchmarks.
- **scripts:** a bunch of python scripts to apply changes to the SOFA code base or scenes. Such as changing requiredPlugin, updating components names, headers inclusion, etc.
- **src/InfinyToolkit:** All the source code of the SOFA components.


## Installation
This plugin should be added as an external plugin of SOFA using the CMAKE_EXTERNAL_DIRECTORIES CMake variable of SOFA. 
See SOFA documentation for more information

## License
This work is dual-licensed under either [GPL](https://github.com/InfinyTech3D/Tearing/blob/main/LICENSE.md) or Commercial License. 
For commercial license request, please contact us by email at contact@infinytech3d.com