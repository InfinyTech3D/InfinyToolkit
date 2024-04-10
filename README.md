# InfinyToolkit SOFA plugin

[![Documentation](https://img.shields.io/badge/info-on_website-green.svg)](https://infinytech3d.com/)
[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions)
[![Discord](https://img.shields.io/badge/chat-on_Discord-darkred.svg)](https://discord.gg/G63t3a8Ra6)
[![Support us](https://img.shields.io/badge/support_us-on_Github_Sponsor-purple.svg)](https://github.com/sponsors/InfinyTech3D)

## Description
This repository gather all open code from InfinyTech3D that could be of any use for the https://www.sofa-framework.org/
Either as components or script. This repository should be sued as a external SOFA plugin.

### Features
Several components are still work in progress or just benchmarks. Here are the components that can be highlighted:

- **MiddleForceField:** basically compute a force field to a barycentric coordinate. Can be useful to fake a simple breathing or beating motions. 
<img align="center" width="50%" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/MiddleForceField.gif">

- **RotationEngine:** allows to apply a succession of rotation to an object.
- **Triangle2RefinedTriangleTopologicalMapping:** define a topological mapping between a coarse triangulation to a refined triangulation.

|<img align="center" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/Triangle2RefinedTriangleTopologicalMapping_coarse_mesh.jpg">|<img align="center" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/Triangle2RefinedTriangleTopologicalMapping_refine_mesh.jpg">|
|--|--|
| Triangle coarse mesh | Mapped triangle refined mesh |

- **NearestTexcoordsMap:** define a mapping for texture coordinates from a surface mesh (obj) to the surface of a volume mesh.

|<img align="center" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/NearestTexcoordsMap_textures.jpg">|<img align="center" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/NearestTexcoordsMap_wireframe.jpg">|<img align="center" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/NearestTexcoordsMap_mapping.jpg">|
|--|--|--|
| NearestTexcoordsMap textures| NearestTexcoordsMap wireframe | textures coordinates mapping|

- **AdvancedCarvingManager:** another version of the SOFA carving manager with several options using performer classes:
    - SimpleCarvingPerformer: similar to SOFA carving manager
    - SurfaceCarvingPerformer: will push the surface without removing element to fake a progressive carving
    - BurningPerformer: will change the texture coordinates of the mesh before carving
    - RefineCarvingPerformer: will refine the volume mesh before carving (using [MeshRefinement plugin](https://github.com/InfinyTech3D/MeshRefinement))
    - CuttingPerformer: will define a fine cut or incision in the mesh (using [MeshRefinement plugin](https://github.com/InfinyTech3D/MeshRefinement))

<img align="center" width="60%" height="auto" src="https://github.com/InfinyTech3D/InfinyToolkit/blob/main/doc/MeshRefinement_AdvancedCarving_penetration.gif">
    
### Architecture
- **examples:** with several examples of the components and some benchmarks.
- **scripts:** a bunch of python scripts to apply changes to the SOFA code base or scenes. Such as:
	- Changing requiredPlugin in scenes
	- Updating components names
	- Update headers inclusion
	- ...
- **src/InfinyToolkit:** All the source code of the SOFA components.


## Installation
This plugin should be added as an external plugin of SOFA using the CMAKE_EXTERNAL_DIRECTORIES CMake variable of SOFA. 
See SOFA documentation for more information

## License
This work is dual-licensed under either [GPL](https://github.com/InfinyTech3D/Tearing/blob/main/LICENSE.md) or Commercial License. 
For commercial license request, please contact us by email at contact@infinytech3d.com
