# InfinyToolkit.MeshRefinement

Cut and refine a tetrahedral mesh during a SOFA simulation, from a scene file, from Python or from
C++.

This sub-plugin provides a single component, **`TetrahedronSubdivisionController`**. It is the
public face of the [MeshRefinement](https://github.com/InfinyTech3D/MeshRefinement) plugin, which
does the actual work: identifying the tetrahedra concerned, splitting them with the right
subdivision pattern, and rebuilding their neighbours so the mesh stays conforming.

| | |
|--|--|
| **Requires** | The `MeshRefinement` plugin, and `Sofa.Component.Topology.Container.Dynamic` |
| **Provides** | `TetrahedronSubdivisionController` |
| **Templates** | `Vec3d` (default). The Cuda types are prepared but not enabled yet |
| **Examples** | [`/examples/MeshRefinement`](../../examples/MeshRefinement) |
| **Tests** | [`tests/`](tests) |

## Contents

- [Installation](#installation)
- [Quick start](#quick-start)
- [What the controller can do](#what-the-controller-can-do)
  - [Cut along a plane](#1-cut-along-a-plane)
  - [Cut between two triangles](#2-cut-between-two-triangles)
  - [Refine a set of tetrahedra](#3-refine-a-set-of-tetrahedra)
  - [Refine the whole mesh](#4-refine-the-whole-mesh)
- [Two-phase cutting](#two-phase-cutting)
- [Creating the cut surface](#creating-the-cut-surface)
- [Data reference](#data-reference)
- [Keyboard shortcuts](#keyboard-shortcuts)
- [Driving it from Python](#driving-it-from-python)
- [Driving it from C++](#driving-it-from-c)
- [Example scenes](#example-scenes)
- [Troubleshooting](#troubleshooting)

## Installation

Both repositories are added as external SOFA plugins, through
`SOFA_EXTERNAL_DIRECTORIES`. `InfinyToolkit.MeshRefinement` is configured automatically as a
sub-plugin of `InfinyToolkit` once `MeshRefinement` has been found - there is no separate option to
turn on, but `MeshRefinement` has to be configured first.

```
cmake -S <sofa-src> -B <sofa-build> \
      -DSOFA_EXTERNAL_DIRECTORIES=<dir containing MeshRefinement and InfinyToolkit> \
      -DPLUGIN_MESHREFINEMENT=ON -DPLUGIN_INFINYTOOLKIT=ON
cmake --build <sofa-build> --config Release --target InfinyToolkit.MeshRefinement
```

In a scene, one `RequiredPlugin` is enough - `MeshRefinement` is pulled in as a dependency:

```xml
<RequiredPlugin pluginName="InfinyToolkit.MeshRefinement"/>
```

## Quick start

The controller sits in the node holding the tetrahedral topology, beside the container, the
modifier and the geometry algorithms. It finds them by itself; there is nothing to link.

```xml
<Node name="Beam">
    <EulerImplicitSolver />
    <CGLinearSolver iterations="25" tolerance="1e-5" threshold="1e-5"/>

    <MechanicalObject src="@../grid" name="Volume" />
    <TetrahedronSetTopologyContainer name="Tetra_topo" src="@../TetraGenerator/Container"/>
    <TetrahedronSetTopologyModifier name="Modifier" />
    <TetrahedronSetGeometryAlgorithms name="GeomAlgo" template="Vec3d" />

    <TetrahedronSubdivisionController name="Cutter"
        cutPointA="-12 0 -12" cutPointB="12 6 -12" cutDir="0 0 1" cutDepth="20"
        drawDebugCut="1"/>

    <DiagonalMass massDensity="2.0" />
    <FastTetrahedralCorotationalForceField youngModulus="1000" poissonRatio="0.3" />
</Node>
```

Run it, press `1` to build the cut and `2` to apply it.

The node **must** contain, next to the controller:

| Component | Why |
|--|--|
| `TetrahedronSetTopologyContainer` | The mesh being modified |
| `TetrahedronSetTopologyModifier` | Applies the topological changes |
| `TetrahedronSetGeometryAlgorithms` | Geometric queries |
| A `MechanicalObject` of the matching template | The positions |

Each missing one is reported by name at init, and the controller then does nothing rather than
misbehaving.

## What the controller can do

Four operations. Each has a key, a C++/Python method, or both.

### 1. Cut along a plane

The cut region is the quad spanned by `cutPointA` and `cutPointB`, extruded along `cutDir` over
`cutDepth`. Every tetrahedron the quad crosses is split along it, and the points on the cut are
duplicated so the two sides come apart.

```xml
<TetrahedronSubdivisionController name="Cutter"
    cutPointA="-12 0 -12" cutPointB="12 6 -12"
    cutDir="0 0 1" cutDepth="20"
    drawDebugCut="1"/>
```

Keys `1` then `2`, or `performCut="1"` to run it at the end of the next time step, or:

```cpp
bool cutFromPlane(const Vec3& pointA, const Vec3& pointB, const Vec3& direction,
                  SReal depth, bool createSurface = false);
```

Make the quad overrun the mesh on the sides where the cut should go all the way through - a quad
that stops inside the mesh produces an incision that stops there, which is often what you want for
a partial cut.

> `cutDepth` does double duty: it is the extrusion length **and** it sets the snapping tolerance
> (`cutDepth * 0.25`). Points of the mesh within that distance of the plane are pulled onto the cut
> instead of a new point being created for them. A very large depth on a small mesh therefore snaps
> more aggressively than you might expect.

### 2. Cut between two triangles

Convenience over the plane version for a caller that has picked triangles rather than computed a
plane - a collision or a mouse pick, typically. The quad runs from the barycentre of `triangleA` to
the barycentre of `triangleB`, extruded *into* the mesh over `depth`, along the normal of
`triangleA`.

```cpp
bool cutFromTriangles(TriangleID triangleA, TriangleID triangleB,
                      SReal depth, bool createSurface = false);
```

There is no key and no Data for this one; it is an API-only entry point. The ids are indices into
the **tetrahedral** topology's triangle array (the same container the controller operates on), not
into a separately mapped surface topology. Out-of-range ids are reported and nothing is cut.

### 3. Refine a set of tetrahedra

Each listed tetrahedron has all six of its edges split, so it becomes eight. Its neighbours are
then rebuilt with the pattern matching however many of *their* edges got split, which is what keeps
the mesh conforming.

```xml
<TetrahedronSubdivisionController name="Refiner"
    testID="40 41 42" refineCriteria="0"/>
```

Key `3`, or:

```cpp
bool refineTetrahedra(const std::set<unsigned int>& ids, SReal criteria = 0.0);
```

`refineCriteria` stops the refinement running away when it is applied repeatedly: it is a fraction
of the mesh's **mean tetrahedron volume**, measured once at init. A tetrahedron whose volume is not
above `refineCriteria x` that mean is left alone. `0`, the default, refines regardless of size.

Ids past the end of the topology are reported and nothing is refined.

### 4. Refine the whole mesh

Every tetrahedron becomes eight. One midpoint is created per edge and shared by all the tetrahedra
using it.

Key `4`, or:

```cpp
bool refineFullMesh();
```

Each call multiplies the tetrahedron count by eight - 5 000 tetrahedra become 40 000, then 320 000.
Start small.

## Two-phase cutting

Building the cut path and applying it are separate steps, so a scene can draw the path and inspect
it before the topology changes:

```cpp
bool prepareCutFromPlane(const Vec3& pointA, const Vec3& pointB, const Vec3& direction, SReal depth);
bool prepareCutFromTriangles(TriangleID triangleA, TriangleID triangleB, SReal depth);
bool applyCut(bool createSurface);
```

`prepareCutFrom*()` computes the intersections and leaves the topology untouched - with
`drawDebugCut="1"` they are drawn immediately. `applyCut()` commits them. `cutFromPlane()` and
`cutFromTriangles()` are exactly the two calls chained, for when the inspection step is not needed.

That is what the `1` / `2` keys are: `1` prepares from the Data, `2` applies.

`delayMode="1"` gives refinement the same treatment on key `3`: the first press computes the
neighbourhood table, the second subdivides from it.

## Creating the cut surface

With `surfaceCut="1"`, applying a cut also builds a renderable surface on each side of the cut and
maps it to the deforming mesh, so the inside of the cut is visible rather than showing through.
`textureName` is applied to both.

This needs a specific scene structure. The tetrahedral node must contain a node holding a
`TriangleSetTopologyContainer` (mapped from the tetrahedra), and **that** node must itself have a
child node - the two cut surfaces are created underneath it:

```xml
<Node name="Cube">
    ...
    <TetrahedronSubdivisionController name="Cutter"
        cutPointA="-6 0 -6" cutPointB="6 3 -6" cutDir="0 0 1" cutDepth="12"
        surfaceCut="1" textureName="textures/colorMap.png"/>

    <Node name="Surface">
        <TriangleSetTopologyContainer name="ContainerTri" />
        <TriangleSetTopologyModifier name="Modifier" />
        <TriangleSetGeometryAlgorithms name="GeomAlgo" />
        <Tetra2TriangleTopologicalMapping input="@../Tetra_topo" output="@ContainerTri" />

        <!-- required: the cut surfaces are created as children of this node -->
        <Node name="Visu">
            <OglModel name="Visual" />
            <IdentityMapping input="@../../Volume" output="@Visual" />
        </Node>
    </Node>
</Node>
```

If either node is missing the surface creation reports an error and is skipped. **The cut itself
still happens** - only the rendered surface is lost.

> `textureName` is read at `init()`, and only when `surfaceCut` is already true. Turning
> `surfaceCut` on later from the GUI cuts and builds the surfaces, but leaves them untextured.

See [`/InfinyToolkit/examples/MeshRefinement/SubdivisionController_CutWithSurface.scn`](../../examples/MeshRefinement/SubdivisionController_CutWithSurface.scn).

## Data reference

| Data | Type | Default | Description |
|--|--|--|--|
| `performCut` | `bool` | `false` | Performs the cut described by the Data below at the end of the next time step, then resets itself to `false` |
| `cutPointA` | `Vec3` | `0 0 0` | First point of the cut quad |
| `cutPointB` | `Vec3` | `0 0 0` | Second point of the cut quad |
| `cutDir` | `Vec3` | `0 -1 0` | Direction the quad is extruded along |
| `cutDepth` | `SReal` | `0` | Extrusion length. Also sets the snapping tolerance, at a quarter of this value |
| `surfaceCut` | `bool` | `false` | Build and render a surface on each side of the cut. [Scene requirements above](#creating-the-cut-surface) |
| `textureName` | `string` | `""` | Texture applied to those surfaces. Only read when `surfaceCut` is true at init |
| `testID` | `set<uint>` | empty | Tetrahedra refined by the `3` key |
| `refineCriteria` | `SReal` | `0` | Fraction of the mesh's mean tetrahedron volume under which a tetrahedron is left alone. `0` refines regardless |
| `delayMode` | `bool` | `false` | Split the `3` key in two presses: compute the neighbourhood table, then subdivide from it |
| `drawTetra` | `bool` | `false` | Draw the tetrahedra currently held by the subdividers |
| `drawScaleTetrahedra` | `float` | `1.0` | Scale of those drawn tetrahedra. Below `1.0` it leaves gaps between them, which makes the subdivision readable |
| `drawDebugCut` | `bool` | `false` | Draw the cut quad and the intersections it computed |

Plus the standard `name`, `printLog` and `listening`. `printLog="1"` also turns on the engine's own
logging, which reports the point and tetrahedron counts before and after each operation - the
quickest way to tell whether an operation did anything.

`listening` is already enabled by the component itself, so the keys work without setting it.

The two `draw*` Data need a GUI: nothing is drawn under `runSofa -g batch`.

## Keyboard shortcuts

| Key | Action |
|--|--|
| `1` | Build the cut path from `cutPointA` / `cutPointB` / `cutDir` / `cutDepth`. The topology is not changed |
| `2` | Apply the cut built by `1`, honouring `surfaceCut` |
| `3` | Refine the tetrahedra listed in `testID`. With `delayMode`, two presses |
| `4` | Refine the whole mesh |

Pressing `2` with no cut prepared does nothing.

## Driving it from Python

```python
controller = node.addObject('TetrahedronSubdivisionController', name='Cutter',
                            cutPointA=[-12, 0, -12], cutPointB=[12, 6, -12],
                            cutDir=[0, 0, 1], cutDepth=20)

# set the Data and let the controller run the cut at the end of the next step
controller.performCut.value = True

# or pick the tetrahedra to refine, then send the key
controller.testID.value = [40, 41, 42]
```

Only `performCut` triggers an operation from a Data. The other three are reached through a key
event or from C++.

## Driving it from C++

Useful when a carving, collision or tool component decides where to cut:

```cpp
#include <InfinyToolkit/MeshRefinement/TetrahedronSubdivisionController.h>

using Controller = sofa::infinytoolkit::TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types>;

auto* controller = node->get<Controller>();

// one-shot cut, with the surface
controller->cutFromPlane({-12, 0, -12}, {12, 6, -12}, {0, 0, 1}, 20.0, true);

// or build it first, draw it, then commit
if (controller->prepareCutFromTriangles(triA, triB, 5.0))
{
    // ... inspect / draw ...
    controller->applyCut(false);
}

// refinement
controller->refineTetrahedra({40, 41, 42});
controller->refineFullMesh();
```

Every method returns `false` rather than throwing: the controller failed to initialise, or an id
was out of range. Each failure also logs an error naming the reason.

Link against `InfinyToolkit.MeshRefinement` and the component is instantiated for `Vec3Types`. Other
vector types need an explicit instantiation in both this plugin and `MeshRefinement`.

## Example scenes

All four are self-contained - the mesh is generated by the scene, so nothing external is needed.

| Scene | Shows | Try |
|--|--|--|
| [`/InfinyToolkit/examples/MeshRefinement/SubdivisionController_CutFromPlane.scn`](../../examples/MeshRefinement/SubdivisionController_CutFromPlane.scn) | Cutting a beam along a plane, with the cut drawn before it is applied | `1` then `2` |
| [`/InfinyToolkit/examples/MeshRefinement/SubdivisionController_CutWithSurface.scn`](../../examples/MeshRefinement/SubdivisionController_CutWithSurface.scn) | The same cut with `surfaceCut="1"`, and the scene structure that needs | `1` then `2` |
| [`/InfinyToolkit/examples/MeshRefinement/SubdivisionController_RefineTetrahedra.scn`](../../examples/MeshRefinement/SubdivisionController_RefineTetrahedra.scn) | Refining three chosen tetrahedra and the neighbours that follow | `3` |
| [`/InfinyToolkit/examples/MeshRefinement/SubdivisionController_RefineFullMesh.scn`](../../examples/MeshRefinement/SubdivisionController_RefineFullMesh.scn) | Refining the whole mesh | `4` |

```
runSofa examples/SubdivisionController_CutFromPlane.scn
```

Further scenes, including Geomagic and CUDA variants and larger anatomical meshes, are in the
`examples/` folder of the MeshRefinement repository.

## Troubleshooting

| Symptom | Cause |
|--|--|
| `No topology found` at init | No `TetrahedronSetTopologyContainer` in the controller's node. Nothing else will run |
| `No TetrahedronSetTopologyModifier found.` / `...GeometryAlgorithms...` / `No BaseMechanicalState found.` | One of the [required components](#quick-start) is missing from the node. The engine reports each by name and stays unusable |
| `Controller is not initialised, cannot cut.` | Follows one of the above. Fix the init error first |
| Nothing happens on `1` and `2` | The cut quad misses the mesh. Turn on `drawDebugCut="1"` and check where it lands - `cutPointA`/`cutPointB` are absolute positions, not offsets |
| Nothing happens on `3` | `testID` is empty, or `refineCriteria` is high enough that every listed tetrahedron is below the volume threshold |
| `Tetrahedron id out of range` | `testID` names a tetrahedron the mesh does not have. Nothing is refined |
| `Triangle id out of range` | Same for `cutFromTriangles` |
| `not possible to compute Cut surface...` | `surfaceCut="1"` without the [node structure it needs](#creating-the-cut-surface). The cut still happened |
| Nothing is drawn | `drawTetra` / `drawDebugCut` are off, or the scene is running under `-g batch` |

## Tests

`InfinyToolkit.MeshRefinement_test` covers the four operations, the two-phase form, the key
handling and the out-of-range guards. Its scenes live in `tests/scenes/` so the test needs nothing
from the MeshRefinement repository at run time.

```
ctest --test-dir <sofa-build> -C Release -R InfinyToolkit.MeshRefinement_test
```

## License

Dual-licensed, GPL or commercial, like the rest of InfinyToolkit - see
[LICENSE.md](../../LICENSE.md). The `MeshRefinement` plugin it drives is licensed separately;
contact contact@infinytech3d.com.
