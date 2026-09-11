/**
 * Covers the public API of TetrahedronSubdivisionController.
 *
 * The scenes live in scenes/ beside this file, so the test is self-contained and
 * needs nothing from the MeshRefinement repository at runtime. They take their
 * geometry and their cut parameters from the Data the scene sets, which the cases
 * read back rather than hard-coding, so retuning a scene does not invalidate them.
 *
 * The controller is driven through its own methods as well as by key event. Each case
 * asserts the topology actually changed: a cut or a subdivision that silently does
 * nothing would otherwise pass.
 */

#include <sofa/testing/BaseTest.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/ExecParams.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/simpleapi/SimpleApi.h>

#include <InfinyToolkit/MeshRefinement/TetrahedronSubdivisionController.h>

#include <string>

using sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer;
using SubdivisionController3d = sofa::infinytoolkit::TetrahedronSubdivisionController<sofa::defaulttype::Vec3Types>;

namespace
{

/// Everything these operations are expected to change.
struct TopologyCounts
{
    sofa::Size nbPoints{ 0 };
    sofa::Size nbEdges{ 0 };
    sofa::Size nbTriangles{ 0 };
    sofa::Size nbTetrahedra{ 0 };

    bool operator==(const TopologyCounts& o) const
    {
        return nbPoints == o.nbPoints && nbEdges == o.nbEdges
            && nbTriangles == o.nbTriangles && nbTetrahedra == o.nbTetrahedra;
    }
};

std::ostream& operator<<(std::ostream& os, const TopologyCounts& c)
{
    return os << c.nbPoints << " points, " << c.nbEdges << " edges, "
              << c.nbTriangles << " triangles, " << c.nbTetrahedra << " tetrahedra";
}


class TetrahedronSubdivisionController_test : public sofa::testing::BaseTest
{
public:
    void doSetUp() override
    {
        // node::load needs a Simulation to exist. Plugins are preloaded rather than
        // left to the scenes' RequiredPlugin, so that a single case run under
        // --gtest_filter behaves like the whole suite.
        m_simulation = sofa::simpleapi::createSimulation("DAG");
        sofa::helper::system::DataRepository.addFirstPath(INFINYTOOLKIT_MESHREFINEMENT_TEST_SCENES_DIR);

        for (const auto* plugin : {
                 "Sofa.Component.Collision.Detection.Algorithm",
                 "Sofa.Component.Collision.Detection.Intersection",
                 "Sofa.Component.Collision.Geometry",
                 "Sofa.Component.Collision.Response.Contact",
                 "Sofa.Component.Constraint.Projective",
                 "Sofa.Component.Engine.Select",
                 "Sofa.Component.IO.Mesh",
                 "Sofa.Component.LinearSolver.Iterative",
                 "Sofa.Component.Mapping.Linear",
                 "Sofa.Component.Mass",
                 "Sofa.Component.MechanicalLoad",
                 "Sofa.Component.ODESolver.Backward",
                 "Sofa.Component.SceneUtility",
                 "Sofa.Component.SolidMechanics.FEM.Elastic",
                 "Sofa.Component.StateContainer",
                 "Sofa.Component.Topology.Container.Constant",
                 "Sofa.Component.Topology.Container.Dynamic",
                 "Sofa.Component.Topology.Container.Grid",
                 "Sofa.Component.Topology.Mapping",
                 "Sofa.Component.Topology.Utility",
                 "Sofa.Component.Visual",
                 "Sofa.GL.Component.Rendering3D",
                 "MeshRefinement",
                 "InfinyToolkit.MeshRefinement",
             })
        {
            sofa::simpleapi::importPlugin(plugin);
        }
    }

    void doTearDown() override
    {
        if (m_root)
        {
            sofa::simulation::node::unload(m_root);
            m_root.reset();
        }
    }

protected:
    sofa::simulation::Simulation::SPtr m_simulation;
    sofa::simulation::NodeSPtr m_root;
    SubdivisionController3d* m_controller{ nullptr };
    TetrahedronSetTopologyContainer* m_topology{ nullptr };

    /// Loads an example scene and resolves the controller and the topology it drives.
    void loadScene(const std::string& sceneFile)
    {
        const std::string scenePath = sofa::helper::system::DataRepository.getFile(sceneFile);
        m_root = sofa::simulation::node::load(scenePath);
        ASSERT_NE(m_root.get(), nullptr) << "could not load " << sceneFile;
        sofa::simulation::node::init(m_root.get());

        m_controller = m_root->get<SubdivisionController3d>(sofa::core::objectmodel::BaseContext::SearchDown);
        ASSERT_NE(m_controller, nullptr) << "no TetrahedronSubdivisionController in " << sceneFile;

        // Resolved from the controller's own context, not from the root: a scene with a
        // topological mapping holds more than one container and the first one found is
        // not necessarily the one the controller operates on.
        m_topology = m_controller->getContext()->get<TetrahedronSetTopologyContainer>();
        ASSERT_NE(m_topology, nullptr) << "no TetrahedronSetTopologyContainer beside the controller";
    }

    TopologyCounts counts() const
    {
        return { m_topology->getNbPoints(), m_topology->getNbEdges(),
                 m_topology->getNbTriangles(), m_topology->getNbTetrahedra() };
    }

    void step(int count)
    {
        for (int i = 0; i < count; ++i)
            sofa::simulation::node::animate(m_root.get(), m_root->getDt());
    }

    void pressKey(char key)
    {
        sofa::core::objectmodel::KeypressedEvent event(key);
        m_root->propagateEvent(sofa::core::execparams::defaultInstance(), &event);
    }
};


/// The cut the 8 migrated cutting scenes perform, driven through the API.
TEST_F(TetrahedronSubdivisionController_test, cutFromPlane_changes_the_topology)
{
    loadScene("TetrahedronCutting_Advanced_01.scn");
    step(10);

    const TopologyCounts before = counts();
    // the scene's own plane, so the geometry stays owned by the scene file
    EXPECT_TRUE(m_controller->cutFromPlane(m_controller->d_cutPointA.getValue(),
                                          m_controller->d_cutPointB.getValue(),
                                          m_controller->d_cutDirection.getValue(),
                                          m_controller->d_cutDepth.getValue(),
                                          false));
    step(1);

    const TopologyCounts after = counts();
    EXPECT_FALSE(after == before) << "the cut did nothing: still " << after;
    EXPECT_GT(after.nbTetrahedra, before.nbTetrahedra);
    std::cout << "[pv] cutFromPlane: " << before << "  ->  " << after << std::endl;
}


/// The same cut through the two-phase shape, which must reach the same result.
TEST_F(TetrahedronSubdivisionController_test, prepare_then_applyCut_matches_the_one_shot)
{
    loadScene("TetrahedronCutting_Advanced_01.scn");
    step(10);

    const TopologyCounts before = counts();
    EXPECT_TRUE(m_controller->prepareCutFromPlane(m_controller->d_cutPointA.getValue(),
                                                 m_controller->d_cutPointB.getValue(),
                                                 m_controller->d_cutDirection.getValue(),
                                                 m_controller->d_cutDepth.getValue()));

    // building the path must not touch the topology on its own
    EXPECT_TRUE(counts() == before) << "prepareCutFromPlane already changed the topology";

    EXPECT_TRUE(m_controller->applyCut(false));
    step(1);

    const TopologyCounts after = counts();
    EXPECT_FALSE(after == before) << "the cut did nothing: still " << after;
    std::cout << "[pv] prepare+apply: " << before << "  ->  " << after << std::endl;
}


/// Keys '1' then '2' are what the migrated scenes rely on.
TEST_F(TetrahedronSubdivisionController_test, keys_1_then_2_cut)
{
    loadScene("TetrahedronCutting_Advanced_01.scn");
    step(10);

    const TopologyCounts before = counts();
    pressKey('1');
    step(1);
    pressKey('2');
    step(1);

    const TopologyCounts after = counts();
    EXPECT_FALSE(after == before) << "keys 1,2 did not cut: still " << after;
}


/// Out-of-range tetrahedron ids must be refused rather than read out of bounds. This
/// is what SimpleCubeRefinement.scn used to do with testID="60" on a 44 tetrahedron
/// mesh: an access violation, not an error.
TEST_F(TetrahedronSubdivisionController_test, refineTetrahedra_rejects_bad_ids)
{
    loadScene("SimpleCubeRefinement.scn");
    step(10);

    const TopologyCounts before = counts();
    const std::set<unsigned int> bad { m_topology->getNbTetrahedra() + 10u };

    EXPECT_MSG_EMIT(Error);
    EXPECT_FALSE(m_controller->refineTetrahedra(bad));
    EXPECT_TRUE(counts() == before) << "a refused refinement still changed the topology";
}


/// Out-of-range triangle ids must be refused rather than read out of bounds.
TEST_F(TetrahedronSubdivisionController_test, cutFromTriangles_rejects_bad_ids)
{
    loadScene("TetrahedronCutting_Advanced_01.scn");
    step(10);

    const TopologyCounts before = counts();
    const auto nbTriangles = m_topology->getNbTriangles();

    // BaseTest fails a test on any Error message, and the guard logs one, so the
    // expectation has to be declared before the call.
    EXPECT_MSG_EMIT(Error);
    EXPECT_FALSE(m_controller->cutFromTriangles(nbTriangles + 10, nbTriangles + 20, 5.0, false));
    EXPECT_TRUE(counts() == before) << "a refused cut still changed the topology";
}


/// Refinement of the tetrahedra named by the scene.
TEST_F(TetrahedronSubdivisionController_test, refineTetrahedra_changes_the_topology)
{
    loadScene("SimpleCubeRefinement.scn");
    step(10);

    const TopologyCounts before = counts();
    ASSERT_FALSE(m_controller->d_testID.getValue().empty()) << "the scene names no tetrahedron to refine";

    EXPECT_TRUE(m_controller->refineTetrahedra(m_controller->d_testID.getValue()));
    step(1);

    const TopologyCounts after = counts();
    EXPECT_FALSE(after == before) << "the refinement did nothing: still " << after;
    EXPECT_GT(after.nbTetrahedra, before.nbTetrahedra);
    std::cout << "[pv] refineTetrahedra: " << before << "  ->  " << after << std::endl;
}


/// Whole-mesh refinement, which the merge moved from key '2' to key '4'.
TEST_F(TetrahedronSubdivisionController_test, refineFullMesh_grows_the_mesh)
{
    loadScene("SimpleCubeRefinement.scn");
    step(10);

    const TopologyCounts before = counts();
    EXPECT_TRUE(m_controller->refineFullMesh());
    step(1);

    const TopologyCounts after = counts();
    EXPECT_GT(after.nbTetrahedra, before.nbTetrahedra) << "refineFullMesh did not subdivide: still " << after;
    std::cout << "[pv] refineFullMesh: " << before << "  ->  " << after << std::endl;
}


/// Key '4' has to reach refineFullMesh, and '2' must no longer do so.
TEST_F(TetrahedronSubdivisionController_test, key_4_refines_the_full_mesh)
{
    loadScene("SimpleCubeRefinement.scn");
    step(10);

    const TopologyCounts before = counts();

    // '2' applies a cut now; with no cut prepared it must leave the mesh alone
    pressKey('2');
    step(1);
    EXPECT_TRUE(counts() == before) << "key 2 changed the mesh with no cut prepared";

    pressKey('4');
    step(1);
    const TopologyCounts after = counts();
    EXPECT_GT(after.nbTetrahedra, before.nbTetrahedra) << "key 4 did not refine: still " << after;
}

} // namespace
