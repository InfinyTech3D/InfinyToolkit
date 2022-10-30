/*****************************************************************************
 *            Copyright (C) - InfinyTech3D - All Rights Reserved             *
 *                                                                           *
 * Unauthorized copying of this file, via any medium is strictly prohibited  *
 * Proprietary and confidential.                                             *
 *                                                                           *
 * Written by Erik Pernod <erik.pernod@infinytech3d.com>, October 2019       *
 ****************************************************************************/

#include <InfinyToolkit/NeedleTracker.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/helper/proximity.h>
#include <limits>

namespace sofa::infinytoolkit
{

int NeedleTrackerClass = core::RegisterObject("Class to track the current position of the needle tip.")
    .add< NeedleTracker >()
    ;


NeedleTracker::NeedleTracker()
    : d_drawDebug(initData(&d_drawDebug, true, "drawBB", "if true, will draw slices BB, ray and intersected triangles"))
    , d_sliceID(initData(&d_sliceID, -1, "sliceID", "ID of the sliced where the needle tip is. -1 if none."))
    , d_sliceName(initData(&d_sliceName, std::string("None"), "sliceName", "ID of the sliced where the needle tip is. -1 if none."))
    , isInit(false)
    , m_needle(NULL)
{

}


NeedleTracker::~NeedleTracker()
{

}


void NeedleTracker::bwdInit()
{
    computeSlicesBB();
}


void NeedleTracker::init()
{
    getContext()->get<MechanicalObject3>(&m_slices, core::objectmodel::Tag("slice"), core::objectmodel::BaseContext::SearchRoot);
    if (m_slices.empty())
        msg_error() << "No slices mechanicalObject found";
    else {
        msg_info() << "found Number of slices: " << m_slices.size();
        m_min.resize(m_slices.size());
        m_max.resize(m_slices.size());
    }

    // get needle position 
    sofa::type::vector<MechanicalObject3*> buffer;
    getContext()->get<MechanicalObject3>(&buffer, core::objectmodel::Tag("needle"), core::objectmodel::BaseContext::SearchRoot);
    if (buffer.empty())
    {
        msg_error() << "No needle mechanicalObject found";
        m_needle = NULL;
        return;
    }
    else 
    {
        msg_info() << "Needle found.";
        m_needle = buffer[0];
    }
}


void NeedleTracker::computeSlicesBB()
{
    for (int i = 0; i < m_slices.size(); ++i)
    {
        MechanicalObject3* meca = m_slices[i];
        Coord& sliceMin = m_min[i];
        Coord& sliceMax = m_max[i];

        // init max BB values
        sliceMin = Coord(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        sliceMax = Coord(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

        size_t nbrDof = meca->getSize();
        for (int j = 0; j < nbrDof; ++j)
        {
            Coord pos = Coord(meca->getPX(j), meca->getPY(j), meca->getPZ(j));

            // check max values
            for (int k = 0; k < 3; ++k)
                if (pos[k] > sliceMax[k])
                    sliceMax[k] = pos[k];

            // check min values
            for (int k = 0; k < 3; ++k)
                if (pos[k] < sliceMin[k])
                    sliceMin[k] = pos[k];
        }
    }

    isInit = true;
}


std::string NeedleTracker::getPositionInSlices(const Coord& tipPosition)
{
    // recompute BB first:
    computeSlicesBB();

    int sliceID = -1;
    
    // first look at the BB
    for (int i = 0; i < m_slices.size(); ++i) 
    {
        bool insideBB = testSliceBBIntersection(i, tipPosition);

        // check close intersection
        if (insideBB)
        {
            bool inside = testSliceDiscretIntersection(i, tipPosition);
            if (inside)
            {
                sliceID = i;
                break;
            }
        }
    }

    d_sliceID.setValue(sliceID);
    if (sliceID == -1)
        d_sliceName.setValue(std::string("None"));
    else
        d_sliceName.setValue(m_slices[sliceID]->getName());
       
    return d_sliceName.getValue();
}


bool NeedleTracker::testSliceBBIntersection(int sliceID, const Coord& tipPosition)
{
    const Coord& min = m_min[sliceID];
    const Coord& max = m_max[sliceID];
    for (int i = 0; i < 3; ++i)
    {
        if (tipPosition[i] < min[i])
            return false;

        if (tipPosition[i] > max[i])
            return false;
    }

    return true;
}

using sofa::core::topology::BaseMeshTopology;
bool NeedleTracker::testSliceDiscretIntersection(int sliceID, const Coord& tipPosition)
{
    // Todo lanc� de rayon ici
    if (sliceID >= m_slices.size()) {
        msg_error() << "SlideId " << sliceID << " out fo bounds.";
        return false;
    }

    MechanicalObject3* meca = m_slices[sliceID];
    // todo use: sofa::core::topology::BaseMeshTopology* _topology = l_topology.get();
    BaseMeshTopology* topo = meca->getContext()->getMeshTopology();

    if (topo == nullptr)
    {
        msg_error() << "No topology found for slice: " << sliceID;
        return false;
    }

    const BaseMeshTopology::SeqTriangles& triangles = topo->getTriangles();
    if (triangles.empty())
    {
        msg_error() << "No triangles found in topology for slice: " << sliceID;
        return false;
    }

    // compute ray direction to first triangle
    const BaseMeshTopology::Triangle& tri0 = triangles[0];
    Coord pos0 = Coord(0.0, 0.0, 0.0);
    for (int i = 0; i < 3; ++i)
    {
        int vId = tri0[i];
        pos0 += Coord(meca->getPX(vId), meca->getPY(vId), meca->getPZ(vId));
    }
    pos0 /= 3;
    m_rayDirection = (pos0 - tipPosition);
    //m_rayDirection = tipPosition + m_rayDirection * 100;
    m_rayOrigin = tipPosition;    

    //static sofa::helper::DistanceSegTri proximitySolver;
    m_triPointInter.clear();
    for (int i = 0; i < triangles.size(); ++i)
    {
        const BaseMeshTopology::Triangle& tri = triangles[i];
        type::Vec3 out;
        auto p0 = type::Vec3(meca->getPX(tri[0]), meca->getPY(tri[0]), meca->getPZ(tri[0]));
        auto p1 = type::Vec3(meca->getPX(tri[1]), meca->getPY(tri[1]), meca->getPZ(tri[1]));
        auto p2 = type::Vec3(meca->getPX(tri[2]), meca->getPY(tri[2]), meca->getPZ(tri[2]));
       
        bool intersect = RayIntersectsTriangle(m_rayOrigin, m_rayDirection, p0, p1, p2, out);

        if (intersect)
        {
            m_triPointInter.push_back(p0);
            m_triPointInter.push_back(p1);
            m_triPointInter.push_back(p2);
        }
    }

    int nbrInter = m_triPointInter.size() / 3;
    if (nbrInter % 2) {
        return true;
    }
    else
        return false;
}



bool NeedleTracker::RayIntersectsTriangle(const Coord& origin, const type::Vec3& rayDirection, const Coord& P0, const Coord& P1, const Coord& P2, Coord& outIntersection)
{
    const float EPSILON = 0.0000001;

    type::Vec3 edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = P1 - P0;
    edge2 = P2 - P0;
    h = rayDirection.cross(edge2);
    a = edge1 * h;

    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.
    f = 1.0 / a;
    s = origin - P0;
    u = f * (s * h);

    if (u < 0.0 || u > 1.0)
        return false;
    q = s.cross(edge1);
    v = f * rayDirection * q;
    if (v < 0.0 || u + v > 1.0)
        return false;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * ( edge2 * q);
    if (t > EPSILON) // ray intersection
    {
        outIntersection = origin + rayDirection * t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}


void NeedleTracker::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::simulation::AnimateEndEvent::checkEventType(event))
    {        
        if (m_needle != NULL && m_needle->getSize() > 0) 
        {
            Coord tipPos = Coord(m_needle->getPX(0), m_needle->getPY(0), m_needle->getPZ(0));
            std::string res = getPositionInSlices(tipPos);
        }
    }
}


void NeedleTracker::draw(const core::visual::VisualParams* vparams)
{
    if (!isInit || m_slices.empty())
        return;

    if (!d_drawDebug.getValue())
        return;

    vparams->drawTool()->setLightingEnabled(false);

    // Draw BB
    {
        for (int i = 0; i < m_slices.size(); ++i)
        {
            float val = (float)i / m_slices.size();
            sofa::type::RGBAColor colorDefault(0.0f, 1.0f, 0.0f, 1.0f);

            if (d_sliceID.getValue() == i)
                colorDefault = sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f);

            sofa::type::vector<Coord> vertices;
            Coord min = m_min[i];
            Coord max = m_max[i];

            // FACE A
            vertices.push_back(Coord(min[0], min[1], min[2]));
            vertices.push_back(Coord(min[0], max[1], min[2]));

            vertices.push_back(Coord(min[0], min[1], min[2]));
            vertices.push_back(Coord(max[0], min[1], min[2]));

            vertices.push_back(Coord(min[0], max[1], min[2]));
            vertices.push_back(Coord(max[0], max[1], min[2]));

            vertices.push_back(Coord(max[0], min[1], min[2]));
            vertices.push_back(Coord(max[0], max[1], min[2]));

            // FACE B
            vertices.push_back(Coord(min[0], min[1], max[2]));
            vertices.push_back(Coord(min[0], max[1], max[2]));

            vertices.push_back(Coord(min[0], min[1], max[2]));
            vertices.push_back(Coord(max[0], min[1], max[2]));

            vertices.push_back(Coord(min[0], max[1], max[2]));
            vertices.push_back(Coord(max[0], max[1], max[2]));

            vertices.push_back(Coord(max[0], min[1], max[2]));
            vertices.push_back(Coord(max[0], max[1], max[2]));

            // Middle
            vertices.push_back(Coord(min[0], min[1], min[2]));
            vertices.push_back(Coord(min[0], min[1], max[2]));

            vertices.push_back(Coord(max[0], min[1], min[2]));
            vertices.push_back(Coord(max[0], min[1], max[2]));

            vertices.push_back(Coord(min[0], max[1], min[2]));
            vertices.push_back(Coord(min[0], max[1], max[2]));

            vertices.push_back(Coord(max[0], max[1], min[2]));
            vertices.push_back(Coord(max[0], max[1], max[2]));

            vparams->drawTool()->drawLines(vertices, 1.0f, colorDefault);
        }
    }

    // Narrow phase debug
    if (!m_triPointInter.empty())// possible intersection detected
    {
        // Draw ray
        vparams->drawTool()->drawLine(m_rayOrigin, m_rayOrigin + m_rayDirection * 100, sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f));

        sofa::type::vector<Coord> vertices;
        for (int i = 0; i < (int)(m_triPointInter.size() / 3); ++i)
        {
            vertices.push_back(m_triPointInter[i * 3]);
            vertices.push_back(m_triPointInter[i * 3 + 1]);

            vertices.push_back(m_triPointInter[i * 3]);
            vertices.push_back(m_triPointInter[i * 3 + 2]);

            vertices.push_back(m_triPointInter[i * 3 + 2]);
            vertices.push_back(m_triPointInter[i * 3 + 1]);
        }

        vparams->drawTool()->drawLines(vertices, 10.0f, sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f));
    }

    vparams->drawTool()->setLightingEnabled(true);
}


} // namespace sofa::infinytoolkit
