#include <vecrender_triangulator.h>
#include <vecrender_mathutils.h>
#include <vecrender_domaintrianglemarker.h>
#include <glm/glm.hpp>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iterator>
#include <map>
#include <utility>

namespace vecrender {

namespace {

constexpr float k_EPSILON = 1e-7f;

struct FaceInfo
{
    int nestingLevel;
    bool inDomain() { return nestingLevel % 2 == 1; }
};

struct VertexInfo
{
    int id = -1;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo, K> Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K, Fbb> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;

struct ConstraintEdgeInfo
{
    const Triangulator_Segment* segment;
    bool border;
};

class CDTHelper
{
    using Edge = std::pair<int, int>;

public:
    void addConstraints(const Triangulator_Segment& segment)
    {
        if (segment.getType() == PathElement::e_LINE) {
            addConstraint(segment, 0, 1);
        } else {
            for (std::size_t i = 0; i < segment.getNumTriangles(); ++i) {
                for (std::size_t j = 0; j < 3; ++j) {
                    addConstraint(segment, segment.getTriangle(i).getIndex(j),
                        segment.getTriangle(i).getIndex((j + 1) % 3));
                }
            }
        }
    }

    CDT& getCDT() { return m_cdt; }

    bool isInnerTriangle(CDT::Face_handle f) const
    {
        const Triangulator_Segment* curSeg = nullptr;
        int numConstraintEdges = 0;
        for (int i = 0; i < 3; ++i) {
            const ConstraintEdgeInfo* info
                = getConstraintEdgeInfo(CDT::Edge(f, i));
            if (info) {
                if (curSeg && curSeg != info->segment) {
                    break;
                } else if (!curSeg) {
                    curSeg = info->segment;
                }
                ++numConstraintEdges;
            }
        }
        return numConstraintEdges < 3;
    }

    const ConstraintEdgeInfo* getConstraintEdgeInfo(CDT::Edge e) const
    {
        int v0 = m_cdt.ccw(e.second);
        int v1 = m_cdt.cw(e.second);
        auto it = m_constraintEdges.find(
            makeEdge(e.first->vertex(v0), e.first->vertex(v1)));
        return it != m_constraintEdges.end() ? &it->second : nullptr;
    }

private:
    void addConstraint(const Triangulator_Segment& segment, int i, int j)
    {
        const glm::vec2& p0 = segment.getPoint(i);
        CDT::Vertex_handle vh0 = m_cdt.insert(CDT::Point(p0.x, p0.y));
        if (vh0->info().id == -1) {
            vh0->info().id = m_nextVertexId++;
        }
        const glm::vec2& p1 = segment.getPoint(j);
        CDT::Vertex_handle vh1 = m_cdt.insert(CDT::Point(p1.x, p1.y));
        if (vh1->info().id == -1) {
            vh1->info().id = m_nextVertexId++;
        }
        m_cdt.insert_constraint(vh0, vh1);

        ConstraintEdgeInfo& info = m_constraintEdges[makeEdge(vh0, vh1)];
        info.segment = &segment;
        info.border = std::abs(i - j) == 1;
    }

    static Edge makeEdge(CDT::Vertex_handle vh0, CDT::Vertex_handle vh1)
    {
        int id0 = vh0->info().id;
        int id1 = vh1->info().id;
        return { std::min(id0, id1), std::max(id0, id1) };
    }

    CDT m_cdt;
    std::map<Edge, ConstraintEdgeInfo> m_constraintEdges;
    int m_nextVertexId = 0;
};

class DomainPolicy
{
public:
    DomainPolicy(CDTHelper& impl)
        : m_impl(impl)
    {
    }

    void setFaceNestingLevel(CDT::Face_handle f, int level)
    {
        f->info().nestingLevel = level;
    }

    int getFaceNestingLevel(CDT::Face_handle f) const
    {
        return f->info().nestingLevel;
    }

    bool isBorderEdge(CDT::Edge e) const
    {
        const ConstraintEdgeInfo* info = m_impl.getConstraintEdgeInfo(e);
        return info && info->border;
    }

private:
    CDTHelper& m_impl;
};

struct CubicClassifier
{
    enum Enum
    {
        e_SERPENTINE,
        e_LOOP,
        e_CUSP_INFLECTION_AT_INF,
        e_CUSP_CUSP_AT_INF,
        e_QUADRATIC,
        e_LINE_OR_POINT,
    };

    static CubicClassifier::Enum classify(
        const glm::vec2* points, float d1, float d2, float d3)
    {
        if (std::abs(d1) > k_EPSILON) {
            float discr = 3.0f * d2 * d2 - 4.0f * d1 * d3;
            if (discr > k_EPSILON) {
                return e_SERPENTINE;
            } else if (discr < -k_EPSILON) {
                return e_LOOP;
            } else {
                return e_CUSP_INFLECTION_AT_INF;
            }
        } else if (std::abs(d2) > k_EPSILON) {
            return e_CUSP_CUSP_AT_INF;
        } else if (std::abs(d3) > k_EPSILON) {
            return e_QUADRATIC;
        } else {
            return e_LINE_OR_POINT;
        }
    }

    static const char* enumToString(CubicClassifier::Enum e)
    {
#define CASE(name)                                                            \
    case e_##name:                                                            \
        return #name;
        switch (e) {
            CASE(SERPENTINE)
            CASE(LOOP)
            CASE(CUSP_CUSP_AT_INF)
            CASE(CUSP_INFLECTION_AT_INF)
            CASE(QUADRATIC)
            CASE(LINE_OR_POINT)
        }
#undef CASE
    }
};

bool loopNeedReverse(float d1, float k1)
{
    return (d1 > k_EPSILON && k1 < -k_EPSILON)
        || (d1 < -k_EPSILON && k1 > k_EPSILON);
}

} // anonymous namespace

// Triangulator_Segment

Triangulator_Segment::Triangulator_Segment(
    const glm::vec2* points, PathElement::Enum type)
    : m_type(type)
{
    setPoints(points);
}

PathElement::Enum Triangulator_Segment::getType() const { return m_type; }

const glm::vec2* Triangulator_Segment::getPoints() const { return m_points; }

const glm::vec2& Triangulator_Segment::getPoint(std::size_t index) const
{
    assert(index < getNumPoints());
    return m_points[index];
}

void Triangulator_Segment::setPoints(const glm::vec2* points)
{
    assert(points);
    std::copy(points, points + getNumPoints(), m_points);
}

std::size_t Triangulator_Segment::getNumPoints() const
{
    return PathElement::getPointNumForElementType(m_type);
}

glm::vec3* Triangulator_Segment::getFactors() { return m_factors; }

glm::vec3& Triangulator_Segment::getFactor(std::size_t index)
{
    assert(index < getNumPoints());
    return m_factors[index];
}

const glm::vec3& Triangulator_Segment::getFactor(std::size_t index) const
{
    assert(index < getNumPoints());
    return m_factors[index];
}

void Triangulator_Segment::setFactors(const glm::vec3* factors)
{
    std::copy(factors, factors + getNumPoints(), m_factors);
}

void Triangulator_Segment::split(float t, Triangulator_Segment& newSegment)
{
    assert(0 < t && t < 1);

    glm::vec2 newPoints[7];
    glm::vec3 newFactors[7];

    switch (m_type) {
    case PathElement::e_LINE: {
        MathUtils::splitLineSegment(m_points, t, newPoints);
        MathUtils::splitLineSegment(m_factors, t, newFactors);
        break;
    }

    case PathElement::e_QUADRATIC:
        MathUtils::splitQuadraticBezier(m_points, t, newPoints);
        MathUtils::splitQuadraticBezier(m_factors, t, newFactors);
        break;

    case PathElement::e_CUBIC:
        MathUtils::splitCubicBezier(m_points, t, newPoints);
        MathUtils::splitCubicBezier(m_factors, t, newFactors);
        break;

    default:
        assert(0);
        break;
    }
    setPoints(newPoints);
    setFactors(newFactors);

    newSegment.setPoints(newPoints + getNumPoints() - 1);
    newSegment.setFactors(newFactors + getNumPoints() - 1);
}

bool Triangulator_Segment::computeFactors(Triangulator_Segment& newSegment)
{
    bool segmentCreated = false;
    switch (m_type) {
    case PathElement::e_LINE:
        m_factors[0] = m_factors[1] = glm::vec3(0);
        break;

    case PathElement::e_QUADRATIC: {
        m_factors[0] = { 0, 0, 0 };
        m_factors[1] = { 0.5, 0, 0.5 };
        m_factors[2] = { 1, 1, 1 };
        break;
    }

    case PathElement::e_CUBIC:
        segmentCreated = computeCubicBezierFactors(newSegment);
        break;

    default:
        assert(0);
        break;
    }
    return segmentCreated;
}

bool Triangulator_Segment::computeCubicBezierFactors(
    Triangulator_Segment& newSegment)
{
    // rows of the coefficient matrix
    glm::vec2 c1 = -3.0f * m_points[0] + 3.0f * m_points[1];
    glm::vec2 c2
        = 3.0f * m_points[0] - 6.0f * m_points[1] + 3.0f * m_points[2];
    glm::vec2 c3
        = -m_points[0] + 3.0f * m_points[1] - 3.0f * m_points[2] + m_points[3];

    float d1 = MathUtils::determinant(c3, c2);
    float d2 = -MathUtils::determinant(c3, c1);
    float d3 = MathUtils::determinant(c2, c1);

    constexpr glm::mat4 k_INVERSE_M(glm::vec4(1, 0, 0, 0),
        glm::vec4(1, 1.0f / 3.0f, 0, 0),
        glm::vec4(1, 2.0f / 3.0f, 1.0f / 3.0f, 0), glm::vec4(1, 1, 1, 1));
    bool segmentCreated = false;
    bool reverse = false;
    CubicClassifier::Enum curveType
        = CubicClassifier::classify(m_points, d1, d2, d2);

    glm::mat4x3 F;
    switch (curveType) {
    case CubicClassifier::e_SERPENTINE:
    case CubicClassifier::e_CUSP_INFLECTION_AT_INF: {
        // serpentine & cusp with inflection at infinity
        auto [tl, tm] = MathUtils::solveQuadratic(-3.0f * d1, 3.0f * d2, -d3);
        glm::vec2 l = { tl, 1.0f };
        glm::vec2 m = { tm, 1.0f };
        l = glm::normalize(l);
        m = glm::normalize(m);
        F[0] = { l.x * m.x, l.x * l.x * l.x, m.x * m.x * m.x };
        F[1] = { -m.y * l.x - l.y * m.x, -3.0f * l.y * l.x * l.x,
            -3.0f * m.y * m.x * m.x };
        F[2] = { l.y * m.y, 3.0f * l.y * l.y * l.x, 3.0f * m.y * m.y * m.x };
        F[3] = { 0.0f, -l.y * l.y * l.y, -m.y * m.y * m.y };
        setFactors(&(F * k_INVERSE_M)[0]);
        reverse = d1 < -k_EPSILON;
        break;
    }

    case CubicClassifier::e_LOOP: {
        // loop
        auto [td, te]
            = MathUtils::solveQuadratic(-d1 * d1, d1 * d2, d3 * d1 - d2 * d2);
        glm::vec2 l = { td, 1.0f };
        glm::vec2 m = { te, 1.0f };
        l = glm::normalize(l);
        m = glm::normalize(m);
        F[0] = { l.x * m.x, l.x * l.x * m.x, l.x * m.x * m.x };
        F[1] = { -m.y * l.x - l.y * m.x,
            -m.y * l.x * l.x - 2.0f * l.y * m.x * l.x,
            -l.y * m.x * m.x - 2.0f * m.y * l.x * m.x };
        F[2] = { l.y * m.y, m.x * l.y * l.y + 2.0f * m.y * l.x * l.y,
            l.x * m.y * m.y + 2.0f * l.y * m.x * m.y };
        F[3] = { 0.0f, -l.y * l.y * m.y, -l.y * m.y * m.y };
        setFactors(&(F * k_INVERSE_M)[0]);

        l.x /= l.y;
        m.x /= m.y;
        if (l.x > k_EPSILON && l.x < 1 - k_EPSILON) {
            split(l.x, newSegment);
            segmentCreated = true;
        } else if (m.x > k_EPSILON && m.x < 1 - k_EPSILON) {
            split(m.x, newSegment);
            segmentCreated = true;
        }
        reverse = loopNeedReverse(d1, m_factors[1].x);
        if (segmentCreated && loopNeedReverse(d1, newSegment.m_factors[1].x)) {
            newSegment.reverseOrientation();
        }
        break;
    }

    case CubicClassifier::e_CUSP_CUSP_AT_INF:
        // cusp with cusp at infinity
        F[0] = { d3, d3 * d3 * d3, 1 };
        F[1] = { -3.0f * d2, -9.0f * d2 * d3 * d3, 0 };
        F[2] = { 0, 27.0f * d2 * d2 * d3, 0 };
        F[3] = { 0, -27.0f * d2 * d2 * d2, 0 };
        setFactors(&(F * k_INVERSE_M)[0]);
        break;

    case CubicClassifier::e_QUADRATIC:
        // quadratic
        m_factors[0] = { 0, 0, 0 };
        m_factors[1] = { 1.0f / 3.0f, 0, 1.0f / 3.0f };
        m_factors[2] = { 2.0f / 3.0f, 1.0f / 3.0f, 2.0f / 3.0f };
        m_factors[3] = { 1, 1, 1 };
        reverse = d3 < -k_EPSILON;
        break;

    case CubicClassifier::e_LINE_OR_POINT:
        std::fill_n(m_factors, getNumPoints(), glm::vec3(0));
        break;
    } // switch (curveType)

    if (reverse) {
        reverseOrientation();
    }

    return segmentCreated;
}

void Triangulator_Segment::reverseOrientation()
{
    for (int i = 0; i < getNumPoints(); ++i) {
        m_factors[i].x = -m_factors[i].x;
        m_factors[i].y = -m_factors[i].y;
    }
}

void Triangulator_Segment::triangulate()
{
    if (m_type == PathElement::e_CUBIC || m_type == PathElement::e_QUADRATIC) {
        m_triangulator.setPoints(m_points, getNumPoints());
    }
}

std::size_t Triangulator_Segment::getNumTriangles() const
{
    if (m_type == PathElement::e_CUBIC || m_type == PathElement::e_QUADRATIC) {
        return m_triangulator.getNumTriangles();
    } else {
        return 0;
    }
}

const LocalTriangulatorTriangle& Triangulator_Segment::getTriangle(
    std::size_t index) const
{
    return m_triangulator.getTriangle(index);
}

Triangulator::Triangulator(const Path& path)
{
    for (auto it = path.elementBegin(); it != path.elementEnd(); ++it) {
        m_segments.emplace_back(it->getPoints(), it->getType());
    }
    for (auto& segment : m_segments) {
        Triangulator_Segment newSegment;
        if (segment.computeFactors(newSegment)) {
            m_segments.push_back(newSegment);
        }
    }
    // XXX: detect overlapping
    // XXX: subdivision for antialising
    constrainedTriangulate();
}

const TriangulatorVertex* Triangulator::getVertices() const
{
    return m_vertices.data();
}

std::size_t Triangulator::numVertices() const { return m_vertices.size(); }

void Triangulator::constrainedTriangulate()
{
    CDTHelper helper;
    for (auto& segment : m_segments) {
        segment.triangulate();
        helper.addConstraints(segment);
    }

    DomainTriangleMarker<CDT> domainMarker;
    DomainPolicy domainPolicy(helper);
    domainMarker.markDomains(helper.getCDT(), domainPolicy);

    for (const auto& segment : m_segments) {
        for (int i = 0; i < segment.getNumTriangles(); ++i) {
            for (int j = 0; j < 3; ++j) {
                TriangulatorVertex& tv = m_vertices.emplace_back();
                auto pointIndex = segment.getTriangle(i).getIndex(j);
                tv.position = segment.getPoint(pointIndex);
                tv.factors = segment.getFactor(pointIndex);
            }
        }
    }
    for (auto fh : helper.getCDT().finite_face_handles()) {
        if (!fh->info().inDomain()) {
            continue;
        }
        if (!helper.isInnerTriangle(fh)) {
            continue;
        }
        for (int i = 0; i < 3; ++i) {
            CDT::Vertex_handle vh = fh->vertex(i);
            TriangulatorVertex& tv = m_vertices.emplace_back();
            tv.position = { vh->point().x(), vh->point().y() };
            tv.factors = { -1, 0, 0 };
        }
    }
}

} // namespace vecrender
