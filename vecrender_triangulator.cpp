#include <vecrender_triangulator.h>
#include <vecrender_mathutils.h>
#include <vecrender_triangulationutils.h>
#include <glm/glm.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <algorithm>
#include <iterator>
#include <queue>
#include <deque>
#include <cassert>

namespace vecrender {

namespace {

constexpr float k_EPSILON = 1e-7f;

struct FaceInfo
{
    int nesting_level;
    bool inDomain() {
        return nesting_level%2 == 1;
    }
};

struct VertexInfo
{
    const Triangulator_Segment* segment;
    int index;

    VertexInfo()
        : segment(nullptr)
        , index(-1)
    {}

    const glm::vec3 getFactor() const 
    {
        assert(segment);
        return segment->getFactor(index);
    }
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo,K> Vb;
typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo,K>     Fbb;
typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Exact_predicates_tag                                Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>  CDT;

CDT::Vertex_handle insertPoint(CDT& cdt, const Triangulator_Segment& segment, int p)
{
    CDT::Vertex_handle vh = 
        cdt.insert(TriangulationUtils::vec2ToPoint<CDT>(segment.getPoint(p)));
    if (!vh->info().segment) {
        vh->info().segment = &segment;
        vh->info().index = p;
    }
    return vh;
}

void insertConstraint(CDT& cdt, const Triangulator_Segment& segment,
                      int p0, int p1)
{
    CDT::Vertex_handle vh0 = insertPoint(cdt, segment, p0);
    CDT::Vertex_handle vh1 = insertPoint(cdt, segment, p1);
    cdt.insert_constraint(vh0, vh1);
}

bool isFaceConstrained(CDT::Face_handle fh)
{
    for (int i = 0; i < 3; ++i) {
        if (fh->is_constrained(i)) {
            return true;
        }
    }
    return false;
}

void markDomains(CDT& ct, CDT::Face_handle start, 
                 int index, std::deque<CDT::Edge>& border)
{
    if (start->info().nesting_level != -1){
        return;
    }

    std::queue<CDT::Face_handle> q;
    q.push(start);
    while (!q.empty()){
        CDT::Face_handle fh = q.front();
        q.pop();
        if (fh->info().nesting_level == -1){
            fh->info().nesting_level = index;
            for(int i = 0; i < 3; i++){
                CDT::Edge e(fh,i);
                CDT::Face_handle n = fh->neighbor(i);
                if(n->info().nesting_level == -1){
                    if(ct.is_constrained(e)) {
                        border.push_back(e);
                    } else {
                        q.push(n);
                    }
                }
            }
        }
    }
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void markDomains(CDT& cdt)
{
    for (CDT::Face_handle f : cdt.all_face_handles()) {
        f->info().nesting_level = -1;
    }
    std::deque<CDT::Edge> border;
    markDomains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()) {
        CDT::Edge e = border.front();
        border.pop_front();
        CDT::Face_handle n = e.first->neighbor(e.second);
        if(n->info().nesting_level == -1){
            markDomains(cdt, n, e.first->info().nesting_level+1, border);
        }
    }
}

struct CubicCurveClassifier {
    enum Enum {
        e_SERPENTINE,
        e_LOOP,
        e_CUSP_INFLECTION_AT_INF,
        e_CUSP_CUSP_AT_INF,
        e_QUADRATIC,
        e_LINE_OR_POINT,
    };

    static CubicCurveClassifier::Enum classify(
        const glm::vec2* points,
        float d1, float d2, float d3) {
        if (std::abs(d1) <= k_EPSILON) {
            float discr = 3.0f * d2 * d2 - 4 * d1 * d3;
            if (discr > k_EPSILON) {
                return e_SERPENTINE;
            } else if (discr < -k_EPSILON) {
                return e_LOOP;
            } else {
                return e_CUSP_INFLECTION_AT_INF;
            }
        } else if (std::abs(d2) <= k_EPSILON) {
            if (std::abs(d3) > k_EPSILON) {
                return e_QUADRATIC;
            } else {
                return e_LINE_OR_POINT;
            }
        } else {
            return e_CUSP_CUSP_AT_INF;
        }
    }
};

bool loopNeedReverse(float d1, float k1)
{
    return (d1 > k_EPSILON && k1 < -k_EPSILON) || 
        (d1 < -k_EPSILON && k1 > k_EPSILON);
}

} // anonymous namespace

// Triangulator_Segment

Triangulator_Segment::Triangulator_Segment(const glm::vec2* points, 
                                           PathElement::Enum type)
    : m_type(type)
{
    setPoints(points);
}

PathElement::Enum Triangulator_Segment::getType() const
{
    return m_type;
}
    
const glm::vec2* Triangulator_Segment::getPoints() const
{
    return m_points;
}

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

glm::vec3* Triangulator_Segment::getFactors()
{
    return m_factors;
}

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
        m_factors[0] = {  0,   0, 0 };
        m_factors[1] = {  0.5, 0, 0.5 };
        m_factors[2] = {  1,   1, 1 };
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
    glm::vec2 c2 = 3.0f * m_points[0] - 6.0f * m_points[1] + 3.0f * m_points[2];
    glm::vec2 c3 = -m_points[0] + 3.0f * m_points[1] - 
                    3.0f * m_points[2] + m_points[3];

    float d1 = MathUtils::determinant(c2, c3);
    float d2 = MathUtils::determinant(c3, c1);
    float d3 = MathUtils::determinant(c1, c2);

    constexpr glm::mat4 k_INVERSE_M(glm::vec4(1, 0, 0, 0),
                                    glm::vec4(1, 1.0f/3.0f, 0, 0),
                                    glm::vec4(1, 2.0f/3.0f, 1.0f/3.0f, 0),
                                    glm::vec4(-1, 1, 1, 1));
    bool segmentCreated = false;
    bool reverse = false;
    if (std::abs(d1) > k_EPSILON || std::abs(d2) > k_EPSILON) {
        glm::mat4x3 F;
        glm::mat3x4 C;
        if (std::abs(d1) <= k_EPSILON) {
            // cusp with cusp at infinity
            F[0] = { d3, d3 * d3 * d3, 1 };
            F[1] = { -3.0f*d2, -9.0f*d2*d3*d3, 0 };
            F[2] = { 0, 27.0f*d2*d2*d3, 0 };
            F[3] = { 0, -27.0f*d2*d2*d2, 0 };
            reverse = true;
        } else {
            glm::vec2 l, m;
            float discr = 3.0f * d2 * d2 - 4 * d1 * d3;
            if (discr >= -k_EPSILON) {
                // serpentine & cusp with inflection at infinity
                auto [tl, tm] = MathUtils::solveQuadratic(-3.0f * d1, 
                                                          3.0f * d2, 
                                                          -d3);
                l = { tl, 2.0f * d1 };
                m = { tm, 2.0f * d1 };
                l = glm::normalize(l);
                m = glm::normalize(m);
                F[0] = { l.x*m.x, l.x*l.x*l.x, m.x*m.x*m.x };
                F[1] = { -m.y*l.x-l.y*m.x, -3.0f*l.y*l.x*l.x, 
                        -3.0f*m.y*m.x*m.x };
                F[2] = { l.y*m.y, 3.0f*l.y*l.y*l.x, 3.0f*m.y*m.y*m.x };
                F[3] = { 0.0f, -l.y*l.y*l.y, -m.y*m.y*m.y };
                reverse = d1 < 0;
            }  else {
                // loop
                auto [td, te] = MathUtils::solveQuadratic(-d1 * d1,
                                                          d1 * d2,
                                                          d3 * d1 - d2 * d2);
                l = { td, 2.0f * d1 };
                m = { te, 2.0f * d1 };
                l = glm::normalize(l);
                m = glm::normalize(m);
                F[0] = { l.x*m.x, l.x*l.x*m.x, l.x*m.x*m.x };
                F[1] = { -m.y*l.x-l.y*m.x, -m.y*l.x*l.x-2.0f*l.y*m.x*l.x, 
                        -l.y*m.x*m.x-2.0f*m.y*l.x*m.x };
                F[2] = { l.y*m.y, m.x*l.y*l.y+2.0f*m.y*l.x*l.y, 
                        l.x*m.y*m.y+2.0f*l.y*m.x*m.y };
                F[3] = { 0.0f, -l.y*l.y*m.y, -l.y*m.y*m.y };
            }

            C = F * k_INVERSE_M;
            for (int i = 0; i < 4; ++i) {
                m_factors[i] = C[i];
            }
            if (discr < -k_EPSILON) { // loop
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
                if (segmentCreated && 
                    loopNeedReverse(d1, newSegment.m_factors[1].x)) {
                    newSegment.reverseOrientation();
                }
            }
        }
    } else {
        if (std::abs(d3) > k_EPSILON) {
            // quadratic
            m_factors[0] = { 0, 0, 0 };
            m_factors[1] = { 1.0f/3.0f, 0, 1.0f/3.0f };
            m_factors[2] = { 2.0f/3.0f, 1.0f/3.0f, 2.0f/3.0f };
            m_factors[3] = { 1, 1, 1 };
            reverse = d3 < -k_EPSILON;
        } else {
            // line or point
            std::fill_n(m_factors, getNumPoints(), glm::vec3(0));
        }
    }

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

const LocalTriangulatorTriangle&
Triangulator_Segment::getTriangle(std::size_t index) const
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
    // detect overlapping
    // furthur subdivision for antialising
    constrainedTriangulate();
}

const Triangulator_Vertex* Triangulator::getVertices() const
{
    return m_vertices.data();
}

std::size_t Triangulator::numVertices() const
{
    return m_vertices.size();
}

void Triangulator::constrainedTriangulate()
{
    CDT cdt;
    for (auto& segment : m_segments) {
        segment.triangulate();
        if (segment.getType() == PathElement::e_LINE) {
            insertConstraint(cdt, segment, 0, 1);
        } else {
            for (std::size_t i = 0; i < segment.getNumTriangles(); ++i) {
                for (int j = 0; j < 3; ++j) {
                    insertConstraint(cdt, segment,
                        segment.getTriangle(i).getIndex(j),
                        segment.getTriangle(i).getIndex((j + 1) % 3));
                }
            }
        }
    }

    markDomains(cdt);
    for (auto fh : cdt.finite_face_handles()) {
        if (!fh->info().inDomain()) {
            continue;
        }
        bool constrained = isFaceConstrained(fh);
        for (int i = 0; i < 3; ++i) {
            CDT::Vertex_handle vh = fh->vertex(i);
            Triangulator_Vertex& tv = m_vertices.emplace_back();
            tv.position = TriangulationUtils::pointToVec2<CDT>(vh->point());
            if (constrained) {
                tv.factors = vh->info().getFactor();
            } else {
                tv.factors = { -1, 0, 0 };
            }
        }
    }
}

} /* vecrender  */ 
