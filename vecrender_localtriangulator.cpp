#include <vecrender_localtriangulator.h>
#include <vecrender_triangulationutils.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <glm/glm.hpp>

namespace vecrender {

namespace {

struct VertexInfo
{
    std::uint8_t index;
};

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo,K> Vb;
typedef CGAL::Triangulation_face_base_2<K>                        Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>               TDS;
typedef CGAL::Triangulation_2<K, TDS>                             DT;

} // anonymous namespace
    
void LocalTriangulatorTriangle::setIndices(std::uint8_t v0,
                                           std::uint8_t v1,
                                           std::uint8_t v2)
{
    m_indices[0] = v0;
    m_indices[1] = v1;
    m_indices[2] = v2;
}

LocalTriangulator::LocalTriangulator()
    : m_points(nullptr)
    , m_numTriangles(0)
{
}

void LocalTriangulator::setPoints(const glm::vec2* points, int numPoints)
{
    assert(points && (numPoints == 3 || numPoints == 4));
    m_points = points;
    m_numTriangles = 0;

    DT dt;
    for (int i = 0; i < numPoints; ++i) {
        DT::Vertex_handle vh = 
            dt.insert(TriangulationUtils::vec2ToPoint<DT>(points[i]));
        vh->info().index = i;
    }
    for (DT::Face_handle fh : dt.finite_face_handles()) {
        addTriangle(fh->vertex(0)->info().index,
                    fh->vertex(1)->info().index,
                    fh->vertex(2)->info().index);
    }
}

void LocalTriangulator::addTriangle(std::uint8_t v0, std::uint8_t v1, 
                                    std::uint8_t v2)
{
    assert(m_numTriangles < m_triangles.size());
    m_triangles[m_numTriangles++].setIndices(v0, v1, v2);
}

} /* vecrender  */ 
