#ifndef INCLUDED_VECRENDER_TRIANGULATOR
#define INCLUDED_VECRENDER_TRIANGULATOR

#include <vecrender_path.h>
#include <vecrender_localtriangulator.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/fwd.hpp>
#include <vector>
#include <cstddef>

namespace vecrender {

class Triangulator;

class Triangulator_Segment
{
    friend class Triangulator;
    Triangulator_Segment() = default;

public:
    // must be public to be used with std::vector
    Triangulator_Segment(const glm::vec2* points, PathElement::Enum type);

    PathElement::Enum getType() const;

    const glm::vec2* getPoints() const;
    const glm::vec2& getPoint(std::size_t index) const;
    void setPoints(const glm::vec2* points);
    std::size_t getNumPoints() const;

    glm::vec3* getFactors();
    glm::vec3& getFactor(std::size_t index);
    const glm::vec3& getFactor(std::size_t index) const;
    void setFactors(const glm::vec3* factors);

    void split(float t, Triangulator_Segment& newSegment);

    // Compute the linear factors for the curve
    // Return true if a new segment is created to resolove
    // the loop curve artifact
    bool computeFactors(Triangulator_Segment& newSegment);

    void triangulate();
    std::size_t getNumTriangles() const;
    const LocalTriangulatorTriangle& getTriangle(std::size_t index) const;

private:
    bool computeCubicBezierFactors(Triangulator_Segment& newSegment);
    void reverseOrientation();

    glm::vec2 m_points[4];
    PathElement::Enum m_type;
    glm::vec3 m_factors[4];
    LocalTriangulator m_triangulator;
};

struct TriangulatorVertex
{
    glm::vec2 position;
    glm::vec3 factors;
#ifdef DEBUG_TRIANGULATION
    enum Enum
    {
        e_NON_DOMAIN,
        e_CONSTRAINT,
        e_INNER
    };
    TriangulatorVertex::Enum type;
#endif
};

class Triangulator
{
public:
    Triangulator(const Path& path);

    const TriangulatorVertex* getVertices() const;
    std::size_t getNumVertices() const;

private:
    void constrainedTriangulate();
    void addVertex(const glm::vec2& pos, const glm::vec3& factors);

    std::vector<TriangulatorVertex> m_vertices;
    std::vector<Triangulator_Segment> m_segments;
};

} // namespace vecrender

#endif /* ifndef INCLUDED_VECRENDER_TRIANGULATOR */
