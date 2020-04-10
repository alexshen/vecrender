#ifndef INCLUDED_VECRENDER_LOCALTRIANGULATOR
#define INCLUDED_VECRENDER_LOCALTRIANGULATOR

#include <glm/fwd.hpp>
#include <array>
#include <cstdint>
#include <cstddef>

namespace vecrender {

class LocalTriangulatorTriangle
{
public:
    void setIndices(std::uint8_t v0, std::uint8_t v1, std::uint8_t v2);
    std::uint8_t getIndex(std::size_t index) const;
private:
    std::array<std::uint8_t, 3> m_indices;
};

class LocalTriangulator 
{
public:
    LocalTriangulator();

    // triangulate the points, numPoints can either be 3 or 4
    void setPoints(const glm::vec2* points, int numPoints);
    std::size_t getNumTriangles() const;
    const LocalTriangulatorTriangle& getTriangle(std::size_t index) const;
private:
    void addTriangle(std::uint8_t v0, std::uint8_t v1, std::uint8_t v2);

    const glm::vec2* m_points;
    std::array<LocalTriangulatorTriangle, 3> m_triangles;
    std::size_t m_numTriangles;
};

// ============================================================================
// INLINE FUNCTION DEFINITIONS
// ============================================================================

inline std::uint8_t LocalTriangulatorTriangle::getIndex(std::size_t index) const
{
    return m_indices[index];
}

inline std::size_t LocalTriangulator::getNumTriangles() const
{
    return m_numTriangles;
}

inline const LocalTriangulatorTriangle&
LocalTriangulator::getTriangle(std::size_t index) const
{
    assert(index < getNumTriangles());
    assert(m_points);
    return m_triangles[index];
}

} // vecrender

#endif /* INCLUDED_VECRENDER_LOCALTRIANGULATOR */
