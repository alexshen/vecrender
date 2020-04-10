#ifndef INCLUDED_VECRENDER_TRIANGULATIONUTILS
#define INCLUDED_VECRENDER_TRIANGULATIONUTILS

#include <glm/vec2.hpp>

namespace vecrender {
    
struct TriangulationUtils
{
    TriangulationUtils() = delete;

    template<typename Triangulation_2>
    static typename Triangulation_2::Point vec2ToPoint(const glm::vec2& p);

    template<typename Triangulation_2>
    static glm::vec2 pointToVec2(const typename Triangulation_2::Point& p);
};

// ============================================================================
// INLINE FUNCTION DEFINITIONS
// ============================================================================

template<typename Triangulation_2>
typename Triangulation_2::Point 
TriangulationUtils::vec2ToPoint(const glm::vec2& p)
{
    return { p.x, p.y };
}

template<typename Triangulation_2>
glm::vec2 
TriangulationUtils::pointToVec2(const typename Triangulation_2::Point& p)
{
    return { p.x(), p.y() };
}

} /* vecrender  */ 

#endif /* INCLUDED_VECRENDER_TRIANGULATIONUTILS */
