#ifndef INCLUDED_VECRENDER_MATHUTILS
#define INCLUDED_VECRENDER_MATHUTILS

#include <glm/glm.hpp>
#include <algorithm>
#include <limits>
#include <utility>
#include <type_traits>
#include <cassert>
#include <cmath>

namespace vecrender {

struct MathUtils {
    MathUtils() = delete;
    
    // Split the cubic bezier curve at the give point
    // The length of newPoints must be at least 7
    template<typename T>
    static void splitCubicBezier(const T* points, float t, T* newPoints);
    
    // Split the quadratic bezier curve at the given point
    // The length of newPoints must be at least 5
    template<typename T>
    static void splitQuadraticBezier(const T* points, float t, T* newPoints);
    
    // Split the quadratic bezier curve at the given point
    // The length of newPoints must be at least 3
    template<typename T>
    static void splitLineSegment(const T* points, float t, T* newPoints);

    template<typename T>
    static bool almostEqual(T a, T b, T absDiff = T(1e-6), 
                            T relDiff = std::numeric_limits<T>::epsilon());

    // Compute the barycentric coordinates for the given point
    // relative the triangle.
    // If the triangle is a degenerate segment, p is projected onto the
    // segment, then the coordinates are computed.
    // If the triangle is a degenerate point, the coordinates are all 0.
    static glm::vec3 computeBarycentricCoords(const glm::vec2* points,
                                              const glm::vec2& p);

    static float determinant(const glm::vec2& d0, const glm::vec2& d1);

    // Return the two solutions to the quadratic equation
    // If there's no solutions, NaNs are returned
    template<typename T>
    static std::pair<T, T> solveQuadratic(T a, T b, T c);
};

// INLINE FUNCTIONS

template<typename T>
void MathUtils::splitCubicBezier(const T* points, float t, T* newPoints)
{
    assert(points && newPoints);
    
    newPoints[0] = points[0];
    newPoints[6] = points[3];
    newPoints[1] = glm::mix(points[0], points[1], t);
    newPoints[5] = glm::mix(points[2], points[3], t);
    auto p11 = glm::mix(points[1], points[2], t);
    newPoints[2] = glm::mix(newPoints[1], p11, t);
    newPoints[4] = glm::mix(p11, newPoints[5], t);
    newPoints[3] = glm::mix(newPoints[2], newPoints[4], t);
}

template<typename T>
void MathUtils::splitQuadraticBezier(const T* points, float t, T* newPoints)
{
    assert(points && newPoints);
    
    newPoints[0] = points[0];
    newPoints[4] = points[2];
    newPoints[1] = glm::mix(points[0], points[1], t);
    newPoints[3] = glm::mix(points[1], points[2], t);
    newPoints[2] = glm::mix(points[1], points[3], t);
}

template<typename T>
void MathUtils::splitLineSegment(const T* points, float t, T* newPoints)
{
    assert(points && newPoints);
    
    newPoints[0] = points[0];
    newPoints[1] = glm::mix(points[0], points[1], t);
    newPoints[2] = points[1];
}

template<typename T>
bool MathUtils::almostEqual(T a, T b, T absDiff, T relDiff)
{
    static_assert(std::is_floating_point_v<T>, 
                  "T must be of floating point type");
    assert(absDiff >= 0 && relDiff >= 0);

    T diff = std::abs(a - b);
    if (diff <= absDiff) {
        return true;
    }

    T absA = std::abs(a);
    T absB = std::abs(b);
    return diff <= std::max(absA, absB) * relDiff;
}

inline float MathUtils::determinant(const glm::vec2& d0, const glm::vec2& d1)
{
    return glm::dot(glm::vec2(-d0.y, d0.x), d1);
}
    
template<typename T>
std::pair<T, T> MathUtils::solveQuadratic(T a, T b, T c)
{
    static_assert(std::is_floating_point_v<T>, 
                  "T must be of floating point type");

    T q = -(b + std::copysign(T(1.0), b) * std::sqrt(b * b - T(4.0) * a * c));
    return { q / a, c / q };
}

} // namespace vecrender

#endif /* INCLUDED_VECRENDER_MATHUTILS */
