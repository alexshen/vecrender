#include <vecrender_mathutils.h>
#include <cassert>

namespace vecrender {

namespace {

int prevVertex(int i) {
    assert(0 <= i && i < 3);
    return (i + 2) % 3;
}

int nextVertex(int i) {
    assert(0 <= i && i < 3);
    return (i + 1) % 3;
}

} // anonymous namspace

glm::vec3 MathUtils::computeBarycentricCoords(const glm::vec2* points,
                                              const glm::vec2& p)
{
    assert(points);

    glm::vec3 coords;
    glm::vec2 dirs[3];
    dirs[0] = points[1] - points[0];
    dirs[1] = points[2] - points[1]; 

    float denom = determinant(dirs[0], dirs[1]);
    if (almostEqual(denom, 0.0f)) {
        // degenerate triangle
        // find the edge with the maximum length
        dirs[2] = points[0] - points[2];
        int maxEdge = 0;
        float maxSqrLen = 0;
        for (int i = 0; i < 3; ++i) {
            float sqr = glm::dot(dirs[i], dirs[i]);
            if (sqr > maxSqrLen) {
                maxSqrLen = sqr;
                maxEdge = i;
            }
        }
        float len = std::sqrt(maxSqrLen);
        // triangle is actually a point
        if (almostEqual(len, 0.0f)) {
            coords = {};
        } else {
            coords[prevVertex(maxEdge)] = 0;
            int nextVert = nextVertex(maxEdge);
            coords[nextVert] = glm::dot(p - points[maxEdge], dirs[maxEdge]);
            coords[maxEdge] = 1.0f - coords[nextVert];
        }
    } else {
        denom = 1.0f / denom;
        coords.x = determinant(dirs[1], p - points[1]) * denom;
        coords.z = determinant(dirs[0], p - points[0]) * denom;
        coords.x = 1.0f - coords.x - coords.z;
    }
    return coords;
}

} /* vecrender  */ 
