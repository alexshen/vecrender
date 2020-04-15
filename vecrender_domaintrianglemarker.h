#ifndef INCLUDED_VECRENDER_DOMAINTRIANGLEMARKER
#define INCLUDED_VECRENDER_DOMAINTRIANGLEMARKER

#include <deque>
#include <queue>

namespace vecrender {

template <typename Triangulation> class DomainTriangleMarker
{
    using FaceHandle = typename Triangulation::Face_handle;
    using Edge = typename Triangulation::Edge;

public:
    // Mark domain triangles
    // MarkPolicy must have
    // 1. void setFaceNestingLevel(Face_handle, int level)
    // 2. int getFaceNestingLevel(Face_handle)
    // 3. bool isBorderEdge(Edge e)
    template <typename DomainPolicy>
    void markDomains(Triangulation& t, DomainPolicy& policy);

private:
    template <typename DomainPolicy>
    void markDomains(FaceHandle start, int level, DomainPolicy& policy);

    std::queue<FaceHandle> m_vistingFaces;
    std::deque<Edge> m_borders;
};

template <typename Triangulation>
template <typename DomainPolicy>
void DomainTriangleMarker<Triangulation>::markDomains(
    Triangulation& t, DomainPolicy& policy)
{
    for (FaceHandle f : t.all_face_handles()) {
        policy.setFaceNestingLevel(f, -1);
    }
    m_borders.clear();
    markDomains(t.infinite_face(), 0, policy);
    while (!m_borders.empty()) {
        Edge e = m_borders.front();
        m_borders.pop_front();
        FaceHandle f = e.first->neighbor(e.second);
        if (policy.getFaceNestingLevel(f) == -1) {
            markDomains(f, policy.getFaceNestingLevel(e.first) + 1, policy);
        }
    }
}

template <typename Triangulation>
template <typename DomainPolicy>
void DomainTriangleMarker<Triangulation>::markDomains(
    FaceHandle start, int level, DomainPolicy& policy)
{
    if (policy.getFaceNestingLevel(start) != -1) {
        return;
    }

    m_vistingFaces = {};
    m_vistingFaces.push(start);
    while (!m_vistingFaces.empty()) {
        FaceHandle curFace = m_vistingFaces.front();
        m_vistingFaces.pop();
        if (policy.getFaceNestingLevel(curFace) != -1) {
            continue;
        }
        policy.setFaceNestingLevel(curFace, level);
        for (int i = 0; i < 3; i++) {
            Edge e(curFace, i);
            FaceHandle adjFace = curFace->neighbor(i);
            if (policy.getFaceNestingLevel(adjFace) != -1) {
                continue;
            }
            if (policy.isBorderEdge(e)) {
                m_borders.push_back(e);
            } else {
                m_vistingFaces.push(adjFace);
            }
        }
    }
}

} // namespace vecrender

#endif /* INCLUDED_VECRENDER_DOMAINTRIANGLEMARKER */
