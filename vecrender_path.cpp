#include <vecrender_path.h>
#include <utility>

namespace vecrender {

// PathElement

std::size_t PathElement::getPointNumForElementType(PathElement::Enum elemType)
{
    switch (elemType) {
    case e_LINE:
        return 2;
    case e_QUADRATIC:
        return 3;
    case e_CUBIC:
        return 4;
    case e_MOVE:
        return 1;
    default:
        assert(0);
        return -1;
    }
}

PathElement::PathElement()
    : m_points(nullptr)
    , m_elemType(PathElement::e_INVALID)
{
}

// Path_ElementIterator

Path_ElementIterator::Path_ElementIterator(const Path& path, bool end)
    : m_path(&path)
{
    if (!end && path.m_elemTypes.size() > 1) {
        m_curElem.m_points = path.m_points.data();
        m_curElem.m_elemType = path.m_elemTypes[1]; // skip the e_MOVE
        m_itElem = path.m_elemTypes.begin() + 1;
    } else {
        m_itElem = path.m_elemTypes.end();
    }
}

Path_ElementIterator& Path_ElementIterator::operator ++()
{
    assert(m_curElem.getType() != PathElement::e_INVALID);
    m_curElem.m_points += m_curElem.numPoints() - 1;
    ++m_itElem;
    if (m_itElem != m_path->m_elemTypes.end() && 
        *m_itElem == PathElement::e_MOVE) {
        ++m_itElem;
    }
    if (m_itElem != m_path->m_elemTypes.end()) {
        m_curElem.m_elemType = *m_itElem;
    } else {
        m_curElem.m_elemType = PathElement::e_INVALID;
    }
    return *this;
}

// Path

Path::Path()
    : m_points()
    , m_elemTypes()
    , m_curSubPathLen(0)
{
}

Path::Path(Path&& rhs)
    : m_points(std::move(rhs.m_points))
    , m_elemTypes(std::move(rhs.m_elemTypes))
    , m_curSubPathLen(std::exchange(rhs.m_curSubPathLen, 0))
{
}

Path& Path::operator =(Path&& rhs)
{
    if (this != &rhs) {
        m_points = std::move(rhs.m_points);
        m_elemTypes = std::move(rhs.m_elemTypes);
        m_curSubPathLen = std::exchange(rhs.m_curSubPathLen, 0);
    }
    return *this;
}

void Path::moveTo(glm::vec2 p)
{
    if (m_curSubPathLen != 1) {
        close();
        m_points.push_back(p);
        m_elemTypes.push_back(PathElement::e_MOVE);
        m_curSubPathLen = 1;
    } else {
        m_points.back() = p;
    }
}

void Path::lineTo(glm::vec2 p)
{
    assert(m_curSubPathLen);
    m_points.push_back(p);
    m_elemTypes.push_back(PathElement::e_LINE);
}

void Path::quadraticTo(glm::vec2 p1, glm::vec2 p2)
{
    assert(m_curSubPathLen);
    m_points.push_back(p1);
    m_points.push_back(p2);
    m_elemTypes.push_back(PathElement::e_QUADRATIC);
}

void Path::cubicTo(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3)
{
    assert(m_curSubPathLen);
    m_points.push_back(p1);
    m_points.push_back(p2);
    m_points.push_back(p3);
    m_elemTypes.push_back(PathElement::e_CUBIC);
}

void Path::close()
{
    if (m_curSubPathLen > 1) {
        const auto& firstPoint = m_points[m_points.size() - m_curSubPathLen];
        if (m_points.back() != firstPoint) {
            lineTo(firstPoint);
        }
        m_curSubPathLen = 0;
    } else if (m_curSubPathLen == 1) {
        m_points.pop_back();
        m_elemTypes.pop_back();
        m_curSubPathLen = 0;
    }
}

Path_ElementIterator Path::elementBegin() const
{
    return Path_ElementIterator(*this, false);
}

Path_ElementIterator Path::elementEnd() const
{
    return Path_ElementIterator(*this, true);
}

} // namespace vecrender
