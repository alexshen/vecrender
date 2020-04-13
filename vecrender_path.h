#ifndef INCLUDED_VECRENDER_PATH
#define INCLUDED_VECRENDER_PATH

#include <glm/vec2.hpp>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>

namespace vecrender {

class Path;
class Path_ElementIterator;

class PathElement {
public:
    enum Enum : char {
        e_LINE,
        e_QUADRATIC,
        e_CUBIC,
        e_MOVE, // for internal use
        e_INVALID
    };

    static std::size_t getPointNumForElementType(PathElement::Enum elemType);

    PathElement(const PathElement&) = default;
    PathElement& operator =(const PathElement&) = default;

    PathElement::Enum getType() const;
    std::size_t getNumPoints() const;
    glm::vec2 getPoint(std::size_t index) const;
    const glm::vec2* getPoints() const;
private:
    friend class Path_ElementIterator;
    PathElement();

    const glm::vec2* m_points;
    PathElement::Enum m_elemType;
};

class Path_ElementIterator {
public:
    using value_type = PathElement;
    using reference = const value_type&;
    using pointer = const value_type*;
    using different_type = std::ptrdiff_t;
    using iterator_category = std::input_iterator_tag;

    Path_ElementIterator(const Path_ElementIterator&) = default;
    Path_ElementIterator& operator =(const Path_ElementIterator&) = default;
    
    reference operator *() const;
    pointer operator ->() const;
    Path_ElementIterator& operator ++();
    Path_ElementIterator operator ++(int);

    bool equal(const Path_ElementIterator& rhs) const;
private:
    friend class Path;
    Path_ElementIterator(const Path& path, bool end);

    const Path* m_path;
    std::vector<PathElement::Enum>::const_iterator m_itElem;

    PathElement m_curElem;
};

bool operator ==(const Path_ElementIterator& lhs, 
                 const Path_ElementIterator& rhs);

bool operator !=(const Path_ElementIterator& lhs, 
                 const Path_ElementIterator& rhs);

class Path {
public:
    Path();

    Path(const Path&) = default;
    Path(Path&& rhs) noexcept;

    Path& operator =(const Path&) = default;
    Path& operator =(Path&& rhs) noexcept;

    void moveTo(glm::vec2 p);
    void lineTo(glm::vec2 p);
    void quadraticTo(glm::vec2 p1, glm::vec2 p2);
    void cubicTo(glm::vec2 p1, glm::vec2 p2, glm::vec2 p3);
    void close();

    Path_ElementIterator elementBegin() const;
    Path_ElementIterator elementEnd() const;
private:
    friend class Path_ElementIterator;

    std::vector<glm::vec2> m_points;
    std::vector<PathElement::Enum> m_elemTypes;
    int m_curSubPathLen;
};
    
// ============================================================================
// INLINE FUNCTION DEFINITIONS
// ============================================================================

// PathElement

inline PathElement::Enum PathElement::getType() const
{
    return m_elemType;
}

inline std::size_t PathElement::getNumPoints() const
{
    return getPointNumForElementType(m_elemType);
}

inline glm::vec2 PathElement::getPoint(std::size_t index) const
{
    assert(index < getNumPoints());
    return m_points[index];
}

inline const glm::vec2* PathElement::getPoints() const
{
    return m_points;
}

// Path_ElementIterator

inline Path_ElementIterator::reference Path_ElementIterator::operator *() const
{
    assert(m_curElem.getType() != PathElement::e_INVALID);
    return m_curElem;
}

inline Path_ElementIterator::pointer Path_ElementIterator::operator ->() const
{
    assert(m_curElem.getType() != PathElement::e_INVALID);
    return &m_curElem;
}

inline Path_ElementIterator Path_ElementIterator::operator ++(int)
{
    Path_ElementIterator tmp(*this);
    ++*this;
    return tmp;
}

inline bool Path_ElementIterator::equal(const Path_ElementIterator& rhs) const
{
    return m_itElem == rhs.m_itElem;
}

} /* vecrender  */ 

// FREE OPERATORS

inline bool vecrender::operator ==(const Path_ElementIterator& lhs, 
                                   const Path_ElementIterator& rhs)
{
    return lhs.equal(rhs);
}

inline bool vecrender::operator !=(const Path_ElementIterator& lhs, 
                                   const Path_ElementIterator& rhs)
{
    return !(lhs == rhs);
}

#endif /* ifndef INCLUDED_VECRENDER_PATH */

// vim:set et sw=4 ts=4 tw=79:
