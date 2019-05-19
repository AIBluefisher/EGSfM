#include "union_find.h"

namespace GraphSfM {
namespace graph {

UnionFind::UnionFind(size_t n)
{
    this->Init(n);
}

void UnionFind::Init(size_t n)
{
    for (size_t i = 0; i < n; i++) {
        _parents.push_back(i);
        _ranks.push_back(0);
    }
}

size_t UnionFind::FindRoot(size_t x)
{
    return (_parents[x] == x) ? x : (_parents[x] = FindRoot(_parents[x]));
}

void UnionFind::Union(size_t x, size_t y)
{
    x = FindRoot(x);
    y = FindRoot(y);
    if (x == y) return;
    
    if (_ranks[x] < _ranks[y]) _parents[x] = y;
    else {
        _parents[y] = x;
        if (_ranks[x] == _ranks[y]) _ranks[x]++;
    }
}

std::vector<size_t> UnionFind::GetRanks() const
{
    return _ranks;
}

std::vector<size_t> UnionFind::GetParents() const
{
    return _parents;
}

} // namespace graph
} // namespace GraphSfM