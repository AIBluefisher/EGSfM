#ifndef GRAPH_UNION_FIND_H
#define GRAPH_UNION_FIND_H

#include <vector>
#include <iostream>

namespace GraphSfM {
namespace graph {

class UnionFind 
{
private:
    std::vector<size_t> _ranks;
    std::vector<size_t> _parents;

public:
    // constructor
    UnionFind() {}
    UnionFind(size_t n);

    // union find operations
    void Init(size_t n);
    size_t FindRoot(size_t x);
    void Union(size_t x, size_t y);

    // get functions
    std::vector<size_t> GetRanks() const;
    std::vector<size_t> GetParents() const;
};

} // namespace graph
} // namespace GraphSfM

#endif