#ifndef UNDIRECTEDGRAPH
#define UNDIRECTEDGRAPH

#include <vector>

struct UndirectedGraph
{
    UndirectedGraph(int N);
    void addEdge(int from, int to);
    std::vector<int> victinity(int u);
    static void intersect(const std::vector<int> &A, std::vector<int> &B);

    std::vector<std::vector<int>> maximalCliques();
    std::vector<int> getConnectedComponent(int v);

    std::vector<int> first, next, dst;
    std::vector<std::vector<int>> cliques;
    std::vector<int> used;

private:
    void BK(const std::vector<int> R, const std::vector<int> P, const std::vector<int> X);
    void addDGEdge(int from, int to);
};

#endif
