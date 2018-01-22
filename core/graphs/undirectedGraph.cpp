#include "core/graphs/undirectedGraph.h"

#include <algorithm>
#include <queue>

UndirectedGraph::UndirectedGraph(int N) : first(N + 1), next(1), dst(1)
{
}

void UndirectedGraph::addDGEdge(int from, int to)
{
    dst.push_back(to);
    next.push_back(first[from]);
    first[from] = (int)next.size() - 1;
}

void UndirectedGraph::addEdge(int from, int to)
{
    addDGEdge(from, to);
    addDGEdge(to, from);
}

std::vector<int> UndirectedGraph::victinity(int u)
{
    std::vector<int> ret;
    int curr = first[u];
    if (curr)
    {
        do
        {
            ret.push_back(dst[curr]);
            curr = next[curr];
        } while(curr);
    }
    std::sort(ret.begin(), ret.end());
    return ret;
}

std::vector<int> UndirectedGraph::getConnectedComponent(int v)
{
    std::vector<int> cc;
    std::queue<int> q;
    used[v] = 1;
    q.push(v);

    while(q.size())
    {
        int u = q.front(); q.pop();

        cc.push_back(u);
        u = first[u];
        while(u)
        {
            if (!used[dst[u]])
            {
                used[dst[u]] = 1;
                q.push(dst[u]);
            }
            u = next[u];
        }

    }
    return cc;
}

void UndirectedGraph::intersect(const std::vector<int> &a, std::vector<int> &b)
{
    int N = (int)a.size(), ptrA = 0, ptrB = 0, M = (int)b.size(), ptrRes = 0;
    for (; ptrA < N && ptrB < M;)
    {
        if (a[ptrA] == b[ptrB])
        {
            b[ptrRes++] = b[ptrB++];
            ptrA++;
            continue;
        }
        if (a[ptrA] < b[ptrB])
        {
            ptrA++;
            continue;
        }
        ptrB++;
    }
    b.resize(ptrRes);
}

std::vector<std::vector<int>> UndirectedGraph::maximalCliques()
{
    used.clear();
    used.resize(first.size());
    cliques.clear();
    for (size_t i = 1; i < first.size(); ++i)
        if (!used[i])
        {
            auto P = getConnectedComponent((int)i);
            if (P.size() < 2)
                continue;
            std::vector<int> R, X;
            BK(R, P, X);
        }
    return cliques;
}

void UndirectedGraph::BK(const std::vector<int> R, const std::vector<int> Pp, const std::vector<int> Xx)
{
    if (!Pp.size() && !Xx.size())
    {
        cliques.push_back(R);
        return;
    }
    std::vector<int> X = Xx;
    std::vector<int> P = Pp;
    for (size_t i = 0; i < Pp.size(); ++i)
    {
        int v = *P.rbegin();
        std::vector<int> V = victinity(v);
        std::vector<int> PP = V, XP = V, RP = R;
        intersect(P, PP);
        intersect(X, XP);
        RP.push_back(v);
        std::sort(RP.begin(), RP.end());
        BK(RP, PP, XP);
        P.resize(P.size() - 1);
        X.push_back(v);
        sort(X.begin(), X.end());
    }
}

