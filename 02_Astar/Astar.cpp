/*
    Astar算法是一种启发式搜索算法，本程序选用曼哈顿距离作为启发式函数
    打印从起点到终点的最短路径和该路径的权值
*/
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
using namespace std;
using uint = unsigned int;
const uint INF = UINT_MAX;

// 路径打印函数
void printPath(int end, const vector<int>& prev)
{
    if (end == -1)
    {
        return;
    }

    printPath(prev[end], prev);

    if (prev[end] != -1)
    {
        cout << "->";
    }
    cout << end;
}

// 曼哈顿距离启发函数
int manhattanHeuristic(int current, int target, const vector<pair<int, int>>& coord)
{
    return abs(coord[current].first - coord[target].first) +
        abs(coord[current].second - coord[target].second);
}

void aStar(const vector<vector<uint>>& graph,
    int start,
    int target,
    const vector<pair<int, int>>& coord)
{
    const int n = graph.size();
    vector<uint> g(n, INF);
    vector<int> prev(n, -1);

    // 定义优先队列元素类型
    priority_queue<pair<uint, int>, vector<pair<uint, int>>, greater<pair<uint, int>>> openSet;

    // 初始化起点
    g[start] = 0;
    uint startF = g[start] + manhattanHeuristic(start, target, coord);
    openSet.push(make_pair(startF, start));

    while (!openSet.empty())
    {
        // 手动解包优先队列元素
        pair<uint, int> topElement = openSet.top();
        uint currentF = topElement.first;
        int u = topElement.second;
        openSet.pop();

        if (u == target)
        {
            break;
        }

        uint currentG = currentF - manhattanHeuristic(u, target, coord);
        if (currentG > g[u])
        {
            continue;
        }

        for (int v = 0; v < n; ++v)
        {
            if (u == v || graph[u][v] == INF)
            {
                continue;
            }

            uint tentativeG = g[u] + graph[u][v];
            if (tentativeG < g[v])
            {
                g[v] = tentativeG;
                prev[v] = u;
                uint newF = tentativeG + manhattanHeuristic(v, target, coord);
                openSet.push(make_pair(newF, v));
            }
        }
    }

    if (g[target] == INF)
    {
        cout << "顶点 " << start << " 到 " << target << " 不可达\n";
    }
    else
    {
        cout << "最短路径: ";
        printPath(target, prev);
        cout << "\n总权值: " << g[target] << endl;
    }
}

int main()
{
    vector<vector<uint>> graph = {
        {0,   2, INF,   4, INF},
        {2,   0,   3,   6, INF},
        {INF, 3,   0, INF,   2},
        {4,   6, INF,   0,   5},
        {INF,INF,  2,   5,   0}
    };

    vector<pair<int, int>> coordinates = {
        {1, 4}, {3, 4}, {4, 3},
        {2, 1}, {4, 1}
    };

    aStar(graph, 0, 4, coordinates);

    return 0;
}