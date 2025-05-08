/*
    Dijkstra算法会计算从起点到任意顶点的最短路径
    本程序会将从起点到所有顶点的最短路径打印出来
*/
#include <iostream>
#include <vector>
#include <queue>
#include <climits>
using namespace std;

const unsigned int INF = UINT_MAX;

// 路径打印函数
void printPath(int v, const vector<int>& prev)
{
    if (v == -1)
    {
        return;
    }
    printPath(prev[v], prev);
    if (prev[v] != -1)
    {
        cout << "->";
    }
    cout << v;
}

void dijkstra(const vector<vector<unsigned int>>& graph, int start)
{
    int n = graph.size();
    vector<unsigned int> dist(n, INF);
    vector<int> prev(n, -1);

    // 定义优先队列类型
    typedef pair<unsigned int, int> pq_element;
    priority_queue<pq_element, vector<pq_element>, greater<pq_element>> pq;

    dist[start] = 0;
    pq.push(make_pair(0, start));

    while (!pq.empty())
    {
        // 手动解包pair
        unsigned int current_dist = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (current_dist > dist[u])
        {
            continue;
        }

        for (int v = 0; v < n; ++v)
        {
            if (graph[u][v] == INF || u == v)
            {
                continue;
            }

            unsigned int new_dist = current_dist + graph[u][v];

            if (new_dist < dist[v])
            {
                dist[v] = new_dist;
                prev[v] = u;
                pq.push(make_pair(new_dist, v));
            }
        }
    }

    // 输出结果
    cout << "起点: " << start << endl;
    for (int i = 0; i < n; ++i)
    {
        if (i == start)
        {
            continue;
        }

        cout << "到顶点 " << i << " 的最短路径: ";
        if (dist[i] == INF)
        {
            cout << "不可达";
        }
        else
        {
            printPath(i, prev);
            cout << "，总权值: " << dist[i];
        }
        cout << endl;
    }
}

int main()
{
    vector<vector<unsigned int>> graph =
    {
        {0,   6,   3, INF, INF, INF},
        {6,   0,   2,   5, INF, INF},
        {3,   2,   0,   3,   4, INF},
        {INF, 5,   3,   0,   2,   3},
        {INF, INF, 4,   2,   0,   5},
        {INF, INF, INF, 3,   5,   0}
    };

    dijkstra(graph, 0);

    return 0;
}