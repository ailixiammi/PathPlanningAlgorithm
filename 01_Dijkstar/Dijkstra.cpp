/*
	Dijkstra：求单源最短路径

	原理：每次都选距离起点最近的点进行更新

    function Dijkstra(Graph, source):
    create vertex set Q

    for each vertex v in Graph:
        dist[v] ← INFINITY
        prev[v] ← UNDEFINED
        add v to Q
    dist[source] ← 0

    while Q is not empty:
        u ← vertex in Q with min dist[u]
        remove u from Q

        for each neighbor v of u:
            alt ← dist[u] + length(u, v)
            if alt < dist[v]:
                dist[v] ← alt
                prev[v] ← u

    return dist[], prev[]

    使用优先级队列（小根堆）优化后的时间复杂度：O(n*logn)
*/

#include <iostream>
#include <vector>
#include <queue>
using namespace std;

// 定义正无穷
using uint = unsigned int;
const uint INF = INT_MAX;

// 邻接矩阵(不存在直接路径记为0)
int adjMatrix[5][5] = {
    {0, 8, 5, 0, 0},
    {8, 0, 3, 1, 0},
    {5, 3, 0, 6, 9},
    {0, 1, 6, 0, 2},
    {0, 0, 9, 2, 0}
};

#if 0
// 算法实现（起点 -> 终点）
int Dijkstra(vector<vector<uint>>& graph, int start, int end)
{
    const int N = graph.size(); // 6

    // 储存各个顶点的最短路径
    vector<uint> dis(N, 0); // 初始全为0
    vector<bool> use(N, false); // false表示未被访问过

    // 把start放入S集合
    use[start] = true;
    // 初始化start到U集合中其他顶点的权值
    for(int i = 0; i < N; i++)
    {
        dis[i] = graph[start][i];
    }

    // 处理U集合中的顶点
    for (int i = 1;i < N;i++)
    {
        // 寻找U集合中权值最小的点
        int k = -1;
        int min = INF;
        for (int j = 0;j < N;j++)
        {
            if (!use[j] && min > dis[j]) // U集合中的点
            {
                min = dis[j];
                k = j;
            }
        }

        if (k == -1)
        {
            break;
        }

        // 把选出的顶点加入S集合中
        use[k] = true;

        // 更新U集合中剩余节点的权值
        for (int j = 0;j < N;j++)
        {
            if (!use[j] && min + graph[k][j] < dis[j])
            {
                dis[j] = min + graph[k][j];
            }
        }
    }

    for (int d : dis)
    {
        cout << d << " ";
    }
    cout << endl;

    return dis[end];
}
#endif

// 使用小根堆的算法实现（起点 -> 终点）
int Dijkstra(vector<vector<uint>>& graph, int start, int end)
{
    const int N = graph.size(); // 6

    // 储存各个顶点的最短路径
    vector<uint> dis(N, 0); // 初始全为0
    vector<bool> use(N, false); // false表示未被访问过

    // 定义小根堆
    priority_queue<pair<uint, int>, vector<pair<uint, int>>, greater<pair<uint, int>>> que;    // pair<权值, 编号>

    // 把start放入S集合
    use[start] = true;
    // 初始化start到U集合中其他顶点的权值
    for (int i = 0; i < N; i++)
    {
        dis[i] = graph[start][i];
        // 把除start顶点的其他顶点全部放入U集合小根堆中
        if (i != start)
        {
            que.emplace(graph[start][i], i);
        }
    }

    // 处理U集合中的顶点
    while(!que.empty())
    {
        // 寻找U集合中权值最小的点
        auto pair = que.top();
        que.pop();
        if (pair.first == INF)  // 意味着剩余的顶点都无法到达
        {
            break;
        }
        int k = pair.second;
        int min = pair.first;
        
        if (use[k])
        {
            continue;
        }

        // 把选出的顶点加入S集合中
        use[k] = true;

        // 更新U集合中剩余节点的权值
        for (int j = 0;j < N;j++)
        {
            if (!use[j] && min + graph[k][j] < dis[j])
            {
                dis[j] = min + graph[k][j];
                // 更新U集合中顶点的权值
                que.emplace(dis[j], j);
            }
        }
    }

    for (int d : dis)
    {
        cout << d << " ";
    }
    cout << endl;

    return dis[end];
}

int main()
{
    vector<vector<uint>> graph = {
        {0, 6, 3, INF, INF, INF},
        {6, 0, 2, 5, INF, INF},
        {3, 2, 0, 3, 4, INF},
        {INF, 5, 3, 0, 2, 3},
        {INF, INF, 4, 2, 0, 5},
        {INF, INF, INF, 3, 5, 0}
    };

    int distance = Dijkstra(graph, 0, 3);

    cout << distance << endl;

    return 0;
}