/*
	Dijkstra����Դ���·��

	ԭ��ÿ�ζ�ѡ�����������ĵ���и���

    function Dijkstra(Graph, source):
    create vertex set Q

    for each vertex v in Graph:
        dist[v] �� INFINITY
        prev[v] �� UNDEFINED
        add v to Q
    dist[source] �� 0

    while Q is not empty:
        u �� vertex in Q with min dist[u]
        remove u from Q

        for each neighbor v of u:
            alt �� dist[u] + length(u, v)
            if alt < dist[v]:
                dist[v] �� alt
                prev[v] �� u

    return dist[], prev[]

    ʹ�����ȼ����У�С���ѣ��Ż����ʱ�临�Ӷȣ�O(n*logn)
*/

#include <iostream>
#include <vector>
#include <queue>
using namespace std;

// ����������
using uint = unsigned int;
const uint INF = INT_MAX;

// �ڽӾ���(������ֱ��·����Ϊ0)
int adjMatrix[5][5] = {
    {0, 8, 5, 0, 0},
    {8, 0, 3, 1, 0},
    {5, 3, 0, 6, 9},
    {0, 1, 6, 0, 2},
    {0, 0, 9, 2, 0}
};

#if 0
// �㷨ʵ�֣���� -> �յ㣩
int Dijkstra(vector<vector<uint>>& graph, int start, int end)
{
    const int N = graph.size(); // 6

    // ���������������·��
    vector<uint> dis(N, 0); // ��ʼȫΪ0
    vector<bool> use(N, false); // false��ʾδ�����ʹ�

    // ��start����S����
    use[start] = true;
    // ��ʼ��start��U���������������Ȩֵ
    for(int i = 0; i < N; i++)
    {
        dis[i] = graph[start][i];
    }

    // ����U�����еĶ���
    for (int i = 1;i < N;i++)
    {
        // Ѱ��U������Ȩֵ��С�ĵ�
        int k = -1;
        int min = INF;
        for (int j = 0;j < N;j++)
        {
            if (!use[j] && min > dis[j]) // U�����еĵ�
            {
                min = dis[j];
                k = j;
            }
        }

        if (k == -1)
        {
            break;
        }

        // ��ѡ���Ķ������S������
        use[k] = true;

        // ����U������ʣ��ڵ��Ȩֵ
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

// ʹ��С���ѵ��㷨ʵ�֣���� -> �յ㣩
int Dijkstra(vector<vector<uint>>& graph, int start, int end)
{
    const int N = graph.size(); // 6

    // ���������������·��
    vector<uint> dis(N, 0); // ��ʼȫΪ0
    vector<bool> use(N, false); // false��ʾδ�����ʹ�

    // ����С����
    priority_queue<pair<uint, int>, vector<pair<uint, int>>, greater<pair<uint, int>>> que;    // pair<Ȩֵ, ���>

    // ��start����S����
    use[start] = true;
    // ��ʼ��start��U���������������Ȩֵ
    for (int i = 0; i < N; i++)
    {
        dis[i] = graph[start][i];
        // �ѳ�start�������������ȫ������U����С������
        if (i != start)
        {
            que.emplace(graph[start][i], i);
        }
    }

    // ����U�����еĶ���
    while(!que.empty())
    {
        // Ѱ��U������Ȩֵ��С�ĵ�
        auto pair = que.top();
        que.pop();
        if (pair.first == INF)  // ��ζ��ʣ��Ķ��㶼�޷�����
        {
            break;
        }
        int k = pair.second;
        int min = pair.first;
        
        if (use[k])
        {
            continue;
        }

        // ��ѡ���Ķ������S������
        use[k] = true;

        // ����U������ʣ��ڵ��Ȩֵ
        for (int j = 0;j < N;j++)
        {
            if (!use[j] && min + graph[k][j] < dis[j])
            {
                dis[j] = min + graph[k][j];
                // ����U�����ж����Ȩֵ
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