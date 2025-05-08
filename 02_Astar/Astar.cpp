/*
    Astar�㷨��һ������ʽ�����㷨��������ѡ�������پ�����Ϊ����ʽ����
    ��ӡ����㵽�յ�����·���͸�·����Ȩֵ
*/
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
using namespace std;
using uint = unsigned int;
const uint INF = UINT_MAX;

// ·����ӡ����
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

// �����پ�����������
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

    // �������ȶ���Ԫ������
    priority_queue<pair<uint, int>, vector<pair<uint, int>>, greater<pair<uint, int>>> openSet;

    // ��ʼ�����
    g[start] = 0;
    uint startF = g[start] + manhattanHeuristic(start, target, coord);
    openSet.push(make_pair(startF, start));

    while (!openSet.empty())
    {
        // �ֶ�������ȶ���Ԫ��
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
        cout << "���� " << start << " �� " << target << " ���ɴ�\n";
    }
    else
    {
        cout << "���·��: ";
        printPath(target, prev);
        cout << "\n��Ȩֵ: " << g[target] << endl;
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