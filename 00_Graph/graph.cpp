/*	分别使用邻接矩阵和邻接表储存图	*/
#include <iostream>
#include <vector>
#include <list>
using namespace std;

// 邻接矩阵
class AdjMatrix
{
private:
	int vertices;	// 图的顶点数量
	vector<vector<int>> adjMatrix;	// 邻接矩阵

public:
	// 使用构造函数初始化邻接矩阵的大小
	AdjMatrix(int v)
	{
		vertices = v;
		adjMatrix.resize(vertices, vector<int>(vertices, 0));
	}

	// 添加边（起点、终点、权重）
	void addEdge(int src, int dest, int weight)
	{
		if (src >= 0 && src < vertices && dest >= 0 && dest < vertices && weight >= 0)
		{
			// 有向图
			adjMatrix[src][dest] = weight;
			//adjMatrix[dest][src] = weight;
		}
		else
		{
			cout << "输入有误！" << endl;
		}
	}

	// 打印邻接矩阵
	void printAdjMatrix()
	{
		cout << "邻接矩阵：" << endl;
		for (auto& row : adjMatrix)
		{
			for (auto& val : row)
			{
				cout << val << " ";
			}
			cout << endl;
		}
	}
};

// 邻接表
class AdjList
{
private:
	struct Edge
	{
		int dest;	// 终点
		int weight;	// 权重

		Edge(int d, int w)
		{
			dest = d;
			weight = w;
		}
	};

	int vectices;
	vector<list<Edge>> adjList;

public:
	AdjList(int v)
	{
		vectices = v;
		adjList.resize(v);
	}

	// 添加边
	void addEdge(int src, int dest, int weight)
	{
		if (src >= 0 && src < vectices && dest >= 0 && dest < vectices && weight >= 0)
		{
			// 有向图
			adjList[src].push_back(Edge(dest, weight));
			//adjList[dest].push_back(Edge(src, weight));
		}
		else
		{
			cout << "输入有误！" << endl;
		}
	}

	void printList()
	{
		for (int i = 0;i < vectices;i++)
		{
			cout << "节点" << i << "的边：";
			for (auto& edge : adjList[i])
			{
				cout << "->[" << edge.dest << "](权重:" << edge.weight << ")";
			}
			cout << endl;
		}
	}
};

int main()
{
	AdjList adjlist(3);

	adjlist.addEdge(1, 3, 1);

	adjlist.printList();

	return 0;
}