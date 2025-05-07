/*	�ֱ�ʹ���ڽӾ�����ڽӱ���ͼ	*/
#include <iostream>
#include <vector>
#include <list>
using namespace std;

// �ڽӾ���
class AdjMatrix
{
private:
	int vertices;	// ͼ�Ķ�������
	vector<vector<int>> adjMatrix;	// �ڽӾ���

public:
	// ʹ�ù��캯����ʼ���ڽӾ���Ĵ�С
	AdjMatrix(int v)
	{
		vertices = v;
		adjMatrix.resize(vertices, vector<int>(vertices, 0));
	}

	// ��ӱߣ���㡢�յ㡢Ȩ�أ�
	void addEdge(int src, int dest, int weight)
	{
		if (src >= 0 && src < vertices && dest >= 0 && dest < vertices && weight >= 0)
		{
			// ����ͼ
			adjMatrix[src][dest] = weight;
			//adjMatrix[dest][src] = weight;
		}
		else
		{
			cout << "��������" << endl;
		}
	}

	// ��ӡ�ڽӾ���
	void printAdjMatrix()
	{
		cout << "�ڽӾ���" << endl;
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

// �ڽӱ�
class AdjList
{
private:
	struct Edge
	{
		int dest;	// �յ�
		int weight;	// Ȩ��

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

	// ��ӱ�
	void addEdge(int src, int dest, int weight)
	{
		if (src >= 0 && src < vectices && dest >= 0 && dest < vectices && weight >= 0)
		{
			// ����ͼ
			adjList[src].push_back(Edge(dest, weight));
			//adjList[dest].push_back(Edge(src, weight));
		}
		else
		{
			cout << "��������" << endl;
		}
	}

	void printList()
	{
		for (int i = 0;i < vectices;i++)
		{
			cout << "�ڵ�" << i << "�ıߣ�";
			for (auto& edge : adjList[i])
			{
				cout << "->[" << edge.dest << "](Ȩ��:" << edge.weight << ")";
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