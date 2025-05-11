/*
	����·��ͼ�㷨
	��ײ��⣺ʹ���߶ε�Բ�ľ�����
	�������ӣ�K�����㷨
	·��������A*�㷨�Ż�
	����������ŷ����þ���

	���ϰ���̫����߲�����̫��ʱ�������Ҳ���·��
	���ڲ����������ԣ�ͬ���Ĳ�������ͬ���ִεĽ��Ҳ��һ��
*/

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <algorithm>
using namespace std;

const double INF = 1e9;
const int NUM_SAMPLES = 100;	// ����������
const int K_NEIGHBORS = 5;		// ÿ������ھ�����
const double MAX_DIST = 2.0;	// ������Ӿ���

// ��ά��ṹ
struct Point
{
	// �������
	double x, y;
	// ���캯����ʼ��
	Point(double x_, double y_) :x(x_), y(y_) {}
	// ����õ㵽��p�ľ���
	double distanceTo(const Point& p) const
	{
		return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
	}
};

// �ϰ���ṹ(��άԲ���ϰ���)
struct Obstacle
{
	// Բ��
	Point center;
	// �뾶
	double radius;
	// ��ʼ��
	Obstacle(Point c_, double r_) :center(c_), radius(r_) {}
	// �жϵ�p�Ƿ����ϰ���ķ�Χ��
	bool contains(const Point& p) const
	{
		// �Ƿ���true���񷵻�false
		return center.distanceTo(p) < radius;
	}
};

// PRM·��ͼ�ڵ�
struct Node
{
	// �ڵ�λ��
	Point pos;
	// �ھӽڵ������
	vector<int> neighbors;
	// ��ʼ��
	Node(Point p) :pos(p) {}
};

class PRM
{
private:
	// �������нڵ�
	vector<Node> nodes;
	// �洢�����е������ϰ���
	vector<Obstacle> obstacles;

	// ��ײ���(��������ao������ab�ϵ�ͶӰ������ȡ�߶�ab��Բ������ĵ�c��ͨ��co��뾶r�Ƚ����ж��Ƿ���ײ)
	bool isCollision(const Point& a, const Point& b) 
	{
		for (const Obstacle& obs : obstacles) 
		{
			// ����ab
			Point vec_ab = { b.x - a.x, b.y - a.y };
			// ����ao
			Point vec_ao = { obs.center.x - a.x, obs.center.y - a.y };
			// ��������vec_ao������vec_ab�ϵ�ͶӰ����projection
			double projection = (vec_ab.x * vec_ao.x + vec_ab.y * vec_ao.y) /
								(vec_ab.x * vec_ab.x + vec_ab.y * vec_ab.y);
			// ������projection��ֵ����������[0.0,1.0]��
			projection = max(0.0, min(1.0, projection));
			// ��������vec_ab��Բ������ĵ�closest
			Point closest = 
			{
				a.x + projection * vec_ab.x,
				a.y + projection * vec_ab.y
			};
			// ͨ�������ж��Ƿ���ײ
			if (obs.center.distanceTo(closest) < obs.radius)
				return true;
		}
		return false;
	}

public:
	// ��ʼ������
	PRM(const vector<Obstacle>& obs) : obstacles(obs) {}
	
	// ѧϰ�׶Σ�����·��ͼ
	void buildRoadmap()
	{
		// �������������
		random_device rd;
		// ����Mersenne Twister�㷨��α�����������
		mt19937 gen(rd());
		// ���ȷֲ������������������������[0, 10)��Χ�ĸ�����
		uniform_real_distribution<double> x_dist(0, 10);
		uniform_real_distribution<double> y_dist(0, 10);

		// �������
		while (nodes.size() < NUM_SAMPLES)
		{
			Point p(x_dist(gen), y_dist(gen));
			bool collision = false;
			// �ж����ɵĵ��Ƿ����ϰ�����ײ
			for (const Obstacle& obs : obstacles)
			{
				if (obs.contains(p))
				{
					collision = true;
					break;
				}
			}
			// û����ײ�������ɵĵ㴢�浽��ͼ��
			if (!collision)
			{
				nodes.emplace_back(p);
			}
		}

		// �����ھӽڵ�
		for (int i = 0; i < nodes.size(); i++)
		{
			// ���ڴ洢��ǰ�ڵ�i�������ڵ�ľ���Ͷ�Ӧ�Ľڵ�����
			vector<pair<double, int>> dist_list;

			// �����벻����MAX_DIST�Ľڵ�ŵ�dist_list��
			for (int j = 0; j < nodes.size(); ++j) 
			{
				if (i == j) continue;
				double d = nodes[i].pos.distanceTo(nodes[j].pos);
				if (d < MAX_DIST)
				{
					dist_list.emplace_back(d, j);
				}
			}

			// ѡ��K_NEIGHBORS��������ھӣ���������dist_list�Ĵ�С
			sort(dist_list.begin(), dist_list.end());
			int k = min(K_NEIGHBORS, (int)dist_list.size());

			// ������ཻ�����ڵ�j��ӵ��ڵ�i���ھ��б���
			for (int n = 0; n < k; ++n) 
			{
				int j = dist_list[n].second;
				if (!isCollision(nodes[i].pos, nodes[j].pos))
				{
					nodes[i].neighbors.push_back(j);
				}
			}
		}
	}

	// ��ѯ�׶Σ�Astar·������
	vector<Point> findPath(const Point& start, const Point& goal)
	{
		// �������յ㵽·��ͼ
		nodes.emplace_back(start);
		nodes.emplace_back(goal);
		int start_id = nodes.size() - 2;
		int goal_id = nodes.size() - 1;

		// ��������յ�
		auto connectNode = [&](int new_id) 
		{
			vector<pair<double, int>> dist_list;
			for (int i = 0; i < nodes.size() - 2; ++i) 
			{
				double d = nodes[new_id].pos.distanceTo(nodes[i].pos);
				if (d < MAX_DIST)
				{
					dist_list.emplace_back(d, i);
				}
			}
			sort(dist_list.begin(), dist_list.end());
			int k = min(K_NEIGHBORS, (int)dist_list.size());
			for (int n = 0; n < k; ++n) 
			{
				int j = dist_list[n].second;
				if (!isCollision(nodes[new_id].pos, nodes[j].pos)) 
				{
					nodes[new_id].neighbors.push_back(j);
					nodes[j].neighbors.push_back(new_id);
				}
			}
		};

		connectNode(start_id);
		connectNode(goal_id);

		// A*�㷨ʵ��
		vector<double> g(nodes.size(), INF);
		vector<int> parent(nodes.size(), -1);
		priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

		g[start_id] = 0;
		double h = nodes[start_id].pos.distanceTo(nodes[goal_id].pos);	// ����������ŷ����þ���
		pq.emplace(h, start_id);

		while (!pq.empty()) 
		{
			double f = pq.top().first;
			int u = pq.top().second;
			pq.pop();

			if (u == goal_id) break;

			for (int v : nodes[u].neighbors) 
			{
				double cost = nodes[u].pos.distanceTo(nodes[v].pos);
				if (g[v] > g[u] + cost) 
				{
					g[v] = g[u] + cost;
					parent[v] = u;
					h = nodes[v].pos.distanceTo(nodes[goal_id].pos);
					pq.emplace(g[v] + h, v);
				}
			}
		}

		// ����·��
		vector<Point> path;
		if (g[goal_id] == INF) return path;

		int current = goal_id;
		while (current != -1) 
		{
			path.push_back(nodes[current].pos);
			current = parent[current];
		}
		reverse(path.begin(), path.end());

		// �Ƴ���ʱ�ڵ�
		nodes.pop_back();
		nodes.pop_back();

		return path;
	}
};

int main()
{
	// ���������ϰ���Ļ���
	vector<Obstacle> obstacles = {
		Obstacle(Point(3, 3), 1.5),
		Obstacle(Point(5, 2), 1.0)
	};

	PRM prm(obstacles);
	prm.buildRoadmap();

	// Ѱ��·��
	Point start(1, 1), goal(9, 9);
	vector<Point> path = prm.findPath(start, goal);

	// ������
	if (path.empty()) {
		cout << "δ�ҵ�·��!" << endl;
	}
	else {
		cout << "�ҵ�·��:" << endl;
		for (const Point& p : path) {
			cout << "(" << p.x << ", " << p.y << ") ";
		}
	}

	return 0;
}