/*
	�Ľ��Ŀ�����չ������㷨

*/
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
using namespace std;

const double INF = 1e9;
const double STEP_SIZE = 0.5;	// ��չ����
const int MAX_ITERATOR = 5000;	// ����������
const double GOAL_RADIUS = 0.5;	// Ŀ������뾶
const double SEARCH_RADIUS = 3.0;	// �½ڵ�������뾶

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

// RRT���ڵ�
struct Node
{
	// �ڵ������ÿռ��е�λ��
	Point pos;
	// ��ǰ�ڵ�ĸ��ڵ������е�����
	int parent;
	// ����㵽��ǰ�ڵ�Ĵ���
	double cost;
	// ��ʼ��
	Node(Point p_, int pa_ = -1, double c_ = 0) : pos(p_), parent(pa_), cost(c_) {}
};

class RRTstar
{
private:
	// �洢RRT���е����нڵ�
	vector<Node> nodes;
	// �洢�����е������ϰ���
	vector<Obstacle> obstacles;
	// ���ڴ���·�����ܴ���
	double total_cost;
	// Ŀ��ڵ�����
	int goal_index = -1; 
	// Ŀ��ڵ�
	Point goal;
	// ����Mersenne Twister�㷨��α�����������
	mt19937 gen;
	// ���ȷֲ������������������������ָ����Χ�ڵĸ�������Ĭ�Ϸ�Χ[0.0, 1.0)
	uniform_real_distribution<double> x_dist;
	uniform_real_distribution<double> y_dist;

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

	// Ѱ������Ľڵ�
	int findNearest(const Point& target)
	{
		// ��¼����ڵ������
		int nearest = 0;
		// ��¼������ڵ�ľ���
		double min_dist = INF;
		// ����RRT�������еĽڵ㣬Ѱ������ڵ�
		for (int i = 0; i < nodes.size(); i++)
		{
			double d = nodes[i].pos.distanceTo(target);
			if (d < min_dist)
			{
				min_dist = d;
				nearest = i;
			}
		}
		return nearest;
	}

	// �½ڵ�Ѱ���ٽ��Ľ��(����С�������뾶�Ľڵ�)
	vector<int> findNearNodes(const Point& newNode)
	{
		// ��¼�ٽ��ڵ���±�����
		vector<int> near_nodes;

		for (int i = 0; i < nodes.size(); i++)
		{
			if (nodes[i].pos.distanceTo(newNode) < SEARCH_RADIUS)
			{
				near_nodes.push_back(i);
			}
		}

		return near_nodes;
	}

public:
	// ���캯��
	RRTstar(Point start, Point goal_, vector<Obstacle> obs_,
		double map_width = 10, double map_height = 10)
		: goal(goal_)
		, obstacles(obs_)
		, x_dist(0, map_width)
		, y_dist(0, map_height)
		, total_cost(0)
	{
		// ��ʼ�����ڵ�
		nodes.emplace_back(start);
		// ��ʼ�������������
		gen.seed(random_device()());
	}

	// ����·����
	bool buildTree()
	{
		// �Ƿ��ҵ�·���ı�־
		bool path_found = false;

		for (int i = 0;i < MAX_ITERATOR;i++)
		{
			// �������
			Point rand_point(x_dist(gen), y_dist(gen));

			// ��10%�ĸ���ֱ�ӽ�Ŀ�������Ϊ����㡣�������ڼ����㷨�ҵ�Ŀ��㣬�ر�����Ŀ��㸽��ʱ��
			if (rand() % 100 < 10)	rand_point = goal;

			// ��չ�½ڵ�
			int nearest_id = findNearest(rand_point);
			Point nearest = nodes[nearest_id].pos;
			// ����Ƕ�
			double theta = atan2(rand_point.y - nearest.y,
				rand_point.x - nearest.x);
			Point new_point = {
				nearest.x + STEP_SIZE * cos(theta),
				nearest.y + STEP_SIZE * sin(theta)
			};

			// ���Ĳ���
			if (!isCollision(nearest, new_point))
			{
				// RRTstar�����Ż�1��Ѱ���½ڵ�����Ÿ��ڵ�
				// �½ڵ�Ѱ���ٽ��Ľ��(����С�������뾶�Ľڵ�)
				vector<int> near_nodes = findNearNodes(new_point);
				// ��ʼ����С�ɱ�
				double min_cost = nodes[nearest_id].cost + nearest.distanceTo(new_point);
				int best_parent = nearest_id;

				// �����ڽ��ڵ�
				for (int near_id : near_nodes)
				{
					double tentative_cost = nodes[near_id].cost + nodes[near_id].pos.distanceTo(new_point);
					if (tentative_cost < min_cost && !isCollision(nodes[near_id].pos, new_point))
					{
						min_cost = tentative_cost;
						best_parent = near_id;
					}
				}

				// ѡ�����Ÿ��ڵ�󣬴���������½ڵ㵽RRT����
				Node new_node(new_point, best_parent, min_cost);
				nodes.push_back(new_node);
				// ����ӽڵ��������¼
				int new_node_index = nodes.size() - 1;  

				// RRTstar�����Ż�2��·���Ż���Rewire��
				// ͨ�������������еĽڵ㣬��һ���Ż�·���ɱ�
				for (int near_id : near_nodes) 
				{
					if (near_id == best_parent) continue;

					// �����µĳɱ�
					double tentative_cost = new_node.cost + new_node.pos.distanceTo(nodes[near_id].pos);
					// �����ڽ��ڵ�ĸ��ڵ�
					if (tentative_cost < nodes[near_id].cost && !isCollision(new_node.pos, nodes[near_id].pos)) 
					{
						nodes[near_id].parent = new_node_index;
						nodes[near_id].cost = tentative_cost;
					}
				}

				// ����Ƿ񵽴�Ŀ������
				if (!path_found && new_point.distanceTo(goal) < GOAL_RADIUS) 
				{
					nodes.emplace_back(goal, nodes.size() - 1, new_node.cost + new_point.distanceTo(goal));
					path_found = true;
				}
			}
		}
		return path_found;
	}

	// ��ȡ·��
	vector<Point> getPath()
	{
		vector<Point> path;

		// û�ҵ�
		if (goal_index == -1) return path;

		// ·������
		int current = goal_index;
		while (current != -1)
		{
			path.push_back(nodes[current].pos);
			current = nodes[current].parent;
		}
		reverse(path.begin(), path.end());		// goal->start => start->goal

		return path;
	}

	// ���ӻ����
	void visualizePath()
	{
		vector<Point> path = getPath();
		if (path.empty())
		{
			cout << "·���滮ʧ�ܣ�" << endl;
			return;
		}

		cout << "===== ·���滮��� =====" << endl;
		cout << "�ڵ�����: " << path.size() << endl;
		cout << "·���ܴ���: " << nodes[goal_index].cost << endl;
		cout << "·������: ";

		for (int i = 0; i < path.size(); i++)
		{
			cout << "(" << path[i].x << ", " << path[i].y << ")";
			if (i != path.size() - 1) cout << " -> ";
		}
		cout << "\n========================" << endl;
	}
};

int main()
{
	vector<Obstacle> obstacles = {
		Obstacle(Point(3, 3), 1.5),
		Obstacle(Point(7, 7), 2.0),
		Obstacle(Point(5, 2), 1.0)
	};

	RRTstar planner(Point(1, 1), Point(9, 9), obstacles);
	if (planner.buildTree()) 
	{
		planner.visualizePath();
	}
	else 
	{
		cout << "Planning failed!" << endl;
	}

	return 0;
}