/*
	概率路线图算法
	碰撞检测：使用线段到圆的距离检测
	邻域连接：K近邻算法
	路径搜索：A*算法优化
	启发函数：欧几里得距离

	当障碍物太多或者采样点太少时，容易找不到路径
	由于采样点的随机性，同样的参数，不同的轮次的结果也不一样
*/

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <algorithm>
using namespace std;

const double INF = 1e9;
const int NUM_SAMPLES = 100;	// 采样点数量
const int K_NEIGHBORS = 5;		// 每个点的邻居数量
const double MAX_DIST = 2.0;	// 最大连接距离

// 二维点结构
struct Point
{
	// 点的坐标
	double x, y;
	// 构造函数初始化
	Point(double x_, double y_) :x(x_), y(y_) {}
	// 计算该点到点p的距离
	double distanceTo(const Point& p) const
	{
		return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
	}
};

// 障碍物结构(二维圆形障碍物)
struct Obstacle
{
	// 圆心
	Point center;
	// 半径
	double radius;
	// 初始化
	Obstacle(Point c_, double r_) :center(c_), radius(r_) {}
	// 判断点p是否在障碍物的范围内
	bool contains(const Point& p) const
	{
		// 是返回true，否返回false
		return center.distanceTo(p) < radius;
	}
};

// PRM路线图节点
struct Node
{
	// 节点位置
	Point pos;
	// 邻居节点的索引
	vector<int> neighbors;
	// 初始化
	Node(Point p) :pos(p) {}
};

class PRM
{
private:
	// 储存所有节点
	vector<Node> nodes;
	// 存储环境中的所有障碍物
	vector<Obstacle> obstacles;

	// 碰撞检测(利用向量ao在向量ab上的投影，来获取线段ab离圆心最近的点c，通过co与半径r比较来判断是否碰撞)
	bool isCollision(const Point& a, const Point& b) 
	{
		for (const Obstacle& obs : obstacles) 
		{
			// 向量ab
			Point vec_ab = { b.x - a.x, b.y - a.y };
			// 向量ao
			Point vec_ao = { obs.center.x - a.x, obs.center.y - a.y };
			// 计算向量vec_ao在向量vec_ab上的投影长度projection
			double projection = (vec_ab.x * vec_ao.x + vec_ab.y * vec_ao.y) /
								(vec_ab.x * vec_ab.x + vec_ab.y * vec_ab.y);
			// 将变量projection的值限制在区间[0.0,1.0]内
			projection = max(0.0, min(1.0, projection));
			// 计算向量vec_ab离圆心最近的点closest
			Point closest = 
			{
				a.x + projection * vec_ab.x,
				a.y + projection * vec_ab.y
			};
			// 通过距离判断是否碰撞
			if (obs.center.distanceTo(closest) < obs.radius)
				return true;
		}
		return false;
	}

public:
	// 初始化环境
	PRM(const vector<Obstacle>& obs) : obstacles(obs) {}
	
	// 学习阶段：构建路线图
	void buildRoadmap()
	{
		// 生成随机数种子
		random_device rd;
		// 基于Mersenne Twister算法的伪随机数生成器
		mt19937 gen(rd());
		// 均匀分布的随机数生成器，用于生成[0, 10)范围的浮点数
		uniform_real_distribution<double> x_dist(0, 10);
		uniform_real_distribution<double> y_dist(0, 10);

		// 随机采样
		while (nodes.size() < NUM_SAMPLES)
		{
			Point p(x_dist(gen), y_dist(gen));
			bool collision = false;
			// 判断生成的点是否与障碍物碰撞
			for (const Obstacle& obs : obstacles)
			{
				if (obs.contains(p))
				{
					collision = true;
					break;
				}
			}
			// 没有碰撞，将生成的点储存到地图中
			if (!collision)
			{
				nodes.emplace_back(p);
			}
		}

		// 连接邻居节点
		for (int i = 0; i < nodes.size(); i++)
		{
			// 用于存储当前节点i到其他节点的距离和对应的节点索引
			vector<pair<double, int>> dist_list;

			// 将距离不超过MAX_DIST的节点放到dist_list中
			for (int j = 0; j < nodes.size(); ++j) 
			{
				if (i == j) continue;
				double d = nodes[i].pos.distanceTo(nodes[j].pos);
				if (d < MAX_DIST)
				{
					dist_list.emplace_back(d, j);
				}
			}

			// 选择K_NEIGHBORS个最近的邻居，但不超过dist_list的大小
			sort(dist_list.begin(), dist_list.end());
			int k = min(K_NEIGHBORS, (int)dist_list.size());

			// 如果不相交，将节点j添加到节点i的邻居列表中
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

	// 查询阶段：Astar路径搜索
	vector<Point> findPath(const Point& start, const Point& goal)
	{
		// 添加起点终点到路线图
		nodes.emplace_back(start);
		nodes.emplace_back(goal);
		int start_id = nodes.size() - 2;
		int goal_id = nodes.size() - 1;

		// 连接起点终点
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

		// A*算法实现
		vector<double> g(nodes.size(), INF);
		vector<int> parent(nodes.size(), -1);
		priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

		g[start_id] = 0;
		double h = nodes[start_id].pos.distanceTo(nodes[goal_id].pos);	// 启发函数是欧几里得距离
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

		// 回溯路径
		vector<Point> path;
		if (g[goal_id] == INF) return path;

		int current = goal_id;
		while (current != -1) 
		{
			path.push_back(nodes[current].pos);
			current = parent[current];
		}
		reverse(path.begin(), path.end());

		// 移除临时节点
		nodes.pop_back();
		nodes.pop_back();

		return path;
	}
};

int main()
{
	// 创建包含障碍物的环境
	vector<Obstacle> obstacles = {
		Obstacle(Point(3, 3), 1.5),
		Obstacle(Point(5, 2), 1.0)
	};

	PRM prm(obstacles);
	prm.buildRoadmap();

	// 寻找路径
	Point start(1, 1), goal(9, 9);
	vector<Point> path = prm.findPath(start, goal);

	// 输出结果
	if (path.empty()) {
		cout << "未找到路径!" << endl;
	}
	else {
		cout << "找到路径:" << endl;
		for (const Point& p : path) {
			cout << "(" << p.x << ", " << p.y << ") ";
		}
	}

	return 0;
}