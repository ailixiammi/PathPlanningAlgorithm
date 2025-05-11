/*
	快速扩展随机树算法
*/
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
using namespace std;

const double INF = 1e9;
const double STEP_SIZE = 0.5;	// 扩展步长
const int MAX_ITERATOR = 1000;	// 最大迭代次数
const double GOAL_RADIUS = 0.5;	// 目标区域半径

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

// RRT树节点
struct Node
{
	// 节点在配置空间中的位置
	Point pos;
	// 当前节点的父节点在树中的索引
	int parent;
	// 从起点到当前节点的代价
	double cost;
	// 初始化
	Node(Point p_, int pa_ = -1, double c_ = 0) : pos(p_), parent(pa_), cost(c_) {}
};

class RRT
{
private:
	// 存储RRT树中的所有节点
	vector<Node> nodes;
	// 存储环境中的所有障碍物
	vector<Obstacle> obstacles;
	// 用于储存路径的总代价
	double total_cost;
	// 目标节点
	Point goal;
	// 基于Mersenne Twister算法的伪随机数生成器
	mt19937 gen;
	// 均匀分布的随机数生成器，用于生成指定范围内的浮点数，默认范围[0.0, 1.0)
	uniform_real_distribution<double> x_dist;
	uniform_real_distribution<double> y_dist;

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

	// 寻找最近的节点
	int findNearest(const Point& target)
	{
		// 记录最近节点的索引
		int nearest = 0;
		// 记录到最近节点的距离
		double min_dist = INF;
		// 遍历RRT树中所有的节点，寻找最近节点
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

public:
	// 构造函数
	RRT(Point start, Point goal_, vector<Obstacle> obs_,
		double map_width = 10, double map_height = 10)
		: goal(goal_)
		, obstacles(obs_)
		,x_dist(0, map_width)
		, y_dist(0, map_height) 
		, total_cost(0.0)
	{
		// 初始化根节点
		nodes.emplace_back(start);
		// 初始化随机数生成器
		gen.seed(random_device()());
	}

	// 构建路径树
	bool buildTree()
	{
		for (int i = 0;i < MAX_ITERATOR;i++)
		{
			// 随机采样
			Point rand_point(x_dist(gen), y_dist(gen));

			// 有10%的概率直接将目标点设置为随机点。这有助于加速算法找到目标点，特别是在目标点附近时。
			if (rand() % 100 < 10)
				rand_point = goal;

			// 扩展新节点
			int nearest_id = findNearest(rand_point);
			Point nearest = nodes[nearest_id].pos;
			// 计算角度
			double theta = atan2(rand_point.y - nearest.y,
				rand_point.x - nearest.x);
			Point new_point = {
				nearest.x + STEP_SIZE * cos(theta),
				nearest.y + STEP_SIZE * sin(theta)
			};

			// 碰撞检测
			if (!isCollision(nearest, new_point)) 
			{
				double new_cost = nodes[nearest_id].cost + nearest.distanceTo(new_point);
				nodes.emplace_back(new_point, nearest_id, new_cost);

				// 终止条件检测
				if (new_point.distanceTo(goal) < GOAL_RADIUS) 
				{
					double goal_cost = nodes[nearest_id].cost + new_point.distanceTo(goal);
					nodes.emplace_back(goal, nodes.size() - 1, goal_cost);
					return true;
				}
			}
		}
		// 如果在MAX_ITER次迭代后仍未找到目标点，则返回false表示失败
		return false;
	}

	// 获取路径
	vector<Point> getPath()
	{
		vector<Point> path;
		total_cost = 0.0;  // 初始化总代价

		// 没找到
		if (nodes.empty() || nodes.back().pos.distanceTo(goal) > GOAL_RADIUS)
		{
			return path;
		}

		// 路径回溯
		int current = nodes.size() - 1;
		while (current != -1)
		{
			path.push_back(nodes[current].pos);
			current = nodes[current].parent;
		}
		reverse(path.begin(), path.end());		// goal->start => start->goal

		// 直接获取目标节点存储的总代价
		total_cost = nodes.back().cost;

		return path;
	}

	// 可视化输出
	void visualizePath() 
	{
		vector<Point> path = getPath();
		if (path.empty()) 
		{
			cout << "路径规划失败！" << endl;
			return;
		}

		cout << "===== 路径规划结果 =====" << endl;
		cout << "节点数量: " << path.size() << endl;
		cout << "路径总代价: " << total_cost << endl;
		cout << "路径序列: ";

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
	// 环境设置
	vector<Obstacle> obstacles = {
		Obstacle(Point(3, 3), 1.5),
		Obstacle(Point(7, 7), 2.0),
		Obstacle(Point(5, 2), 1.0)
	};

	Point start(1, 1), goal(9, 9);

	// 规划流程
	RRT planner(start, goal, obstacles);
	bool success = planner.buildTree();

	if (success) 
	{
		planner.visualizePath();
	}
	else 
	{
		cout << "规划失败，达到最大迭代次数！" << endl;
	}

	return 0;
}