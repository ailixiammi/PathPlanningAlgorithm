#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>
using namespace std;

const double INF = numeric_limits<double>::max();
const double STEP_SIZE = 0.5;
const int MAX_ITERATIONS = 5000;
const double GOAL_RADIUS = 0.5;
const double SEARCH_RADIUS = 3.0;  // ���������뾶

// ��ά��ṹ
struct Point {
    double x, y;
    Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}

    double distanceTo(const Point& p) const {
        return hypot(x - p.x, y - p.y);
    }
};

// �ϰ���ṹ��Բ�Σ�
struct Obstacle {
    Point center;
    double radius;
    Obstacle(Point c, double r) : center(c), radius(r) {}

    bool contains(const Point& p) const {
        return center.distanceTo(p) < radius;
    }
};

// RRT*�ڵ�ṹ
struct RRTNode {
    Point position;
    int parent;
    double cost;
    RRTNode(Point p = Point(), int pa = -1, double c = 0)
        : position(p), parent(pa), cost(c) {
    }
};

class RRTStar {
private:
    vector<RRTNode> nodes;
    vector<Obstacle> obstacles;
    Point goal;
    mt19937 gen;
    uniform_real_distribution<double> x_dist;
    uniform_real_distribution<double> y_dist;
    int goal_index = -1;  // Ŀ��ڵ�����

    // ��ײ��⣨�߶����ϰ����⣩
    bool isCollision(const Point& a, const Point& b) {
        for (const Obstacle& obs : obstacles) {
            Point vec_ab = { b.x - a.x, b.y - a.y };
            Point vec_ao = { obs.center.x - a.x, obs.center.y - a.y };

            double projection = (vec_ab.x * vec_ao.x + vec_ab.y * vec_ao.y) /
                (vec_ab.x * vec_ab.x + vec_ab.y * vec_ab.y);
            projection = max(0.0, min(1.0, projection));

            Point closest = {
                a.x + projection * vec_ab.x,
                a.y + projection * vec_ab.y
            };

            if (obs.center.distanceTo(closest) < obs.radius)
                return true;
        }
        return false;
    }

    // Ѱ������ڵ�
    int findNearest(const Point& target) {
        int nearest = 0;
        double min_dist = INF;
        for (size_t i = 0; i < nodes.size(); ++i) {
            double d = nodes[i].position.distanceTo(target);
            if (d < min_dist) {
                min_dist = d;
                nearest = i;
            }
        }
        return nearest;
    }

    // Ѱ������ڵ�
    vector<int> findNearNodes(const Point& target) {
        vector<int> near_nodes;
        for (size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i].position.distanceTo(target) < SEARCH_RADIUS) {
                near_nodes.push_back(i);
            }
        }
        return near_nodes;
    }

public:
    RRTStar(Point start, Point goal_, vector<Obstacle> obs,
        double map_width = 10, double map_height = 10)
        : goal(goal_), obstacles(obs),
        x_dist(0, map_width), y_dist(0, map_height) {
        nodes.emplace_back(start);
        gen.seed(random_device()());
    }

    // ���㷨����
    bool plan() {
        bool path_found = false;
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // ���������10%���ʲ���Ŀ��㣩
            Point rand_point = (rand() % 100 < 10) ?
                goal : Point(x_dist(gen), y_dist(gen));

            // ���������
            int nearest_id = findNearest(rand_point);
            Point nearest = nodes[nearest_id].position;

            // ��չ�½ڵ�
            Point direction = {
                rand_point.x - nearest.x,
                rand_point.y - nearest.y
            };
            double length = hypot(direction.x, direction.y);
            Point new_point = {
                nearest.x + (direction.x / length) * STEP_SIZE,
                nearest.y + (direction.y / length) * STEP_SIZE
            };

            if (!isCollision(nearest, new_point)) {
                // ����ڵ�����
                vector<int> near_nodes = findNearNodes(new_point);

                // ѡ�����Ÿ��ڵ�
                double min_cost = nodes[nearest_id].cost + nearest.distanceTo(new_point);
                int best_parent = nearest_id;
                for (int near_id : near_nodes) {
                    double tentative_cost = nodes[near_id].cost +
                        nodes[near_id].position.distanceTo(new_point);
                    if (tentative_cost < min_cost &&
                        !isCollision(nodes[near_id].position, new_point)) {
                        min_cost = tentative_cost;
                        best_parent = near_id;
                    }
                }

                // ����½ڵ�
                RRTNode new_node(new_point, best_parent, min_cost);
                nodes.push_back(new_node);
                int new_node_index = nodes.size() - 1;

                // ���²��ߣ�Rewire��
                for (int near_id : near_nodes) {
                    if (near_id == best_parent) continue;

                    double tentative_cost = new_node.cost +
                        new_node.position.distanceTo(nodes[near_id].position);
                    if (tentative_cost < nodes[near_id].cost &&
                        !isCollision(new_node.position, nodes[near_id].position)) {
                        nodes[near_id].parent = new_node_index;
                        nodes[near_id].cost = tentative_cost;
                    }
                }

                // ����Ƿ񵽴�Ŀ��
                if (!path_found && new_point.distanceTo(goal) < GOAL_RADIUS) {
                    if (!isCollision(new_point, goal)) {
                        nodes.emplace_back(goal, new_node_index,
                            new_node.cost + new_point.distanceTo(goal));
                        goal_index = nodes.size() - 1;
                        path_found = true;
                    }
                }
            }
        }
        return goal_index != -1;
    }

    // ��ȡ·��
    vector<Point> getPath() {
        vector<Point> path;
        if (goal_index == -1) return path;

        int current = goal_index;
        while (current != -1) {
            path.push_back(nodes[current].position);
            current = nodes[current].parent;
        }
        reverse(path.begin(), path.end());
        return path;
    }

    // ���ӻ����
    void visualize() {
        vector<Point> path = getPath();
        if (path.empty()) {
            cout << "·���滮ʧ�ܣ�" << endl;
            return;
        }

        cout << "====== RRT* ·���滮��� ======\n";
        cout << "�ܴ���: " << nodes[goal_index].cost << endl;
        cout << "·���ڵ���: " << path.size() << endl;
        cout << "·����������:\n";

        for (size_t i = 0; i < path.size(); ++i) {
            printf("(%.2f, %.2f)", path[i].x, path[i].y);
            if (i != path.size() - 1) cout << " -> ";
            if ((i + 1) % 5 == 0) cout << endl;
        }
        cout << "\n===============================" << endl;
    }
};

int main() {
    // �����ϰ��ﻷ��
    vector<Obstacle> obstacles = {
        Obstacle(Point(3, 3), 1.5),
        Obstacle(Point(7, 7), 2.0),
        Obstacle(Point(5, 2), 1.0)
    };

    // ���������յ�
    Point start(1, 1), goal(9, 9);

    // �����滮����ִ���㷨
    RRTStar planner(start, goal, obstacles);
    if (planner.plan()) {
        planner.visualize();
    }
    else {
        cout << "�滮ʧ�ܣ�δ�ҵ�����·����" << endl;
    }

    return 0;
}