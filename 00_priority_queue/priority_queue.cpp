#include <iostream>
#include <vector>
#include <queue>
using namespace std;

void print1(priority_queue<int>& pq)
{
	while (!pq.empty())
	{
		cout << pq.top() << " ";	// 打印堆顶元素，但并不会将堆顶出堆
		pq.pop();	// 出堆
	}
	cout << endl;
}

void print2(priority_queue<int, vector<int>, greater<int>>& pq)
{
	while (!pq.empty())
	{
		cout << pq.top() << " ";	// 打印堆顶元素，但并不会将堆顶出堆
		pq.pop();	// 出堆
	}
	cout << endl;
}

void print3(priority_queue<pair<int, int>>& pq)
{
	while (!pq.empty())
	{
		cout << "first: " << pq.top().first << " second: " << pq.top().second;	// 打印堆顶元素，但并不会将堆顶出堆
		cout << endl;
		pq.pop();	// 出堆
	}
}

void print4(priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>>& pq)
{
	while (!pq.empty())
	{
		cout << "first: " << pq.top().first << " second: " << pq.top().second;	// 打印堆顶元素，但并不会将堆顶出堆
		cout << endl;
		pq.pop();	// 出堆
	}
}

int main()
{
	// 默认构造（大根堆）
	priority_queue<int> pqEmp;
	priority_queue<int> pqMax;

	pqMax.push(10);
	pqMax.push(20);
	pqMax.push(5);

	print1(pqMax);	// 20 10 5

	// 自定义比较函数（小根堆）
	priority_queue<int, vector<int>, greater<int>> pqMin;

	pqMin.push(10);
	pqMin.push(20);
	pqMin.push(5);
	pqMin.push(5);

	print2(pqMin);	// 5 5 10 20

	// 队列大小size()
	cout << "pqEmp的大小为：" << pqEmp.size() << endl;
	cout << "pqMax的大小为：" << pqMax.size() << endl;
	cout << "pqMin的大小为：" << pqMin.size() << endl;

	// 队列存放pair<first, second>
	// 默认按照第一个元素进行优先级排序
	priority_queue<pair<int, int>> pqPair1;	// 按第一个元素降序

	pqPair1.push({ 10, 1 });
	pqPair1.push({ 20, 2 });
	pqPair1.push({ 5, 3 });

	print3(pqPair1);

	// 自定义按照第一个元素进行升序排序
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pqPair2;

	pqPair2.push({ 10, 1 });
	pqPair2.push({ 20, 2 });
	pqPair2.push({ 5, 3 });

	print4(pqPair2);
}