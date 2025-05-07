#include <iostream>
#include <vector>
#include <queue>
using namespace std;

void print1(priority_queue<int>& pq)
{
	while (!pq.empty())
	{
		cout << pq.top() << " ";	// ��ӡ�Ѷ�Ԫ�أ��������Ὣ�Ѷ�����
		pq.pop();	// ����
	}
	cout << endl;
}

void print2(priority_queue<int, vector<int>, greater<int>>& pq)
{
	while (!pq.empty())
	{
		cout << pq.top() << " ";	// ��ӡ�Ѷ�Ԫ�أ��������Ὣ�Ѷ�����
		pq.pop();	// ����
	}
	cout << endl;
}

void print3(priority_queue<pair<int, int>>& pq)
{
	while (!pq.empty())
	{
		cout << "first: " << pq.top().first << " second: " << pq.top().second;	// ��ӡ�Ѷ�Ԫ�أ��������Ὣ�Ѷ�����
		cout << endl;
		pq.pop();	// ����
	}
}

void print4(priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>>& pq)
{
	while (!pq.empty())
	{
		cout << "first: " << pq.top().first << " second: " << pq.top().second;	// ��ӡ�Ѷ�Ԫ�أ��������Ὣ�Ѷ�����
		cout << endl;
		pq.pop();	// ����
	}
}

int main()
{
	// Ĭ�Ϲ��죨����ѣ�
	priority_queue<int> pqEmp;
	priority_queue<int> pqMax;

	pqMax.push(10);
	pqMax.push(20);
	pqMax.push(5);

	print1(pqMax);	// 20 10 5

	// �Զ���ȽϺ�����С���ѣ�
	priority_queue<int, vector<int>, greater<int>> pqMin;

	pqMin.push(10);
	pqMin.push(20);
	pqMin.push(5);
	pqMin.push(5);

	print2(pqMin);	// 5 5 10 20

	// ���д�Сsize()
	cout << "pqEmp�Ĵ�СΪ��" << pqEmp.size() << endl;
	cout << "pqMax�Ĵ�СΪ��" << pqMax.size() << endl;
	cout << "pqMin�Ĵ�СΪ��" << pqMin.size() << endl;

	// ���д��pair<first, second>
	// Ĭ�ϰ��յ�һ��Ԫ�ؽ������ȼ�����
	priority_queue<pair<int, int>> pqPair1;	// ����һ��Ԫ�ؽ���

	pqPair1.push({ 10, 1 });
	pqPair1.push({ 20, 2 });
	pqPair1.push({ 5, 3 });

	print3(pqPair1);

	// �Զ��尴�յ�һ��Ԫ�ؽ�����������
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pqPair2;

	pqPair2.push({ 10, 1 });
	pqPair2.push({ 20, 2 });
	pqPair2.push({ 5, 3 });

	print4(pqPair2);
}