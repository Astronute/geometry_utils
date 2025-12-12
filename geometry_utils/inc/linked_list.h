#ifndef __LINKED_LIST_H__
#define __LINKED_LIST_H__

#include <iostream>
#include <memory>
#include "Eigen/Dense"

struct EventNode;
struct StatusNode;

enum class FillState {
	Unknown,
	Filled,
	Empty
};

struct FillInfo {
	FillState above = FillState::Unknown;
	FillState below = FillState::Unknown;
};

struct Segment {
	int id;
	Eigen::Vector2d start, end;
	FillInfo myFill; // 当前多边形的填充状态
	FillInfo otherFill; // 另一个多边形的填充状态

	Segment() {}

	Segment(Eigen::Vector2d s, Eigen::Vector2d e): start(s), end(e){
	}
};

struct EventNode {
	bool isStart; // 是否为起点
	bool primary; // true:主多边形 false:次多边形
	Eigen::Vector2d pos; // 坐标
	Segment* seg = nullptr; // 对应线段
	EventNode* other = nullptr; // 所在线段的另一个(事件点/端点)
	EventNode* prev = nullptr;
	EventNode* next = nullptr;
	StatusNode* status = nullptr; // 终点事件才会指向状态点

	EventNode() {}
};

struct StatusNode {
	EventNode* ev = nullptr; // 指向起点事件
	StatusNode* prev = nullptr; // 上一个状态点
	StatusNode* next = nullptr; // 下一个状态点

	StatusNode() {}
};



class EventList {
public:
	EventList() {
		root = new EventNode;
	}

	~EventList() {
		delete root;
	}

	bool isEmpty();

	bool exists(EventNode* node);

	EventNode* getHead();

	bool detach(EventNode* node);

	void remove(EventNode* node);

	void printList();

	EventNode* root;

};

class StatusList {
public:
	StatusList() {
		root = new StatusNode;
	}
	~StatusList() {
		delete root;
	}

	bool isEmpty();

	bool exists(StatusNode* node);

	bool detach(StatusNode* node);

	void remove(StatusNode* node);

	void addAbove(StatusNode* above, StatusNode* below);

	void printList();

	StatusNode* root;
};


#endif // !__LINKED_LIST_H__

