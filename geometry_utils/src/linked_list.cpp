#include "linked_list.h"

bool EventList::isEmpty() {
	return root->next == nullptr;
}

bool EventList::exists(EventNode* node) {
	if (node == nullptr || node == root) {
		return false;
	}
	return true;
}

bool EventList::detach(EventNode* node) {
	if (node == nullptr || node == root) {
		DEBUG_PRINT(" EventList detach Node error");
		return false;
	}
	node->prev->next = node->next;
	if (node->next) {
		node->next->prev = node->prev;
	}
	node->prev = nullptr;
	node->next = nullptr;

	return true;
}

void EventList::remove(EventNode* node) {
	//if (!node) {
	//	return;
	//}
	//if (detach(node)) {
	//	if (node->other) {
	//		if (node->other->other == node) {
	//			node->other->other = nullptr;
	//		}
	//		node->other = nullptr;
	//	}
	//	node->status = nullptr;
	//	node->seg = nullptr;
	//	delete node;
	//}
	//else {
	//	std::cout << " EventList remove Node error" << std::endl;
	//}
}

EventNode* EventList::getHead() {
	return root->next;
}

void EventList::printList() {
	EventNode* iter = root->next;
	while (iter != nullptr) {
		//if (int(iter->isStart))
		DEBUG_PRINT(" isstart: " << int(iter->isStart) << ", pos: " << iter->pos << " -> " << iter->other->pos);
		iter = iter->next;
	}
}

/////////////////////////////////////////////////////////////////////////////////


bool StatusList::isEmpty() {
	return root->next == nullptr;
}

bool StatusList::exists(StatusNode* node) {
	if (node == nullptr || node == root) {
		return false;
	}
	return true;
}

bool StatusList::detach(StatusNode* node) {
	if (node == nullptr || node == root) {
		DEBUG_PRINT(" StatusList detach Node error");
		return false;
	}
	node->prev->next = node->next;
	if (node->next) {
		node->next->prev = node->prev;
	}
	node->prev = nullptr;
	node->next = nullptr;

	return true;
}

void StatusList::remove(StatusNode* node) {
	//if (detach(node)) {
	//	node->ev->other->status = nullptr;
	//	node->ev = nullptr;
	//	delete node;
	//}
	//else {
	//	std::cout << " EventList remove Node error" << std::endl;
	//}
}

void StatusList::addAbove(StatusNode* above, StatusNode* below) {
	if (below == root && below == nullptr) {
		DEBUG_PRINT(" add above illegal ");
		return;
	}
	above->next = below;
	above->prev = below->prev;
	below->prev->next = above;
	below->next = above;
}

void StatusList::printList() {
	StatusNode* iter = root->next;
	while (iter != nullptr) {
		DEBUG_PRINT("pos: " << iter->ev->pos << " -> " << iter->ev->other->pos);
		iter = iter->next;
	}
}