#ifndef pqueue_H
#define pqueue_H

#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>

template<typename val, typename priority, typename hash>
class pqueue {
	public:
		val pop() {
			val tmp = heap[0].first;
			size_t size = heap.size() - 1;
			swap(heap[0], heap[size]);
			keys[heap[0].first] = 0;
			heap.pop_back();
			if (size > 1) {
				reheapifyDown(0);
			}
			return tmp;
		}
		void push(val key, priority priority) {
			size_t size = heap.size();
			heap.push_back({ key, priority });
			keys[key] = size;
			reheapifyUp(size);
		}
		void updatePriority(val key, priority priority) {
			heap[keys[key]].second = priority;
			reheapifyUp(keys[key]);
		}
		bool empty() {
			return heap.size() == 0;
		}

	private:
		std::vector<std::pair<val, priority>> heap;
		std::unordered_map<val, size_t, hash> keys;
		void reheapifyUp(size_t i) {
			if (i == 0) return;
			size_t parentIdx = (i - 1) / 2;
			if (heap[i].second < heap[parentIdx].second) {
				std::swap(heap[i], heap[parentIdx]);
				keys[heap[i].first] = i;
				keys[heap[parentIdx].first] = parentIdx;
				reheapifyUp(parentIdx);
			}
		}
		void reheapifyDown(size_t i) {
			size_t size = heap.size();
			size_t leftIdx = 2 * i + 1;
			size_t rightIdx = leftIdx + 1;
			size_t minIdx = i;
			if (leftIdx < size && heap[leftIdx].second < heap[minIdx].second) minIdx = leftIdx;
			if (rightIdx < size && heap[rightIdx].second < heap[minIdx].second) minIdx = rightIdx;
			if (minIdx == i) return;
			swap(heap[i], heap[minIdx]);
			keys[heap[i].first] = i;
			keys[heap[minIdx].first] = minIdx;
			reheapifyDown(minIdx);
		}
};

#endif
