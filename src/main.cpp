#ifdef NDEBUG
#define main WinMain
#endif

#include <array>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

#include <raylib.h>
#include <pqueue.h>

constexpr int windowW = 1280, windowH = 720, gridSize = 20;

using namespace std;
using Point = pair<int, int>;
using Grid = array<array<unsigned char, windowH / gridSize>, windowW / gridSize>;

Grid grid;
Point start = { 0, 0 };
Point finish = { windowW / gridSize - 1, windowH / gridSize - 1 };
unsigned char state = 0;

class Pathfinder {
	public:
		Pathfinder() : m_grid{ grid } {}
		Pathfinder(Grid& grid, Point start, Point finish) : m_grid{ grid }, m_start{ start }, m_finish{ finish }, m_cur{ m_start } {
			m_pq.push(m_start, 0.0);
			m_parents[m_start] = m_start;
			m_distances[m_start] = 0.0;
			m_grid[m_start.first][m_start.second] = 3;
		}
		void operator= (const Pathfinder& other) {
			m_grid = other.m_grid;
			m_start = other.m_start;
			m_finish = other.m_finish;
			m_pq = other.m_pq;
			m_parents = other.m_parents;
			m_distances = other.m_distances;
		}
		void step() {
			if (!m_pq.empty()) {
				Point cur = m_pq.pop();
				m_grid[cur.first][cur.second] = 3;
				if (cur == m_finish) {
					m_cur = m_finish;
					m_reached = true;
					state++;
					return;
				}
				for (int i = -1; i < 2; i++) {
					for (int j = -1; j < 2; j++) {
						Point tmp{ cur.first + i, cur.second + j };
						if (tmp == cur) continue;
						if (isValid(tmp)) {
							double step = tmp.first != cur.first && tmp.second != cur.second ? sqrt(2) : 1.0;
							if (m_distances[tmp] == 0.0) {
								m_distances[tmp] = DBL_MAX;
								m_grid[tmp.first][tmp.second] = 4;
								m_pq.push(tmp, DBL_MAX);
							}
							if (m_distances[cur] + step < m_distances[tmp]) {
								m_distances[tmp] = m_distances[cur] + step;
								m_parents[tmp] = cur;
								m_pq.updatePriority(tmp, abs(m_finish.first - tmp.first) + abs(m_finish.second - tmp.second) + m_distances[cur] + step);
							}
						}
					}
				}
			}
			else state++;
		}
		void backtrack() {
			if (!m_reached || m_cur == m_start) state++;
			else {
				m_grid[m_cur.first][m_cur.second] = 2;
				m_cur = m_parents[m_cur];
			}
		}
	private:
		struct hash_pair {
			size_t operator()(const pair<int, int>& p) const {
				auto hash1 = hash<int>{}(p.first);
				auto hash2 = hash<int>{}(p.second);

				if (hash1 != hash2) {
					return hash1 ^ hash2;
				}
				return hash1;
			}
		};
		Grid& m_grid;
		Point m_start;
		Point m_finish;
		Point m_cur;
		pqueue<Point, double, hash_pair> m_pq;
		unordered_map<Point, Point, hash_pair> m_parents;
		unordered_map<Point, double, hash_pair> m_distances;
		bool m_reached = false;
		bool isValid(Point point) {
			return point.first >= 0 && point.first < windowW / gridSize && point.second >= 0 && point.second < windowH / gridSize && grid[point.first][point.second] != 1;
		}

} astar;

void handleInput() {
	if (state == 1 || state == 2) {
		if (IsKeyDown(KEY_BACKSPACE)) state = 3;
		else return;
	}
	if (state == 3) {
		if (IsKeyDown(KEY_BACKSPACE)) {
			for (auto& i : grid) {
				for (auto& j : i) {
					if (j > 1) j = 0;
				}
			}
			state = 0;
		}
	}
	else {
		int mouseX = GetMouseX() / gridSize, mouseY = GetMouseY() / gridSize;
		if (IsKeyDown(KEY_DELETE)) grid = { { 0 } };
		else if (IsKeyDown(KEY_ENTER)) {
			state++;
			astar = Pathfinder(grid, start, finish);
		}
		else if (mouseX >= 0 && mouseX < windowW / gridSize && mouseY >= 0 && mouseY < windowH / gridSize) {
			if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
				if (IsKeyDown(KEY_LEFT_SHIFT)) start = { mouseX, mouseY };
				else if (Point{ mouseX, mouseY } != start && Point{ mouseX, mouseY } != finish) grid[mouseX][mouseY] = 1;
			}
			if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
				if (IsKeyDown(KEY_LEFT_SHIFT)) finish = { mouseX, mouseY };
				grid[mouseX][mouseY] = 0;
			}
		}
	}
}

void render() {
	constexpr static array<Color, 5> colMap = { BLACK, WHITE, DARKPURPLE, DARKBLUE, SKYBLUE };
	BeginDrawing();
	for (int x = 0; x < windowW / gridSize; x++) {
		for (int y = 0; y < windowH / gridSize; y++) {
			Color cellCol;
			if (Point{ x, y } == start) cellCol = GREEN;
			else if (Point{ x, y } == finish) cellCol = RED;
			else cellCol = colMap[grid[x][y]];
			DrawRectangle(x * gridSize, y * gridSize, gridSize, gridSize, cellCol);
		}
	}
	EndDrawing();
}

int main() {
	InitWindow(windowW, windowH, "A* Pathfinder");
	SetTargetFPS(240);
	while (!WindowShouldClose()) {
		handleInput();
		if (state == 1) astar.step();
		else if (state == 2) astar.backtrack();
		render();
	}
	CloseWindow();
}
