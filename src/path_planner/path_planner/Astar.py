import sys
import heapq
import numpy as np
from matplotlib import pyplot as plt


class Point:
    """栅格上的一个点，包含坐标和寻路时的父节点引用。"""

    __slots__ = ("x", "y", "parent")

    def __init__(self, x: int, y: int):
        self.x = int(x)
        self.y = int(y)
        self.parent = None  # type: Point | None

    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return f"Point(x={self.x}, y={self.y})"

    def dist(self, ptb: "Point"):
        """计算到另一点的欧氏距离和方向角（弧度）。"""
        dx = ptb.x - self.x
        dy = ptb.y - self.y
        c = dx ** 2 + dy ** 2
        rad = np.arctan2(dy, dx)  # -pi, pi
        return c ** 0.5, rad


class AStar:
    """
    栅格地图上的 A* 路径规划。

    地图约定：
      - self.map[y, x] == 0      -> 可通行
      - self.map[y, x] >= threshold -> 障碍
    """

    def __init__(self, mp: np.ndarray, obstacle_threshold: int = 100):
        """
        :param mp: 2D numpy 数组，值 0 表示空闲，>= obstacle_threshold 表示障碍
        """
        if mp.ndim != 2:
            raise ValueError("map 必须是 2D numpy 数组")
        self.map = mp.astype(np.int16)
        self.size = self.map.shape  # (height, width)
        self.obstacle_threshold = obstacle_threshold

        self.path: list[Point] = []

    def IsObstacle(self, x: int, y: int) -> bool:
        """判断是否是障碍：>= obstacle_threshold 认为是障碍。"""
        return self.map[y, x] >= self.obstacle_threshold

    def IsValidPoint(self, x: int, y: int) -> bool:
        """判断位置 (x, y) 是否在地图范围且不是障碍。"""
        if x < 0 or y < 0:
            return False
        h, w = self.size
        if x >= w or y >= h:
            return False
        return not self.IsObstacle(x, y)

    def _neighbors(self, x: int, y: int):
        """
        8 邻域，对角点 (x+dx, y+dy) 只有在 (x+dx, y) 和 (x, y+dy) 都可达时才允许。
        """
        dirs = [
            (-1, 0), (1, 0), (0, -1), (0, 1),      # 4 邻域
            (-1, -1), (1, -1), (-1, 1), (1, 1),    # 4 对角
        ]
        for dx, dy in dirs:
            nx, ny = x + dx, y + dy
            if not self.IsValidPoint(nx, ny):
                continue

            # 对角防“卡角”
            if dx != 0 and dy != 0:
                if not (self.IsValidPoint(x + dx, y) and self.IsValidPoint(x, y + dy)):
                    continue

            yield nx, ny

    @staticmethod
    def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
        """启发函数：欧氏距离。"""
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def _reconstruct_path(self, came_from: dict[tuple[int, int], tuple[int, int]], current: tuple[int, int]):
        """从 came_from 字典中反向构建路径（Point 列表）。"""
        path_pix: list[tuple[int, int]] = [current]
        while current in came_from:
            current = came_from[current]
            path_pix.append(current)
        path_pix.reverse()

        # 转成 Point 列表
        points: list[Point] = []
        for x, y in path_pix:
            p = Point(x, y)
            if points:
                p.parent = points[-1]
            points.append(p)

        self.path = points
        return self.path

    def Run(self, s: Point, g: Point):
        """
        A* 主入口。

        :param s: 起点（地图索引坐标）
        :param g: 终点（地图索引坐标）
        :return: Point 列表路径，如果失败返回 None
        """
        # 起点终点合法性检查
        if not self.IsValidPoint(s.x, s.y):
            print("AStar.Run: 起点在障碍或地图外，无法规划路径")
            self.path = []
            return None
        if not self.IsValidPoint(g.x, g.y):
            print("AStar.Run: 终点在障碍或地图外，无法规划路径")
            self.path = []
            return None

        start = (int(s.x), int(s.y))
        goal = (int(g.x), int(g.y))

        open_heap: list[tuple[float, int, tuple[int, int]]] = []
        heapq.heapify(open_heap)

        g_score: dict[tuple[int, int], float] = {start: 0.0}
        f_score: dict[tuple[int, int], float] = {start: self._heuristic(*start, *goal)}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}

        # 用于打破 f 值相等时的堆冲突
        counter = 0
        heapq.heappush(open_heap, (f_score[start], counter, start))

        closed: set[tuple[int, int]] = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue

            if current == goal:
                return self._reconstruct_path(came_from, current)

            closed.add(current)
            cx, cy = current

            for nx, ny in self._neighbors(cx, cy):
                neighbor = (nx, ny)
                if neighbor in closed:
                    continue

                # 走到邻居的 cost（8 邻域距离）
                step_cost = ((nx - cx) ** 2 + (ny - cy) ** 2) ** 0.5
                tentative_g = g_score[current] + step_cost

                if tentative_g < g_score.get(neighbor, sys.maxsize):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(nx, ny, *goal)
                    f_score[neighbor] = f
                    counter += 1
                    heapq.heappush(open_heap, (f, counter, neighbor))

        print("No path found, algorithm failed!!!")
        self.path = []
        return None

    def show(self):
        """使用 matplotlib 显示当前 self.path 在地图上的效果。"""
        if not self.path:
            print("AStar.show: 当前没有路径可显示")
            return

        z = np.copy(self.map)
        for p in self.path:
            if 0 <= p.y < z.shape[0] and 0 <= p.x < z.shape[1]:
                z[p.y, p.x] = 150

        plt.imshow(z, cmap="gray")
        plt.title("A* Path")
        plt.show()

    def update(self, newst: Point):
        """
        根据机器人新位置，重新规划到原路径中间某点，做局部修正。

        :param newst: 新起点（地图索引坐标）
        :return: 更新后的 self.path（Point 列表）
        """
        if not self.path:
            # 当前没有路径，直接返回
            return self.path

        maxstep = 15
        if len(self.path) > maxstep:
            midp = self.path[maxstep]
            idx = maxstep
        else:
            midp = self.path[-1]
            idx = len(self.path) - 1

        p1 = Point(int(newst.x), int(newst.y))
        # 局部重规划
        new_astar = AStar(self.map, self.obstacle_threshold)
        sub_path = new_astar.Run(p1, Point(int(midp.x), int(midp.y)))

        if sub_path is None or not sub_path:
            # 如果局部重规划失败，就保留原路径
            print("AStar.update: 局部重规划失败，保留原有路径")
            return self.path

        print(len(sub_path), len(self.path), newst.x, newst.y, midp.x, midp.y)

        if idx >= maxstep and len(self.path) > maxstep + 1:
            # 替换前 maxstep 段为新的局部路径
            self.path = sub_path + self.path[maxstep + 1:]
        else:
            # 直接采用局部路径
            self.path = sub_path

        return self.path

    def getspeed(self):
        """
        返回从路径起点到前方某个路径点的距离和角度，用于简单速度规划 demo。
        (仅在独立测试 main() 里用)
        """
        if not self.path:
            return 0.0, 0.0

        if len(self.path) > 20:
            pt = self.path[20]
        else:
            pt = self.path[-1]

        st = self.path[0]
        return st.dist(pt)


def main():
    print("AStar standalone test...")
    mp = (np.random.random((120, 160)) > 0.95) * 100
    stx, sty = 10, 10
    edx, edy = 150, 110
    mp[sty, stx] = 0
    mp[edy, edx] = 0
    astar = AStar(mp)
    points = astar.Run(s=Point(stx, sty), g=Point(edx, edy))
    if points:
        print("path length:", len(points))
        print("getspeed:", astar.getspeed())
        astar.show()
        while len(astar.path) > 20:
            cp = astar.path[5]
            astar.update(cp)
            print("getspeed:", astar.getspeed())
            astar.show()
    else:
        print("No path found in standalone test.")


if __name__ == '__main__':
    main()
