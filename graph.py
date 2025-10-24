import math
from distutils.command.build_scripts import first_line_re

import matplotlib.pyplot as plt
import networkx as nx
import heapq
# ============================================================
# Graph Class (with direction support)
# ============================================================

class Graph:
    """Directed/Undirected weighted graph for building map."""

    def __init__(self):
        self.edges = {}

    def add_edge(self, u: str, v: str, w: float, bidirectional: bool = True):
        """Add edge u→v (and optionally v→u) with weight w."""
        self.edges.setdefault(u, []).append((v, w))
        if bidirectional:
            self.edges.setdefault(v, []).append((u, w))

    def dijkstra(self, start: str, end: str):
        """Simple Dijkstra shortest path."""
        import heapq
        pq = [(0, start, [])]
        visited = set()
        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]
            if node == end:
                return path, cost
            for neighbor, weight in self.edges.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))
        return [], float("inf")

    # 新增的方法
    def dijkstra_k_shortest(self, start: str, end: str, k=2):
        """找到k条最短路径"""
        import heapq
        import copy

        pq = [(0, start, [])]
        visited = {}
        shortest_paths = []

        while pq and len(shortest_paths) < k:
            cost, node, path = heapq.heappop(pq)

            if node not in visited:
                visited[node] = 0
            visited[node] += 1

            if visited[node] > k * 2:  # 限制访问次数，避免无限循环
                continue

            new_path = path + [node]

            if node == end:
                # 检查是否重复路径
                if not any(p == new_path for p, c in shortest_paths):
                    shortest_paths.append((new_path, cost))
                continue

            for neighbor, weight in self.edges.get(node, []):
                neighbor_visit_count = visited.get(neighbor, 0)
                if neighbor_visit_count < k * 2:  # 宽松的限制
                    heapq.heappush(pq, (cost + weight, neighbor, new_path))

        # 按成本排序并返回前k个
        shortest_paths.sort(key=lambda x: x[1])
        while len(shortest_paths) < k:
            shortest_paths.append(([], float("inf")))

        return shortest_paths[:k]

    def get_second_shortest(self, start: str, end: str):
        """获取第二短路径"""
        shortest_path, shortest_cost = self.dijkstra(start, end)

        if not shortest_path or shortest_cost == float("inf"):
            return [], float("inf")

        second_shortest_cost = float("inf")
        second_shortest_path = []

        # 排除最短路径中的每条边，寻找替代路径
        for i in range(len(shortest_path) - 1):
            u, v = shortest_path[i], shortest_path[i + 1]

            # 临时移除边 u->v
            original_edges = self.edges.get(u, [])
            self.edges[u] = [(n, w) for n, w in original_edges if n != v]

            # 寻找替代路径
            alt_path, alt_cost = self.dijkstra(start, end)

            # 恢复边
            self.edges[u] = original_edges

            if alt_path and alt_cost < second_shortest_cost:
                second_shortest_path = alt_path
                second_shortest_cost = alt_cost

        return second_shortest_path, second_shortest_cost

    def find_alternative_routes(self, start: str, end: str, num_routes=2):
        """找到多条备选路线"""
        print(f"\n寻找从 {start} 到 {end} 的 {num_routes} 条最短路径:")
        print("=" * 60)

        paths = self.dijkstra_k_shortest(start, end, num_routes)

        for i, (path, cost) in enumerate(paths, 1):
            if path:
                print(f"第{i}短路径: {cost:.2f}秒")
                print(f"路径: {' → '.join(path)}")
            else:
                print(f"第{i}短路径: 未找到")
            print("-" * 40)

        return paths

    def dijkstra_extra(self, start: str, end: str):
        """Enhanced Dijkstra: returns full path, total time, and safe segment times before/between/after E nodes."""
        import heapq

        def get_edge_weight(u, v):
            """Return weight of edge u->v, raise if not found."""
            for nb, w in self.edges.get(u, []):
                if nb == v:
                    return w
            raise ValueError(f"Edge weight not found for {u} -> {v}")

        pq = [(0, start, [])]
        visited = set()

        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]

            # ---------- 到达终点 ----------
            if node == end:
                total_time = cost
                E_nodes = [n for n in path if ("E1" in n) or ("E2" in n)]

                # 如果少于两个 E 节点，返回安全默认值
                if len(E_nodes) < 2:
                    e1 = E_nodes[0] if len(E_nodes) >= 1 else None
                    e2 = None
                    return {
                        "path": path,
                        "total_time": total_time,
                        "segments": {"before": 0.0, "between": 0.0, "after": total_time},
                        "E_nodes": (e1, e2)  # 永远返回 tuple，不为 None
                    }

                # ---------- 正常情况 ----------
                e1 = E_nodes[0]
                e2 = E_nodes[1]
                idx_e1 = path.index(e1)
                idx_e2 = path.index(e2)

                before = 0.0
                between = 0.0
                after = 0.0

                for i in range(len(path) - 1):
                    u, v = path[i], path[i + 1]
                    w = get_edge_weight(u, v)
                    if i < idx_e1:
                        before += w
                    elif idx_e1 <= i < idx_e2:
                        between += w
                    else:
                        after += w

                return {
                    "path": path,
                    "total_time": total_time,
                    "segments": {"before": before, "between": between, "after": after},
                    "E_nodes": (e1, e2)
                }

            # ---------- 普通 Dijkstra 更新 ----------
            for neighbor, weight in self.edges.get(node, []):
                if neighbor not in visited:
                    heapq.heappush(pq, (cost + weight, neighbor, path))

        # ---------- 无法到达 ----------
        return {
            "path": [],
            "total_time": float("inf"),
            "segments": {"before": 0.0, "between": 0.0, "after": 0.0},
            "E_nodes": (None, None)
        }


# ============================================================
# Graph Initialization
# ============================================================
def initial_six_graphs(speed_land=1.5, speed_stair=0.5):
    # add_1_E_1_graph(speed_land=1.5, speed_stair=0.5)
    stair_graph = inital_graph(speed_land, speed_stair)
    add_1E1_graph = inital_graph(speed_land, speed_stair)
    add_1_E_1_graph(add_1E1_graph)
    add_1E2_graph = inital_graph(speed_land, speed_stair)
    add_1_E_2_graph(add_1E2_graph)
    add_2E1_graph = inital_graph(speed_land, speed_stair)
    add_2_E_1_graph(add_2E1_graph)
    add_2E2_graph = inital_graph(speed_land, speed_stair)
    add_2_E_2_graph(add_2E2_graph)
    add_3E1_graph = inital_graph(speed_land, speed_stair)
    add_3_E_1_graph(add_3E1_graph)
    add_3E2_graph = inital_graph(speed_land, speed_stair)
    add_3_E_2_graph(add_3E2_graph)
    final_graph = inital_graph(speed_land, speed_stair)
    add_elevator_connect(final_graph)
    return stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, final_graph

def inital_graph(speed_land, speed_stair):
    # --- Initialize building graph ---
    graph = Graph()
    stair_length_1 = math.sqrt((175-150)**2+(1.76-1.10)**2)+math.sqrt((175-150)**2+(3.5-2.07)**2)
    stair_length_2 = math.sqrt((405.39-380.67)**2+(1.76-1.10)**2)+math.sqrt((405.39-380.67)**2+(3.5-2.07)**2)
    v_l = speed_land
    v_s = speed_stair

    add_floor1_graph = add_floor1(graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor2_graph = add_floor2(add_floor1_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor3_graph = add_floor3(add_floor2_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor4_graph = add_floor4(add_floor3_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor5_graph = add_floor5(add_floor4_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor6_graph = add_floor6(add_floor5_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor7_graph = add_floor7(add_floor6_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor8_graph = add_floor8(add_floor7_graph, stair_length_1, stair_length_2, v_l, v_s)
    add_floor9_graph = add_floor9(add_floor8_graph, stair_length_1, stair_length_2, v_l, v_s)

    stair_graph = add_floor9_graph
    # add_1E1_graph = add_1_E_1_graph(stair_graph)
    # add_1E2_graph = add_1_E_2_graph(stair_graph)
    # add_2E1_graph = add_2_E_1_graph(stair_graph)
    # add_2E2_graph = add_2_E_2_graph(stair_graph)
    # add_3E1_graph = add_3_E_1_graph(stair_graph)
    # add_3E2_graph = add_3_E_2_graph(stair_graph)
    # final_graph = add_elevator_connect(add_floor9_graph)

    return stair_graph#, add_1E1_graph,add_1E2_graph , add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph#, final_graph

def calculate_elevator_time_need(n):
    time_need = 1.5+1.5+1.75*n+1.5
    return time_need
def add_1_E_1_graph(graph):
    add_1E1_graph = graph
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    add_1E1_graph.add_edge("1_1_E1", "2_1_E1", one_layer)
    add_1E1_graph.add_edge("2_1_E1", "3_1_E1", one_layer)
    add_1E1_graph.add_edge("1_1_E1", "3_1_E1", two_layer)
    return add_1E1_graph

def add_1_E_2_graph(graph):
    add_1E2_graph = graph
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    add_1E2_graph.add_edge("1_1_E2", "2_1_E2", one_layer)
    add_1E2_graph.add_edge("2_1_E2", "3_1_E2", one_layer)
    add_1E2_graph.add_edge("1_1_E2", "3_1_E2", two_layer)
    return add_1E2_graph

def add_2_E_1_graph(graph):
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    three_layer = calculate_elevator_time_need(3)
    four_layer = calculate_elevator_time_need(4)
    five_layer = calculate_elevator_time_need(5)
    six_layer = calculate_elevator_time_need(6)
    seven_layer = calculate_elevator_time_need(7)
    eight_layer = calculate_elevator_time_need(8)
    add_2E1_graph = graph
    add_2E1_graph.add_edge("1_2_E1", "2_2_E1", one_layer)
    add_2E1_graph.add_edge("1_2_E1", "3_2_E1", two_layer)
    add_2E1_graph.add_edge("1_2_E1", "4_2_E1", three_layer)
    add_2E1_graph.add_edge("1_2_E1", "5_2_E1", four_layer)
    add_2E1_graph.add_edge("1_2_E1", "6_2_E1", five_layer)
    add_2E1_graph.add_edge("1_2_E1", "7_2_E1", six_layer)
    add_2E1_graph.add_edge("1_2_E1", "8_2_E1", seven_layer)
    add_2E1_graph.add_edge("1_2_E1", "9_2_E1", eight_layer)
    add_2E1_graph.add_edge("2_2_E1", "3_2_E1", one_layer)
    add_2E1_graph.add_edge("2_2_E1", "4_2_E1", two_layer)
    add_2E1_graph.add_edge("2_2_E1", "5_2_E1", three_layer)
    add_2E1_graph.add_edge("2_2_E1", "6_2_E1", four_layer)
    add_2E1_graph.add_edge("2_2_E1", "7_2_E1", five_layer)
    add_2E1_graph.add_edge("2_2_E1", "8_2_E1", six_layer)
    add_2E1_graph.add_edge("2_2_E1", "9_2_E1", seven_layer)
    add_2E1_graph.add_edge("3_2_E1", "4_2_E1", one_layer)
    add_2E1_graph.add_edge("3_2_E1", "5_2_E1", two_layer)
    add_2E1_graph.add_edge("3_2_E1", "6_2_E1", three_layer)
    add_2E1_graph.add_edge("3_2_E1", "7_2_E1", four_layer)
    add_2E1_graph.add_edge("3_2_E1", "8_2_E1", five_layer)
    add_2E1_graph.add_edge("3_2_E1", "9_2_E1", six_layer)
    add_2E1_graph.add_edge("4_2_E1", "5_2_E1", one_layer)
    add_2E1_graph.add_edge("4_2_E1", "6_2_E1", two_layer)
    add_2E1_graph.add_edge("4_2_E1", "7_2_E1", three_layer)
    add_2E1_graph.add_edge("4_2_E1", "8_2_E1", four_layer)
    add_2E1_graph.add_edge("4_2_E1", "9_2_E1", five_layer)
    add_2E1_graph.add_edge("5_2_E1", "6_2_E1", one_layer)
    add_2E1_graph.add_edge("5_2_E1", "7_2_E1", two_layer)
    add_2E1_graph.add_edge("5_2_E1", "8_2_E1", three_layer)
    add_2E1_graph.add_edge("5_2_E1", "9_2_E1", four_layer)
    add_2E1_graph.add_edge("6_2_E1", "7_2_E1", one_layer)
    add_2E1_graph.add_edge("6_2_E1", "8_2_E1", two_layer)
    add_2E1_graph.add_edge("6_2_E1", "9_2_E1", three_layer)
    add_2E1_graph.add_edge("7_2_E1", "8_2_E1", one_layer)
    add_2E1_graph.add_edge("7_2_E1", "9_2_E1", two_layer)
    add_2E1_graph.add_edge("8_2_E1", "9_2_E1", one_layer)
    return add_2E1_graph
def add_2_E_2_graph(graph):
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    three_layer = calculate_elevator_time_need(3)
    four_layer = calculate_elevator_time_need(4)
    five_layer = calculate_elevator_time_need(5)
    six_layer = calculate_elevator_time_need(6)
    seven_layer = calculate_elevator_time_need(7)
    eight_layer = calculate_elevator_time_need(8)
    add_2E2_graph = graph
    add_2E2_graph.add_edge("1_2_E2", "2_2_E2", one_layer)
    add_2E2_graph.add_edge("1_2_E2", "3_2_E2", two_layer)
    add_2E2_graph.add_edge("1_2_E2", "4_2_E2", three_layer)
    add_2E2_graph.add_edge("1_2_E2", "5_2_E2", four_layer)
    add_2E2_graph.add_edge("1_2_E2", "6_2_E2", five_layer)
    add_2E2_graph.add_edge("1_2_E2", "7_2_E2", six_layer)
    add_2E2_graph.add_edge("1_2_E2", "8_2_E2", seven_layer)
    add_2E2_graph.add_edge("1_2_E2", "9_2_E2", eight_layer)
    add_2E2_graph.add_edge("2_2_E2", "3_2_E2", one_layer)
    add_2E2_graph.add_edge("2_2_E2", "4_2_E2", two_layer)
    add_2E2_graph.add_edge("2_2_E2", "5_2_E2", three_layer)
    add_2E2_graph.add_edge("2_2_E2", "6_2_E2", four_layer)
    add_2E2_graph.add_edge("2_2_E2", "7_2_E2", five_layer)
    add_2E2_graph.add_edge("2_2_E2", "8_2_E2", six_layer)
    add_2E2_graph.add_edge("2_2_E2", "9_2_E2", seven_layer)
    add_2E2_graph.add_edge("3_2_E2", "4_2_E2", one_layer)
    add_2E2_graph.add_edge("3_2_E2", "5_2_E2", two_layer)
    add_2E2_graph.add_edge("3_2_E2", "6_2_E2", three_layer)
    add_2E2_graph.add_edge("3_2_E2", "7_2_E2", four_layer)
    add_2E2_graph.add_edge("3_2_E2", "8_2_E2", five_layer)
    add_2E2_graph.add_edge("3_2_E2", "9_2_E2", six_layer)
    add_2E2_graph.add_edge("4_2_E2", "5_2_E2", one_layer)
    add_2E2_graph.add_edge("4_2_E2", "6_2_E2", two_layer)
    add_2E2_graph.add_edge("4_2_E2", "7_2_E2", three_layer)
    add_2E2_graph.add_edge("4_2_E2", "8_2_E2", four_layer)
    add_2E2_graph.add_edge("4_2_E2", "9_2_E2", five_layer)
    add_2E2_graph.add_edge("5_2_E2", "6_2_E2", one_layer)
    add_2E2_graph.add_edge("5_2_E2", "7_2_E2", two_layer)
    add_2E2_graph.add_edge("5_2_E2", "8_2_E2", three_layer)
    add_2E2_graph.add_edge("5_2_E2", "9_2_E2", four_layer)
    add_2E2_graph.add_edge("6_2_E2", "7_2_E2", one_layer)
    add_2E2_graph.add_edge("6_2_E2", "8_2_E2", two_layer)
    add_2E2_graph.add_edge("6_2_E2", "9_2_E2", three_layer)
    add_2E2_graph.add_edge("7_2_E2", "8_2_E2", one_layer)
    add_2E2_graph.add_edge("7_2_E2", "9_2_E2", two_layer)
    add_2E2_graph.add_edge("8_2_E2", "9_2_E2", one_layer)
    return add_2E2_graph
def add_3_E_1_graph(graph):
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    three_layer = calculate_elevator_time_need(3)
    four_layer = calculate_elevator_time_need(4)
    five_layer = calculate_elevator_time_need(5)
    add_3E1_graph = graph
    add_3E1_graph.add_edge("1_3_E1", "2_3_E1", one_layer)
    add_3E1_graph.add_edge("1_3_E1", "3_3_E1", two_layer)
    add_3E1_graph.add_edge("1_3_E1", "4_3_E1", three_layer)
    add_3E1_graph.add_edge("1_3_E1", "5_3_E1", four_layer)
    add_3E1_graph.add_edge("1_3_E1", "6_3_E1", five_layer)
    add_3E1_graph.add_edge("2_3_E1", "3_3_E1", one_layer)
    add_3E1_graph.add_edge("2_3_E1", "4_3_E1", two_layer)
    add_3E1_graph.add_edge("2_3_E1", "5_3_E1", three_layer)
    add_3E1_graph.add_edge("2_3_E1", "6_3_E1", four_layer)
    add_3E1_graph.add_edge("3_3_E1", "4_3_E1", one_layer)
    add_3E1_graph.add_edge("3_3_E1", "5_3_E1", two_layer)
    add_3E1_graph.add_edge("3_3_E1", "6_3_E1", three_layer)
    add_3E1_graph.add_edge("4_3_E1", "5_3_E1", one_layer)
    add_3E1_graph.add_edge("4_3_E1", "6_3_E1", two_layer)
    add_3E1_graph.add_edge("5_3_E1", "6_3_E1", one_layer)
    return add_3E1_graph
def add_3_E_2_graph(graph):
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    three_layer = calculate_elevator_time_need(3)
    four_layer = calculate_elevator_time_need(4)
    five_layer = calculate_elevator_time_need(5)
    add_3E2_graph = graph
    add_3E2_graph.add_edge("1_3_E2", "2_3_E2", one_layer)
    add_3E2_graph.add_edge("1_3_E2", "3_3_E2", two_layer)
    add_3E2_graph.add_edge("1_3_E2", "4_3_E2", three_layer)
    add_3E2_graph.add_edge("1_3_E2", "5_3_E2", four_layer)
    add_3E2_graph.add_edge("1_3_E2", "6_3_E2", five_layer)
    add_3E2_graph.add_edge("2_3_E2", "3_3_E2", one_layer)
    add_3E2_graph.add_edge("2_3_E2", "4_3_E2", two_layer)
    add_3E2_graph.add_edge("2_3_E2", "5_3_E2", three_layer)
    add_3E2_graph.add_edge("2_3_E2", "6_3_E2", four_layer)
    add_3E2_graph.add_edge("3_3_E2", "4_3_E2", one_layer)
    add_3E2_graph.add_edge("3_3_E2", "5_3_E2", two_layer)
    add_3E2_graph.add_edge("3_3_E2", "6_3_E2", three_layer)
    add_3E2_graph.add_edge("4_3_E2", "5_3_E2", one_layer)
    add_3E2_graph.add_edge("4_3_E2", "6_3_E2", two_layer)
    add_3E2_graph.add_edge("5_3_E2", "6_3_E2", one_layer)
    return add_3E2_graph


def add_elevator_connect(graph):
    one_layer = calculate_elevator_time_need(1)
    two_layer = calculate_elevator_time_need(2)
    three_layer = calculate_elevator_time_need(3)
    four_layer = calculate_elevator_time_need(4)
    five_layer = calculate_elevator_time_need(5)
    six_layer = calculate_elevator_time_need(6)
    seven_layer = calculate_elevator_time_need(7)
    eight_layer = calculate_elevator_time_need(8)
    # add elevator for building 1
    graph.add_edge("1_1_E1", "2_1_E1", one_layer)
    graph.add_edge("2_1_E1", "3_1_E1", one_layer)
    graph.add_edge("1_1_E1", "3_1_E1", two_layer)
    graph.add_edge("1_1_E2", "2_1_E2", one_layer)
    graph.add_edge("2_1_E2", "3_1_E2", one_layer)
    graph.add_edge("1_1_E2", "3_1_E2", two_layer)
    # add elevator for building 3
    graph.add_edge("1_3_E1", "2_3_E1", one_layer)
    graph.add_edge("1_3_E1", "3_3_E1", two_layer)
    graph.add_edge("1_3_E1", "4_3_E1", three_layer)
    graph.add_edge("1_3_E1", "5_3_E1", four_layer)
    graph.add_edge("1_3_E1", "6_3_E1", five_layer)
    graph.add_edge("2_3_E1", "3_3_E1", one_layer)
    graph.add_edge("2_3_E1", "4_3_E1", two_layer)
    graph.add_edge("2_3_E1", "5_3_E1", three_layer)
    graph.add_edge("2_3_E1", "6_3_E1", four_layer)
    graph.add_edge("3_3_E1", "4_3_E1", one_layer)
    graph.add_edge("3_3_E1", "5_3_E1", two_layer)
    graph.add_edge("3_3_E1", "6_3_E1", three_layer)
    graph.add_edge("4_3_E1", "5_3_E1", one_layer)
    graph.add_edge("4_3_E1", "6_3_E1", two_layer)
    graph.add_edge("5_3_E1", "6_3_E1", one_layer)
    graph.add_edge("1_3_E2", "2_3_E2", one_layer)
    graph.add_edge("1_3_E2", "3_3_E2", two_layer)
    graph.add_edge("1_3_E2", "4_3_E2", three_layer)
    graph.add_edge("1_3_E2", "5_3_E2", four_layer)
    graph.add_edge("1_3_E2", "6_3_E2", five_layer)
    graph.add_edge("2_3_E2", "3_3_E2", one_layer)
    graph.add_edge("2_3_E2", "4_3_E2", two_layer)
    graph.add_edge("2_3_E2", "5_3_E2", three_layer)
    graph.add_edge("2_3_E2", "6_3_E2", four_layer)
    graph.add_edge("3_3_E2", "4_3_E2", one_layer)
    graph.add_edge("3_3_E2", "5_3_E2", two_layer)
    graph.add_edge("3_3_E2", "6_3_E2", three_layer)
    graph.add_edge("4_3_E2", "5_3_E2", one_layer)
    graph.add_edge("4_3_E2", "6_3_E2", two_layer)
    graph.add_edge("5_3_E2", "6_3_E2", one_layer)
    # add elevator for building 2
    graph.add_edge("1_2_E1", "2_2_E1", one_layer)
    graph.add_edge("1_2_E1", "3_2_E1", two_layer)
    graph.add_edge("1_2_E1", "4_2_E1", three_layer)
    graph.add_edge("1_2_E1", "5_2_E1", four_layer)
    graph.add_edge("1_2_E1", "6_2_E1", five_layer)
    graph.add_edge("1_2_E1", "7_2_E1", six_layer)
    graph.add_edge("1_2_E1", "8_2_E1", seven_layer)
    graph.add_edge("1_2_E1", "9_2_E1", eight_layer)
    graph.add_edge("2_2_E1", "3_2_E1", one_layer)
    graph.add_edge("2_2_E1", "4_2_E1", two_layer)
    graph.add_edge("2_2_E1", "5_2_E1", three_layer)
    graph.add_edge("2_2_E1", "6_2_E1", four_layer)
    graph.add_edge("2_2_E1", "7_2_E1", five_layer)
    graph.add_edge("2_2_E1", "8_2_E1", six_layer)
    graph.add_edge("2_2_E1", "9_2_E1", seven_layer)
    graph.add_edge("3_2_E1", "4_2_E1", one_layer)
    graph.add_edge("3_2_E1", "5_2_E1", two_layer)
    graph.add_edge("3_2_E1", "6_2_E1", three_layer)
    graph.add_edge("3_2_E1", "7_2_E1", four_layer)
    graph.add_edge("3_2_E1", "8_2_E1", five_layer)
    graph.add_edge("3_2_E1", "9_2_E1", six_layer)
    graph.add_edge("4_2_E1", "5_2_E1", one_layer)
    graph.add_edge("4_2_E1", "6_2_E1", two_layer)
    graph.add_edge("4_2_E1", "7_2_E1", three_layer)
    graph.add_edge("4_2_E1", "8_2_E1", four_layer)
    graph.add_edge("4_2_E1", "9_2_E1", five_layer)
    graph.add_edge("5_2_E1", "6_2_E1", one_layer)
    graph.add_edge("5_2_E1", "7_2_E1", two_layer)
    graph.add_edge("5_2_E1", "8_2_E1", three_layer)
    graph.add_edge("5_2_E1", "9_2_E1", four_layer)
    graph.add_edge("6_2_E1", "7_2_E1", one_layer)
    graph.add_edge("6_2_E1", "8_2_E1", two_layer)
    graph.add_edge("6_2_E1", "9_2_E1", three_layer)
    graph.add_edge("7_2_E1", "8_2_E1", one_layer)
    graph.add_edge("7_2_E1", "9_2_E1", two_layer)
    graph.add_edge("8_2_E1", "9_2_E1", one_layer)

    graph.add_edge("1_2_E2", "2_2_E2", one_layer)
    graph.add_edge("1_2_E2", "3_2_E2", two_layer)
    graph.add_edge("1_2_E2", "4_2_E2", three_layer)
    graph.add_edge("1_2_E2", "5_2_E2", four_layer)
    graph.add_edge("1_2_E2", "6_2_E2", five_layer)
    graph.add_edge("1_2_E2", "7_2_E2", six_layer)
    graph.add_edge("1_2_E2", "8_2_E2", seven_layer)
    graph.add_edge("1_2_E2", "9_2_E2", eight_layer)
    graph.add_edge("2_2_E2", "3_2_E2", one_layer)
    graph.add_edge("2_2_E2", "4_2_E2", two_layer)
    graph.add_edge("2_2_E2", "5_2_E2", three_layer)
    graph.add_edge("2_2_E2", "6_2_E2", four_layer)
    graph.add_edge("2_2_E2", "7_2_E2", five_layer)
    graph.add_edge("2_2_E2", "8_2_E2", six_layer)
    graph.add_edge("2_2_E2", "9_2_E2", seven_layer)
    graph.add_edge("3_2_E2", "4_2_E2", one_layer)
    graph.add_edge("3_2_E2", "5_2_E2", two_layer)
    graph.add_edge("3_2_E2", "6_2_E2", three_layer)
    graph.add_edge("3_2_E2", "7_2_E2", four_layer)
    graph.add_edge("3_2_E2", "8_2_E2", five_layer)
    graph.add_edge("3_2_E2", "9_2_E2", six_layer)
    graph.add_edge("4_2_E2", "5_2_E2", one_layer)
    graph.add_edge("4_2_E2", "6_2_E2", two_layer)
    graph.add_edge("4_2_E2", "7_2_E2", three_layer)
    graph.add_edge("4_2_E2", "8_2_E2", four_layer)
    graph.add_edge("4_2_E2", "9_2_E2", five_layer)
    graph.add_edge("5_2_E2", "6_2_E2", one_layer)
    graph.add_edge("5_2_E2", "7_2_E2", two_layer)
    graph.add_edge("5_2_E2", "8_2_E2", three_layer)
    graph.add_edge("5_2_E2", "9_2_E2", four_layer)
    graph.add_edge("6_2_E2", "7_2_E2", one_layer)
    graph.add_edge("6_2_E2", "8_2_E2", two_layer)
    graph.add_edge("6_2_E2", "9_2_E2", three_layer)
    graph.add_edge("7_2_E2", "8_2_E2", one_layer)
    graph.add_edge("7_2_E2", "9_2_E2", two_layer)
    graph.add_edge("8_2_E2", "9_2_E2", one_layer)

    return graph

def add_a_building_layer(graph, current_building, up_building, right_building, stair_length_1, stair_length_2,v_l,v_s):
    graph.add_edge(current_building+"Left_2", current_building+"A", (60-3)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Left_2", current_building+"Left_1", (113-92)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Left_1", current_building+"Left_2", (113-92)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Ar", current_building+"Left_1", (60-3)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Ar", current_building+"A", (113 - 92)/v_l, bidirectional=False)
    graph.add_edge(current_building+"A", current_building+"Ar", (113 - 92)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Br", current_building+"B", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"B", current_building+"Br", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Cr", current_building+"C", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"C", current_building+"Cr", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Dr", current_building+"D", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"D", current_building+"Dr", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Er", current_building+"E", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"E", current_building+"Er", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Fr", current_building+"F", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"F", current_building+"Fr", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Gr", current_building+"G", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"G", current_building+"Gr", (113 - 92) / v_l, bidirectional=False)

    graph.add_edge(current_building+"Ewr", current_building+"Ew", (113 - 92) / v_l, bidirectional=False)
    graph.add_edge(current_building+"Ew", current_building+"Ewr", (113 - 92) / v_l, bidirectional=False)


    graph.add_edge(current_building+"A", current_building+"B", (75-60)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Br", current_building+"Ar", (75-60)/v_l, bidirectional=False)

    graph.add_edge(current_building+"B", current_building+"Er", (77-75)/v_l, bidirectional=False)
    graph.add_edge(current_building+"E", current_building+"Br", (77-75)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Stair1_1", current_building+"E",  (93-77)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Stair1_1", current_building+"Stair1_2", (122-93)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Cr", current_building+"Stair1_2",  (143-122)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Er", current_building+"Stair1_2",  (122-77+113-92)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Er", current_building+"C", (153-77)/v_l, bidirectional=False)

    graph.add_edge(current_building+"F", current_building+"Cr", (200-143)/v_l, bidirectional=False)
    graph.add_edge(current_building+"C", current_building+"Fr", (200-143)/v_l, bidirectional=False)


    graph.add_edge(current_building+"Fr", current_building+"D", (220-200)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Dr", current_building+"F", (220-200)/v_l, bidirectional=False)

    graph.add_edge(current_building+"G", current_building+"Dr", (293-220)/v_l, bidirectional=False)
    graph.add_edge(current_building+"D", current_building+"Gr", (293-220)/v_l, bidirectional=False)

    graph.add_edge(current_building+"Ew", current_building+"G", (380-293)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Stair2_1", current_building+"G",  (380-293)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Ew", current_building+"Stair2_2", stair_length_2/v_l)

    graph.add_edge(current_building+"Gr", current_building+"Ewr", (380-293)/v_l,bidirectional=False)
    graph.add_edge(current_building+"Gr", current_building+"E1", (380-293+150-100)/v_l,bidirectional=False)
    graph.add_edge(current_building+"Gr", current_building+"E2", (410-293+150-100)/v_l,bidirectional=False)
    graph.add_edge(current_building+"Gr", current_building+"Stair2_2", (380-293)/v_l,bidirectional=False)


    graph.add_edge(current_building+"Ew", current_building+"E1", (150-110)/v_l,bidirectional=False)
    graph.add_edge(current_building+"E1", current_building+"Ew", (150-110)/v_l,bidirectional=False)

    graph.add_edge(current_building+"E1", current_building+"E2", (410-380)/v_l,bidirectional=False)
    graph.add_edge(current_building+"E2", current_building+"E1", (410-380)/v_l,bidirectional=False)


    graph.add_edge(current_building+"Right_1",current_building+"Ew", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Ewr", current_building+"Right_2", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Right_2", current_building+"Right_1", (113-92)/v_l,bidirectional=False)
    graph.add_edge(current_building+"Right_1", current_building+"Right_2", (113-92)/v_l,bidirectional=False)


    #Building 1 first floor to second floor
    if up_building != "":
        graph.add_edge(up_building + "Stair1_1", current_building + "Stair1_1", stair_length_1 / v_s,
                       bidirectional=False)
        graph.add_edge(current_building + "Stair1_2", up_building + "Stair1_2", stair_length_1 / v_s,
                       bidirectional=False)
        graph.add_edge(up_building + "Stair2_1", current_building + "Stair2_1", stair_length_2 / v_s,
                       bidirectional=False)
        graph.add_edge(current_building + "Stair2_2", up_building + "Stair2_2", stair_length_2 / v_s,
                       bidirectional=False)
    # 一层 一号楼 几号楼梯 左1右2
    if right_building != "":
        graph.add_edge(right_building + "Left_1", current_building + "Right_1", 200 / v_l, bidirectional=False)
        graph.add_edge(current_building + "Right_2", right_building + "Left_2", 200 / v_l, bidirectional=False)
        graph.add_edge(right_building + "Left_1", right_building + "Left_2", 200 / v_l)
    # Building1-2

    return graph


def add_floor1(graph,stair_length_1,stair_length_2,v_l,v_s):
    # Floor 1
    # Building 1
    current_building = "1_1_"
    up_building = "2_1_"
    right_building = "1_2_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "1_2_"
    up_building = "2_2_"
    right_building = "1_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "1_3_"
    up_building = "2_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

def add_floor2(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 3
    # Building 1
    current_building = "2_1_"
    up_building = "3_1_"
    right_building = "2_2_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "2_2_"
    up_building = "3_2_"
    right_building = "2_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "2_3_"
    up_building = "3_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)


    return graph

def add_floor3(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    current_building = "3_1_"
    up_building = ""
    right_building = "3_2_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "3_2_"
    up_building = "4_2_"
    right_building = "3_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "3_3_"
    up_building = "4_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

def add_floor4(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "4_2_"
    up_building = "5_2_"
    right_building = "4_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "4_3_"
    up_building = "5_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

def add_floor5(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "5_2_"
    up_building = "6_2_"
    right_building = "5_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "5_3_"
    up_building = "6_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)


    return graph

def add_floor6(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "6_2_"
    up_building = "7_2_"
    right_building = "6_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "6_3_"
    up_building = "7_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)


    return graph

def add_floor7(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "7_2_"
    up_building = "8_2_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

def add_floor8(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "8_2_"
    up_building = "9_2_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

def add_floor9(graph, stair_length_1, stair_length_2, v_l, v_s):

    current_building = "9_2_"
    up_building = ""
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    return graph

# ============================================================
# Test
# ============================================================
# 在 __main__ 中添加测试
if __name__ == "__main__":
    stair_graph, add_1E1_graph,add_1E2_graph , add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph,final_graph= initial_six_graphs(speed_land=1.5, speed_stair=0.5)
    # # Example pathfinding
    #stair
    path_stair, cost_stair = stair_graph.dijkstra("1_1_Left_2", "6_3_F")
    print("\nStair Shortest path:", path_stair)
    print("Stair Total travel time:", round(cost_stair, 2), "s")
    #1E1
    # path_1E1, cost_1E1 = add_1E1_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\n1E1 Shortest path:", path_1E1)
    # print("1E1 Total travel time:", round(cost_1E1, 2), "s")

    result_1E1 = add_3E1_graph.dijkstra_extra("1_1_Left_2", "6_3_B")
    path1E1 = result_1E1["path"]
    time_1E1 = result_1E1["total_time"]
    before_1E1 = result_1E1["segments"]["before"]
    after_1E1 = result_1E1["segments"]["after"]
    between_1E1 = result_1E1["segments"]["between"]
    start_1E1 = result_1E1["E_nodes"][0]
    end_1E1 = result_1E1["E_nodes"][1]
    print(result_1E1["path"])
    print(result_1E1["total_time"])
    print(result_1E1["segments"])
    print(result_1E1["segments"]["before"])
    print(result_1E1["segments"]["between"])
    print(result_1E1["segments"]["after"])
    print(result_1E1["E_nodes"])
    print(result_1E1["E_nodes"][0])
    print(result_1E1["E_nodes"][1])

    #1E2
    # path_1E2, cost_1E2 = add_1E2_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\n1E2 Shortest path:", path_1E2)
    # print("1E2 Total travel time:", round(cost_1E2, 2), "s")
    # #2E1
    # path_2E1, cost_2E1 = add_2E1_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\n2E1 Shortest path:", path_2E1)
    # print("2E1 Total travel time:", round(cost_2E1, 2), "s")
    # #2E2
    # path_2E2, cost_2E2 = add_2E2_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\n2E2 Shortest path:", path_2E2)
    # print("2E2 Total travel time:", round(cost_2E2, 2), "s")
    # #3E1
    # path_3E1, cost_3E1 = add_3E1_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\n3E1 Shortest path:", path_3E1)
    # print("3E1 Total travel time:", round(cost_3E1, 2), "s")
    # #3E2
    # path_3E2, cost_3E2 = add_3E2_graph.dijkstra("1_1_Left_2", "6_3_F")
    # print("\n3E2 Shortest path:", path_3E2)
    # print("3E2 Total travel time:", round(cost_3E2, 2), "s")
    # #total_path
    # path_total, cost_total = final_graph.dijkstra("1_1_Left_2", "6_3_B")
    # print("\nTotal Shortest path:", path_total)
    # print("Total Total travel time:", round(cost_total, 2), "s")

    # # 测试第二短路径
    # start = "1_1_Left_2"
    # end = "6_3_B"
    #
    # # 使用新方法
    # routes = graph.find_alternative_routes(start, end, 3)
    #
    # # 单独获取第二短路径
    # second_path, second_cost = graph.get_second_shortest(start, end)
    # if second_path:
    #     print(f"\n第二短路径: {second_cost:.2f}秒")
    #     print(f"路径: {' → '.join(second_path)}")
