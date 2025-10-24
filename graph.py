import math
import matplotlib.pyplot as plt
import networkx as nx

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


# ============================================================
# Graph Initialization
# ============================================================


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
    final_graph = add_elevator_connect(add_floor9_graph)

    return final_graph

def calculate_elevator_time_need(n):
    time_need = 1.5+1.5+1.75*n+1.5
    return time_need

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
    graph.add_edge(current_building+"Left_2", current_building+"Left_1", (113-92)/v_l)
    graph.add_edge(current_building+"Ar", current_building+"Left_1", (60-3)/v_l, bidirectional=False)

    graph.add_edge(current_building+"A", current_building+"Ar", (113 - 92)/v_l)
    graph.add_edge(current_building+"B", current_building+"Br", (113 - 92) / v_l)
    graph.add_edge(current_building+"C", current_building+"Cr", (113 - 92) / v_l)
    graph.add_edge(current_building+"D", current_building+"Dr", (113 - 92) / v_l)
    graph.add_edge(current_building+"E", current_building+"Er", (113 - 92) / v_l)
    graph.add_edge(current_building+"F", current_building+"Fr", (113 - 92) / v_l)
    graph.add_edge(current_building+"G", current_building+"Gr", (113 - 92) / v_l)
    graph.add_edge(current_building+"Ew", current_building+"Ewr", (113 - 92) / v_l)

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
    graph.add_edge(current_building+"Gr", current_building+"Stair2_2", (380-293)/v_l,bidirectional=False)


    graph.add_edge(current_building+"Ew", current_building+"E1", (150-110)/v_l)
    graph.add_edge(current_building+"E1", current_building+"E2", (410-380)/v_l)

    graph.add_edge(current_building+"Right_1",current_building+"Ew", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Ewr", current_building+"Right_2", (433-380)/v_l, bidirectional=False)
    graph.add_edge(current_building+"Right_2", current_building+"Right_1", (113-92)/v_l)

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

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
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

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
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

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor4(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "4_2_"
    up_building = "5_2_"
    right_building = "4_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "4_3_"
    up_building = "5_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor5(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "5_2_"
    up_building = "6_2_"
    right_building = "5_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "5_3_"
    up_building = "6_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor6(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "6_2_"
    up_building = "7_2_"
    right_building = "6_3_"
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    current_building = "6_3_"
    up_building = "7_3_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor7(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "7_2_"
    up_building = "8_2_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    # current_building = "1_3_"
    # up_building = "2_3_"
    # right_building = ""
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor8(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "8_2_"
    up_building = "9_2_"
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    # current_building = "1_3_"
    # up_building = "2_3_"
    # right_building = ""
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph

def add_floor9(graph, stair_length_1, stair_length_2, v_l, v_s):
    # Floor 1
    # Building 1
    # current_building = "1_1_"
    # up_building = "2_1_"
    # right_building = "1_2_"
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 2################################
    current_building = "9_2_"
    up_building = ""
    right_building = ""
    add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)
    # Building 3
    # current_building = "1_3_"
    # up_building = "2_3_"
    # right_building = ""
    # add_a_building_layer(graph,current_building,up_building,right_building,stair_length_1,stair_length_2,v_l,v_s)

    # --- Print connectivity summary ---
    print("=== Graph Connectivity Summary ===")
    for node, neighbors in graph.edges.items():
        print(f"{node}:")
        for n, w in neighbors:
            print(f"   → {n} (time={w:.2f}s)")
    print(f"\nTotal Nodes: {len(graph.edges)}")
    return graph
# ============================================================
# Test
# ============================================================

if __name__ == "__main__":
    graph = inital_graph(speed_land=1.5, speed_stair=0.5)

    # # Example pathfinding
    # path, cost = graph.dijkstra("1_1_Left_2", "1_1_Ewr")
    # print("\nShortest path:", path)
    # print("Total travel time:", round(cost, 2), "s")
