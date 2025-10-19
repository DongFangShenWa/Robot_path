#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多机器人调度与路径规划（演示实现）
Features:
- Graph nodes are (ord, floor) represented by string keys "ord:floor"
- Edges have types: 'flat' (same floor), 'stair' (cross-floor), 'lift' (elevator link)
- EdgeTag records occupied_until (time). LiftTag records schedule and capacity (here capacity=1).
- FindShortestPath implements a Dijkstra-like algorithm that allows waiting for resource availability.
- Greedy task assignment: for each task (by time), choose robot with minimal travel time to task.start.
"""

from heapq import heappush, heappop
from collections import defaultdict, namedtuple
import math

# ---- Data classes ----
class Robot:
    def __init__(self, id, rtype, skill, ord, floor, v_flat, v_stair):
        self.id = id
        self.type = rtype
        self.skill = skill
        self.ord = ord
        self.floor = floor
        self.position = (ord, floor)
        self.v_flat = v_flat      # speed on flat edges (m/s)
        self.v_stair = v_stair    # speed on stairs (m/s)
        self.available_time = 0.0 # next time robot is free

    def speed_for_edge(self, edge_type):
        if edge_type == 'stair':
            return self.v_stair
        else:
            return self.v_flat

    def __repr__(self):
        return f"Robot({self.id}, pos={self.position}, skill={self.skill}, avail={self.available_time})"

class Task:
    def __init__(self, id, skill_req, start, target, time):
        self.id = id
        self.skill_req = skill_req
        self.start = start      # (ord, floor)
        self.target = target    # (ord, floor)
        self.time = time        # task arrival / earliest start (could be deadline in other designs)

    def __repr__(self):
        return f"Task({self.id}, {self.start}->{self.target}, skill={self.skill_req}, time={self.time})"

Edge = namedtuple('Edge', ['u','v','length','etype','id'])  # etype: 'flat'|'stair'|'lift'

# ---- Resource tags ----
class EdgeTag:
    def __init__(self):
        # maps edge_id -> occupied_until_time (float)
        self.occupied_until = defaultdict(float)  # default 0.0 (free)

    def get_free_time(self, edge_id):
        return self.occupied_until.get(edge_id, 0.0)

    def reserve(self, edge_id, start_time, duration):
        end = start_time + duration
        # We assume single reservation per edge per time; if overlapping, this simply moves the occupied_until forward.
        self.occupied_until[edge_id] = max(self.occupied_until.get(edge_id, 0.0), end)

class LiftTag:
    def __init__(self, lift_ids):
        # For each lift_id, track next_free_time (capacity=1)
        self.next_free = {lid: 0.0 for lid in lift_ids}
        # If you want richer schedule, replace next_free with a list of reservations.
    def get_free_time(self, lift_id):
        return self.next_free.get(lift_id, 0.0)
    def reserve(self, lift_id, start_time, duration):
        end = start_time + duration
        self.next_free[lift_id] = max(self.next_free.get(lift_id, 0.0), end)

# ---- Graph ----
class Graph:
    def __init__(self):
        # adjacency: node -> list of Edge
        self.adj = defaultdict(list)
        self.edges = {}  # edge_id -> Edge

    def add_edge(self, u, v, length, etype, edge_id=None):
        if edge_id is None:
            edge_id = f"{u}-{v}"
        e = Edge(u, v, length, etype, edge_id)
        self.adj[u].append(e)
        # also add reverse (bidirectional)
        e_rev = Edge(v, u, length, etype, edge_id)  # same id for both directions (shared resource)
        self.adj[v].append(e_rev)
        self.edges[edge_id] = e

    def neighbors(self, node):
        return self.adj.get(node, [])

# ---- Pathfinding (Dijkstra-like with time and availability) ----
def FindShortestPath(G: Graph, robot: Robot, start_node, end_node, current_time, edge_tag: EdgeTag, lift_tag: LiftTag, lift_ids_map=None):
    """
    Return: (path_nodes_list, arrival_time) or (None, math.inf) if unreachable.
    This implementation allows waiting when an edge is occupied: it computes earliest arrival time to each node.
    lift_ids_map: dict mapping edge_id -> lift_id if edge etype is 'lift'.
    """
    if lift_ids_map is None:
        lift_ids_map = {}

    INF = float('inf')
    dist = defaultdict(lambda: INF)  # earliest arrival time at node (absolute time)
    prev = {}  # node -> (prev_node, edge_used_id, start_time_on_edge)
    pq = []

    # initial time to start node is max(robot.available_time, current_time)
    start_time0 = max(robot.available_time, current_time)
    dist[start_node] = start_time0
    heappush(pq, (dist[start_node], start_node))

    while pq:
        t_u, u = heappop(pq)
        # If popped time is larger than stored dist, skip
        if t_u > dist[u] + 1e-9:
            continue
        if u == end_node:
            break

        for edge in G.neighbors(u):
            v = edge.v
            # travel time depends on robot's speed for that edge type
            speed = robot.speed_for_edge(edge.etype)
            if speed <= 0:
                continue  # cannot traverse
            travel_time = edge.length / speed

            # earliest time robot can start traversing this edge:
            earliest_arrival_at_u = dist[u]
            # check edge availability
            edge_free = edge_tag.get_free_time(edge.id)
            start_on_edge = max(earliest_arrival_at_u, edge_free)  # wait if needed

            # if lift, check lift availability via lift_tag
            if edge.etype == 'lift':
                lift_id = lift_ids_map.get(edge.id, edge.id)  # default lift id = edge id
                lift_free = lift_tag.get_free_time(lift_id)
                start_on_edge = max(start_on_edge, lift_free)

            new_arrival_time = start_on_edge + travel_time

            # relax
            if new_arrival_time + 1e-9 < dist[v]:
                dist[v] = new_arrival_time
                prev[v] = (u, edge.id, start_on_edge)
                heappush(pq, (dist[v], v))

    if dist[end_node] == INF:
        return None, INF

    # reconstruct path nodes and edge ids
    path_nodes = []
    cur = end_node
    edges_used = []
    while cur != start_node:
        if cur not in prev:
            break
        u, edge_id, start_on_edge = prev[cur]
        path_nodes.append(cur)
        edges_used.append((edge_id, start_on_edge))
        cur = u
    path_nodes.append(start_node)
    path_nodes.reverse()
    edges_used.reverse()  # align with path between nodes

    # return both path nodes and list of edges with start times (useful for reservation)
    return (path_nodes, edges_used), dist[end_node]

# ---- Reservation update after choosing path ----
def ReservePath(edge_tag: EdgeTag, lift_tag: LiftTag, edges_used, graph_edges, robot, start_time0, lift_ids_map=None):
    """
    edges_used: list of (edge_id, start_time_on_edge) in the order of traversal.
    Will reserve each edge / lift for the robot using durations computed from graph length and robot speed.
    Returns final end time.
    """
    if lift_ids_map is None:
        lift_ids_map = {}
    t = start_time0
    for edge_id, start_time_on_edge in edges_used:
        edge = graph_edges[edge_id]
        # compute travel time using robot speed
        travel_time = edge.length / robot.speed_for_edge(edge.etype)
        # if start_time_on_edge may be earlier than current t due to rounding, enforce monotonicity:
        start = max(t, start_time_on_edge)
        # reserve edge
        edge_tag.reserve(edge_id, start, travel_time)
        if edge.etype == 'lift':
            lift_id = lift_ids_map.get(edge_id, edge_id)
            lift_tag.reserve(lift_id, start, travel_time)
        t = start + travel_time
    return t

# ---- Main greedy scheduler ----
def greedy_schedule(graph: Graph, robots, tasks, current_time, edge_tag: EdgeTag, lift_tag: LiftTag, lift_ids_map=None):
    # tasks sorted by their time (earliest first)
    tasks_sorted = sorted(tasks, key=lambda x: x.time)
    solutions = []  # list of dicts {task, robot, path_nodes, start_time, end_time}
    failures = []

    for task in tasks_sorted:
        # candidates: robots with needed skill and available before task.time (we allow robots that can start at or after task.time)
        candidates = [r for r in robots if r.skill == task.skill_req]
        best = None
        best_arrival = float('inf')
        best_details = None

        for r in candidates:
            # compute path from robot current pos to task.start
            path_info, arrival_time = FindShortestPath(graph, r, r.position, task.start, current_time, edge_tag, lift_tag, lift_ids_map)
            if path_info is None:
                continue
            # if robot can't reach until after some large time, skip (or allow)
            if arrival_time < best_arrival:
                best_arrival = arrival_time
                best = r
                best_details = path_info  # (path_nodes, edges_used)

        if best is None:
            failures.append((task, "no feasible robot/path"))
            continue

        # Now compute path from task.start to task.target (robot is assumed to arrive at task.start at best_arrival)
        # We'll treat robot as starting at task.start at time best_arrival
        # For the pathfinding call, set robot.available_time temporarily to best_arrival for correct waiting behavior
        saved_avail = best.available_time
        best.available_time = best_arrival
        path2_info, arrival_time2 = FindShortestPath(graph, best, task.start, task.target, best_arrival, edge_tag, lift_tag, lift_ids_map)
        # restore robot.available_time for now; final will be set after reservation
        best.available_time = saved_avail

        if path2_info is None:
            # cannot reach target from start (should be rare)
            failures.append((task, "can't reach target from start"))
            continue

        # Reserve both route segments sequentially:
        # 1) robot current position -> task.start
        path1_nodes, edges_used1 = best_details
        # edges_used1 contains edge_id and start times relative to current_time; but we will compute reservations starting at max(robot.available_time, current_time)
        start_time_segment1 = max(best.available_time, current_time)
        end1 = ReservePath(edge_tag, lift_tag, edges_used1, graph.edges, best, start_time_segment1, lift_ids_map)

        # Update robot position to task.start and available_time to end1 (robot now at task.start at end1)
        best.position = task.start
        best.available_time = end1

        # 2) task.start -> task.target (starting at best.available_time)
        path2_nodes, edges_used2 = path2_info
        end2 = ReservePath(edge_tag, lift_tag, edges_used2, graph.edges, best, best.available_time, lift_ids_map)

        # update robot final state
        best.position = task.target
        best.available_time = end2

        solutions.append({
            'task': task,
            'robot': best,
            'path_to_start': path1_nodes,
            'path_to_target': path2_nodes,
            'start_time': start_time_segment1,
            'arrive_start': end1,
            'arrive_target': end2
        })

    return solutions, failures

# ---- Example usage / small scenario ----
def example_run():
    # build graph: nodes are strings "A:1", "B:1", etc
    G = Graph()
    # Add flat edges on floor 1
    G.add_edge("A:1", "B:1", length=10.0, etype='flat', edge_id="A1-B1")
    G.add_edge("B:1", "C:1", length=15.0, etype='flat', edge_id="B1-C1")
    # stairs connecting floors at node B
    G.add_edge("B:1", "B:2", length=5.0, etype='stair', edge_id="B1-B2")
    # elevator connecting floors at node A (use same edge id for each direction)
    G.add_edge("A:1", "A:2", length=2.0, etype='lift', edge_id="LIFT_A_1_2")

    # create lift map (edge_id -> lift_id). Here single lift id "LIFT_A"
    lift_ids_map = {"LIFT_A_1_2": "LIFT_A"}

    # robots
    robots = [
        Robot(id="R1", rtype="humanoid", skill="carry", ord="A", floor=1, v_flat=1.5, v_stair=0.8),
        Robot(id="R2", rtype="wheel", skill="inspect", ord="B", floor=1, v_flat=2.0, v_stair=0.0),  # can't use stairs
        Robot(id="R3", rtype="dog", skill="carry", ord="C", floor=1, v_flat=1.2, v_stair=0.9),
    ]

    # tasks: move from start to target, with required skill
    tasks = [
        Task(id="T1", skill_req="carry", start=("A",1), target=("B",2), time=0.0),
        Task(id="T2", skill_req="inspect", start=("C",1), target=("A",2), time=5.0),
    ]

    # initial tags
    edge_tag = EdgeTag()
    lift_tag = LiftTag(lift_ids=["LIFT_A"])

    # schedule
    current_time = 0.0
    sols, fails = greedy_schedule(G, robots, tasks, current_time, edge_tag, lift_tag, lift_ids_map)

    print("=== Solutions ===")
    for s in sols:
        t = s['task']
        r = s['robot']
        print(f"Task {t.id} assigned to Robot {r.id}")
        print(f"  path to start: {s['path_to_start']}, arrive at start: {s['arrive_start']:.2f}")
        print(f"  path to target: {s['path_to_target']}, arrive at target: {s['arrive_target']:.2f}")
        print()

    print("=== Failures ===")
    for f in fails:
        print(f)

    print("=== Final Robot States ===")
    for r in robots:
        print(r)

if __name__ == "__main__":
    example_run()
