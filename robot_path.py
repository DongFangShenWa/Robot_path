"""
============================================================
Full Implementation (with Dijkstra Path Planning)
Multi-Robot Task Scheduling and Path Planning in Multi-Floor Building
------------------------------------------------------------
Now integrates real graph-based shortest path computation
Author: Zhiyi Mou
Date: 2025-10-24
============================================================
"""

import heapq
import math
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple
from graph import inital_graph  # 导入你构建的图

# ============================================================
# Graph and Pathfinding
# ============================================================

# Graph class is defined in graph.py, so we don't redefine it here.
# We will get the graph object from inital_graph()

# ============================================================
# Core Data Structures
# ============================================================

class Elevator:
    """Elevator with schedule and availability logic."""
    def __init__(self, eid: int, bldg_num: int, local_id: str, current_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num  # 建筑编号 (1, 2, or 3)
        self.local_id = local_id  # 电梯本地ID ('E1' or 'E2')
        self.state = "idle"
        self.current_floor = current_floor
        self.schedule: List[Tuple[float, float, int, int, int]] = []  # (start, end, from_floor, to_floor, robot_id)

    def is_available(self, desired_start: float, duration: float, from_floor: int) -> Tuple[bool, float]:
        """
        返回 (available, extra_time):
          - If available: (True, elevator arrival time)
          - If not: (False, waiting time + arrival time)
        """
        # 假设电梯移动1层需要1.5s (这个可以调整)
        floor_diff = abs(self.current_floor - from_floor)
        arrive_time = floor_diff * 1.5  # per-floor movement time

        # 检查时间冲突
        effective_start_time = desired_start + arrive_time

        for (s, e, *_rest) in self.schedule:
            # 检查时间段 [effective_start_time, effective_start_time + duration]
            # 是否与 [s, e] 重叠
            if not (effective_start_time + duration <= s or effective_start_time >= e):
                wait_time = e - effective_start_time  # 需要等待的时间
                # 总的额外时间 = 电梯到达时间 + 冲突等待时间
                return False, arrive_time + wait_time

        # 没有冲突，额外时间 = 电梯到达时间
        return True, arrive_time

    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        """Append usage record and update current floor."""
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])
        self.current_floor = to_floor
        print(f"     [Elevator {self.id} Reserved: R{robot_id}, {from_floor}->{to_floor}, Time: {start_time:.2f}s - {end_time:.2f}s]")


class Robot:
    def __init__(self, rid: int, skill: str, position: str, available_time: float = 0.0):
        self.id = rid
        self.skill = skill
        self.position = position
        self.available_time = available_time  # 机器人可以开始新任务的最早时间


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str):
        self.id = tid
        self.skill = skill
        self.start = start  # 任务发布地点 (目前未使用，但保留)
        self.target = target # 任务目标地点


# ============================================================
# Helper Functions
# ============================================================

def get_node_details(node_str: str) -> Tuple[int, int]:
    """从节点名称 'floor_bldg_...' 中解析楼层和建筑ID."""
    try:
        parts = node_str.split('_')
        floor = int(parts[0])
        bldg = int(parts[1])
        return floor, bldg
    except (ValueError, IndexError):
        print(f"Warning: Could not parse node details from '{node_str}'")
        return -1, -1


def travel_time_to_elevator(robot: Robot, elevator: Elevator, graph) -> float:
    """计算机器人从当前位置到目标电梯在同一楼层入口的时间."""
    # 1. 获取机器人当前在哪一层
    robot_floor, _ = get_node_details(robot.position)

    # 2. 确定目标电梯节点 (在电梯自己的楼，但在机器人的楼层)
    #    例如：机器人_pos = "1_1_A", 电梯 = (Bldg 2, E1) -> 目标 = "1_2_E1"
    elevator_node = f"{robot_floor}_{elevator.bldg_num}_{elevator.local_id}"

    # 3. 计算机器人从当前位置，步行到目标电梯口的时间
    #    Dijkstra 会自动处理跨楼（例如 1_1_A -> 1_1_Right_2 -> 1_2_Left_2 -> 1_2_E1）
    _path, time = graph.dijkstra(robot.position, elevator_node)
    return time


def travel_time_exit(elevator: Elevator, target_pos: str, graph) -> float:
    """计算从电梯出口到目标位置的时间."""
    # 1. 获取目标在哪一层
    target_floor, _ = get_node_details(target_pos)

    # 2. 确定电梯的出口节点 (在电梯自己的楼，在目标楼层)
    #    例如：目标_pos = "5_2_D", 电梯 = (Bldg 2, E1) -> 目标 = "5_2_E1"
    elevator_node = f"{target_floor}_{elevator.bldg_num}_{elevator.local_id}"

    # 3. 计算从电梯口步行到最终目标的时间
    _path, time = graph.dijkstra(elevator_node, target_pos)
    return time


def calculate_elevator_time_need(n: int) -> float:
    """
    计算电梯跨越n层所需的时间 (基于 graph.py).
    (开门1.5s + 关门1.5s + 运行1.75s*n + 开门1.5s)
    这与PDF中的 (1,2) [cite: 33] 和 (1,3) [cite: 36] 示例一致.
    """
    if n == 0:
        return 0
    return 1.5 + 1.5 + 1.75 * n + 1.5

def generate_elevator_table() -> Dict[Tuple[int, int], float]:
    """
    自动生成电梯时间查询表.
    """
    table = {}
    # 假设所有电梯都服务1-9层 (为简单起见，后续可按建筑细分)
    # 实际在 `multi_robot_scheduling` 中会通过 `travel_time` 过滤掉不在同一建筑的电梯
    for i in range(1, 10):
        for j in range(1, 10):
            if i == j:
                continue
            n_floors = abs(i - j)
            time = calculate_elevator_time_need(n_floors)
            table[(i, j)] = time
    return table


def lookup_elevator_time(elevator_table: Dict[Tuple[int, int], float], from_floor: int, to_floor: int) -> float:
    """Lookup pre-measured elevator travel time."""
    if (from_floor, to_floor) in elevator_table:
        return elevator_table[(from_floor, to_floor)]
    return float("inf") # 理论上 generate_elevator_table 已覆盖


def visualize_schedules(elevators: List[Elevator]):
    """Draw Gantt chart for elevator usage."""
    fig, ax = plt.subplots(figsize=(12, 7))
    y_labels = []
    y_pos = []
    color_map = plt.cm.get_cmap("tab10")

    for i, e in enumerate(elevators):
        y_labels.append(f"Elevator {e.id} (Bldg {e.bldg_num})")
        y_pos.append(i)
        for j, (s, end, f1, f2, rid) in enumerate(e.schedule):
            color = color_map(rid % 10)
            ax.barh(i, end - s, left=s, color=color, alpha=0.8, edgecolor='black')
            ax.text(s + (end - s) / 2, i, f"R{rid} ({f1}→{f2})", ha="center", va="center", fontsize=9, color="white", fontweight='bold')

    ax.set_yticks(y_pos)
    ax.set_yticklabels(y_labels)
    ax.set_xlabel("Time (s)")
    ax.set_title("Elevator Usage Schedule")
    plt.grid(axis='x', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()


# ============================================================
# Core Algorithm
# ============================================================

def multi_robot_scheduling(tasks: List[Task], robots: List[Robot], elevators: List[Elevator],
                           elevator_table: Dict[Tuple[int, int], float], graph):
    """
    Main scheduling and path planning algorithm.
    为每个任务寻找最优机器人
    """
    results = []

    for task in tasks:
        print(f"\n--- [TASK {task.id}] Processing Task: Go to {task.target} (Skill: {task.skill}) ---")

        candidate_robots = [r for r in robots if r.skill == task.skill]
        if not candidate_robots:
            print(f"  No suitable robot found for task {task.id}")
            continue

        best_overall_finish_time = float('inf')
        best_robot = None
        best_path_desc = ""
        best_travel_time = 0.0

        # 存储最优机器人的最优电梯选择
        best_elevator_choice = {}

        # 1. 遍历所有符合条件的机器人
        for r in candidate_robots:
            print(f"  Evaluating Robot {r.id} (at {r.position}, free at {r.available_time:.2f}s)")

            robot_best_finish_time = float('inf')
            robot_best_path = ""
            robot_best_travel_time = 0.0
            robot_elevator_choice = {}

            # --- Option A: Stairs or walking path via Dijkstra ---
            _path, stair_time = graph.dijkstra(r.position, task.target)

            if stair_time == float('inf'):
                print(f"    - Path (Stairs): Unreachable")
            else:
                total_stair_finish_time = r.available_time + stair_time
                print(f"    - Path (Stairs): {stair_time:.2f}s travel. (Finishes at {total_stair_finish_time:.2f}s)")
                if total_stair_finish_time < robot_best_finish_time:
                    robot_best_finish_time = total_stair_finish_time
                    robot_best_path = "Stairs/Dijkstra"
                    robot_best_travel_time = stair_time
                    robot_elevator_choice = {} # 重置电梯选择

            # --- Option B: Elevators ---
            for e in elevators:
                # 2. 检查机器人和目标点是否在电梯所在的建筑
                t_move = travel_time_to_elevator(r, e, graph)
                if t_move == float('inf'):
                    # print(f"    - Path (Elevator {e.id}): Robot cannot reach this elevator.")
                    continue

                t_exit = travel_time_exit(e, task.target, graph)
                if t_exit == float('inf'):
                    # print(f"    - Path (Elevator {e.id}): Target cannot be reached from this elevator.")
                    continue

                from_floor, _ = get_node_details(r.position)
                to_floor, _ = get_node_details(task.target)

                if from_floor == to_floor:
                    continue # 同层，不需要电梯

                # 4. 查找电梯行程时间
                travel_t = lookup_elevator_time(elevator_table, from_floor, to_floor)
                if travel_t == float('inf'):
                    continue # 电梯不去这些楼层

                # 5. 检查电梯可用性
                desired_start = r.available_time + t_move
                _available, extra_time = e.is_available(desired_start, travel_t, from_floor)

                # 6. 计算总时间
                total_elevator_travel_time = t_move + extra_time + travel_t + t_exit
                total_elevator_finish_time = r.available_time + total_elevator_travel_time

                # print(f"    - Path (Elevator {e.id}): {total_elevator_travel_time:.2f}s travel (move={t_move:.2f}, wait+arrival={extra_time:.2f}, ride={travel_t:.2f}, exit={t_exit:.2f}). (Finishes at {total_elevator_finish_time:.2f}s)")

                if total_elevator_finish_time < robot_best_finish_time:
                    robot_best_finish_time = total_elevator_finish_time
                    robot_best_path = f"Elevator_{e.id}"
                    robot_best_travel_time = total_elevator_travel_time
                    robot_elevator_choice = {
                        'e': e,
                        'start_time': r.available_time + t_move + extra_time, # 电梯实际开始运行时间
                        'duration': travel_t,
                        'from': from_floor,
                        'to': to_floor
                    }

            # 7. 比较这个机器人的最佳时间与所有机器人的最佳时间
            if robot_best_finish_time < best_overall_finish_time:
                best_overall_finish_time = robot_best_finish_time
                best_robot = r
                best_path_desc = robot_best_path
                best_travel_time = robot_best_travel_time
                best_elevator_choice = robot_elevator_choice

        # --- 任务分配和状态更新 ---
        if best_robot:
            print(f"  => [ASSIGNED] Task {task.id} to Robot {best_robot.id}. Path: {best_path_desc}, Travel Time: {best_travel_time:.2f}s, Finishes at: {best_overall_finish_time:.2f}s")

            # 更新机器人状态
            best_robot.available_time = best_overall_finish_time
            best_robot.position = task.target

            # 如果使用了电梯，更新电梯时间表
            if best_elevator_choice.get('e'):
                e = best_elevator_choice['e']
                e.reserve(
                    best_elevator_choice['start_time'],
                    best_elevator_choice['duration'],
                    best_elevator_choice['from'],
                    best_elevator_choice['to'],
                    best_robot.id
                )

            results.append((task.id, best_robot.id, best_path_desc, best_travel_time))
        else:
            print(f"  => [FAILED] No path found for any robot to complete task {task.id}")
            results.append((task.id, -1, "Unreachable", float('inf')))

    return results


# ============================================================
# Example Execution
# ============================================================

if __name__ == "__main__":

    # --- 1. 初始化图 ---
    # 假设陆地速度1.5 m/s, 楼梯速度 0.5 m/s
    print("Initializing building graph...")
    graph = inital_graph(speed_land=1.5, speed_stair=0.5)
    print("Graph initialized.")

    # --- 2. 初始化电梯时间表 ---
    elevator_table = generate_elevator_table()

    # --- 3. 初始化电梯 (共6台) ---
    # 建筑1 (1-3层): E1, E2
    # 建筑2 (1-9层): E1, E2
    # 建筑3 (1-6层): E1, E2
    elevators = [
        Elevator(eid=1, bldg_num=1, local_id='E1', current_floor=1),
        Elevator(eid=2, bldg_num=1, local_id='E2', current_floor=1),
        Elevator(eid=3, bldg_num=2, local_id='E1', current_floor=1),
        Elevator(eid=4, bldg_num=2, local_id='E2', current_floor=1),
        Elevator(eid=5, bldg_num=3, local_id='E1', current_floor=1),
        Elevator(eid=6, bldg_num=3, local_id='E2', current_floor=1),
    ]

    # --- 4. 初始化机器人  ---
    robots = []
    start_pos = "1_1_Left_1" # 统一出发点
    # 5 个机器狗
    for i in range(5):
        robots.append(Robot(rid=i + 1, skill="dog", position=start_pos))
    # 5 个双足机器人
    for i in range(5):
        robots.append(Robot(rid=i + 6, skill="bipedal", position=start_pos))

    print(f"Initialized {len(robots)} robots at {start_pos}.")

    # --- 5. 定义任务列表 ---
    tasks = [
        Task(tid=1, skill="dog", start="1_1_A", target="3_1_C"),        # 任务1: 狗, 楼内, 1->3层 (Bldg 1)
        Task(tid=2, skill="bipedal", start="1_1_A", target="5_2_D"),    # 任务2: 人, 跨楼, 1->5层 (Bldg 1 -> Bldg 2)
        Task(tid=3, skill="dog", start="1_1_A", target="8_2_E"),        # 任务3: 狗, 跨楼, 1->8层 (Bldg 1 -> Bldg 2)
        Task(tid=4, skill="bipedal", start="1_1_A", target="4_3_F"),    # 任务4: 人, 跨楼, 1->4层 (Bldg 1 -> Bldg 3)
        Task(tid=5, skill="dog", start="1_1_A", target="2_1_G"),        # 任务5: 狗, 楼内, 1->2层 (Bldg 1, 抢占电梯)
        Task(tid=6, skill="dog", start="1_1_A", target="9_2_A"),        # 任务6: 狗, 跨楼, 1->9层 (Bldg 1 -> Bldg 2)
    ]

    # --- 6. 运行调度 ---
    result = multi_robot_scheduling(tasks, robots, elevators, elevator_table, graph)

    # --- 7. 显示结果 ---
    print("\n" + "=" * 70)
    print("Task Execution Results:")
    print("-" * 70)
    for t_id, r_id, path, time_cost in result:
        if r_id != -1:
            print(f"Task {t_id:2d} → Robot {r_id:2d}: Path={path:15s} | Travel Time={time_cost:6.2f}s")
        else:
            print(f"Task {t_id:2d} → FAILED: Path unreachable.")
    print("=" * 70)

    # # --- 8. 可视化电梯调度 ---
    # visualize_schedules(elevators)