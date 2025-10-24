"""
============================================================
Interactive Multi-Robot Scheduler Terminal
------------------------------------------------------------
- Skill-based robot-task assignment
- Terminal input: add tasks anytime
- After each task, print each robot's status
Author: Zhiyi Mou
Date: 2025-10-25
============================================================
"""

from typing import Dict
from graph import initial_six_graphs  # 假设已有图类，支持 dijkstra 和 dijkstra_extra

# ============================================================
# Core Classes
# ============================================================

class Elevator:
    def __init__(self, eid: int, bldg_num: int, local_id: str, current_floor: int = 1):
        self.id = eid
        self.bldg_num = bldg_num
        self.local_id = local_id
        self.current_floor = current_floor
        self.schedule = []  # (start_time, end_time, from_floor, to_floor, robot_id)

    def reserve(self, start_time: float, duration: float, from_floor: int, to_floor: int, robot_id: int):
        end_time = start_time + duration
        self.schedule.append((start_time, end_time, from_floor, to_floor, robot_id))
        self.schedule.sort(key=lambda x: x[0])
        self.current_floor = to_floor
        print(f"[Elevator {self.id} Reserved] R{robot_id}: {from_floor}->{to_floor}, {start_time:.2f}s - {end_time:.2f}s")


class Robot:
    def __init__(self, rid: int, skill: str, position: str):
        self.id = rid
        self.skill = skill
        self.position = position
        self.available_time = 0.0  # 初始可用


class Task:
    def __init__(self, tid: int, skill: str, start: str, target: str):
        self.id = tid
        self.skill = skill
        self.start = start
        self.target = target


# ============================================================
# Elevator Utilities
# ============================================================

def init_six_elevators() -> Dict[str, Elevator]:
    elevators = {}
    elevators["1_E1"] = Elevator(1, 1, "E1")
    elevators["1_E2"] = Elevator(2, 1, "E2")
    elevators["2_E1"] = Elevator(3, 2, "E1")
    elevators["2_E2"] = Elevator(4, 2, "E2")
    elevators["3_E1"] = Elevator(5, 3, "E1")
    elevators["3_E2"] = Elevator(6, 3, "E2")
    return elevators


def compute_elevator_wait_time(elevator: Elevator, from_floor: int, start_time: float, duration: float) -> float:
    travel_time_to_start = abs(elevator.current_floor - from_floor) * 1.75
    effective_start = start_time + travel_time_to_start
    for (s, e, *_rest) in elevator.schedule:
        if not (effective_start + duration <= s or effective_start >= e):
            wait_time = e - effective_start
            return travel_time_to_start + wait_time
    return travel_time_to_start  # 没有冲突


# ============================================================
# Path Selection
# ============================================================

def select_best_path_with_elevator(tid: int, start_pos: str, target_pos: str,
                                   stair_graph, add_1E1_graph, add_1E2_graph,
                                   add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph,
                                   elevators: dict):
    path_results = {}

    # 楼梯路径
    path_stair, cost_stair = stair_graph.dijkstra(start_pos, target_pos)
    path_results["stair"] = {
        "path": path_stair,
        "base_time": cost_stair,
        "actual_time": cost_stair,
        "type": "stair"
    }

    # 电梯路径
    graph_map = {
        "1_E1": add_1E1_graph,
        "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph,
        "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph,
        "3_E2": add_3E2_graph,
    }

    for eid, g in graph_map.items():
        res = g.dijkstra_extra(start_pos, target_pos)
        before = res["segments"]["before"]
        between = res["segments"]["between"]
        after = res["segments"]["after"]
        start_e, end_e = res["E_nodes"]
        base_time = res["total_time"]

        actual_time = base_time
        wait_time = 0

        if start_e and end_e:
            from_floor = int(start_e.split("_")[0])
            wait_time = compute_elevator_wait_time(elevators[eid], from_floor, before, between)
            actual_time = before + wait_time + between + after

        path_results[eid] = {
            "path": res["path"],
            "base_time": base_time,
            "actual_time": actual_time,
            "wait_time": wait_time,
            "before": before,
            "between": between,
            "after": after,
            "start_e": start_e,
            "end_e": end_e,
            "type": "elevator",
            "eid": eid
        }

    # 选择最短时间方案
    best_key = min(path_results.keys(), key=lambda k: path_results[k]["actual_time"])
    best_info = path_results[best_key]

    print(f"\nTask {tid} selected route: {best_key}, Total time: {best_info['actual_time']:.2f}s (wait {best_info.get('wait_time',0):.2f}s)")
    print(f"Path: {best_info['path']}\n")

    # 更新 schedule
    if best_info["type"] == "elevator" and best_info["start_e"] and best_info["end_e"]:
        eid = best_info["eid"]
        from_floor = int(best_info["start_e"].split("_")[0])
        elevators[eid].reserve(
            start_time=best_info["before"] + best_info.get('wait_time', 0),
            duration=best_info["between"],
            from_floor=from_floor,
            to_floor=int(best_info["end_e"].split("_")[0]),
            robot_id=tid
        )

    return best_info


# ============================================================
# Scheduler
# ============================================================

class Scheduler:
    def __init__(self, robots, elevators, stair_graph, elevator_graphs):
        self.robots = robots
        self.elevators = elevators
        self.stair_graph = stair_graph
        self.elevator_graphs = elevator_graphs

    def find_feasible_robots(self, task, current_time):
        return [r for r in self.robots if r.skill == task.skill and r.available_time <= current_time]

    def assign_task(self, task, current_time=0.0):
        feasible_robots = self.find_feasible_robots(task, current_time)
        if not feasible_robots:
            return {"error": f"No feasible robot for Task {task.id} (skill: {task.skill})"}

        robot = min(feasible_robots, key=lambda r: r.available_time)
        start_time = max(current_time, robot.available_time)

        best_info = select_best_path_with_elevator(
            tid=robot.id,
            start_pos=robot.position,
            target_pos=task.target,
            stair_graph=self.stair_graph,
            add_1E1_graph=self.elevator_graphs["1_E1"],
            add_1E2_graph=self.elevator_graphs["1_E2"],
            add_2E1_graph=self.elevator_graphs["2_E1"],
            add_2E2_graph=self.elevator_graphs["2_E2"],
            add_3E1_graph=self.elevator_graphs["3_E1"],
            add_3E2_graph=self.elevator_graphs["3_E2"],
            elevators=self.elevators
        )

        # 更新机器人状态
        robot.position = task.target
        robot.available_time = start_time + best_info["actual_time"]

        return {
            "robot_id": robot.id,
            "task_id": task.id,
            "start_time": start_time,
            "end_time": robot.available_time,
            "path_info": best_info
        }


# ============================================================
# Terminal Interface
# ============================================================

def start_terminal_scheduler():
    # 初始化图
    stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, final_graph = initial_six_graphs(
        speed_land=1.5, speed_stair=0.5
    )

    elevators = init_six_elevators()
    robots = [
        Robot(0, "dog", "1_1_Left_1"),
        Robot(1, "cat", "1_1_Left_1"),
        Robot(2, "dog", "1_1_Left_1")
    ]
    elevator_graphs = {
        "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
    }

    scheduler = Scheduler(robots, elevators, stair_graph, elevator_graphs)

    current_time = 0.0
    task_counter = 0

    print("=== Multi-Robot Scheduler Terminal ===")
    print("输入任务格式：<skill> <target_position>，例如：dog 6_3_G")
    print("输入 'exit' 退出")

    while True:
        user_input = input("New Task> ").strip()
        if user_input.lower() == "exit":
            print("退出调度系统")
            break
        if len(user_input.split()) != 2:
            print("格式错误，请输入：<skill> <target_position>")
            continue
        skill, target = user_input.split()
        task = Task(task_counter, skill, "", target)
        result = scheduler.assign_task(task, current_time)
        if "error" in result:
            print(result["error"])
        else:
            print(f"任务分配完成: Robot {result['robot_id']}, start {result['start_time']:.2f}, end {result['end_time']:.2f}")
            print(f"路径信息: {result['path_info']}")
            current_time = max(current_time, result['end_time'])
            task_counter += 1

            # 打印所有机器人状态
            print("\n--- Current Robot Status ---")
            for r in robots:
                print(f"Robot {r.id}: skill={r.skill}, position={r.position}, available_time={r.available_time:.2f}")
            print("----------------------------\n")


if __name__ == "__main__":
    start_terminal_scheduler()
