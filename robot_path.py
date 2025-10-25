import time
from os.path import realpath
from typing import Dict
from graph import initial_six_graphs  # 假设已有图类，支持 dijkstra 和 dijkstra_extra
from node import show_path_with_coords,get_coordinates_from_node
import math
import threading
# ============================================================
# Core Classes
# ============================================================

# 全局变量用于存储实时路径和机器人状态
current_paths = {}  # 存储每个任务的路径信息
robot_status = {}   # 存储每个机器人的实时状态

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
        self.available_time = 0.0  # 实际秒表时间


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

def select_best_path_with_elevator(
    tid: int,
    start_pos: str,
    target_pos: str,
    stair_graph,
    add_1E1_graph,
    add_1E2_graph,
    add_2E1_graph,
    add_2E2_graph,
    add_3E1_graph,
    add_3E2_graph,
    elevators: dict,
    current_time: float
):
    """
    对同一任务计算楼梯路径 + 六种电梯路径
    每种路径会根据当前电梯 schedule 判断等待时间
    最终选择总耗时最短的路径返回
    """
    path_results = {}
    # 在函数开头声明使用全局变量
    global current_paths

    # --------------------------
    # 1. 楼梯路径（无冲突）
    # --------------------------
    path_stair, cost_stair = stair_graph.dijkstra(start_pos, target_pos)
    if path_stair and not math.isinf(cost_stair):
        path_results["stair"] = {
            "path": path_stair,
            "actual_time": cost_stair,
            "wait_time": 0.0,
            "before": 0.0,
            "between": 0.0,
            "after": 0.0,
            "eid": None,
            "type": "stair"
        }

    # --------------------------
    # 2. 电梯路径
    # --------------------------
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
        if not res or "total_time" not in res or not res["path"]:
            continue

        before = res["segments"]["before"]
        between = res["segments"]["between"]
        after = res["segments"]["after"]
        start_e, end_e = res["E_nodes"]

        if not start_e or not end_e or math.isinf(res["total_time"]):
            continue

        elev = elevators[eid]
        from_floor = int(start_e.split("_")[0])


        # 计算电梯到达所需时间
        travel_to_start = abs(elev.current_floor - from_floor) * 1.75

        # effective_start = absolute_arrival + travel_to_start
        robot_arrival = current_time + before
        elevator_ready = current_time + travel_to_start
        effective_start = max(robot_arrival, elevator_ready)
        # --- 检查电梯 schedule 是否冲突
        wait_time = 0.0
        for (s, e, _from, _to, _rid) in elev.schedule:
            # 判断是否冲突
            if not (effective_start + between <= s or effective_start >= e):
                # 若冲突，需要等到 e
                wait_time = max(wait_time, e - effective_start)

        # --- 总耗时
        actual_time = before + travel_to_start + wait_time + between + after

        path_results[eid] = {
            "path": res["path"],
            "actual_time": actual_time,
            "wait_time": wait_time + travel_to_start,
            "before": before,
            "between": between,
            "after": after,
            "start_e": start_e,
            "end_e": end_e,
            "eid": eid,
            "type": "elevator"
        }

    # --------------------------
    # 3. 选择最短路径
    # --------------------------
    if not path_results:
        print(f"[!] Task {tid} failed: No valid path found from {start_pos} to {target_pos}")
        return {"error": "no_valid_path"}

    # best_key = min(path_results.keys(), key=lambda k: path_results[k]["actual_time"])
    # best_info = path_results[best_key]

    # --------------------------
    # 4. 输出结果并 Reserve
    # --------------------------


    # 在选择最佳路径后更新全局变量
    best_key = min(path_results.keys(), key=lambda k: path_results[k]["actual_time"])
    best_info = path_results[best_key]
    real_path = show_path_with_coords(best_info["path"])

    current_paths[tid] = {
        "route": best_key,
        "path": best_info["path"],
        "real_path": real_path,
        "total_time": best_info['actual_time'],
        "wait_time": best_info['wait_time']
    }

    print(f"\nTask {tid} selected route: {best_key}, Total time: {best_info['actual_time']:.2f}s (wait {best_info['wait_time']:.2f}s)")
    print(f"Path: {best_info['path']}\n")
    print(f"Real Path: {real_path}\n")

    if best_info["type"] == "elevator":
        eid = best_info["eid"]
        elev = elevators[eid]
        from_floor = int(best_info["start_e"].split("_")[0])
        to_floor = int(best_info["end_e"].split("_")[0])

        reserve_start_abs = current_time + best_info["before"] + best_info["wait_time"]
        elev.reserve(
            start_time=reserve_start_abs,
            duration=best_info["between"],
            from_floor=from_floor,
            to_floor=to_floor,
            robot_id=tid
        )
        print(f"[Elevator Reserved] {eid}: R{tid}: {from_floor}->{to_floor}, {reserve_start_abs:.2f}s - {reserve_start_abs + best_info['between']:.2f}s")

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
        return [r for r in self.robots if r.skill == task.skill]

    def assign_task(self, task, current_time):
        feasible_robots = self.find_feasible_robots(task, current_time)
        global robot_status

        if not feasible_robots:
            return {"error": f"No robot matches skill '{task.skill}' for Task {task.id}"}

        robot = min(feasible_robots, key=lambda r: r.available_time)
        if current_time < robot.available_time:
            return {
                "error": f"Task {task.id} failed: Robot {robot.id} busy until {robot.available_time - current_time:.2f}s later"}

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
            elevators=self.elevators,
            current_time=current_time
        )

        if "error" in best_info:
            return {"error": f"Task {task.id} failed: No valid path from {robot.position} to {task.target}"}

        robot.position = task.target
        robot.real_position = get_coordinates_from_node(robot.position)
        robot.available_time = current_time + best_info["actual_time"]
        # 在任务分配完成后更新机器人状态
        robot_status[robot.id] = {
            "position": robot.position,
            "real_position": robot.real_position,
            "available_time": robot.available_time,
            "current_task": task.id,
            "skill": robot.skill
        }

        return {
            "robot_id": robot.id,
            "task_id": task.id,
            "start_time": current_time,
            "end_time": robot.available_time,
            "path_info": best_info
        }



# ============================================================
# Terminal Interface (real-time)
# ============================================================

def start_terminal_scheduler():


    # 初始化图与对象
    stair_graph, add_1E1_graph, add_1E2_graph, add_2E1_graph, add_2E2_graph, add_3E1_graph, add_3E2_graph, _ = initial_six_graphs(
        speed_land=1.5, speed_stair=0.5
    )
    elevators = init_six_elevators()
    robots = [
        Robot(0, "dog", "1_1_Left_1"),
        Robot(1, "dog", "1_1_Left_1"),
        Robot(2, "human", "1_1_Left_1"),
        Robot(3, "human", "1_1_Left_1"),
    ]
    elevator_graphs = {
        "1_E1": add_1E1_graph, "1_E2": add_1E2_graph,
        "2_E1": add_2E1_graph, "2_E2": add_2E2_graph,
        "3_E1": add_3E1_graph, "3_E2": add_3E2_graph
    }

    scheduler = Scheduler(robots, elevators, stair_graph, elevator_graphs)

    start_timestamp = time.time()
    task_counter = 0

    print("=== Multi-Robot Real-Time Scheduler ===")
    print("输入任务格式：<skill> <target_position>，例如：dog 6_3_G, human 4_3_A")
    print("6_3_G: 代表任务地点为 6楼3号楼G房间")
    print("一号楼共3层， 二号楼共9层， 三号楼共6层， 每层有任务地点 A B C D E F G")
    print("初始所有机器人位于一号楼一层左侧")
    print("设定所有机器人靠右侧行走， 避免发生相撞")
    print("输入 'exit' 退出, 输入 'robot' 查看当前机器人状态")

    while True:
        user_input = input("New Task> ").strip()
        if user_input.lower() == "exit":
            print("退出调度系统")
            break

        if user_input.lower() == "robot":
            print("\n--- 当前机器人状态 ---")
            now = time.time() - start_timestamp
            for r in robots:
                state = "空闲" if r.available_time <= now else f"忙碌({r.available_time - now:.1f}s)"
                print(
                    f"Robot {r.id} ({r.skill}): pos={r.position},available_time={r.available_time:.2f}, 状态={state}")
            print("----------------------\n")
            continue

        if len(user_input.split()) != 2:
            print("格式错误，请输入：<skill> <target_position>")
            continue

        skill, target = user_input.split()
        current_time = time.time() - start_timestamp
        task = Task(task_counter, skill, "", target)
        result = scheduler.assign_task(task, current_time)

        if "error" in result:
            print(f"[!] {result['error']}")
        else:
            print(f"[OK] 任务分配成功: Robot {result['robot_id']}, Start={result['start_time']:.2f}s, End={result['end_time']:.2f}s")
            task_counter += 1

        # 打印状态
        print("\n--- 当前机器人状态 (实时) ---")
        now = time.time() - start_timestamp
        for r in robots:
            state = "空闲" if r.available_time <= now else f"忙碌({r.available_time - now:.1f}s)"
            print(f"Robot {r.id} ({r.skill}): pos={r.position}, available_time={r.available_time:.2f}, 状态={state}")
        print("----------------------------\n")

        # 在每次任务分配后更新全局机器人状态
        for r in robots:
            robot_status[r.id] = {
                "position": r.position,
                "real_position": get_coordinates_from_node(r.position),
                "available_time": r.available_time,
                "skill": r.skill,
                "status": "空闲" if r.available_time <= now else f"忙碌({r.available_time - now:.1f}s)"
            }

        # print("**********************************")
        # print(get_current_paths())
        # print("**********************************")
        # print(get_robot_status())


def get_current_paths():
    """获取当前所有路径信息"""
    return current_paths


def get_robot_status():
    """获取当前所有机器人状态"""
    return robot_status

def clear_history():
    """清空历史记录"""
    global current_paths, robot_status
    current_paths.clear()
    robot_status.clear()

if __name__ == "__main__":
    start_terminal_scheduler()
