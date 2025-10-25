# ============================================================
# 后端实时运行版本：前端添加任务
# ============================================================
# ============================================================
# robot_server.py
# 后端调度系统（支持前端下发任务）
# ============================================================

from flask import Flask, request, jsonify
import CORS
import threading
import queue
import time

# ============================================================
# 模拟全局变量与对象（实际项目中应由 graph / robot 模块导入）
# ============================================================

task_queue = queue.Queue()
robot_status = {}          # 存放机器人状态
current_paths = {}         # 存放当前路径信息
task_counter = 0
start_timestamp = time.time()

# ============================================================
# 模拟任务与调度逻辑
# ============================================================

class Task:
    def __init__(self, tid, skill, robot_type, target):
        self.id = tid
        self.skill = skill
        self.robot_type = robot_type
        self.target = target

class DummyScheduler:
    """模拟调度系统"""
    def __init__(self):
        self.available_robots = [1, 2]

    def assign_task(self, task, current_time):
        if not self.available_robots:
            return {"error": "暂无可用机器人"}
        rid = self.available_robots.pop(0)
        result = {
            "robot_id": rid,
            "start_time": current_time,
            "end_time": current_time + 10.0
        }
        # 模拟任务完成后机器人恢复空闲
        threading.Timer(5.0, lambda: self.available_robots.append(rid)).start()
        return result


def initialize_scheduler_system():
    """初始化调度系统"""
    global scheduler
    scheduler = DummyScheduler()
    print("🧠 调度系统初始化完成。")


def get_robot_status():
    """返回当前机器人状态"""
    return robot_status


def get_current_paths():
    """返回路径信息"""
    return current_paths


def update_robot_status():
    """模拟更新机器人状态"""
    global robot_status
    robot_status = {
        1: {"state": "idle"},
        2: {"state": "busy" if task_queue.qsize() > 0 else "idle"}
    }


# ============================================================
# Flask 应用与路由
# ============================================================

app = Flask(__name__)
CORS(app)  # 允许前端跨域访问

@app.route('/add_task', methods=['POST'])
def add_task():
    """前端通过 POST /add_task 添加任务"""
    data = request.get_json()
    skill = data.get("skill")
    target = data.get("target")
    if not skill or not target:
        return jsonify({"error": "缺少 skill 或 target"}), 400

    task_queue.put((skill, target))
    return jsonify({"message": f"任务已加入队列: {skill} -> {target}"}), 200


@app.route('/status', methods=['GET'])
def get_status():
    """前端查看机器人状态"""
    return jsonify(get_robot_status())


@app.route('/paths', methods=['GET'])
def get_paths():
    """查看当前路径信息"""
    return jsonify(get_current_paths())


# ============================================================
# 后台调度线程
# ============================================================

def scheduler_loop():
    """持续运行的后台调度循环，从队列中取任务执行"""
    global task_counter, scheduler, start_timestamp
    initialize_scheduler_system()
    print("✅ 后端调度系统启动成功，可通过 /add_task 接口下发任务。")

    while True:
        try:
            if not task_queue.empty():
                skill, target = task_queue.get()
                print(f"\n[New Task] {skill} -> {target}")
                current_time = time.time() - start_timestamp
                task = Task(task_counter, skill, "", target)
                result = scheduler.assign_task(task, current_time)

                if "error" in result:
                    print(f"[!] {result['error']}")
                else:
                    print(f"[OK] 任务分配成功: R{result['robot_id']}, Start={result['start_time']:.2f}s, End={result['end_time']:.2f}s")
                    task_counter += 1

                update_robot_status()
                print("--- 当前机器人状态 ---")
                for rid, info in robot_status.items():
                    print(f"R{rid}: {info}")
                print("----------------------\n")

            time.sleep(0.5)  # 避免CPU占用过高
        except Exception as e:
            print(f"[Error in scheduler loop] {e}")
            time.sleep(1)


# ============================================================
# 主程序入口
# ============================================================

if __name__ == "__main__":
    # 启动后台调度线程
    scheduler_thread = threading.Thread(target=scheduler_loop, daemon=True)
    scheduler_thread.start()

    # 启动 Flask 服务
    app.run(host="0.0.0.0", port=5050)
