# 局部相对坐标（单位：cm 或自定义单位）
local_coords = {
    'A': (60.00, 100.00),    'Ar': (60.00, 113.000),
    'B': (75.00, 100.00),    'Br': (75.00, 113.00),
    'C': (143.00, 100.00),    'Cr': (143.00, 113.00),
    'D': (220.00, 100.00),    'Dr': (220.00, 113.00),
    'E': (77.00, 113.00),    'Er': (77.00, 100.00),
    'F': (200.00, 113.00 ),    'Fr': (200.00, 100.00),
    'G': (293.00, 113.00),    'Gr': (293.00, 100.00),
    'Left_1': (3.00, 113.00),    'Left_2': (3.00, 100.00),
    'Right_1': (433.00, 113.00),    'Right_2': (433.00, 100.00),
    'E1':(380.00, 167.00),    'E2':(405.00, 167.00),
    'Stair1_1':(93.00, 150.00), 'Stair1_2':(122.00, 150.00),
    'Stair2_1':(380.00, 199.00), 'Stair2_2':(380.00, 167.00),
}


def get_coordinates_from_node(node: str):
    """
    将节点名（如 '1_2_A'）解析为坐标 (x, y, z)
    building间x相差63000, floor间z相差300
    """
    try:
        parts = node.split('_')
        building = int(parts[1])
        floor = int(parts[0])
        room = parts[2]
        if len(parts) == 4:
            room = parts[2]+'_'+parts[3]

    except Exception:
        print(f"节点 {node} 格式错误，无法解析！")
        return None

    # 从local_coords获取房间的局部坐标
    if room not in local_coords:
        print(f"节点 {node} 中的房间 {room} 未在local_coords中定义！")
        return None

    local_x, local_y = local_coords[room]

    # 计算全局坐标
    x = local_x + (building - 1) * 633.00
    y = local_y
    z = 1.10 + (floor - 1) * 3.50

    return (int(x*100), int(y*100), int(z*100))



def show_path_with_coords(path_list):
    # print("Path with Coordinates:")
    new_path = []
    for node in path_list:
        coord = get_coordinates_from_node(node)
        new_path.append(coord)
        # if coord:
        #     print(f"{node:<12} -> 坐标 (x={coord[0]}, y={coord[1]}, z={coord[2]})")
    # print(new_path)
    return new_path

if __name__ == "__main__":
    # 测试示例
    nodes = ["1_1_A", "1_1_B", "1_1_C", "1_1_D", "1_1_E", "2_3_B", "3_2_Left_1"]
    show_path_with_coords(nodes)
