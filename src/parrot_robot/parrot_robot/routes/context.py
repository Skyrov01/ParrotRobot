ros_node = None

def set_ros_node(node):
    global ros_node
    ros_node = node
    print(f"[CONTEXT] ros_node set to: {ros_node}")

def get_ros_node():
    global ros_node
    if ros_node is None:
        print("[CONTEXT] Warning: ros_node is not set")
    else:
        print(f"[CONTEXT] Returning ros_node: {ros_node}")      
    return ros_node
