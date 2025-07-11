# routes/context.py
ros_node = None

def set_ros_node(node):
    global ros_node
    ros_node = node