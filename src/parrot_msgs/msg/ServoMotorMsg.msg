# parrot_msgs/msg/ServoMotorMsg.msg
string target       # "head", "left_wing", "right_wing"
string method       # "instant", "linear", "ease"
float32 position    # angle in degrees
float32 speed       # movement speed
