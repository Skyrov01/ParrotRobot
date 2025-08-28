# parrot_msgs/msg/ServoMotorStatus.msg

# --- Status codes (use like an enum)
uint8 IDLE=0
uint8 MOVING=1
uint8 HOLD=2
uint8 ERROR=3

# --- Identity
string target            # "head_rotate", "left_wing", ...

# --- Kinematics
float32 commanded_deg    # target angle in degrees
float32 measured_deg     # if you have no feedback, publish = commanded_deg

# --- Motion profile
string method            # "Instant", "Linear", "Ease-In-Out"
float32 speed            # your existing semantic (same units you use today)

# --- State
uint8 status             # one of the codes above
bool saturated           # true if endstop/limit reached
bool fault               # true if driver or safety issue detected
string message           # optional human-readable info

# --- Timing
builtin_interfaces/Time stamp