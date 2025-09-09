# Generic command for high-level behaviours
string behaviour_type     # e.g. "agree", "disagree", "wave", "lookaround"

# Generic numeric parameters

string amplitude          # how far to move - low, medium, high
float32 speed             # movement speed
int32 repetitions         # for repetitive actions (e.g. wave)

# Optional extra info
string target             # which wing ("left" or "right") or other target