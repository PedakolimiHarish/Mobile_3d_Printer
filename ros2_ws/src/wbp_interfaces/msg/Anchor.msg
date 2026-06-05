# Anchor.msg
# Single anchor observation.
# This is an observation, not truth.

string anchor_id

# Anchor type (informational only)
uint8 anchor_type

# Pose of the anchor in the site/world frame
geometry_msgs/PoseStamped pose

# Estimator confidence [0.0 - 1.0]
float32 confidence

# Last time this anchor was observed
builtin_interfaces/Time last_seen

# Construction phase this anchor belongs to
string phase_id

# Monotonic version; increments if anchor is moved or reinstalled
uint32 version
