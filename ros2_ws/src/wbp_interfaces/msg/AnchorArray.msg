# AnchorArray.msg
# Snapshot of all anchors observed at a point in time.

builtin_interfaces/Time stamp

# Identifier for the construction site or project
string site_id

# Observed anchors (unordered snapshot)
Anchor[] anchors
