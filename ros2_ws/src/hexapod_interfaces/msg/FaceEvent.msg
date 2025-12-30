# Event notification for face detection/recognition

# Event type constants
uint8 UNKNOWN_FACE_DETECTED=1
uint8 KNOWN_FACE_DETECTED=2

std_msgs/Header header

# Event type (see constants above)
uint8 event_type

# Identity name if recognized (empty for unknown)
string identity

# Face location in the image
sensor_msgs/RegionOfInterest bounding_box

# Recognition confidence/distance (lower = better match for known faces)
float32 confidence
