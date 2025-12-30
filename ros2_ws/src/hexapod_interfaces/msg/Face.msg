# A detected face with optional recognition result

std_msgs/Header header

# Face location in image
sensor_msgs/RegionOfInterest bounding_box

# Detection confidence (0.0-1.0)
float32 confidence

# Recognition results
bool recognized                    # True if face was matched to known identity
string identity                    # Name if recognized, empty if unknown
float32 recognition_distance       # Distance to closest known face (lower = better match)
