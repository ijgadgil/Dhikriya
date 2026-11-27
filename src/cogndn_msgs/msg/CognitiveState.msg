# Output of one VLA cycle (llava-phi3 with schema-constrained decoding)
std_msgs/Header header
string action                 # keep_lane | slow_down | stop | lane_change_left | lane_change_right | turn_left | turn_right | go_straight
InformationNeed[] needs
string rationale
string zone                   # zone at capture time
float32 x                     # world position at capture time
float32 y
string[] context_names        # verified NDN Data that was in the prompt (C2)
string[] context_digests
float32 latency_s
string model
bool valid                    # false if the model output failed validation (action defaults to keep_lane)
