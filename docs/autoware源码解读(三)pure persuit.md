需要一堆离散点，使用pure persuit进行几何跟踪

还需要一个目标速度

搜索最临近路点

速度控制器用一个P控制器

autoware里面每一个waypoint里面包含的信息有：
```
# global id
int32 gid
# local id
int32 lid
geometry_msgs/PoseStamped pose
geometry_msgs/TwistStamped twist
DTLane dtlane
int32 change_flag
WaypointState wpstate

uint32 lane_id
uint32 left_lane_id
uint32 right_lane_id
uint32 stop_line_id
float32 cost
float32 time_cost

# Lane Direction
# FORWARD	     = 0
# FORWARD_LEFT       = 1
# FORWARD_RIGHT      = 2
# BACKWARD           = 3
# BACKWARD_LEFT	     = 4
# BACKWARD_RIGHT     = 5
# STANDSTILL         = 6
uint32 direction
```
