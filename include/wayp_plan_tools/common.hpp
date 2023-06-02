#ifndef COMMON_HPP_
#define COMMON_HPP_

// #include <iostream>
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/pose_array.hpp"

namespace common_wpt
{
    enum METRICS
    {
        CUR_LAT_DIST_SIGNED = 0, // current lateral distance to the waypoint (signed, cross-track error)
        CUR_LAT_DIST_ABS,        // current lateral distance to the waypoint (absolute value)
        AVG_LAT_DISTANCE,        // average lateral distance over time
        MAX_LAT_DISTANCE,        // maximum lateral distance over time
        CUR_WAYPOINT_ID,         // current waypoint ID
        TRG_WAYPOINT_ID,         // target waypoint ID
        TRG_WAY_LON_DIST,        // target waypoint longitudinal distance
        NOT_USED_YET
    };
} // namespace common_wpt

#endif // COMMON_HPP_