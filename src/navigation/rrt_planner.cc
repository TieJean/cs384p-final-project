#include "rrt_planner.h"
#include <limits>

namespace rrt_planner {

RRTPlanner::RRTPlanner() 
  : global_goal_mloc_(0,0),
    global_goal_mangle_(0),
    global_goal_set_(false) {

  // root_ = TreeNode(nullptr, 0);
}

void RRTPlanner::SetMap(const string &map_file) {
  map_.Load(map_file);
}

void RRTPlanner::SetGlobalGoal(const Vector2f &loc, const float angle) {
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
}
  
// checks if current location is close enough to the goal location
// bool RRTPlanner::AtGoal(const Vector2f& robot_mloc) {
//   if (!global_goal_set_) { return true; }
//   return (robot_mloc - global_goal_mloc_).norm() < CONFIG_STOP_DIST;
// }

void RRTPlanner::VisualizePath(VisualizationMsg& global_viz_msg) {

}


}

