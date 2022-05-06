#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"

#include <vector>

using namespace std;
using namespace Eigen;
using amrl_msgs::VisualizationMsg;

namespace rrt_planner {

class RRTPlanner {

public:
  RRTPlanner();
  void SetMap(const string &map_file);
  void SetGlobalGoal(const Vector2f &loc, const float angle);
  void GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle);
  Vector2f GetLocalGoal(const Vector2f& robot_mloc, const float robot_mangle);
  bool AtGoal(const Vector2f& robot_mloc);
  void VisualizePath(VisualizationMsg& global_viz_msg);

private:


};

}

#endif // RRTPLANNER_H_