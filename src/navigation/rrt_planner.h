#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"

#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;
using amrl_msgs::VisualizationMsg;

namespace rrt_planner {

struct State {
  Vector2f loc;
  float angle;
};

struct Control {
  float velocity;
  float curvature;
};

struct Trajectory {
  vector<State> state;
  vector<Control> control;
  float time;
};

struct TreeNode {
  TreeNode* parent;
  float cost;
  vector<TreeNode*> children;
  TreeNode() {}
  TreeNode(TreeNode* parent, float cost) 
    : parent(parent), cost(cost) {}
};

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
  vector_map::VectorMap map_;
  Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;

  TreeNode* root_;
  void Steer(const State& start_state, const State goal_state);
};

}

#endif // RRTPLANNER_H_