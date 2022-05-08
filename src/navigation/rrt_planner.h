#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "vector_map/vector_map.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "KDTree/KDTree.hpp"
#include "util/random.h"
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;
using amrl_msgs::VisualizationMsg;

namespace rrt_planner {

struct State {
  Vector2f loc;
  float angle;
  State() {}
  State(const Vector2f& loc, const float angle) 
    : loc(loc), angle(angle) {}

  friend std::ostream& operator<<(std::ostream& o, const State& s) {
    o << "loc: " << s.loc.transpose()  << ", angle: "  << s.angle;
    return o;
  }
};

struct Control {
  float a;
  float c;
};

struct Trajectory {
  vector<State> state;
  vector<Control> control;
  float time;
};

struct TreeNode {
  shared_ptr<TreeNode> parent;
  float cost;
  Vector2f location;
  vector<shared_ptr<TreeNode>> children;
  TreeNode() {}
  TreeNode(const TreeNode& parent, float cost) {
    this->parent = make_shared<TreeNode>(parent);
  }
};

class RRTPlanner {

public:
  RRTPlanner();
  void SetMap(const string &map_file);
  void SetGlobalGoal(const Vector2f &loc, const float angle);
  void GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle);
  Vector2f GetLocalGoal(const Vector2f& robot_mloc, const float robot_mangle);
  bool AtGoal(const Vector2f& robot_mloc);
  bool AtGoal(const State& state_baselink);
  bool AtGoalState_(const State& state, const State& goal_state);
  void VisualizeTraj(const Trajectory& traj, VisualizationMsg& global_viz_msg);
  void VisualizePath(VisualizationMsg& global_viz_msg);
  Trajectory Steer_(const State& start_state, 
                    const State& goal_state,
                    State& next_state); // TODO move to private

private:
  vector_map::VectorMap map_;
  Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;
  TreeNode root_;

  const float t_interval_ = 0.5;
  const float kEpsilon = 1e-4;

  const float MAX_VELOCITY = 1.0;
  const float MIN_CURVATURE = -1.7;
  const float MAX_CURVATURE = 1.7;
  const float SAFE_MARGIN = 0.1;
  const float CAR_LENGTH = 0.4826;
  const float CAR_LENGTH_SAFE = CAR_LENGTH + SAFE_MARGIN * 2;
  const float CAR_BASE = 0.343;
  const float CAR_WIDTH = 0.2667;
  const float CAR_WIDTH_SAFE = CAR_WIDTH + SAFE_MARGIN * 2;

  bool SteerOneStep_(const State& start_state, 
                     const State& goal_state,
                     State& next_state, Control& control_to_next_state) ;
  bool SteerOneStepByControl_(const State& curr_state, 
                              const Control& control, 
                              State& next_state);
  State GetNextStateByCurvature_(const State& curr_state, const float curvature);
  tuple<Vector2f, float, float, float, int> GetTravelledArc_(const State& baselink_state, const float curvature);
  float GetTravelledDistOneStep_();
  float GetCost_(const Control& u);
};

}

#endif // RRTPLANNER_H_