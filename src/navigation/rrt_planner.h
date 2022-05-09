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
#include <limits>

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
  Trajectory() {}
  Trajectory(const vector<State>& state, const vector<Control>& control, const float time) 
    : state(state), control(control), time(time) {}
  friend std::ostream& operator<<(std::ostream& o, const Trajectory& traj) {
    for (const State& s : traj.state) {
      o << "-->" << s << endl;
    }
    return o;
  }
};

struct TreeNode {
  shared_ptr<TreeNode> parent;
  State state;
  float cost;
  Trajectory trajectory;
  set<shared_ptr<TreeNode>> children;

  TreeNode() {
    this->cost = std::numeric_limits<float>::max();
  }
  TreeNode(const TreeNode& node) 
    : parent(node.parent), state(node.state), cost(node.cost) {}
  TreeNode(const State& state, const float cost) : state(state), cost(cost) {}
  TreeNode(const TreeNode& parent, const State& state, const float cost) {
    this->parent = make_shared<TreeNode>(parent);
    this->state = state;
    this->cost = cost;
  }
  friend std::ostream& operator<<(std::ostream& o, const TreeNode& s) {
    o << "state - " << s.state << "; cost - " << s.cost;
    return o;
  }
};

void printTree(const TreeNode& goal_node);

class RRTPlanner {

public:
  RRTPlanner();
  void SetMap(const string &map_file);
  void SetGlobalGoal(const Vector2f &loc, const float angle);
  Vector2f GetGlobalGoal();
  bool RetrieveGlobalPlan_(); // TODO move to private
  bool GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle);
  bool isGlobalPlanReady();
  Trajectory GetGlobalTraj(); // TODO for debug
  Vector2f GetLocalGoal(const Vector2f& robot_mloc, const float robot_mangle);
  bool AtGoal(const Vector2f& robot_mloc);
  bool AtGoal(const State& state_baselink);
  bool AtGoalState_(const State& state, const State& goal_state);
  void VisualizeTraj(const Trajectory& traj, VisualizationMsg& global_viz_msg);
  void VisualizePath(VisualizationMsg& global_viz_msg);
  Trajectory Steer_(const State& start_state, 
                    const State& goal_state,
                    State& next_state); // TODO move to private
  bool Steer_(const State& start_state,  const State& goal_state,
              State& next_state, Trajectory& traj);
  void PrintFinalPath();

private:
  vector_map::VectorMap map_;
  Vector2f global_goal_mloc_;
	float global_goal_mangle_;
  bool global_goal_set_;
  shared_ptr<TreeNode> root_;
  shared_ptr<TreeNode> goal_;
  util_random::Random rng_;
  Trajectory global_plan_;
  int path_start_idx_;
  bool found_gloabal_plan_;

  float t_interval_ = 0.5;
  float kEpsilon = 1e-4;

  float MAX_VELOCITY = 1.0;
  float MIN_CURVATURE = -1.7;
  float MAX_CURVATURE = 1.7;
  float SAFE_MARGIN = 0.1;
  float CAR_LENGTH = 0.4826;
  float CAR_LENGTH_SAFE = CAR_LENGTH + SAFE_MARGIN * 2;
  float CAR_BASE = 0.343;
  float CAR_WIDTH = 0.2667;
  float CAR_WIDTH_SAFE = CAR_WIDTH + SAFE_MARGIN * 2;

  bool SteerOneStep_(const State& start_state, 
                     const State& goal_state,
                     State& next_state, Control& control_to_next_state) ;
  bool SteerOneStepByControl_(const State& curr_state, 
                              const Control& control, 
                              State& next_state);
  State GetNextStateByCurvature_(const State& curr_state, const float curvature);
  tuple<Vector2f, float, float, float, int> GetTravelledArc_(const State& baselink_state, const float curvature);
  float GetTravelledDistOneStep_();
  float GetTrajCost_(const Trajectory& traj);
  float GetCostOneStep_(const Control& u);
  bool IsStateCollisionFree_(const State& state);
  bool IsStateLocCollisionFree_(const Vector2f loc);
  bool IsRandStateBad_(const State& start_state, const State& rand_state);
};

}

#endif // RRTPLANNER_H_