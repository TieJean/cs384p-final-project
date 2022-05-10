#include <glog/logging.h>
#include <cmath>
#include <tuple>
#include <algorithm>
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/util/random.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "kd_tree.h"
#include "rrt_planner.h"
#include "config_reader/config_reader.h"

CONFIG_FLOAT(RRT_STOP_DIST, "RRT_STOP_DIST");
CONFIG_FLOAT(RRT_W_A, "RRT_W_A");
CONFIG_FLOAT(RRT_W_C, "RRT_W_C");
CONFIG_FLOAT(RRT_CLEARANCE, "RRT_CLEARANCE");
CONFIG_INT(RRT_SEARCH_BUFFER, "RRT_SEARCH_BUFFER");
CONFIG_FLOAT(RRT_LOCAL_HORIZON, "RRT_LOCAL_HORIZON");

using namespace geometry;
using geometry::line2f;

namespace rrt_planner {

config_reader::ConfigReader config_reader_({"config/navigation.lua"});

float distBtwStates(const State& state1, const State& state2) {
  return (state1.loc - state2.loc).norm();
}

float distBtwTreeNodes(const TreeNode& node1, const TreeNode& node2) {
  return distBtwStates(node1.state, node2.state);
}

float angleMod(const float& angle) {
  float ret = angle;
  if (ret < M_PI && ret > -M_PI) { return ret; }
  if (ret < 0) {
    while (ret  + 2 * M_PI < 0) {
      ret += (2 * M_PI);
    }
  } else {
    while (ret - 2 * M_PI > 0) {
      ret -= (2 * M_PI);
    }
  }
  if (ret > M_PI) {
    ret = 2 * M_PI - ret;
  } else if (angle < -M_PI) {
    ret = 2 * M_PI + ret;
  }
  return ret;
}

void pointInBaselinkToWolrd(const State& baselink_state,
                            const Vector2f& point_in_baselink,
                            Vector2f& point_in_world) {
  point_in_world = Rotation2Df(baselink_state.angle) * point_in_baselink + baselink_state.loc;
}

void printTree(const shared_ptr<TreeNode> goal_node) {
  cout << "---print tree---" << endl;
  shared_ptr<TreeNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    cout << *curr_node << endl;
    cout << curr_node->trajectory << endl;
    curr_node = (curr_node->parent);
  }
}

RRTPlanner::RRTPlanner() 
  : global_goal_mloc_(0,0),
    global_goal_mangle_(0),
    global_goal_set_(false) {
  this->path_start_idx_ = 0;
  this->found_gloabal_plan_ = false;
}

Trajectory RRTPlanner::GetGlobalTraj() {
  return this->global_plan_;
}

void RRTPlanner::PrintFinalPath() {
  printTree(goal_);
}

bool RRTPlanner::isGlobalPlanReady() {
  if (found_gloabal_plan_) {
    found_gloabal_plan_ = false;
    return true;
  } else {
    return found_gloabal_plan_;
  }
}

void RRTPlanner::SetMap(const string &map_file) {
  map_.Load(map_file);
}

void RRTPlanner::SetGlobalGoal(const Vector2f &loc, const float angle) {
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
}

Vector2f RRTPlanner::GetGlobalGoal() {
  return global_goal_mloc_;
}

bool RRTPlanner::IsStateCollisionFree_(const State& state) {
  return IsStateLocCollisionFree_(state.loc);
}

bool RRTPlanner::IsStateLocCollisionFree_(const Vector2f loc) {
  const float HALF_SIDE_LEN = 0.1;

  bool up_side_collide = false;
  bool left_side_collide = false;
  bool right_side_collide = false;
  bool bottom_side_collide = false;
  Vector2f point_top_left     = loc + Vector2f(-HALF_SIDE_LEN,  HALF_SIDE_LEN);
  Vector2f point_top_right    = loc + Vector2f( HALF_SIDE_LEN,  HALF_SIDE_LEN);
  Vector2f point_bottom_left  = loc + Vector2f(-HALF_SIDE_LEN, -HALF_SIDE_LEN);
  Vector2f point_bottom_right = loc + Vector2f( HALF_SIDE_LEN, -HALF_SIDE_LEN);
  for (line2f map_line : map_.lines) {
    up_side_collide     = (MinDistanceLineLine(map_line.p0, map_line.p1, point_top_left, point_top_right)       > kEpsilon);
    left_side_collide   = (MinDistanceLineLine(map_line.p0, map_line.p1, point_top_left, point_bottom_left)     > kEpsilon);
    right_side_collide  = (MinDistanceLineLine(map_line.p0, map_line.p1, point_top_right, point_top_right)      > kEpsilon);
    bottom_side_collide = (MinDistanceLineLine(map_line.p0, map_line.p1, point_bottom_left, point_bottom_right) > kEpsilon);
  }
  return (!up_side_collide) && (!left_side_collide) && (!right_side_collide) && (!bottom_side_collide);
}

bool RRTPlanner::IsRandStateBad_(const State& start_state, const State& rand_state) {
  float factor_thresh = 1.5;
  float rand_dist_to_goal  = (rand_state.loc  - global_goal_mloc_).norm();
  float start_dist_to_goal = (start_state.loc - global_goal_mloc_).norm();
  float start_dist_to_rand = (start_state.loc - rand_state.loc).norm();
  // rand is too far away from goal compared to start
  // start is too far away from rand compared to goal
  return rand_dist_to_goal > factor_thresh * start_dist_to_goal || start_dist_to_rand > factor_thresh * start_dist_to_goal;
}

bool RRTPlanner::RetrieveGlobalPlan_() {
  if (goal_->parent == nullptr) { return false; }
  global_plan_.state.clear();
  global_plan_.control.clear();
  shared_ptr<TreeNode> curr_node = goal_;

#if 1
  while (curr_node != root_) {
    assert(curr_node->trajectory.state.size() == curr_node->trajectory.control.size());
    for (int i = curr_node->trajectory.state.size()-1; i >= 0; --i) {
      global_plan_.state.push_back(curr_node->trajectory.state[i]);
    }
    for (int i = curr_node->trajectory.state.size()-1; i >= 0; --i) {
      global_plan_.control.push_back(curr_node->trajectory.control[i]);
    }
    curr_node = curr_node->parent;
  }
  std::reverse(global_plan_.state.begin(),   global_plan_.state.end());
  std::reverse(global_plan_.control.begin(), global_plan_.control.end());
  return true;
#endif

#if 0
  bool ret = false;
  while (curr_node->parent != nullptr) {
    assert(curr_node->trajectory.state.size() == curr_node->trajectory.control.size());
    for (int i = curr_node->trajectory.state.size()-1; i >= 0; --i) {
      global_plan_.state.push_back(curr_node->trajectory.state[i]);
    }
    for (int i = curr_node->trajectory.state.size()-1; i >= 0; --i) {
      global_plan_.control.push_back(curr_node->trajectory.control[i]);
    }
    if (curr_node == root_) {
      ret = true;
      break;
    }
    curr_node = curr_node->parent;
  }
  if (ret) {
    std::reverse(global_plan_.state.begin(),   global_plan_.state.end());
    std::reverse(global_plan_.control.begin(), global_plan_.control.end());
    cout << "global_plan_.state.size(): " << endl;
  } 
  return ret;
#endif
}

Vector2f RRTPlanner::GetLocalGoal(const Vector2f& robot_mloc, const float robot_mangle) {
  if (!global_goal_set_) { return robot_mloc; }
  
  int n_states = global_plan_.state.size();
  path_start_idx_ = path_start_idx_ <= CONFIG_RRT_SEARCH_BUFFER ? 0 : path_start_idx_ - CONFIG_RRT_SEARCH_BUFFER;
  int i = path_start_idx_;
  while (i < n_states && (global_plan_.state[i].loc-robot_mloc).norm() <  CONFIG_RRT_LOCAL_HORIZON) { ++i; }
  if (i == n_states) { return global_goal_mloc_; }
  if (i == path_start_idx_) {
    if (!GetGlobalPlan(robot_mloc, robot_mangle)) { // TODO
      LOG(FATAL) << "Failed to find global plan!" << endl;
    }
    found_gloabal_plan_ = true;
  }
  path_start_idx_ = i;
  return global_plan_.state[i].loc;
}

// including goal_node; in reverse order
void RRTPlanner::GetAncesterNodes_(const shared_ptr<TreeNode> goal_node, 
                                   vector<shared_ptr<TreeNode>>& nodes) {
  nodes.push_back(goal_node);
  shared_ptr<TreeNode> curr_node = goal_node;
  curr_node = curr_node->parent;
  while (curr_node != nullptr) {
    nodes.push_back(curr_node);
    curr_node = curr_node->parent;
  }
}

bool RRTPlanner::GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle, VisualizationMsg& global_viz_msg) {
  
// implement RRT*: https://docs.google.com/presentation/d/1RcltuVrbIx6wGGV1e5iqGIvMAVDnLJxu08OF-Pb0V4Y/edit#slide=id.ga2146f52c9_0_123

  const size_t MAX_N_ITER = 25000;
  const size_t EFF_N_ITER = 200;
  size_t effective_n_iter = 0;
  bool found_path_to_goal = false;
  State start_state(odom_loc, angleMod(odom_angle));
  State goal_state(global_goal_mloc_, 0.0);
  cout << "start state: " << start_state << endl;
  cout << "goa   state: " << goal_state << endl;
  vector<shared_ptr<TreeNode>> tree_nodes; // TODO redundant, replace me with tree traversal 
  float radius = max(distBtwStates(start_state, goal_state), (float)10.0); // delibrately don't divide by 2

  shared_ptr<TreeNode> new_root_node = make_shared<TreeNode>(start_state, 0.0);
  shared_ptr<TreeNode> goal_node     = make_shared<TreeNode>(goal_state, std::numeric_limits<float>::max());
  new_root_node->parent = nullptr;  // TODO redundant
  goal_node->parent = nullptr; // TODO redundant
  tree_nodes.push_back(new_root_node);
  // for (size_t i = 0; i < MAX_N_ITER || effective_n_iter > EFF_N_ITER; ++i) { // TODO FIXME
  for (size_t i = 0; i < MAX_N_ITER; ++i) { // TODO FIXME
      for (const auto& node : tree_nodes) {
        std::cout<<"node loc:" << node->state.loc<<std::endl;
        visualization::DrawCross(node->state.loc,1,0xFF0000, global_viz_msg);
    }
    if (effective_n_iter > EFF_N_ITER) { break; }
  // cout << "i: " << i << ", effective_n_iter: " << effective_n_iter << endl;
    float x_rand = rng_.UniformRandom(-35, -12);
    float y_rand = rng_.UniformRandom(0, 20);
    State rand_state(Vector2f(x_rand,y_rand), 0.0);
    if (IsRandStateBad_(start_state, rand_state)) { continue; }
    // if (!IsStateCollisionFree_(rand_state)) { continue; } // TODO add me back

    shared_ptr<TreeNode> rand_node = make_shared<TreeNode>();
    rand_node->state = rand_state;
    shared_ptr<TreeNode> nearest_node = make_shared<TreeNode>();
    float min_dist_to_rand_node = std::numeric_limits<float>::max();
    // find nearest node in set 
    vector<shared_ptr<TreeNode>> nodes_around_rand;
    for (const auto& node : tree_nodes) {
      float dist_to_rand_node = distBtwTreeNodes(*node, *rand_node);
      if (dist_to_rand_node < 0.25) {continue;} // tread them as the same node
      if (dist_to_rand_node <= radius) {
        nodes_around_rand.push_back(node);
        if (dist_to_rand_node < min_dist_to_rand_node) {
          min_dist_to_rand_node = dist_to_rand_node;
          nearest_node = node;
        }
      }
    }
    // found nothing
    if (min_dist_to_rand_node > radius) { continue; }
    // cout << "nearest_node: " << *nearest_node << endl;
    // cout << "rand_node: " << *rand_node << endl;

    shared_ptr<TreeNode> new_node = make_shared<TreeNode>();
    new_node->cost = std::numeric_limits<float>::max(); // TODO redundant
    State new_state_nearest_node, new_state;
    Trajectory traj_to_new_state;
    float cost_to_new_state;
    shared_ptr<TreeNode> candidate_parent;

    // find pink node (nearest_node) and green node (new_state_nearest_node)
    if (!SteerToRandNode_(nearest_node->state, rand_node->state, new_state_nearest_node, traj_to_new_state)) { continue; }
    new_node->state = new_state_nearest_node;
    cost_to_new_state = nearest_node->cost + GetTrajCost_(traj_to_new_state);
    new_node->cost = cost_to_new_state < new_node->cost ? cost_to_new_state : new_node->cost;
    new_node->trajectory = traj_to_new_state;
    candidate_parent = nearest_node;
    
    // steer from all blue nodes (nodes_around_rand \ nearest_node) to the green nodes (new_state_nearest_node)
    for (const auto& node : nodes_around_rand) {
      if (node == nearest_node) { continue; }
      if (!Steer_(node->state, new_state_nearest_node, new_state, traj_to_new_state)) { continue; }
      cost_to_new_state = node->cost + GetTrajCost_(traj_to_new_state);
      if (cost_to_new_state < new_node->cost) {
        new_node->cost = cost_to_new_state;
        new_node->trajectory = traj_to_new_state;
        new_node->state = new_state;
        candidate_parent = node;
      }
    }
    // add new node to its parent's children
    new_node->parent = candidate_parent;
    new_node->parent->children.insert(new_node);
    tree_nodes.push_back(new_node); // TODO redundant
    // printTree(*new_node);

    // add goal state to the set
    if (distBtwTreeNodes(*new_node, *goal_node) < radius) {
      nodes_around_rand.push_back(goal_node);
    }
    for (const auto& node : nodes_around_rand) {
      if (node == new_node->parent) { continue; }
      if (!Steer_(new_node->state, node->state, new_state, traj_to_new_state)) { continue; }
      if (node == goal_node) { found_path_to_goal = true; }
      cost_to_new_state = new_node->cost + GetTrajCost_(traj_to_new_state);
      if (cost_to_new_state < node->cost) {
        node->cost = cost_to_new_state;
        node->trajectory = traj_to_new_state;
        if (node->parent != nullptr) {
          node->parent->children.erase(node);
        }
        node->parent = new_node;
        node->parent->children.insert(node);
      }
    }
    effective_n_iter += 1;
    radius = std::max(radius * 0.98, 1.0);
    std::cout<<"# tree nodes: "<<tree_nodes.size()<<std::endl;
    // for (const auto& node : tree_nodes) {
      // std::cout<<"traj size"<<node->trajectory.state.size()<<std::endl;
      // VisualizeTraj(node->trajectory,global_viz_msg,0x000000);
    // }

  }
  root_ = new_root_node;
  goal_ = goal_node;
  RetrieveGlobalPlan_();
  cout << "effective_n_iter: " << effective_n_iter << ", final radius: " << radius << endl;
  // printTree(*goal_);
  return found_path_to_goal;
}

// implement RRT*: https://docs.google.com/presentation/d/1RcltuVrbIx6wGGV1e5iqGIvMAVDnLJxu08OF-Pb0V4Y/edit#slide=id.ga2146f52c9_0_123
  bool RRTPlanner::GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle) {
  const size_t MAX_N_ITER = 25000;
  const size_t EFF_N_ITER = 200;
  size_t effective_n_iter = 0;
  bool found_path_to_goal = false;
  State start_state(odom_loc, angleMod(odom_angle));
  State goal_state(global_goal_mloc_, 0.0);
  cout << "start state: " << start_state << endl;
  cout << "goa   state: " << goal_state << endl;
  vector<shared_ptr<TreeNode>> tree_nodes; // TODO redundant, replace me with tree traversal 
  float radius = max(distBtwStates(start_state, goal_state), (float)10.0); // delibrately don't divide by 2

  shared_ptr<TreeNode> new_root_node = make_shared<TreeNode>(start_state, 0.0);
  shared_ptr<TreeNode> goal_node     = make_shared<TreeNode>(goal_state, std::numeric_limits<float>::max());
  new_root_node->parent = nullptr;  // TODO redundant
  goal_node->parent = nullptr; // TODO redundant
  tree_nodes.push_back(new_root_node);
  // for (size_t i = 0; i < MAX_N_ITER || effective_n_iter > EFF_N_ITER; ++i) { // TODO FIXME
  for (size_t i = 0; i < MAX_N_ITER; ++i) { // TODO FIXME
    if (effective_n_iter > EFF_N_ITER) { break; }
  cout << "i: " << i << ", effective_n_iter: " << effective_n_iter << endl;
    float x_rand = rng_.UniformRandom(-35, -12);
    float y_rand = rng_.UniformRandom(0, 20);
    State rand_state(Vector2f(x_rand,y_rand), 0.0);
    if (IsRandStateBad_(start_state, rand_state)) { continue; }
    // if (!IsStateCollisionFree_(rand_state)) { continue; } // TODO add me back

    shared_ptr<TreeNode> rand_node = make_shared<TreeNode>();
    rand_node->state = rand_state;
    shared_ptr<TreeNode> nearest_node = make_shared<TreeNode>();
    float min_dist_to_rand_node = std::numeric_limits<float>::max();
    // find nearest node in set 
    vector<shared_ptr<TreeNode>> nodes_around_rand;
    for (const auto& node : tree_nodes) {
      float dist_to_rand_node = distBtwTreeNodes(*node, *rand_node);
      if (dist_to_rand_node < 0.25) {continue;} // tread them as the same node
      if (dist_to_rand_node <= radius) {
        nodes_around_rand.push_back(node);
        if (dist_to_rand_node < min_dist_to_rand_node) {
          min_dist_to_rand_node = dist_to_rand_node;
          nearest_node = node;
        }
      }
    }
    // found nothing
    if (min_dist_to_rand_node > radius) { continue; }
    // cout << "nearest_node: " << *nearest_node << endl;
    // cout << "rand_node: " << *rand_node << endl;

    shared_ptr<TreeNode> new_node = make_shared<TreeNode>();
    new_node->cost = std::numeric_limits<float>::max(); // TODO redundant
    State new_state_nearest_node, new_state;
    Trajectory traj_to_new_state;
    float cost_to_new_state;
    shared_ptr<TreeNode> candidate_parent;

    // find pink node (nearest_node) and green node (new_state_nearest_node)
    if (!SteerToRandNode_(nearest_node->state, rand_node->state, new_state_nearest_node, traj_to_new_state)) { continue; }
    new_node->state = new_state_nearest_node;
    cost_to_new_state = nearest_node->cost + GetTrajCost_(traj_to_new_state);
    new_node->cost = cost_to_new_state < new_node->cost ? cost_to_new_state : new_node->cost;
    new_node->trajectory = traj_to_new_state;
    candidate_parent = nearest_node;
  
    // steer from all blue nodes (nodes_around_rand \ nearest_node) to the green nodes (new_state_nearest_node)
    for (const auto& node : nodes_around_rand) {
      if (node == nearest_node) { continue; }
      if (!Steer_(node->state, new_state_nearest_node, new_state, traj_to_new_state)) { continue; }
      cost_to_new_state = node->cost + GetTrajCost_(traj_to_new_state);
      if (cost_to_new_state < new_node->cost) {
        new_node->cost = cost_to_new_state;
        new_node->trajectory = traj_to_new_state;
        new_node->state = new_state;
        candidate_parent = node;
      }
    }
    // add new node to its parent's children
    new_node->parent = candidate_parent;
    new_node->parent->children.insert(new_node);
    tree_nodes.push_back(new_node); // TODO redundant
    // printTree(*new_node);

    // add goal state to the set
    if (distBtwTreeNodes(*new_node, *goal_node) < radius) {
      nodes_around_rand.push_back(goal_node);
    }
    for (const auto& node : nodes_around_rand) {
      if (node == new_node->parent) { continue; }
      if (!Steer_(new_node->state, node->state, new_state, traj_to_new_state)) { continue; }
      if (node == goal_node) { found_path_to_goal = true; }
      cost_to_new_state = new_node->cost + GetTrajCost_(traj_to_new_state);
      if (cost_to_new_state < node->cost) {
        node->cost = cost_to_new_state;
        node->trajectory = traj_to_new_state;
        if (node->parent != nullptr) {
          node->parent->children.erase(node);
        }
        node->parent = new_node;
        node->parent->children.insert(node);
      }
    }
    effective_n_iter += 1;
    radius = std::max(radius * 0.98, 1.0);
  }
  root_ = new_root_node;
  goal_ = goal_node;
  RetrieveGlobalPlan_();
  cout << "effective_n_iter: " << effective_n_iter << ", final radius: " << radius << endl;
  // printTree(*goal_);
  return found_path_to_goal;
}
  
// checks if current location is close enough to the goal location
bool RRTPlanner::AtGoal(const Vector2f& robot_mloc) {
  if (!global_goal_set_) { return true; }
  return (robot_mloc - global_goal_mloc_).norm() < CONFIG_RRT_STOP_DIST;
}

bool RRTPlanner::AtGoal(const State& state_baselink) {
  return AtGoal(state_baselink.loc);
}

bool RRTPlanner::AtGoalState_(const State& state, const State& goal_state) {
  return distBtwStates(state, goal_state) < 0.25; // TODO hardcoded for now
}

tuple<Vector2f, float> 
getTurningCenterAndRadiusByCurvature(const State& baselink_state, const float curvature) {
  if (abs(curvature) < kEpsilon) {
    LOG(FATAL) << "getTurningCenterByCurvature assumes non-zero curvature, but input curvature is 0!";
  }
  float r_c = 1.0 / curvature; // has sign
  Vector2f turning_center_in_world;
  pointInBaselinkToWolrd(baselink_state, Vector2f(0, r_c), turning_center_in_world);
  return std::make_tuple(turning_center_in_world, r_c);
}

/**
 * @param baselink_state 
 * @param curvature 
 * @return <0> turning center in world frame
 * @return <1> turning radius (non-negvalue)
 * @return <2> starting angle
 * @return <3> ending andle
 * @return <4> rotation_sign 
 */
tuple<Vector2f, float, float, float, int>
RRTPlanner::GetTravelledArc_(const State& baselink_state, const float curvature) {
  float dist_traveled = GetTravelledDistOneStep_();

  auto center_and_r_c = getTurningCenterAndRadiusByCurvature(baselink_state, curvature);
  Vector2f a_center = std::get<0>(center_and_r_c);
  float r_c = std::get<1>(center_and_r_c);
  float a_radius = abs(r_c);
  float a_delta = dist_traveled / r_c; // positive if counterclock-wise
  float a_angle_start = r_c > 0 ? -M_PI_2 + baselink_state.angle : M_PI_2 + baselink_state.angle + a_delta;
  float a_angle_end = r_c > 0 ? -M_PI_2 + baselink_state.angle + a_delta : baselink_state.angle + M_PI_2;
  // int rotation_sign = r_c > 0 ? 1 : -1;
  int rotation_sign = 1; // TODO check for correctness
  return std::make_tuple(a_center, a_radius, a_angle_start, a_angle_end, rotation_sign);
}

// For non-zero curvature, this function assume abs(theta) < M_PI
State RRTPlanner::GetNextStateByCurvature_(const State& curr_state, const float curvature) {
  Vector2f next_loc;
  float next_angle;
  float dist_traveled = GetTravelledDistOneStep_();
  if (abs(curvature) < kEpsilon) {
    Vector2f curr_loc = curr_state.loc;
    float curr_angle = curr_state.angle;
    next_loc = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_traveled, 0);
    next_angle = curr_angle;
  } else {
    float r_c = 1.0 / curvature; // has sign
    float theta_c = dist_traveled / r_c; // has sign
    float delta_x_in_baselink = sin(theta_c) * r_c;
    float delta_y_in_baselink = (1 - cos(theta_c)) * r_c;
    pointInBaselinkToWolrd(curr_state, Vector2f(delta_x_in_baselink, delta_y_in_baselink), next_loc);
    next_angle = angleMod(curr_state.angle + theta_c);
  }
  State next_state;
  next_state.loc = next_loc;
  next_state.angle = next_angle;
  return next_state;
}

// helper function for Steer
bool RRTPlanner::SteerOneStepByControl_(const State& curr_state, const Control& control, State& next_state) {
  float c = control.c;
  float dist_to_front = CAR_BASE;
  // float dist_to_side = CAR_WIDTH / 2.0;
  float dist_traveled = GetTravelledDistOneStep_(); // TODO Fix hardcoded max velocity 1.0
  Vector2f curr_loc = curr_state.loc;
  float curr_angle = curr_state.angle;
    
  float distance = 0.0;
  if (abs(c) < kEpsilon) { // if curvature is 0
  // cout << "case1" << endl;
    Vector2f car_front_p = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_to_front, 0);
    Vector2f car_end_p   = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_to_front + dist_traveled, 0);
    for (line2f map_line : map_.lines) {
      distance = MinDistanceLineLine(map_line.p0, map_line.p1, car_front_p, car_end_p);
      if (distance < CONFIG_RRT_CLEARANCE) { 
        // cout << "p0: " << map_line.p0.transpose() << ", p1: " << map_line.p1.transpose() << endl;
        break; 
      }
    }
  } else { // for non-zero curvatures
    // cout << "case2: c = " << c << endl;
    auto arc_travelled  = GetTravelledArc_(curr_state, c);
    Vector2f a_center   = std::get<0>(arc_travelled);
    float a_radius      = std::get<1>(arc_travelled);
    float a_angle_start = std::get<2>(arc_travelled);
    float a_angle_end   = std::get<3>(arc_travelled);
    // int rotation_sign   = std::get<4>(arc_travelled);
    // cout << "curr_state: " << curr_state << endl;
    for (line2f map_line : map_.lines) {
      // distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, a_angle_start, a_angle_end, rotation_sign);
      if (c > 0) {
        distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, a_angle_start, a_angle_end, (int)1);
      } else {
        distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, a_angle_start, a_angle_end, (int)1);
        // distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, (float)(0.0), (float)(0.175), 1);
      }
      if (distance < kEpsilon) { // TODO FIXME
        // cout << "distance: " << distance << ", curvature: " << control.c << endl;
        // cout << "p0: " << map_line.p0.transpose() << ", p1: " << map_line.p1.transpose() << endl;
        // cout << "a_center: " << a_center.transpose() << ", a_radius: " << a_radius 
        //      << ", a_angle_start: " << a_angle_start << ", a_angle_end: " << a_angle_end << endl;
        break; 
      }
    }
  }
  if (distance >= CONFIG_RRT_CLEARANCE) {
    next_state = GetNextStateByCurvature_(curr_state, c);
  }
  return distance >= CONFIG_RRT_CLEARANCE;
}

// greedy, but no optimality guarantee
// pick curvature that can take robot closest to goal_state
bool RRTPlanner::SteerOneStep_(const State& start_state, 
                               const State& goal_state,
                               State& next_state, Control& control_to_next_state) {
  const float CURVATURE_STEP = 0.05;
  
  float best_dist = std::numeric_limits<float>::max();
  State next_state_by_curvature;
  bool foundCollisionFreeCurvature = false;
  for (float c = MIN_CURVATURE; c <= MAX_CURVATURE; c += CURVATURE_STEP) {
    Control control;
    control.c = c;
    control.a = 0; // TODO FIXME
    if (SteerOneStepByControl_(start_state, control, next_state_by_curvature)) {
      // cout << "c = " << c << endl;
      // cout << "next_state_by_curvature: " << next_state_by_curvature << endl;
      foundCollisionFreeCurvature = true;
      float dist_to_goal_state = distBtwStates(next_state_by_curvature, goal_state);
      if (dist_to_goal_state < best_dist) {
        // cout << "dist_to_goal_state: " << dist_to_goal_state 
        //      << ", best_dist: " << best_dist << ", c: " << control.c << endl;
        next_state = next_state_by_curvature;
        control_to_next_state =  control;
        best_dist = dist_to_goal_state;
      }
    } else {
      continue;
    }
  }
  return foundCollisionFreeCurvature;
}

bool RRTPlanner::SteerByMaxIter_(const State& start_state, 
                                 const State& goal_state,
                                 const size_t MAX_ITER,
                                 State& next_state, Trajectory& traj) {
  traj.state.clear();
  traj.control.clear();
  traj.time = 0;
  bool found_traj = false;
  State curr_state, next_state_one_step;
  Control next_control_one_step;

  curr_state = start_state;
  for (size_t t = 0; t < MAX_ITER; ++t) {
    if (SteerOneStep_(curr_state, goal_state, next_state_one_step, next_control_one_step)) {
      // cout << "curr_state: " << curr_state << endl;
      traj.state.emplace_back(curr_state);
      traj.control.emplace_back(next_control_one_step);
    } else {
      return found_traj;
    }
    traj.time += t_interval_;
    if (AtGoalState_(next_state_one_step, goal_state)) { 
      // cout << "at goal state" << endl;
      found_traj = true;
      break; 
    }
    if (AtGoal(next_state_one_step)) { 
      found_traj = true;
      break; 
    }
    curr_state = next_state_one_step;
  }
  next_state = next_state_one_step;
  return found_traj;
}

bool RRTPlanner::Steer_(const State& start_state, 
                        const State& goal_state,
                        State& next_state,
                        Trajectory& traj) {
  return SteerByMaxIter_(start_state, goal_state, 20, next_state, traj);
}

bool RRTPlanner::SteerToRandNode_(const State& start_state, 
                                  const State& goal_state,
                                  State& next_state,
                                  Trajectory& traj) {
  return SteerByMaxIter_(start_state, goal_state, 10, next_state, traj);
}

float RRTPlanner::GetTrajCost_(const Trajectory& traj) {
  float total_cost = 0.0;
  for (const auto& u : traj.control) {
    total_cost += GetCostOneStep_(u);
  }
  return total_cost;
}

float RRTPlanner::GetCostOneStep_(const Control& u) {
  return (CONFIG_RRT_W_A * abs(u.a) + CONFIG_RRT_W_C * abs(u.c)) * t_interval_;
}

float RRTPlanner::GetTravelledDistOneStep_() {
  return MAX_VELOCITY * t_interval_;
}

void RRTPlanner::VisualizePath(VisualizationMsg& global_viz_msg) {

}

void RRTPlanner::VisualizeTraj(const Trajectory& traj, VisualizationMsg& global_viz_msg,uint32_t color) {
  if (traj.state.size() < 4) {return;}
  for (size_t t = 0; t < traj.state.size()-4; ++t) { // TODO FIXME
    if (abs(traj.control[t].c) < kEpsilon) {
      float dist_traveled = GetTravelledDistOneStep_();
      Vector2f baselink_end_in_world;
      pointInBaselinkToWolrd(traj.state[t], Vector2f(dist_traveled, 0), baselink_end_in_world);
      visualization::DrawLine(traj.state[t].loc, baselink_end_in_world, color, global_viz_msg);
    } else {
      auto arc = GetTravelledArc_(traj.state[t], traj.control[t].c);
      Vector2f center   = std::get<0>(arc);
      float radius      = std::get<1>(arc);
      float start_angle = std::get<2>(arc);
      float end_angle   = std::get<3>(arc);
      // visualization::DrawCross(center, 0.3, 0xFF0000, global_viz_msg);
      // visualization::DrawCross(traj.state[t].loc, 0.3, 0xFF0000, global_viz_msg);
      // cout << "center: " << center.transpose() << ", radius: " << radius << ", start_angle: " << start_angle << ", end_angle: " << end_angle << endl;
      visualization::DrawArc(center, radius, start_angle, end_angle, color, global_viz_msg);
    }
  }
}

}
//0x000000