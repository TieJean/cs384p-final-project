#include <glog/logging.h>
#include <cmath>
#include <tuple>
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/util/random.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"
#include "kd_tree.h"
#include "rrt_planner.h"
#include "config_reader/config_reader.h"
#include <limits>

CONFIG_FLOAT(RRT_STOP_DIST, "RRT_STOP_DIST");
CONFIG_FLOAT(RRT_W_A, "RRT_W_A");
CONFIG_FLOAT(RRT_W_C, "RRT_W_C");
CONFIG_FLOAT(RRT_CLEARANCE, "RRT_CLEARANCE");

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

void pointInBaselinkToWolrd(const State& baselink_state,
                            const Vector2f& point_in_baselink,
                            Vector2f& point_in_world) {
  point_in_world = Rotation2Df(baselink_state.angle) * point_in_baselink + baselink_state.loc;
}

RRTPlanner::RRTPlanner() 
  : global_goal_mloc_(0,0),
    global_goal_mangle_(0),
    global_goal_set_(false) {
  // tree = KDTree();
}

void RRTPlanner::SetMap(const string &map_file) {
  map_.Load(map_file);
}

void RRTPlanner::SetGlobalGoal(const Vector2f &loc, const float angle) {
  global_goal_mloc_ = loc;
  global_goal_mangle_ = angle;
  global_goal_set_ = true;
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
  float factor_thresh = 2.0;
  float rand_dist_to_goal  = (rand_state.loc  - global_goal_mloc_).norm();
  float start_dist_to_goal = (start_state.loc - global_goal_mloc_).norm();
  float start_dist_to_rand = (start_state.loc - rand_state.loc).norm();
  // rand is too far away from goal compared to start
  // start is too far away from rand compared to goal
  return rand_dist_to_goal > factor_thresh * start_dist_to_goal || start_dist_to_rand > factor_thresh * start_dist_to_goal;
}

// implement RRT*
void RRTPlanner::GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle) {
  State start_state(odom_loc, odom_angle);
  State goal_state(global_goal_mloc_, global_goal_mangle_);
  // vector<TreeNode> tree_nodes; // TODO redundant, replace me with tree traversal 
  vector<shared_ptr<TreeNode>> tree_nodes; // TODO redundant, replace me with tree traversal 
  double radius = distBtwStates(start_state, goal_state); // delibrately don't divide by 2

  TreeNode new_root_node(start_state, 0.0);
  TreeNode goal_node(goal_state, std::numeric_limits<float>::max());
  new_root_node.parent = nullptr;  // TODO redundant
  goal_node.parent = nullptr; // TODO redundant
  tree_nodes.push_back(make_shared<TreeNode>(new_root_node));
  tree_nodes.push_back(make_shared<TreeNode>(goal_node));

  while (false) { // TODO FIXME
    float x_rand = rng_.UniformRandom(-50, 50);
    float y_rand = rng_.UniformRandom(-50, 50);
    State rand_state(Vector2f(x_rand,y_rand), 0.0);
    if (IsRandStateBad_(start_state, rand_state)) { continue; }

    shared_ptr<TreeNode> rand_node;
    rand_node->state = rand_state;
    shared_ptr<TreeNode> nearest_node;
    float min_dist_to_rand_node = std::numeric_limits<float>::max();

    vector<shared_ptr<TreeNode>> nodes_around_rand;
    for (const auto& node : tree_nodes) {
      float dist_to_rand_node = distBtwTreeNodes(*node, *rand_node);
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

    State new_state;
    Steer_(nearest_node->state, rand_node->state, new_state);
    for (const auto& node : nodes_around_rand) {
      if (node == nearest_node) { continue; }
    }

  }

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
  return distBtwStates(state, goal_state) < 0.5; // TODO hardcoded for now
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
    next_angle = curr_state.angle + theta_c;
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
        cout << "p0: " << map_line.p0.transpose() << ", p1: " << map_line.p1.transpose() << endl;
        break; 
      }
    }
  } else { // for non-zero curvatures
    // cout << "case2" << endl;
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

/**
 * Given start_state and goal_state, compute next_state (closest reachable state from start to goal)
 * Return Trajectory from start_state to next_state
 */
Trajectory RRTPlanner::Steer_(const State& start_state, 
                              const State& goal_state,
                              State& next_state) {
  const size_t MAX_ITER = 1;

  Trajectory traj;
  traj.time = 0;
  State next_state_one_step;
  Control next_control_one_step;
  for (size_t t = 0; t < MAX_ITER; ++t) {
    if (SteerOneStep_(start_state, goal_state, next_state_one_step, next_control_one_step)) {
      traj.state.emplace_back(next_state_one_step);
      traj.control.emplace_back(next_control_one_step);
    } else {
      return traj;
    }
    if (AtGoalState_(next_state_one_step, goal_state)) { break; }
    if (AtGoal(next_state_one_step)) { break; }
  }
  next_state = next_state_one_step;
  return traj;
}

bool RRTPlanner::Steer_(const State& start_state, 
                        const State& goal_state,
                        State& next_state,
                        Trajectory& traj) {
  const size_t MAX_ITER = 1;

  traj.time = 0;
  bool found_traj = false;
  State next_state_one_step;
  Control next_control_one_step;
  for (size_t t = 0; t < MAX_ITER; ++t) {
    if (SteerOneStep_(start_state, goal_state, next_state_one_step, next_control_one_step)) {
      found_traj = true;
      traj.state.emplace_back(next_state_one_step);
      traj.control.emplace_back(next_control_one_step);
    } else {
      return found_traj;
    }
    if (AtGoalState_(next_state_one_step, goal_state)) { break; }
    if (AtGoal(next_state_one_step)) { break; }
  }
  next_state = next_state_one_step;
  return found_traj;
}

float RRTPlanner::GetTrajCost(const Trajectory& traj) {
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

void RRTPlanner::VisualizeTraj(const Trajectory& traj, VisualizationMsg& global_viz_msg) {
  for (size_t t = 0; t < traj.state.size(); ++t) {
    auto arc = GetTravelledArc_(traj.state[t], traj.control[t].c);
    Vector2f center   = std::get<0>(arc);
    float radius      = std::get<1>(arc);
    float start_angle = std::get<2>(arc);
    float end_angle   = std::get<3>(arc);
    // visualization::DrawCross(center, 0.3, 0xFF0000, global_viz_msg);
    // visualization::DrawCross(traj.state[t].loc, 0.3, 0xFF0000, global_viz_msg);
    // cout << "center: " << center.transpose() << ", radius: " << radius << ", start_angle: " << start_angle << ", end_angle: " << end_angle << endl;
    visualization::DrawArc(center, radius, start_angle, end_angle, 0x000000, global_viz_msg);
  }
}


}

