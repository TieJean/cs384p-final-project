#include <glog/logging.h>
#include <cmath>
#include <tuple>
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
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

// implement RRT*
void RRTPlanner::GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle) {
  // double x_rand= UniformRandom(-50,50);
  // double y_rand= UniformRandom(-50,50);
  // Vector2f rand_point(x_rand,y_rand);
  // double radius = 1;
  // // check if randomly sampled point is within radius of nearest node
  // auto nearest_points = tree.neighborhood_points(rand_point, radius);
  // for (Vector2f tree_point : nearest_points) {
  //   // check if there is no collision along path to random point from nearest point  
  //    if (collision_free (tree_point,rand_point)){
  //     Vector2f intermediate_point;


      // add node to kd tree then check nodes from parent to see if any nodes nee  to rearange connection     Steer_(tree_point,rand_point,intermediate_point);
      // update tree_point to point to intermediate_point    }
}
  
// checks if current location is close enough to the goal location
bool RRTPlanner::AtGoal(const Vector2f& robot_mloc) {
  if (!global_goal_set_) { return true; }
  return (robot_mloc - global_goal_mloc_).norm() < CONFIG_RRT_STOP_DIST;
}

bool RRTPlanner::AtGoal(const State& state_baselink) {
  return AtGoal(state_baselink.loc);
}

void pointInBaselinkToWolrd(const State& baselink_state,
                            const Vector2f& point_in_baselink,
                            Vector2f& point_in_world) {
  point_in_world = Rotation2Df(baselink_state.angle) * point_in_baselink + baselink_state.loc;
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
RRTPlanner::getTravelledArc_(const State& baselink_state, const float curvature) {
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
    float delta_y_in_baselink = -(1 - cos(theta_c)) * r_c;
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
    auto arc_travelled  = getTravelledArc_(curr_state, c);
    Vector2f a_center   = std::get<0>(arc_travelled);
    float a_radius      = std::get<1>(arc_travelled);
    float a_angle_start = std::get<2>(arc_travelled);
    float a_angle_end   = std::get<3>(arc_travelled);
    int rotation_sign   = std::get<4>(arc_travelled);
    // cout << "curr_state: " << curr_state << endl;
    for (line2f map_line : map_.lines) {
      distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, a_angle_start, a_angle_end, rotation_sign);
      // if (map_line.p0.x() > -36 && map_line.p0.x() < -30 && map_line.p0.y() > 18 && map_line.p0.y() < 19) {
      //   cout << "p0: " << map_line.p0.transpose() << ", p1: " << map_line.p1.transpose() << endl;
      //   cout << "distance: " << distance << ", CONFIG_RRT_CLEARANCE: " << CONFIG_RRT_CLEARANCE << endl;
      //   cout << "a_center: " << a_center.transpose() << ", a_radius: " << a_radius 
      //        << ", a_angle_start: " << a_angle_start << ", a_angle_end: " << a_angle_end << ", rotation_sign: " << rotation_sign << endl;
      //   cout << endl;
      // }
      if (distance < CONFIG_RRT_CLEARANCE) { 
        cout << "p0: " << map_line.p0.transpose() << ", p1: " << map_line.p1.transpose() << endl;
        break; 
      }
    }
  }
  if (distance >= CONFIG_RRT_CLEARANCE) {
    next_state = GetNextStateByCurvature_(curr_state, c);
  }
  return distance >= CONFIG_RRT_CLEARANCE;
}

float distBtwStates(const State& state1, const State& state2) {
  return (state1.loc - state2.loc).norm();
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
      foundCollisionFreeCurvature = true;
      float dist_to_goal_state = distBtwStates(start_state, next_state_by_curvature);
      if (dist_to_goal_state < best_dist) {
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
    if (AtGoal(next_state_one_step)) { break; }
  }
  next_state = next_state_one_step;
  return traj;
}


float RRTPlanner::GetCost_(const Control& u) {
  return (CONFIG_RRT_W_A * abs(u.a) + CONFIG_RRT_W_C * abs(u.c)) * t_interval_;
}

float RRTPlanner::GetTravelledDistOneStep_() {
  return MAX_VELOCITY * t_interval_;
}

void RRTPlanner::VisualizePath(VisualizationMsg& global_viz_msg) {

}

void RRTPlanner::VisualizeTraj(const Trajectory& traj, VisualizationMsg& global_viz_msg) {
  for (size_t t = 0; t < traj.state.size(); ++t) {
    auto arc = getTravelledArc_(traj.state[t], traj.control[t].c);
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

