#include <cmath>
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
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

State RRTPlanner::GetNextStateByCurvature(const State& curr_state, const float curvature) {
  if (curvature < kEpsilon) {
    // Vector2f curr_loc = curr_state.loc;
    // float curr_angle = curr_state.angle;
    // float dist_traveled = MAX_VELOCITY * t_interval_;
    // curr_loc + curr_angle * Vector2f(dist_traveled, 0);
  }
  return State();
}

// helper function for Steer
void RRTPlanner::SteerOneStep_(const State& curr_state, State& next_state) {
  const float CURVATURE_STEP = 0.05;
  float dist_to_front = CAR_BASE;
  // float dist_to_side = CAR_WIDTH / 2.0;
  float dist_traveled = MAX_VELOCITY * t_interval_; // TODO Fix hardcoded max velocity 1.0
  Vector2f curr_loc = curr_state.loc;
  float curr_angle = curr_state.angle;

  for (float c = MIN_CURVATURE; c <= MAX_CURVATURE; c += CURVATURE_STEP) {
    if (abs(c) < kEpsilon) {
      Vector2f car_front_p = curr_loc + curr_angle * Vector2f(dist_to_front, 0);
      Vector2f car_end_p   = curr_loc + curr_angle * Vector2f(dist_to_front + dist_traveled, 0);
      float distance = 0.0;
      for (line2f map_line : map_.lines) {
        distance = MinDistanceLineLine(map_line.p0, map_line.p1, car_front_p, car_end_p);
        if (distance < CONFIG_RRT_CLEARANCE) { break; }
      }
      if (distance >= CONFIG_RRT_CLEARANCE) {

      }
    }
  }
}

/**
 * Given start_state and goal_state, compute next_state (closest reachable state from start to goal)
 * Return Trajectory from start_state to next_state
 */
Trajectory RRTPlanner::Steer_(const State& start_state, 
                              const State& goal_state,
                              State& next_state) {
  return Trajectory();
}


float RRTPlanner::GetCost_(const Control& u) {
  return (CONFIG_RRT_W_A * abs(u.a) + CONFIG_RRT_W_C * abs(u.c)) * t_interval_;
}

void RRTPlanner::VisualizePath(VisualizationMsg& global_viz_msg) {

}


}

