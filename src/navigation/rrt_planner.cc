#include <glog/logging.h>
#include <cmath>
#include <tuple>
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
vector<pair<float,float>> RRTPlanner::findNearestPoints( pair<float,float> rand_point, double radius){
    vector<pair<float,float>> inRadius;
    for (auto& it: rrt_tree) {
    Vector2f rand_point_vector(rand_point.first,rand_point.second);
    Vector2f tree_vector(it.first.first,it.first.second);
    if ((tree_vector-rand_point_vector).norm()<radius){
        inRadius.push_back(it.first);
    }
}
return inRadius;
}
// implement RRT*
bool RRTPlanner::isCollisionFree(const State& start_state, const State& end_state,State& next_state){

    //check if end_state is within the map and a wall isnt withing a bounding box around end_state
    //check to see if there is a valid path from start_state to end_state 
    return false;
}
Trajectory RRTPlanner::getTrajCost(const State& start_state, const State& end_state){
    return NULL;
}
      // implement RRT*

void RRTPlanner::GetGlobalPlan(const Vector2f& odom_loc, const float odom_angle) {
    // double x_rand= UniformRandom(-50,50);
    // double y_rand= UniformRandom(-50,50);
    double radius = 1;
    pair<float,float> rand_point =make_pair(0,0);
    Vector2f rand_point_vector(0,0);
    TreeNode rand_point_;
    rand_point_.state.loc= rand_point_vector; 
    
    auto nearest_points = findNearestPoints(rand_point,radius);


    TreeNode min_cost_point;
    TreeNode rand_tree_point;
    min_cost_point.cost=INFINITY;

    bool collision_free_path = false;
    State intermediate_state;
    State closest_intermediate_state;
    int i=0;
    vector<pair<float,float>> temp_children;
    for (auto tree_point : nearest_points) {
        TreeNode tree_point_ = rrt_tree[tree_point];

        // check if there is no collision along path to random point from nearest point probably change with steer_
        
        if (isCollisionFree (tree_point_.state,rand_point_.state,intermediate_state)){
            collision_free_path=true;
            
            //find lowest cost path to rand_point
            double intermediate_cost = GetCost_(tree_point_.state,intermediate_state);

            //calculate cost from tree_point to intermediate_point

            if (tree_point_.cost+intermediate_cost<min_cost_point.cost){
                min_cost_point=tree_point_;

                rand_point_.cost = tree_point_.cost+ intermediate_cost;
                closest_intermediate_state= intermediate_state;
                temp_children.push_back(tree_point);
                i+=1;
            }
        }
        
      
    }


    // add intermediate point to tree and remove parent from nn search
    if(collision_free_path){
        temp_children.erase(temp_children.begin()+i);
        nearest_points= temp_children;
        rand_point_.state=closest_intermediate_state;
        rand_point_.parent = min_cost_point; 
        min_cost_point.children.push_back(rand_point_);
        auto new_node = make_pair(rand_point,rand_point_);
        rrt_tree.insert(new_node);
    }
            //update points in radius to point to intermediate point


    for (auto tree_point : nearest_points) {
        TreeNode tree_point_ = rrt_tree[tree_point];
        //calculate cost from rand_point to tree_point
        double cost_to_reach_tree_point= GetCost_(rand_point_.state,tree_point_.state);
        if (rand_point_.cost+cost_to_reach_tree_point<tree_point_.cost){
            tree_point_.parent.children.erase(tree_point_);
            tree_point_.parent= rand_point_;
            rand_point_.children.push_back(tree_point_);
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
  float a_angle_start = baselink_state.angle;
  float a_angle_end = a_angle_start + a_delta;
  int rotation_sign = r_c > 0 ? 1 : -1;
  return std::make_tuple(a_center, a_radius, a_angle_start, a_angle_end, rotation_sign);
}

// For non-zero curvature, this function assume abs(theta) < M_PI
State RRTPlanner::GetNextStateByCurvature_(const State& curr_state, const float curvature) {
  Vector2f next_loc;
  float next_angle;
  float dist_traveled = GetTravelledDistOneStep_();
  if (curvature < kEpsilon) {
    Vector2f curr_loc = curr_state.loc;
    float curr_angle = curr_state.angle;
    next_loc = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_traveled, 0);
    next_angle = curr_angle;
  } else {
    float r_c = 1.0 / curvature; // has sign
    float theta_c = dist_traveled / r_c; // has sign
    float delta_x_in_baselink = sin(theta_c) * r_c;
    float delta_y_in_baselink = -(1 - cos(theta_c)) * r_c;
    Vector2f next_loc;
    pointInBaselinkToWolrd(curr_state, Vector2f(delta_x_in_baselink, delta_y_in_baselink), next_loc);
    next_angle = curr_state.angle + theta_c;
  }
  return State(next_loc, next_angle);
}

// helper function for Steer
void RRTPlanner::SteerOneStepByControl_(const State& curr_state, const Control& control, State& next_state) {
  float c = control.c;
  float dist_to_front = CAR_BASE;
  // float dist_to_side = CAR_WIDTH / 2.0;
  float dist_traveled = GetTravelledDistOneStep_(); // TODO Fix hardcoded max velocity 1.0
  Vector2f curr_loc = curr_state.loc;
  float curr_angle = curr_state.angle;
    
  float distance = 0.0;
  if (abs(c) < kEpsilon) { // if curvature is 0
    Vector2f car_front_p = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_to_front, 0);
    Vector2f car_end_p   = curr_loc + Rotation2Df(curr_angle) * Vector2f(dist_to_front + dist_traveled, 0);
    for (line2f map_line : map_.lines) {
      distance = MinDistanceLineLine(map_line.p0, map_line.p1, car_front_p, car_end_p);
      if (distance < CONFIG_RRT_CLEARANCE) { break; }
    }
  } else { // for non-zero curvatures
    auto arc_travelled  = getTravelledArc_(curr_state, c);
    Vector2f a_center   = std::get<0>(arc_travelled);
    float a_radius      = std::get<1>(arc_travelled);
    float a_angle_start = std::get<2>(arc_travelled);
    float a_angle_end   = std::get<3>(arc_travelled);
    int rotation_sign   = std::get<4>(arc_travelled);
    for (line2f map_line : map_.lines) {
      distance = MinDistanceLineArc(map_line.p0, map_line.p1, a_center, a_radius, a_angle_start, a_angle_end, rotation_sign);
      if (distance < CONFIG_RRT_CLEARANCE) { break; }
    }
  }
  if (distance >= CONFIG_RRT_CLEARANCE) {
    next_state = GetNextStateByCurvature_(curr_state, c);
  }
    
}

float distBtwStates(const State& state1, const State& state2) {
  return (state1.loc - state2.loc).norm();
}

// greedy, but no optimality guarantee
// pick curvature that can take robot closest to goal_state
Control RRTPlanner::SteerOneStep_(const State& start_state, 
                               const State& goal_state,
                               State& next_state) {
  const float CURVATURE_STEP = 0.05;
  
  float best_dist = std::numeric_limits<float>::max();
  Control control_to_next_state;
  State next_state_by_curvature;
  for (float c = MIN_CURVATURE; c <= MAX_CURVATURE; c += CURVATURE_STEP) {
    Control control;
    control.c = c;
    control.a = 0; // TODO FIXME
    SteerOneStepByControl_(start_state, control, next_state_by_curvature);
    float dist_to_goal_state = distBtwStates(start_state, next_state_by_curvature);
    if (dist_to_goal_state < best_dist) {
      next_state = next_state_by_curvature;
      control_to_next_state =  control;
      best_dist = dist_to_goal_state;
    }
  }
  return control_to_next_state;
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
  for (size_t t = 0; t < MAX_ITER; ++t) {
    Control control = SteerOneStep_(start_state, goal_state, next_state_one_step);
    traj.state.emplace_back(next_state_one_step);
    traj.control.emplace_back(control);
    if (AtGoal(next_state_one_step)) { break; }
  }

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


}


