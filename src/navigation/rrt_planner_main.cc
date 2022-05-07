#include "rrt_planner.h"

using namespace rrt_planner;

int main() {
  RRTPlanner planner;
  planner.SetMap("maps/GDC1.txt");
  planner.SetGlobalGoal(Vector2f(-32, 20), 0.0);
  State start_state(Vector2f(-32, 20), 0.0);
  State goal_state(Vector2f(-30, 20), 0.0);

  State baselink_state(Vector2f(-32, 20), 0);
  auto ret = planner.getTravelledArc_(baselink_state, 0.5);
  Vector2f a_center = std::get<0>(ret);
  float a_radius = std::get<1>(ret);
  float a_angle_start = std::get<2>(ret);
  float a_angle_end = std::get<3>(ret);
  int rotation_sign = std::get<4>(ret);

  cout << "a_center: " << a_center.transpose() << endl;
  cout << "a_radius: " << a_radius << endl;
  cout << "a_angle_start: " << a_angle_start << endl;
  cout << "a_angle_end: " << a_angle_end << endl;
  cout << "rotation_sign: " << rotation_sign << endl;
  
}