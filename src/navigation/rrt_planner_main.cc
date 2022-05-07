#include "rrt_planner.h"

using namespace rrt_planner;

int main() {
  State baselink_state(Vector2f(1, 0), -M_PI_2);
  Vector2f point_in_baselink(1, 0);
  Vector2f point_in_world;
  pointInBaselinkToWolrd(baselink_state, point_in_baselink, point_in_world);
  cout << point_in_world.transpose() << endl;
}