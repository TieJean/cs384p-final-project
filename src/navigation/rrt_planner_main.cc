#include "shared/math/geometry.h"
#include "rrt_planner.h"

using namespace rrt_planner;
using namespace geometry ;

int main() {
  RRTPlanner planner;
  planner.SetMap("maps/GDC1.txt");
  planner.SetGlobalGoal(Vector2f(-32, 20), 0.0);
  State start_state(Vector2f(-32, 20), -M_PI_2);
  State goal_state(Vector2f(-30, 20), 0.0);

  Control next_control;
  next_control.a = 0;
  next_control.c = -1.0;
  State next_state;
  if (planner.SteerOneStep_(start_state, goal_state, next_state, next_control)) {
    cout << next_state << endl;
    cout << "next_control.c: " << next_control.c << endl;
  } else {
    cout << "cannot steer one step" << endl;
  }
  

}