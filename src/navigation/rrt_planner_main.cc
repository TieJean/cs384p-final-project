#include "shared/math/geometry.h"
#include "rrt_planner.h"

using namespace rrt_planner;
using namespace geometry ;

int main() {
  RRTPlanner planner;
  planner.SetMap("maps/GDC1.txt");
  planner.SetGlobalGoal(Vector2f(-32, 20), 0.0);
  State start_state(Vector2f(-32, 18), M_PI_2);
  State goal_state(Vector2f(-30, 20), 0.0);

  Control control;
  control.a = 0;
  control.c = -1.0;
  State next_state;
  planner.SteerOneStepByControl_(start_state, control, next_state);
  cout << next_state << endl;

}