#include "main.hpp"

static int phase = 0;
static double tactic_start_time = 0;
static status tactic_status = RUNNING;
uint32_t tactic_delay = 0xFFFFFFFF;

status tactic_template()
{
switch (phase)
    {
    case 0:
      if (rot_to_xy(750, 125, FORWARD, MAX_ANG_VEL) == SUCCESS)
        phase = 1;
      break;

    case 1:
      if (delay_nb_2(&tactic_delay, 2000))
      {
        tactic_start_time = get_time();
        phase = 2;
      }
      break;

    case 2:
      if (move_to_xy(750, 125, FORWARD, MAX_VEL, MAX_ANG_VEL) == SUCCESS)
        phase = 3;
      break;

    case 3:
      if (move_on_path(0, 100, -90, FORWARD, true, MAX_VEL) == SUCCESS)
        phase = 4;
      break;

    case 4:
      if (move_on_dir(200, FORWARD, MAX_VEL) == SUCCESS)
        phase = 5;
      break;

    case 5:
      if (move_on_path(-750, -750, -90, FORWARD, false, MAX_VEL) == SUCCESS)
        phase = 6;
      break;

    case 6:
      if (rot_to_angle(-90, MAX_ANG_VEL) == SUCCESS)
        phase = 7;
      break;

    case 7:
      if (move_on_dir(175, FORWARD, MAX_VEL/4) == SUCCESS)
        phase = 8;
      break;

    case 8:
      if (delay_nb_2(&tactic_delay, 2000))
        phase = 9;
      break;

    case 9:
      if (move_on_dir(175, BACKWARD, MAX_VEL/2) == SUCCESS)
        phase = 10;
      break;

    case 10:
      if (move_to_xy(-1125, 600, FORWARD, MAX_VEL, MAX_ANG_VEL) == SUCCESS)
      {
        std::cout << " " << std::endl;
        std::cout << "tactic time = " << get_time() - tactic_start_time << std::endl;
        phase = 99;
        tactic_status = SUCCESS;
      }
      break;
}
return tactic_status;
}