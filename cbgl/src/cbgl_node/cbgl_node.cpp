/*
 * cbgl - [IROS'24] Globally localise your 2D LIDAR in a 2D map in no time
 *
 * Copyright (c) 2024 Alexandros PHILOTHEOU
 *
 * Licensed under the MIT License.
 * See LICENSE.MIT for details.
 */
#include <cbgl_node/cbgl.h>

#include <cstdio>
#include <exception>

/**
 * @brief Runs the node, and turns a refusal to start into a diagnosis rather
 * than a core dump.
 * @details The node refuses a configuration it cannot work from by throwing,
 * and without a handler anywhere the runtime terminates at the throw point
 * instead of unwinding. Nothing is destroyed, the reason reaches only the
 * terminate handler's own output rather than the ROS log, the advertisements
 * this node made are never withdrawn from the master, and the operator is
 * shown a process that died on signal six with a core file beside it. Catching
 * here costs nothing and turns all of that into one sentence and an exit code.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbgl_node");

  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    CBGL cbgl(nh, nh_private);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    /* Written twice on purpose. The logger reaches an operator watching the
     * ROS output, and the stream reaches one reading a terminal or a launch
     * log, which is where somebody debugging a node that will not start
     * actually looks. A refusal nobody can read is barely better than a
     * crash. */
    fprintf(stderr, "%s\n", ex.what());
    fflush(stderr);
    ROS_FATAL("%s", ex.what());
    ros::shutdown();
    return 1;
  }

  return 0;
}
