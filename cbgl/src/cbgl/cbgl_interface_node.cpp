// Copyright 2026 Alexandros Filotheou
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @brief Instantiates the interface class to be used for triggering the
 * functionality the node provides
 */
#include <exception>

#include "cbgl/cbgl_interface.hpp"

/**
 * @brief Runs the node until it is asked to stop, and reports a refusal to
 * start.
 * @details The executor is multi-threaded and takes the framework's own thread
 * count, which is the number of cores with a floor of two. Two is what the
 * service contract needs: one thread carries a run and another answers the
 * request that arrives while that run lasts, which is what lets the second
 * request be refused rather than queued. A single threaded executor would
 * answer it only once the run it was asking about had finished.
 * A setting outside its range is refused by the constructor, which throws with
 * the sentence inside `what()`. Catching it here is what turns that into a log
 * line an operator can read and an exit code a launch can act on, rather than
 * the abort an uncaught exception leaves.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<cbgl::Interface>();
    exec.add_node(node);
    exec.spin();
  }
  catch (const std::exception & problem)
  {
    RCLCPP_FATAL(rclcpp::get_logger("cbgl"), "%s", problem.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
