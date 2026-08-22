/*
 * cbgl - [IROS'24] Globally localise your 2D LIDAR in a 2D map in no time
 *
 * Copyright (c) 2024 Alexandros PHILOTHEOU
 *
 * Licensed under the MIT License.
 * See LICENSE.MIT for details.
 */
#ifndef CBGL_H
#define CBGL_H

#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <utime.h>
#include <cmath>
#include <iostream>
#include <time.h>
#include <limits.h>
#include <boost/assign.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_matrix.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Duration.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/fsm_core.h>
#include "utils/map/map.h"
#include "utils/pf/pf.h"
#include <csm/csm_all.h>  /* csm defines min and max, but Eigen complains */
#include <egsl/egsl_macros.h>

#undef min
#undef max

#include "utils/range_libc/includes/RangeLib.h"
#include "utils/range_libc/vendor/lodepng/lodepng.h"
#include "utils/occupancy_grid_utils/ray_tracer.h"


class CBGL
{
  public:

    CBGL(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~CBGL();

  private:

    /* **** ros */

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    /* Name of this package */
    std::string PKG_NAME;

    /* Cached fftw plans? */
    bool cachedFFTW3Plans_;

    /* subscribers */
    ros::Subscriber scan_subscriber_;
    ros::Subscriber map_subscriber_;

    /* services */
    ros::ServiceServer global_localisation_service_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    /* static, cached */
    tf::Transform base_to_laser_;

    /* static, cached, calculated from base_to_laser_ */
    tf::Transform laser_to_base_;

    /* publishers */
    ros::Publisher global_pose_publisher_;
    ros::Publisher execution_time_publisher_;
    ros::Publisher best_particle_publisher_;
    ros::Publisher all_hypotheses_publisher_ ;
    ros::Publisher all_hypotheses_caer_publisher_ ;
    ros::Publisher top_caer_hypotheses_publisher_ ;
    ros::Publisher world_scan_publisher_;
    ros::Publisher map_scan_publisher_;

    /* **** parameters */

    std::string base_frame_id_;
    std::string fixed_frame_id_;
    std::string odom_frame_id_;
    bool tf_broadcast_;

    std::string scan_topic_;
    std::string map_topic_;
    std::string map_file_;
    std::string output_pose_topic_;
    std::string global_localisation_service_name_;

    std::string laser_z_orientation_;
    bool do_undersample_scan_;
    int undersample_rate_;

    /* cache a scan's number of rays */
    unsigned int nrays_;
    int sgn_;
    double angle_min_;
    double angle_inc_;
    double map_res_;
    unsigned int map_hgt_;

    /* cache the map origin's orientation, and its sine and cosine, so that a
     * cast into the grid need not derive them from a quaternion every time */
    double map_yaw_;
    double map_sin_yaw_;
    double map_cos_yaw_;

    /* For generating the hypothesis set H */
    pf_t *pf_hyp_;
    map_t* map_hyp_;

    /* How to scanMap the map? */
    std::string map_scan_method_;

    bool do_icp_;
    bool do_fsm_;

    /* FSM Params */
    FSM::input_params ip_;

    /* Forward plan (DFT) */
    fftw_plan r2rp_;

    /* Backward plan (IDFT) */
    fftw_plan c2rp_;

    /* Covariance params */
    double initial_cov_xx_;
    double initial_cov_yy_;
    double initial_cov_aa_;

    bool received_scan_;
    bool received_map_;
    bool received_pose_cloud_;
    bool running_;
    bool received_start_signal_;

    /* fixed-to-base tf (pose of base frame in fixed frame) */
    tf::Transform f2b_;

    /* the map */
    nav_msgs::OccupancyGrid map_;

    /* The map converted to a suitable structure for 3rd-party lib ray-casting */
    ranges::OMap omap_;

    /* 3rd-party ray-casters (range_libc) */
    ranges::RayMarching rm_;
    ranges::CDDTCast cddt_;
    ranges::BresenhamsLine br_;

    /* The world scan received latest */
    sensor_msgs::LaserScan::Ptr latest_world_scan_;

    sm_params input_;
    sm_result output_;

    /* Crucial CBGL params */
    std::vector<geometry_msgs::Pose::Ptr> dispersed_particles_;
    int dl_;
    int da_;
    int top_k_caers_;
    bool publish_pose_sets_;



    /***************************************************************************
     * @brief Broadcasts the estimated pose as the odom<--map transform
     */
    void broadcast_global_pose_tf(
      const geometry_msgs::Pose& pose);

    /***************************************************************************
     * @brief Calculation of the cumulative absolute error per ray metric
     */
    double caer(const sensor_msgs::LaserScan::Ptr& sr,
      const sensor_msgs::LaserScan::Ptr& sv);
    double caer(const std::vector<float>& sr,
      const std::vector<float>& sv);

    /***************************************************************************
     * Used by FSM
     */
    void cacheFFTW3Plans(const unsigned int& sz);

    /***************************************************************************
     * @brief Convert an OccupancyGrid map message into the internal
     * representation. This allocates a map_t and returns it. Stolen from amcl.
     */
    map_t* convertMap(const nav_msgs::OccupancyGrid& map_msg);

    /***************************************************************************
     * @brief Convert an OccupancyGrid map message into a unsigned char* array.
     * If this array is immediately inputted to lodepng::encode the latter
     * returns 0.
     * @param[in] map_msg [const nav_msgs::OccupancyGrid&] The map message to be
     * converted.
     * @param[out] converted [unsigned char*] The output array
     * @ return void
     */
    void convertMapToPNG(
      const nav_msgs::OccupancyGrid& map_msg,
      unsigned char* converted);

    /***************************************************************************
     * @brief Convert an OccupancyGrid map message into a unsigned char* array,
     * encode it into PNG, and store the image to file.
     * @param[in] map_msg [const nav_msgs::OccupancyGrid&] The map message to be
     * converted to png
     * @param[in] filename [const std::string&] The name of the png file to be
     * stored
     * @ return void
     */
    void convertMapToPNG(
      const nav_msgs::OccupancyGrid& map_msg,
      const std::string& filename);

    /***************************************************************************
     * @brief This function uses the icp result for correcting the amcl pose.
     * Given the icp output, this function express this correction in the
     * map frame. The result is the icp-corrected pose in the map frame.
     * @param[in,out] icp_corrected_pose
     * [geometry_msgs::Pose::Ptr&] The icp-corrected amcl
     * pose
     * @return void
     */
    void correctICPPose(geometry_msgs::Pose::Ptr& icp_corrected_pose,
      const tf::Transform& f2b);

    /***************************************************************************
     * @brief Creates a transform from a 2D pose (x,y,theta)
     * @param[in] x [const double&] The x-wise coordinate of the pose
     * @param[in] y [const double&] The y-wise coordinate of the pose
     * @param[in] theta [const double&] The orientation of the pose
     * @param[in,out] t [tf::Transform&] The returned transform
     */
    void createTfFromXYTheta(const double& x, const double& y,
      const double& theta, tf::Transform& t);

   /****************************************************************************
    * @brief Given the amcl pose and a world scan, this function corrects
    * the pose by ICP-ing the world scan and a map scan taken at the amcl pose
    * @param[in] amcl_pose_msg [const geometry_msgs::Pose::Ptr&]
    * The amcl pose
    * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
    * laser scan
    * @return void
    * pose
    */
    void doFSM(const geometry_msgs::Pose::Ptr& amcl_pose_msg,
      const sensor_msgs::LaserScan::Ptr& latest_world_scan,
      sm_result* output, tf::Transform* f2b);

    /***************************************************************************
     * @brief Given the amcl pose and a world scan, this function corrects
     * the pose by ICP-ing the world scan and a map scan taken at the amcl pose
     * @param[in] amcl_pose_msg
     * [const geometry_msgs::Pose::Ptr&] The amcl pose
     * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
     * laser scan
     * @return void
     */
    void doICP(const geometry_msgs::Pose::Ptr& amcl_pose_msg,
      const sensor_msgs::LaserScan::Ptr& latest_world_scan,
      sm_result* output, tf::Transform* f2b);

    /***************************************************************************
     * @brief Extracts the yaw component from the input pose's quaternion.
     * @param[in] pose [const geometry_msgs::Pose&] The input pose
     * @return [double] The pose's yaw
     */
    double extractYawFromPose(const geometry_msgs::Pose& pose);

    /***************************************************************************
     * @brief Calculates the area of the free space of a map
     * @param[in] map [map_t*] Guess what
     * @return [double] Its area
     */
    double freeArea(map_t* map);

    /***************************************************************************
     * @brief Finds the transform between the laser frame and the base frame
     * @param[in] frame_id [const::string&] The laser's frame id
     * @return [bool] True when the transform was found, false otherwise.
     */
    bool getBaseToLaserTf (const std::string& frame_id);

    /***************************************************************************
     * @brief Given the robot's pose in the map frame, this function returns the
     * laser's pose in the map frame.
     * @param[in] robot_pose [const geometry_msgs::Pose&] The robot's pose in the
     * map frame.
     * @return [geometry_msgs::Pose] The pose of the laser in the map frame.
     */
    geometry_msgs::Pose getCurrentLaserPose(
      const geometry_msgs::Pose& robot_pose);

    /***************************************************************************
     * @brief This is where all the magic happens
     * @param[in] pose_msg [const geometry_msgs::Pose::Ptr&]
     * The amcl pose wrt to the map frame
     * @return void
     */
    void handleInputPose(const geometry_msgs::Pose::Ptr& pose_msg,
      sm_result* output, tf::Transform* f2b);

    /***************************************************************************
     * @brief Initializes parameters
     * @return void
     */
    void initParams();

    /***************************************************************************
     * @brief Initialises the ray-casters from the RangeLib library. Since they
     * take as arguments the maximum range of the lidar and the resolution of the
     * map, and these are unknown before being received, this function should be
     * called ONCE, exactly after the first scan and the map are received.
     * @param void
     * @return void
     */
    void initRangeLibRayCasters();

    /***************************************************************************
     * Identify contiguous regions of false measurements
     */
    std::vector<float> interpolateRanges(
      const std::vector<float>& ranges, const float& idd);

    /***************************************************************************
     * @brief Converts a LaserScan laser scan to a LDP structure.
     * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input
     * scan
     * @param[out] ldp [LDP&] The output LDP scan
     * @return void
     */
    void laserScanToLDP(const sensor_msgs::LaserScan::Ptr& scan_msg,
      LDP& ldp);

    /***************************************************************************
     * @brief Converts a LDP structure to a LaserScan laser scan
     * @param[in] ldp [const LDP&] The input LDP laser scan
     * @return [sensor_msgs::LaserScan::Ptr] The output LaserScan laser scan
     */
    sensor_msgs::LaserScan::Ptr ldpTolaserScan(const LDP& ldp);

    /***************************************************************************
     * @brief Builds an occupancy grid message from a map description YAML
     * file and its accompanying image, read once at start up.
     * @param[in] filename [const std::string&] The path to the map
     * description YAML file
     * @return [nav_msgs::OccupancyGrid] The map, in message form
     */
    nav_msgs::OccupancyGrid loadMapFromFile(const std::string& filename);

    /***************************************************************************
     * @brief Stores the map upon receipt. (The map does not change through time)
     * @param[in] map_msg [const nav_msgs::OccupancyGrid] The map
     * @return void
     */
    void mapCallback(const nav_msgs::OccupancyGrid& map);

    /***************************************************************************
     * @brief Publishes the pipeline's latest execution time.
     * @param[in] start [const ros::Time&] The start time of the pipeline's
     * execution
     * @param[in] end [const ros::Time&] The end time of the pipeline's execution
     */
    void measureExecutionTime(const ros::Time& start, const ros::Time& end);

    /***************************************************************************
     * @brief Checks if there are nan's in an input pose.
     * @param[in] pose_msg [const geometry_msgs::Pose::Ptr&]
     * The input pose
     * @return [bool] True if there is at least one nan in the input_pose, false
     * otherwise.
     */
    bool nanInPose(
      const geometry_msgs::Pose::Ptr& pose_msg);

    /***************************************************************************
     * @brief Returns the number of rays that correspond to an angle range
     * based on known values of minimum and maximum angle, and the number of rays
     * that correspond to them.
     * @param[in] angle_min [const double&] The minimum angle
     * @param[in] angle_max [const double&] The maximum angle
     * @param[in] num_rays [const int&] The number of rays that correspond to the
     * interval [angle_min, angle_max]
     * @param[in] new_range [const double&] The angle range over which we seek the
     * number of rays
     * @return [int] The number of rays corresponding to new_range
     */
    int numRaysFromAngleRange(const double& angle_min, const double& angle_max,
      const int& num_rays, const double& new_range);

    /***************************************************************************
     * @brief The amcl cloud pose callback. This is the point of entry
     * @param[in] pose_cloud_msg [const geometry_msgs::PoseArray::Ptr&]
     * The amcl pose wrt to the map frame
     * @return void
     */
    void poseCloudCallback(
      const geometry_msgs::PoseArray::Ptr& pose_cloud_msg);

    /***************************************************************************
     * @brief
     * @return void
     */
    void processPoseCloud();

    /***************************************************************************
     * @brief The champion function of the ICP operation.
     * @param[in] world_scan_ldp [LDP&] The world scan in LDP form.
     * @param[in] map_scan_ldp [LDP&] The map scan in LDP form.
     * @return void
     */
    void processScan(LDP& world_scan_ldp, LDP& map_scan_ldp,
      sm_result* output, tf::Transform* f2b);

    /***************************************************************************
     * @brief Decodes a PGM image file into an 8-bit greyscale pixel buffer.
     * @param[in] filename [const std::string&] The path to the PGM file
     * @param[out] width [unsigned int*] The image's width, in pixels
     * @param[out] height [unsigned int*] The image's height, in pixels
     * @param[out] pixels [std::vector<unsigned char>*] The decoded pixels,
     * one byte per pixel, row-major, top row first
     * @return void
     */
    void readPGM(const std::string& filename, unsigned int* width,
      unsigned int* height, std::vector<unsigned char>* pixels);

    /***************************************************************************
     * For use with FSM
     */
    std::vector<double> retypeScan(const sensor_msgs::LaserScan::Ptr& scan_msg);

    /***************************************************************************
     * @brief The laser scan callback
     * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input
     * scan message
     * @return void
     */
    void scanCallback(const sensor_msgs::LaserScan::Ptr& scan_msg);

    /***************************************************************************
     * @brief Given the robot's pose and a choice to scan over an angle of 2π,
     * this function simulates a range scan that has the physical world
     * substituted for the map.
     * @param[in] robot_pose
     * [const geometry_msgs::Pose::Ptr&] The robot's pose.
     * @param[in] scan_method [const std::string&] Which method to use for
     * scanning the map. Currently supports vanilla (Bresenham's method)
     * and ray_marching.
     * @param[in] do_fill_map_scan [const bool&] A choice to scan over an angle of
     * 2π.
     * @return [sensor_msgs::LaserScan::Ptr] The map scan in LaserScan form.
     */
    sensor_msgs::LaserScan::Ptr scanMap(
      const geometry_msgs::Pose::Ptr& robot_pose,
      const std::string& scan_method,
      const bool& do_fill_map_scan);
    sensor_msgs::LaserScan::Ptr scanMapPanoramic(
      const geometry_msgs::Pose::Ptr& robot_pose,
      const std::string& scan_method);

    /***************************************************************************
     * @brief Takes all pose hypotheses, computes their caer against the real
     * scan and ranks them. Only p% of all poses are then considered for sm2
     */
    std::vector<geometry_msgs::Pose::Ptr> siftThroughCAERPanoramic(
      const std::vector<geometry_msgs::Pose::Ptr>& init_hypotheses);

    /***************************************************************************
     * @brief User calls this service and cbgl functionality is executed
    */
    bool startSignalService(std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);

    /***************************************************************************
     * @brief Pose disperser
     * @return [pf_vector_t] Uniformly random pose set over arg
     */
    static pf_vector_t uniformPoseGenerator(void* arg);

    /***************************************************************************
     * @brief Wraps an angle in the [-π, π] interval
     * @param[in,out] angle [double&] The angle to be expressed in [-π,π]
     * @return void
     */
    void wrapAngle(double& angle);

    /***************************************************************************
     * @brief Wraps a pose's orientation in the [-π,π] interval
     * @param[in,out] pose [geometry_msgs::Pose::Ptr&] The
     * pose whose input will be wrapped in [-π,π]
     * @return void
     */
    void wrapPoseOrientation(geometry_msgs::Pose::Ptr& pose);
};

#endif  /* CBGL_H */
