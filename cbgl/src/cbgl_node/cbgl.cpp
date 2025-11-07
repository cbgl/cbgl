/*
 * cbgl - [IROS'24] Globally localise your 2D LIDAR in a 2D map in no time
 *
 * Copyright (c) 2024 Alexandros PHILOTHEOU
 *
 * Licensed under the MIT License.
 * See LICENSE.MIT for details.
 */
#include <cbgl_node/cbgl.h>

#define DEBUG_EXECUTION_TIMES 0

/*******************************************************************************
 * @brief Constructor
 * @param[in] nh [ros::NodeHandle]
 * @param[in] nh_private [ros::NodeHandle]
 */
CBGL::CBGL(
  ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  received_scan_(false),
  received_map_(false),
  received_pose_cloud_(false),
  received_start_signal_(false),
  running_(false),
  do_icp_(true),
  do_fsm_(false),
  omap_(ranges::OMap(1,1)),
  rm_(ranges::RayMarching(omap_, 1)),
  cddt_(ranges::CDDTCast(omap_, 1, 1)),
  br_(ranges::BresenhamsLine(omap_, 1)),
  pf_hyp_(NULL),
  map_hyp_(NULL),
  nrays_(0),
  sgn_(0),
  angle_min_(0.0),
  angle_inc_(0.0),
  map_res_(0.0),
  map_hgt_(0.0),
  top_k_caers_(10)
{
  /* initialise needed parameters */
  initParams();

  /* publishers */

  /* The result */
  global_pose_publisher_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      output_pose_topic_, 1);

  /* Publisher of execution time */
  execution_time_publisher_ =
    nh_.advertise<std_msgs::Duration>(
      ros::this_node::getName() + "/execution_time", 10);

  /* Publisher of the id of the best hypothesis */
  best_particle_publisher_ =
    nh_.advertise<std_msgs::UInt64>(
      ros::this_node::getName() + "/best_particle", 10);

  /* Self-described; for debugging/analysis purposes */
  all_hypotheses_publisher_ =
    nh_.advertise<geometry_msgs::PoseArray>(
      ros::this_node::getName() + "/particlecloud_all",
        std::numeric_limits<unsigned int>::max());
  all_hypotheses_caer_publisher_ =
    nh_.advertise<geometry_msgs::PoseArray>(
      ros::this_node::getName() + "/particlecloud_all_caers",
        std::numeric_limits<unsigned int>::max());
  top_caer_hypotheses_publisher_ =
    nh_.advertise<geometry_msgs::PoseArray>(
      ros::this_node::getName() + "/particlecloud_top_k_caers", top_k_caers_);

  /* The world scan is published through this publisher */
  world_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/world_scan", 1);

  /* The map scan is published through this publisher */
  map_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/map_scan", 1);

  /* subscribers */

  /* This is the map */
  map_subscriber_ = nh_.subscribe(map_topic_, 1,
    &CBGL::mapCallback, this);

  /* This is the scan subscriber */
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1,
    &CBGL::scanCallback, this);

  /* Call me for a fun time */
  global_localisation_service_ =
    nh_.advertiseService(
      ros::this_node::getName() + "/" + global_localisation_service_name_,
      &CBGL::startSignalService, this);

  ROS_INFO("[CBGL] BORN READY");
}

/*******************************************************************************
 * @brief Destructor
 * @params void
 */
CBGL::~CBGL()
{
  ROS_INFO("[CBGL] Destroying CBGL");
}

/*****************************************************************************
 * @brief Broadcasts the estimated pose as the odom<--map transform
 */
void CBGL::broadcast_global_pose_tf(
  const geometry_msgs::Pose& pose)
{
  /*
   * (base --> map) = (base --> odom) x (odom --> map)
   * therefore
   * (odom --> map) = (odom --> base) x (base --> map)
   */

  /* base <-- odom */
  ros::Time t = ros::Time::now();
  tf::StampedTransform odom_to_base_tf;
  try
  {
    tf_listener_.waitForTransform(base_frame_id_, odom_frame_id_,
      t, ros::Duration(1.0));
    tf_listener_.lookupTransform (base_frame_id_, odom_frame_id_,
      t, odom_to_base_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("[CBGL] Could not get transform from %s to %s. Error: %s",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
  }

  /* map <-- base */
  tf::Transform base_to_map_tf;
  createTfFromXYTheta(pose.position.x, pose.position.y, extractYawFromPose(pose),
    base_to_map_tf);

  /* (odom --> map) */
  tf::Transform odom_to_map_tf = base_to_map_tf * odom_to_base_tf;
  tf_broadcaster_.sendTransform(
    tf::StampedTransform(odom_to_map_tf, ros::Time::now(),
      fixed_frame_id_, odom_frame_id_));
}

/*******************************************************************************
 * @brief Calculation of the cumulative absolute error per ray metric
*/
double CBGL::caer(
  const sensor_msgs::LaserScan::Ptr& sr,
  const sensor_msgs::LaserScan::Ptr& sv)
{
  double c = 0;
  for (unsigned int i = 0; i < nrays_; i++)
  {
    if (sr->ranges[i] > 0.0 && sv->ranges[i] > 0.0)
      c += fabs(sr->ranges[i] - sv->ranges[i]);
  }

  return c;
}

/*******************************************************************************
*/
double CBGL::caer(
  const std::vector<float>& sr,
  const std::vector<float>& sv)
{
  double c = 0;
  for (unsigned int i = 0; i < nrays_; i++)
  {
    if (sr[i] > 0.0 && sv[i] > 0.0)
      c += fabs(sr[i] - sv[i]);
  }

  return c;
}

/*******************************************************************************
 * Used by FSM
 */
  void
CBGL::cacheFFTW3Plans(const unsigned int& sz)
{
  /* Create forward  plan */
  double* r2r_in;
  double* r2r_out;

  r2r_in = (double*) fftw_malloc(sz * sizeof(double));
  r2r_out = (double*) fftw_malloc(sz * sizeof(double));

  r2rp_ = fftw_plan_r2r_1d(sz, r2r_in, r2r_out, FFTW_R2HC, FFTW_MEASURE);

  /* Create backward plan */
  fftw_complex* c2r_in;
  double* c2r_out;

  c2r_in = (fftw_complex*) fftw_malloc(sz * sizeof(fftw_complex));
  c2r_out = (double*) fftw_malloc(sz * sizeof(double));

  c2rp_ = fftw_plan_dft_c2r_1d(sz, c2r_in, c2r_out, FFTW_MEASURE);
}

/*******************************************************************************
 * @brief Convert an OccupancyGrid map message into the internal
 * representation. This allocates a map_t and returns it. Stolen from amcl.
 */
  map_t*
CBGL::convertMap(const nav_msgs::OccupancyGrid& map_msg)
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  /* Convert to player format */
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

/*******************************************************************************
* @brief Convert an OccupancyGrid map message into a unsigned char* array.
* If this array is immediately inputted to lodepng::encode the latter returns 0.
* @param[in] map_msg [const nav_msgs::OccupancyGrid&] The map message to be
* converted.
* @param[out] converted [unsigned char*] The output array
* @ return void
*/
void CBGL::convertMapToPNG(
  const nav_msgs::OccupancyGrid& map_msg,
  unsigned char* converted)
{
  unsigned int width = map_msg.info.width;
  unsigned int height = map_msg.info.height;

  unsigned int ic = 0;
  for (size_t i = 0; i < map_msg.data.size(); i++)
  {
    /* black */
    if (map_msg.data[i] == 100)
    {
      converted[ic+0] = 0;
      converted[ic+1] = 0;
      converted[ic+2] = 0;
      converted[ic+3] = 255;
    }

    /* white */
    if (map_msg.data[i] == 0)
    {
      converted[ic+0] = 254;
      converted[ic+1] = 254;
      converted[ic+2] = 254;
      converted[ic+3] = 255;
    }

    /* gray */
    if (map_msg.data[i] == -1)
    {
      converted[ic+0] = 205;
      converted[ic+1] = 205;
      converted[ic+2] = 205;
      converted[ic+3] = 255;
    }
    ic += 4;
  }

  /* Flip y axis */
  unsigned row_size = width * 4;  /* 4 bytes per pixel (RGBA) */
  std::vector<uint8_t> temp_row(row_size);
  for (unsigned y = 0; y < height / 2; ++y)
  {
    uint8_t* top_row = &converted[y * row_size];
    uint8_t* bottom_row = &converted[(height-1- y) * row_size];
    std::copy(bottom_row, bottom_row + row_size, temp_row.begin());
    std::copy(top_row, top_row + row_size, bottom_row);
    std::copy(temp_row.begin(), temp_row.end(), top_row);
  }
}

/*******************************************************************************
* @brief Convert an OccupancyGrid map message into a unsigned char* array,
* encode it into PNG, and store the image to file.
* @param[in] map_msg [const nav_msgs::OccupancyGrid&] The map message to be
* converted to png
* @param[in] filename [const std::string&] The name of the png file to be stored
* @ return void
*/
void CBGL::convertMapToPNG(
  const nav_msgs::OccupancyGrid& map_msg,
  const std::string& filename)
{
  unsigned char converted[4*map_msg.data.size()];
  convertMapToPNG(map_msg, converted);

  /* Store png to disc */
  unsigned int width = map_msg.info.width;
  unsigned int height = map_msg.info.height;
  std::vector<uint8_t> png;
  unsigned error = lodepng::encode(png, converted, width, height, LCT_RGBA, 8);
  if (error)
    throw std::runtime_error("Error encoding PNG: " +
      std::string(lodepng_error_text(error)));

  error = lodepng::save_file(png, filename);
}

/*******************************************************************************
 * @brief This function uses the csm_icp result for correcting the amcl pose.
 * Given the icp output, this function express this correction in the
 * map frame. The result is the icp-corrected pose in the map frame.
 * @param[in,out] icp_corrected_pose
 * [geometry_msgs::Pose::Ptr&] The icp-corrected amcl pose
 * @return void
 */
  void
CBGL::correctICPPose(
  geometry_msgs::Pose::Ptr& icp_corrected_pose,
  const tf::Transform& f2b)
{
  if (!nanInPose(icp_corrected_pose))
  {
    tf::Transform map_to_base_tf;
    tf::poseMsgToTF(*icp_corrected_pose, map_to_base_tf);

    /* Now express the icp-corrected amcl pose in terms of the map frame ... */
    tf::Transform icp_corrected_pose_tf = map_to_base_tf * f2b;

    /* ... and convert the transform into a message */
    tf::poseTFToMsg(icp_corrected_pose_tf, *icp_corrected_pose);

    /* Make sure the orientation is in the [-π,π] interval */
    wrapPoseOrientation(icp_corrected_pose);
  }
  else
  {
    /* icp has failed: the icp-corrected pose falls back to the amcl pose */
    ROS_ERROR("[CBGL] ICP has failed; falling back to amcl pose");
  }
}

/*******************************************************************************
 * @brief Creates a transform from a 2D pose (x,y,theta)
 * @param[in] x [const double&] The x-wise coordinate of the pose
 * @param[in] y [const double&] The y-wise coordinate of the pose
 * @param[in] theta [const double&] The orientation of the pose
 * @param[in,out] t [tf::Transform&] The returned transform
 */
  void
CBGL::createTfFromXYTheta(
  const double& x,
  const double& y,
  const double& theta,
  tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  q.normalize();
  t.setRotation(q);
}

/*******************************************************************************
 * @brief Given the amcl pose and a world scan, this function corrects
 * the pose by ICP-ing the world scan and a map scan taken at the amcl pose
 * @param[in] amcl_pose_msg [const geometry_msgs::Pose::Ptr&]
 * The amcl pose
 * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
 * laser scan
 * @return void
 * pose
 */
  void
CBGL::doFSM(
  const geometry_msgs::Pose::Ptr& amcl_pose_msg,
  const sensor_msgs::LaserScan::Ptr& latest_world_scan,
  sm_result* output, tf::Transform* f2b)
{
  /* Measure execution time */
#if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_icp = ros::Time::now();
#endif

  geometry_msgs::Pose::Ptr icp_corrected_pose =
    boost::make_shared<geometry_msgs::Pose>(*amcl_pose_msg);

  /* Obtain the map scan ... */
  sensor_msgs::LaserScan::Ptr map_scan = scanMap(icp_corrected_pose,
    map_scan_method_, false);

  std::tuple<double,double,double> origin;
  std::get<0>(origin) = 0.0;
  std::get<1>(origin) = 0.0;
  std::get<2>(origin) = 0.0;


  std::vector<double> sr = retypeScan(latest_world_scan);
  std::vector<double> sv = retypeScan(map_scan);

  /* The reference scan in 2d points */
  std::vector< std::pair<double,double> > vp;
  FSM::Utils::scan2points(sv, origin, &vp);

  FSM::output_params op;
  std::tuple<double,double,double> diff;

  /*
   * Do your magic thing
   */
  FSM::Match::fmtdbh(sr, origin, vp, r2rp_, c2rp_, ip_, &op, &diff);

  double dx = std::get<0>(diff);
  double dy = std::get<1>(diff);
  double dt = std::get<2>(diff);

  /* the correction of the laser's position, in the laser frame */
  tf::Transform corr_ch_l;
  createTfFromXYTheta(dx,dy,dt, corr_ch_l);

  /* the correction of the base's position, in the base frame */
  f2b_ = base_to_laser_ * corr_ch_l * laser_to_base_;
  *f2b = f2b_;
  output->x[0] = dx;
  output->x[1] = dy;
  output->x[2] = dt;
  output->valid = 1;

  /* Delete the map scan ptr and the LDPs */
  map_scan.reset();

#if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("doFSM() took %.2f ms",
    (ros::Time::now() - start_icp).toSec() * 1000);
#endif
}

/*******************************************************************************
 * @brief Given the amcl pose and a world scan, this function corrects
 * the pose by ICP-ing the world scan and a map scan taken at the amcl pose
 * @param[in] amcl_pose_msg [const geometry_msgs::Pose::Ptr&]
 * The amcl pose
 * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
 * laser scan
 * @return void
 * pose
 */
  void
CBGL::doICP(
  const geometry_msgs::Pose::Ptr& amcl_pose_msg,
  const sensor_msgs::LaserScan::Ptr& latest_world_scan,
  sm_result* output, tf::Transform* f2b)
{
  /* Measure execution time */
#if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_icp = ros::Time::now();
#endif

  geometry_msgs::Pose::Ptr icp_corrected_pose =
    boost::make_shared<geometry_msgs::Pose>(*amcl_pose_msg);

  /* Convert the latest world scan into LDP form */
  LDP world_scan_ldp;
  laserScanToLDP(latest_world_scan, world_scan_ldp);

  /* Obtain the map scan ... */
  sensor_msgs::LaserScan::Ptr map_scan = scanMap(icp_corrected_pose,
    map_scan_method_, false);

  /* .. and turn into a suitable structure (LDP) for processing */
  LDP map_scan_ldp;
  laserScanToLDP(map_scan, map_scan_ldp);

  /* Do the ICP thing */
  processScan(world_scan_ldp, map_scan_ldp, output, f2b);

  /* Delete the map scan ptr and the LDPs */
  map_scan.reset();
  ld_free(world_scan_ldp);
  ld_free(map_scan_ldp);

#if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("doICP() took %.2f ms",
    (ros::Time::now() - start_icp).toSec() * 1000);
#endif
}

/*******************************************************************************
 * @brief Extracts the yaw component from the input pose's quaternion.
 * @param[in] pose [const geometry_msgs::Pose&] The input pose
 * @return [double] The pose's yaw
 */
  double
CBGL::extractYawFromPose(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);

  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  wrapAngle(yaw);

  return yaw;
}

/*******************************************************************************
 * @brief Calculates the area of the free space of a map
 * @param[in] map [map_t*] Guess what
 * @return [double] Its area
 */
  double
CBGL::freeArea(map_t* map)
{
  double area = 0.0;
  for (unsigned int c = 0; c < map->size_x * map->size_y; c++)
  {
    map_cell_t* cell = map->cells + c;

    if (cell->occ_state == -1)
      area -= cell->occ_state;
  }

  area *= map->scale * map->scale;

  return area;
}

/*******************************************************************************
 * @brief Finds the transform between the laser frame and the base frame
 * @param[in] frame_id [const::string&] The laser's frame id
 * @return [bool] True when the transform was found, false otherwise.
 */
  bool
CBGL::getBaseToLaserTf(const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(base_frame_id_, frame_id, t, ros::Duration(1.0));

    /*
     * The direction of the transform returned will be from the base_frame_id_
     * to the frame_id. Which if applied to data, will transform data in
     * the frame_id into the base_frame_id_.
     */
    tf_listener_.lookupTransform(base_frame_id_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("[CBGL] Could not get initial transform from");
    ROS_WARN("base frame to %s: %s", frame_id.c_str(), ex.what());

    return false;
  }

  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

/*******************************************************************************
 * @brief Given the robot's pose in the map frame, this function returns the
 * laser's pose in the map frame.
 * @param[in] robot_pose [const geometry_msgs::Pose&] The robot's pose in the
 * map frame.
 * @return [geometry_msgs::Pose] The pose of the laser in the map frame.
 */
  geometry_msgs::Pose
CBGL::getCurrentLaserPose(const geometry_msgs::Pose& robot_pose)
{
  /* Transform the robot_pose to a transform */
  tf::Transform map_to_base_tf;
  tf::poseMsgToTF(robot_pose, map_to_base_tf);

  /* Get the laser's pose in the map frame (as a transform) */
  tf::Transform map_to_laser_tf = map_to_base_tf * base_to_laser_;

  /* Convert the transform into a message */
  geometry_msgs::Pose laser_pose;
  tf::poseTFToMsg(map_to_laser_tf, laser_pose);

  /* Return the laser's pose */
  return laser_pose;
}

/*******************************************************************************
 * @brief This is where all the magic happens
 * @param[in] pose_msg [const geometry_msgs::Pose::Ptr&]
 * The amcl pose wrt to the map frame
 * @return void
 */
  void
CBGL::handleInputPose(const geometry_msgs::Pose::Ptr& pose_msg,
  sm_result* output, tf::Transform* f2b)
{
  /* Return if the pose received from amcl contains nan's */
  if (nanInPose(pose_msg))
  {
    ROS_ERROR("[CBGL] pose contains nan's; aborting ...");
    return;
  }

  /* Bail if already running */
  if (running_)
  {
    ROS_ERROR("[CBGL] Already running; aborting ...");
    return;
  }

  /* Set status to running to block further execution */
  running_ = true;

  sensor_msgs::LaserScan::Ptr world_scan =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  geometry_msgs::Pose::Ptr global_pose;

  if (do_icp_)
    doICP(pose_msg, world_scan, output, f2b);
  if (do_fsm_)
    doFSM(pose_msg, world_scan, output, f2b);

  /* GUARD stand-down */
  running_ = false;
}

/*******************************************************************************
 * @brief Initializes parameters
 * @return void
 */
  void
CBGL::initParams()
{
  if (!nh_private_.getParam("base_frame_id", base_frame_id_))
  {
    base_frame_id_ = "base_footprint";
    ROS_ERROR("[CBGL] base_frame_id");
  }
  if (!nh_private_.getParam("fixed_frame_id", fixed_frame_id_))
  {
    fixed_frame_id_ = "map";
    ROS_ERROR("[CBGL] fixed_frame_id");
  }
  if (!nh_private_.getParam("odom_frame_id", odom_frame_id_))
  {
    odom_frame_id_ = "odom";
    ROS_ERROR("[CBGL] odom_frame_id");
  }
  if (!nh_private_.getParam("tf_broadcast", tf_broadcast_))
  {
    tf_broadcast_ = false;
    ROS_ERROR("[CBGL] tf_broadcast");
  }
  if (!nh_private_.getParam("scan_topic", scan_topic_))
  {
    scan_topic_ = "/front_laser/scan";
    ROS_ERROR("[CBGL] scan_topic");
  }
  if (!nh_private_.getParam("map_topic", map_topic_))
  {
    map_topic_ = "/map";
    ROS_ERROR("[CBGL] map_topic");
  }
  if (!nh_private_.getParam("output_pose_topic", output_pose_topic_))
  {
    output_pose_topic_ = "/initialpose";
    ROS_ERROR("[CBGL] output_pose_topic");
  }
  if (!nh_private_.getParam("dl", dl_))
  {
    dl_ = 40;
    ROS_ERROR("[CBGL] param dl");
  }
  if (!nh_private_.getParam("da", da_))
  {
    da_ = 32;
    ROS_ERROR("[CBGL] param da");
  }
  if (!nh_private_.getParam("top_k_caers", top_k_caers_))
  {
    top_k_caers_ = 10;
    ROS_ERROR("[CBGL] top_k_caers");
  }
  if (!nh_private_.getParam("map_scan_method", map_scan_method_))
  {
    map_scan_method_ = "vanilla";
    ROS_ERROR("[CBGL] map_scan_method");
  }
  if (!nh_private_.getParam("publish_pose_sets", publish_pose_sets_))
  {
    publish_pose_sets_ = false;
    ROS_ERROR("[CBGL] publish_pose_sets");
  }
  if (!nh_private_.getParam("global_localisation_service_name",
      global_localisation_service_name_))
  {
    global_localisation_service_name_ = "/global_localization";
    ROS_ERROR("[CBGL] global_localisation_service_name");
  }
  if (!nh_private_.getParam("laser_z_orientation", laser_z_orientation_))
  {
    laser_z_orientation_ = "upwards";
    ROS_ERROR("[CBGL] laser_z_orientation");
  }
  if (!nh_private_.getParam("do_undersample_scan", do_undersample_scan_))
  {
    do_undersample_scan_ = false;
    ROS_ERROR("[CBGL] do_undersample_scan");
  }
  if (!nh_private_.getParam("undersample_rate", undersample_rate_))
  {
    undersample_rate_ = 1;
    ROS_ERROR("[CBGL] undersample_rate");
  }
  /*
   * The angular progression of scanning depends on how the laser is mounted
   * on the robot:
   * a. On the turtlebot the laser faces upwards;
   * b. On the rb1 the laser faces downwards
   */
  if (laser_z_orientation_.compare("upwards") == 0)
    sgn_ = -1;
  else if (laser_z_orientation_.compare("downwards") == 0)
    sgn_ = 1;
  else
  {
    ROS_ERROR("[CBGL] Please provide a valid value");
    ROS_ERROR("       for param laser_z_orientation");
  }


  /*
   * **** CSM parameters - comments copied from algos.h (by Andrea Censi)
   */

  /* Maximum angular displacement between scans */
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
  {
    input_.max_angular_correction_deg = 45.0;
    ROS_ERROR("[CBGL] max_angular_correction_deg");
  }

  /* Maximum translation between scans (m) */
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
  {
    input_.max_linear_correction = 0.50;
    ROS_ERROR("[CBGL] max_linear_correction");
  }

  /* Maximum ICP cycle iterations */
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
  {
    input_.max_iterations = 10;
    ROS_ERROR("[CBGL] max_iterations");
  }

  /* A threshold for stopping (m) */
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
  {
    input_.epsilon_xy = 0.000001;
    ROS_ERROR("[CBGL] epsilon_xy");
  }

  /* A threshold for stopping (rad) */
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
  {
    input_.epsilon_theta = 0.000001;
    ROS_ERROR("[CBGL] epsilon_theta");
  }

  /* Maximum distance for a correspondence to be valid */
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
  {
    input_.max_correspondence_dist = 0.3;
    ROS_ERROR("[CBGL] max_correspondence_dist");
  }

  /* Noise in the scan (m) */
  if (!nh_private_.getParam ("sigma", input_.sigma))
  {
    input_.sigma = 0.010;
    ROS_ERROR("[CBGL] sigma");
  }

  /* Use smart tricks for finding correspondences. */
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
  {
    input_.use_corr_tricks = 1;
    ROS_ERROR("[CBGL] use_corr_tricks");
  }

  /* Restart: Restart if error is over threshold */
  if (!nh_private_.getParam ("restart", input_.restart))
  {
    input_.restart = 0;
    ROS_ERROR("[CBGL] restart");
  }

  /* Restart: Threshold for restarting */
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
  {
    input_.restart_threshold_mean_error = 0.01;
    ROS_ERROR("[CBGL] restart_threshold_mean_error");
  }

  /* Restart: displacement for restarting. (m) */
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
  {
    input_.restart_dt = 1.0;
    ROS_ERROR("[CBGL] restart_dt");
  }

  /* Restart: displacement for restarting. (rad) */
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
  {
    input_.restart_dtheta = 0.1;
    ROS_ERROR("[CBGL] restart_dtheta");
  }

  /* Max distance for staying in the same clustering */
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
  {
    input_.clustering_threshold = 0.25;
    ROS_ERROR("[CBGL] clustering_threshold");
  }

  /* Number of neighbour rays used to estimate the orientation */
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
  {
    input_.orientation_neighbourhood = 20;
    ROS_ERROR("[CBGL] orientation_neighbourhood");
  }

  /* If 0, it's vanilla ICP */
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
  {
    input_.use_point_to_line_distance = 1;
    ROS_ERROR("[CBGL] use_point_to_line_distance");
  }

  /* Discard correspondences based on the angles */
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
  {
    input_.do_alpha_test = 0;
    ROS_ERROR("[CBGL] do_alpha_test");
  }

  /* Discard correspondences based on the angles - threshold angle, in degrees */
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
  {
    input_.do_alpha_test_thresholdDeg = 20.0;
    ROS_ERROR("[CBGL] do_alpha_test_thresholdDeg");
  }

  /*
   * Percentage of correspondences to consider: if 0.9,
   * always discard the top 10% of correspondences with more error
   */
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
  {
    input_.outliers_maxPerc = 0.90;
    ROS_ERROR("[CBGL] outliers_maxPerc");
  }

  /*
   * Parameters describing a simple adaptive algorithm for discarding.
   * 1) Order the errors.
   * 2) Choose the percentile according to outliers_adaptive_order.
   * (if it is 0.7, get the 70% percentile)
   * 3) Define an adaptive threshold multiplying outliers_adaptive_mult
   * with the value of the error at the chosen percentile.
   * 4) Discard correspondences over the threshold.
   * This is useful to be conservative; yet remove the biggest errors.
   */
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
  {
    input_.outliers_adaptive_order = 0.7;
    ROS_ERROR("[CBGL] outliers_adaptive_order");
  }

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
  {
    input_.outliers_adaptive_mult = 2.0;
    ROS_ERROR("[CBGL] outliers_adaptive_mult");
  }

  /*
   * If you already have a guess of the solution, you can compute the polar angle
   * of the points of one scan in the new position. If the polar angle is not a monotone
   * function of the readings index, it means that the surface is not visible in the
   * next position. If it is not visible, then we don't use it for matching.
   */
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
  {
    input_.do_visibility_test = 0;
    ROS_ERROR("[CBGL] do_visibility_test");
  }

  /* no two points in laser_sens can have the same corr. */
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
  {
    input_.outliers_remove_doubles = 1;
    ROS_ERROR("[CBGL] outliers_remove_doubles");
  }

  /* If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov */
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
  {
    input_.do_compute_covariance = 0;
    ROS_ERROR("[CBGL] do_compute_covariance");
  }

  /* Checks that find_correspondences_tricks gives the right answer */
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
  {
    input_.debug_verify_tricks = 0;
    ROS_ERROR("[CBGL] debug_verify_tricks");
  }

  /*
   * If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
   * incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
   */
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
  {
    input_.use_ml_weights = 0;
    ROS_ERROR("[CBGL] use_ml_weights");
  }

  /*
   * If 1, the field 'readings_sigma' in the second scan is used to weight the
   * correspondence by 1/sigma^2
   */
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
  {
    input_.use_sigma_weights = 0;
    ROS_ERROR("[CBGL] use_sigma_weights");
  }

  if (!nh_private_.getParam ("gpm_theta_bin_size_deg", input_.gpm_theta_bin_size_deg))
  {
    input_.gpm_theta_bin_size_deg = 5;
    ROS_ERROR("[CBGL] gpm_theta_bin_size_deg");
  }
  if (!nh_private_.getParam ("gpm_extend_range_deg", input_.gpm_extend_range_deg))
  {
    input_.gpm_extend_range_deg = 15;
    ROS_ERROR("[CBGL] gpm_extend_range_deg");
  }
  if (!nh_private_.getParam ("gpm_interval", input_.gpm_interval))
  {
    input_.gpm_interval = 1;
    ROS_ERROR("[CBGL] gpm_interval");
  }

  if (!nh_private_.getParam ("initial_cov_xx", initial_cov_xx_))
    initial_cov_xx_ = 0.05;
  if (!nh_private_.getParam ("initial_cov_yy", initial_cov_yy_))
    initial_cov_yy_ = 0.05;
  if (!nh_private_.getParam ("initial_cov_aa", initial_cov_aa_))
    initial_cov_aa_ = 0.06854;

  if (!nh_private_.getParam("do_icp", do_icp_))
  {
    do_icp_ = true;
    ROS_ERROR("[CBGL] do_icp");
  }
  if (!nh_private_.getParam("do_fsm", do_fsm_))
  {
    do_fsm_ = false;
    ROS_ERROR("[CBGL] do_fsm");
  }

  /* FSM Params */
  int int_param;
  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("num_iterations", int_param))
  {
    ROS_WARN("[CBGL] no num_iterations param found; resorting to defaults");
    ip_.num_iterations = 2;
  }
  else
    ip_.num_iterations = static_cast<unsigned int>(int_param);

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("xy_bound", ip_.xy_bound))
  {
    ROS_WARN("[CBGL] no xy_bound param found; resorting to defaults");
    ip_.xy_bound = 0.2;
  }

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("t_bound", ip_.t_bound))
  {
    ROS_WARN("[CBGL] no t_bound param found; resorting to defaults");
    ip_.t_bound = M_PI/4;
  }

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("max_counter", int_param))
  {
    ROS_WARN("CBGL] no max_counter param found; resorting to defaults");
    ip_.max_counter = 200;
  }
  else
    ip_.max_counter = static_cast<unsigned int>(int_param);

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("min_magnification_size", int_param))
  {
    ROS_WARN("CBGL] no min_magnification_size param found; resorting to defaults");
    ip_.min_magnification_size = 0;
  }
  else
    ip_.min_magnification_size = static_cast<unsigned int>(int_param);

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("max_magnification_size", int_param))
  {
    ROS_WARN("CBGL] no max_magnification_size param found; resorting to defaults");
    ip_.max_magnification_size = 3;
  }
  else
    ip_.max_magnification_size = static_cast<unsigned int>(int_param);

  /* ------------------------------------------------------------------------ */
  if (!nh_private_.getParam ("max_recoveries", int_param))
  {
    ROS_WARN("[CBGL] no max_recoveries param found; resorting to defaults");
    ip_.max_recoveries = 10;
  }
  else
    ip_.max_recoveries = static_cast<unsigned int>(int_param);

  /* state variables */
  f2b_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  /* Initialize output_ vectors as Null for error-checking */
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  assert(ip_.num_iterations > 0);
  assert(ip_.xy_bound >= 0.0);
  assert(ip_.t_bound >= 0.0);
  assert(ip_.max_counter > 0);
  /* assert(ip_.min_magnification_size >= 0); */
  assert(ip_.max_magnification_size >= ip_.min_magnification_size);
  /* assert(ip_.max_recoveries >= 0); */
  assert(undersample_rate_ >= 0);
}

/*******************************************************************************
 * @brief Initialises the ray-casters from the RangeLib library. Since they
 * take as arguments the maximum range of the lidar and the resolution of the
 * map, and these are unknown before being received, this function should be
 * called ONCE, exactly after the first scan and the map are received.
 * @param void
 * @return void
 */
void CBGL::initRangeLibRayCasters()
{
  /* Max laser range in pixels */
  float max_range_rc = latest_world_scan_->range_max / map_res_;

  /* Init ray-casters */
  if (map_scan_method_.compare("bresenham") == 0)
    br_ = ranges::BresenhamsLine(omap_, max_range_rc);

  if (map_scan_method_.compare("ray_marching") == 0)
    rm_ = ranges::RayMarching(omap_, max_range_rc);

  if (map_scan_method_.compare("cddt") == 0)
    cddt_ = ranges::CDDTCast(omap_, max_range_rc, 720);
}

/*******************************************************************************
 * Identify contiguous regions of false measurements
 */
std::vector<float> CBGL::interpolateRanges(
  const std::vector<float>& ranges, const float& idd)
{
  std::vector< std::vector <int> > regions;
  int i = 0;
  int j;
  while(i < nrays_)
  {
    if (ranges[i] <= idd)
    {
      int region_begin = i;
      int region_end;
      bool broke = false;
      for (j = region_begin+1; j < nrays_; j++)
      {
        if (ranges[j] <= idd)
          continue;
        if (ranges[j] > idd)
        {
          region_end = j-1;
          {
            broke = true;
            break;
          }
        }
      }

      if (j == nrays_ && !broke)
        region_end = j-1;

      std::vector<int> region;
      region.push_back(region_begin);
      region.push_back(region_end);
      regions.push_back(region);

      i = j+1;
    }
    else
      i++;
  }

  /* Inflate to consecutive indices */
  for (unsigned int i = 0; i < regions.size(); i++)
  {
    int begin = regions[i][0];
    int end = regions[i][regions[i].size()-1];

    regions[i].clear();

    for (unsigned int j = begin; j <= end; j++)
      regions[i].push_back(j);
  }

  /* Is the first index 0 and the last equal to the size-1? */
  int num_regions = regions.size();
  int numel_last_region = regions[num_regions-1].size();

  if (regions[0][0] == 0 &&
    regions[num_regions-1][numel_last_region-1] == ranges.size()-1)
  {
    for (int i = 0; i < regions[0].size(); i++)
      regions[num_regions-1].push_back(regions[0][i]);

    regions.erase(regions.begin(), regions.begin()+1);
  }

  /*
   * for (int i = 0; i < regions.size(); i++)
   * {
   * printf("region %d\n", i);
   * for (int j = 0; j < regions[i].size(); j++)
   * printf("%d,", regions[i][j]);
   * printf("\n");
   * }
   */

  std::vector<float> ranges_interp = ranges;

  for (unsigned int i = 0; i < regions.size(); i++)
  {
    int interp_begin = regions[i][0]-1;
    int interp_end = regions[i][regions[i].size()-1]+1;

    if (interp_begin < 0)
      interp_begin = ranges.size()-1;

    if (interp_end >= ranges.size())
      interp_end = 0;

    float range_a = ranges[interp_begin];
    float range_b = ranges[interp_end];
    float interp = (range_a + range_b)/2;

    for (unsigned int j = 0; j < regions[i].size(); j++)
      ranges_interp[regions[i][j]]= interp;
  }

  /*
   * for (int j = 0; j < ranges.size(); j++)
   * printf("%f\n", ranges_interp[j]);
   */

  return ranges_interp;
}

/*******************************************************************************
 * @brief Converts a LaserScan laser scan to a LDP structure.
 * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input scan
 * @param[out] ldp [LDP&] The output LDP scan
 * @return void
 */
  void
CBGL::laserScanToLDP(
  const sensor_msgs::LaserScan::Ptr& scan_msg,
  LDP& ldp)
{
  unsigned int n = nrays_ ;
  ldp = ld_alloc_new(n);

  float min_range = scan_msg->range_min;
  float max_range = scan_msg->range_max;
  float angle_min = scan_msg->angle_min;
  float angle_inc = scan_msg->angle_increment;

  for (unsigned int i = 0; i < n; i++)
  {
    double r = scan_msg->ranges[i];

    if (std::isfinite(r))
    {
      if (r >= min_range && r <= max_range)
      {
        ldp->readings[i] = r;
        ldp->valid[i] = 1;
      }
      else if (r < min_range)
      {
        ldp->readings[i] = min_range;
        ldp->valid[i] = 0;
      }
      else if (r > max_range)
      {
        ldp->readings[i] = max_range;
        ldp->valid[i] = 0;
      }
    }
    else
    {
      ldp->readings[i] = max_range;  /* for invalid range */
      ldp->valid[i] = 0;
    }

    if (r == 0.0)
    {
      ldp->readings[i] = 0.0;
      ldp->valid[i] = 0;
    }

    ldp->cluster[i] = -1;
    ldp->theta[i] = angle_min + i * angle_inc;
  }

  ldp->min_theta = angle_min;
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

/*******************************************************************************
 * @brief Converts a LDP structure to a LaserScan laser scan
 * @param[in] ldp [const LDP&] The input LDP laser scan
 * @return [sensor_msgs::LaserScan::Ptr] The output LaserScan laser scan
 */
  sensor_msgs::LaserScan::Ptr
CBGL::ldpTolaserScan(const LDP& ldp)
{
  sensor_msgs::LaserScan::Ptr laser_scan =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  laser_scan->angle_min = ldp->min_theta;
  laser_scan->angle_max = ldp->max_theta;

  unsigned int n = ldp->nrays;

  laser_scan->angle_increment =
    (laser_scan->angle_max - laser_scan->angle_min) / n;

  laser_scan->ranges.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    if (ldp->valid[i] == 1)
      laser_scan->ranges[i] = ldp->readings[i];
    else
      laser_scan->ranges[i] = 0.0;
  }

  return laser_scan;
}

/*******************************************************************************
 * @brief Stores the map upon receipt. (The map does not change through time)
 * @param[in] map_msg [const nav_msgs::OccupancyGrid] The map
 * @return void
 */
  void
CBGL::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
  /* Map used in dispersal of hypotheses */
  map_hyp_ = convertMap(map_msg);

  /* Find area */
  double area = freeArea(map_hyp_);

  /* TODO(li9i) make these depend on the map's area */
  unsigned int min_particles_ = dl_ * area;
  unsigned int max_particles_ = dl_ * area;

  /* The set of hypotheses */
  pf_hyp_ = pf_alloc(min_particles_, max_particles_, 0.001, 0.1,
    (pf_init_model_fn_t)CBGL::uniformPoseGenerator, (void *)map_hyp_);
  pf_hyp_->pop_err = 0.01;
  pf_hyp_->pop_z = 0.99;

  pf_init(pf_hyp_, pf_vector_zero(), pf_matrix_zero());

  map_ = map_msg;
  map_res_ = map_.info.resolution;
  map_hgt_ = map_.info.height;

  /* Re-set the map png file */
  unsigned char converted[4*map_msg.data.size()];
  convertMapToPNG(map_msg, converted);
  omap_ = ranges::OMap(converted);

  received_map_ = true;
  if (received_scan_ && received_pose_cloud_ && received_start_signal_)
    processPoseCloud();
}

/*******************************************************************************
 * @brief Publishes the pipeline's latest execution time.
 * @param[in] start [const ros::Time&] The start time of the pipeline's
 * execution
 * @param[in] end [const ros::Time&] The end time of the pipeline's execution
 */
  void
CBGL::measureExecutionTime(
  const ros::Time& start,
  const ros::Time& end)
{
  ros::Duration d = end-start;
  std_msgs::Duration duration_msg;
  duration_msg.data = d;
  execution_time_publisher_.publish(duration_msg);
}

/*******************************************************************************
 * @brief Checks if there are nan's in an input pose.
 * @param[in] pose_msg [const geometry_msgs::Pose::Ptr&]
 * The input pose
 * @return [bool] True if there is at least one nan in the input_pose, false
 * otherwise.
 */
  bool
CBGL::nanInPose(
  const geometry_msgs::Pose::Ptr& pose_msg)
{
  if (std::isnan(pose_msg->position.x) ||
    std::isnan(pose_msg->position.y) ||
    std::isnan(pose_msg->orientation.z) ||
    std::isnan(pose_msg->orientation.w))
    return true;
  else
    return false;
}

/*******************************************************************************
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
  int
CBGL::numRaysFromAngleRange(
  const double& angle_min,
  const double& angle_max,
  const int& num_rays,
  const double& new_range)
{
  double v = num_rays * new_range / (angle_max - angle_min);
  int v_int = static_cast<int>(v);

  /* We shall return an even number of rays either way */
  if (v_int % 2 == 0)
    return v;
  else
    return ceil(v);
}

/*******************************************************************************
 * @brief The amcl cloud pose callback. This is the point of entry
 * @param[in] pose_cloud_msg [const geometry_msgs::PoseArray::Ptr&]
 * The amcl pose wrt to the map frame
 * @return void
 */
  void
CBGL::poseCloudCallback(
  const geometry_msgs::PoseArray::Ptr& pose_cloud_msg)
{
  /* Store all the filter's particles */
  dispersed_particles_.clear();

  /* Push back all particles from amcl */
  for (int i = 0; i < pose_cloud_msg->poses.size(); i++)
  {
    geometry_msgs::Pose::Ptr pose_i =
      boost::make_shared<geometry_msgs::Pose>(pose_cloud_msg->poses[i]);

    dispersed_particles_.push_back(pose_i);
  }

  ROS_INFO("[CBGL] There are %zux%d particles in total",
    dispersed_particles_.size(), da_);

  /* Process cloud if all other conditions are satisfied */
  received_pose_cloud_ = true;
  if (received_scan_ && received_map_ && received_start_signal_)
    processPoseCloud();
}

/*******************************************************************************
 * @brief
 * @return void
 */
void CBGL::processPoseCloud()
{
  ros::Time start = ros::Time::now();

  /*
   * Construct the 3rd-party ray-casters AFTER both a scan and the map is
   * received
   */
  bool cond = received_scan_ &&
    received_pose_cloud_     &&
    received_start_signal_   &&
    received_map_;

  if (!cond)
    return;

  /* Init raycasters with laser details */
  initRangeLibRayCasters();

  /* Publish all hypotheses */
  if (publish_pose_sets_)
  {
    geometry_msgs::PoseArray pa1;
    pa1.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> pv1;
    for (unsigned int i = 0; i < dispersed_particles_.size(); i++)
      pv1.push_back(*dispersed_particles_[i]);
    pa1.poses = pv1;
    all_hypotheses_publisher_.publish(pa1);
  }

  /* Do not take ALL hypotheses, but only those with the 10 best caers */
  std::vector<geometry_msgs::Pose::Ptr> caer_best_particles =
    siftThroughCAERPanoramic(dispersed_particles_);

  ROS_INFO("[CBGL] I will consider only %zu hypotheses",
    caer_best_particles.size());

  /* Publish top caer hypotheses */
  if (publish_pose_sets_)
  {
    geometry_msgs::PoseArray pa2;
    pa2.header.stamp = ros::Time::now();
    std::vector<geometry_msgs::Pose> pv2;
    for (unsigned int i = 0; i < caer_best_particles.size(); i++)
      pv2.push_back(*caer_best_particles[i]);
    pa2.poses = pv2;
    top_caer_hypotheses_publisher_.publish(pa2);
  }

  /* After CAER phase: scan--to--map-scan match the top X best hypotheses */
  std::vector<sm_result> outputs;
  std::vector<tf::Transform> f2bs;
  double score = -1.0;
  double score_i = -1.0;
  int score_idx = -1;
  for (int i = 0; i < caer_best_particles.size(); i++)
  {
    ROS_INFO(
      "[CBGL] Processing hypothesis %d: (x,y,t) = (%f,%f,%f)", i,
      caer_best_particles[i]->position.x,
      caer_best_particles[i]->position.y,
      extractYawFromPose(*caer_best_particles[i]));

    sm_result output;
    tf::Transform f2b;
    handleInputPose(caer_best_particles[i], &output, &f2b);

    outputs.push_back(output);
    f2bs.push_back(f2b);


    /*
     * How to evaluate the best hypotheses AFTER matching?
     * Via ICP's internal metric or CAER?
     */
    bool icps_score = false;
    if (icps_score)
      score_i = output.valid * output.nvalid / fabs(output.error);
    else
    {
      /* Apply transform to this hypothesis so that we may take the CAER */
      geometry_msgs::Pose::Ptr corrected_pose =
        boost::make_shared<geometry_msgs::Pose>(*caer_best_particles[i]);
      correctICPPose(corrected_pose, f2b);

      /* Have CAER as score_i instead of ICP's metric */
      double c = caer(latest_world_scan_,
        scanMap(corrected_pose, map_scan_method_, false));
      score_i = 1.0 / (c+1e-10);
    }

    if (score_i > score)
    {
      score = score_i;
      score_idx = i;
    }
  }

  ROS_INFO("[CBGL] ---------------------");
  ROS_INFO("[CBGL] Best hypothesis id = %d", score_idx);
  ROS_INFO("[CBGL] ---------------------");

  /*
   * At this point what we have identified is the particle from whose pose
   * the virtual laser scan once transformed has the best match to
   * the input scan
   */
  geometry_msgs::Pose::Ptr global_pose =
    boost::make_shared<geometry_msgs::Pose>(*caer_best_particles[score_idx]);

  if (outputs[score_idx].valid == 1)
  {
    correctICPPose(global_pose, f2bs[score_idx]);

    ROS_INFO("[CBGL] Pose found: (%f,%f,%f)",
      global_pose->position.x,
      global_pose->position.y,
      extractYawFromPose(*global_pose));
    ROS_INFO("--------------------------------------------------------------------------");
  }
  else
    ROS_ERROR("[CBGL] No valid pose found");

  /* Publish execution time */
  measureExecutionTime(start, ros::Time::now());

  /* Publish global pose */
  geometry_msgs::PoseWithCovarianceStamped global_pose_wcs;
  global_pose_wcs.pose.pose = *global_pose;

  boost::array<double,36> covar;
  covar[6*0+0] = initial_cov_xx_;
  covar[6*1+1] = initial_cov_yy_;
  covar[6*5+5] = initial_cov_aa_;
  global_pose_wcs.pose.covariance =  covar;
  global_pose_wcs.header.stamp = ros::Time::now();
  global_pose_wcs.header.frame_id = "/map";
  global_pose_publisher_.publish(global_pose_wcs);

  /* Broadcast pose as transform */
  if (tf_broadcast_)
    broadcast_global_pose_tf(global_pose_wcs.pose.pose);

  /* Publish the index of the best particle by caer standards */
  std_msgs::UInt64 np_msg;
  if (outputs[score_idx].valid == 1)
    np_msg.data = score_idx;
  else
    np_msg.data = -1;
  best_particle_publisher_.publish(np_msg);

  /* Re-close down */
  received_start_signal_ = false;
}

/*******************************************************************************
 * @brief The champion function of the ICP operation.
 * @param[in] world_scan_ldp [LDP&] The world scan in LDP form.
 * @param[in] map_scan_ldp [LDP&] The map scan in LDP form.
 * @return void
 */
  void
CBGL::processScan(LDP& world_scan_ldp, LDP& map_scan_ldp,
  sm_result* output, tf::Transform* f2b)
{
  ros::WallTime start = ros::WallTime::now();

  /*
   * CSM is used in the following way:
   * The scans are always in the laser frame
   * The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
   * The new scan (currLDPScan) has a pose equal to the movement
   * of the laser in the laser frame since the last scan
   * The computed correction is then propagated using the tf machinery
   */

  map_scan_ldp->odometry[0] = 0.0;
  map_scan_ldp->odometry[1] = 0.0;
  map_scan_ldp->odometry[2] = 0.0;

  map_scan_ldp->estimate[0] = 0.0;
  map_scan_ldp->estimate[1] = 0.0;
  map_scan_ldp->estimate[2] = 0.0;

  map_scan_ldp->true_pose[0] = 0.0;
  map_scan_ldp->true_pose[1] = 0.0;
  map_scan_ldp->true_pose[2] = 0.0;

  input_.laser_ref  = map_scan_ldp;
  input_.laser_sens = world_scan_ldp;

  input_.first_guess[0] = 0.0;
  input_.first_guess[1] = 0.0;
  input_.first_guess[2] = 0.0;

  /* If they are non-Null, free covariance gsl matrices to avoid leaking memory */
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  /* scan match - using point to line icp from CSM */

  sm_icp(&input_, &output_);

  /* https://github.com/AndreaCensi/csm/blob/master/sm/csm/algos.h#L138 */
  if (output_.valid)
  {
    /* the correction of the laser's position, in the laser frame */
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    /* the correction of the base's position, in the base frame */
    f2b_ = base_to_laser_ * corr_ch_l * laser_to_base_;
  }
  else
  {
    f2b_.setIdentity();
    ROS_WARN("[CBGL] Error in scan matching");
  }

  *output = output_;
  *f2b = f2b_;

  /* statistics */
  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("[CBGL] Scan matcher total duration: %.1f ms", dur);
}

/*******************************************************************************
 * For use with FSM
*/
  std::vector<double>
CBGL::retypeScan(const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  std::vector<double> ret_vector;

  for (auto i(0); i < scan_msg->ranges.size(); i++)
    ret_vector.push_back(scan_msg->ranges[i]);

  return ret_vector;
}

/*******************************************************************************
 * @brief The laser scan callback
 * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input scan
 * message
 * @return void
 */
  void
CBGL::scanCallback(
  const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  if (!std::isfinite(scan_msg->ranges[0]))  /* premature scan */
    return;

  /* Undersample range scan if desirable ------------------------------------ */
  sensor_msgs::LaserScan::Ptr s_ =
    boost::make_shared<sensor_msgs::LaserScan>(*scan_msg);
  if (do_undersample_scan_)
  {
    std::vector<float> uranges;
    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
      if (fmod(i,undersample_rate_) == 0)
        uranges.push_back(scan_msg->ranges[i]);
    }

    /* The new scan message */
    s_->ranges = uranges;
    s_->angle_increment = undersample_rate_ * scan_msg->angle_increment;
  }

  /* Store scan properties -------------------------------------------------- */
  nrays_ = s_->ranges.size();
  angle_inc_ = s_->angle_increment;
  angle_min_ = s_->angle_min;
  input_.min_reading = s_->range_min;
  input_.max_reading = s_->range_max;


  /* Store the latest scan -------------------------------------------------- */
  latest_world_scan_ =
    boost::make_shared<sensor_msgs::LaserScan>(*s_);

  for (unsigned int i = 0; i < s_->ranges.size(); i++)
  {
    if (!std::isfinite(s_->ranges[i]))
      latest_world_scan_->ranges[i] = 0;
  }


  /* What's the lowest range? */
  double latest_world_scan_min_range_now =
    *min_element(s_->ranges.begin(), s_->ranges.end());

  /* Remove garbage measurements if lowest measured range <= min sensor range */
  if (latest_world_scan_min_range_now <= latest_world_scan_->range_min)
    latest_world_scan_->ranges = interpolateRanges(latest_world_scan_->ranges,
      latest_world_scan_->range_min);


  /* if first scan cache needed stuff --------------------------------------- */
  if (!received_scan_)
  {
    /* cache the static tf from base to laser if output is required as transform */
    if (tf_broadcast_)
    {
      if (!getBaseToLaserTf(s_->header.frame_id))
      {
        ROS_WARN("[CBGL] Skipping scan");
        return;
      }
    }

    if (do_fsm_)
      cacheFFTW3Plans(s_->ranges.size());
  }

  /* Good to go ------------------------------------------------------------- */
  received_scan_ = true;
  if (received_map_ && received_pose_cloud_ && received_start_signal_)
    processPoseCloud();
}

/*******************************************************************************
 * @brief Given the robot's pose and a choice to scan over an angle of 2π,
 * this function simulates a range scan that has the physical world substituted
 * for the map.
 * @param[in] robot_pose [const geometry_msgs::Pose::Ptr&]
 * The robot's pose.
 * @param[in] scan_method [const std::string&] Which method to use for
 * scanning the map. Currently supports vanilla (Bresenham's method)
 * and ray_marching.
 * @param[in] do_fill_map_scan [const bool&] A choice to scan over an angle of
 * 2π.
 * @return [sensor_msgs::LaserScan::Ptr] The map scan in LaserScan form.
 */
  sensor_msgs::LaserScan::Ptr
CBGL::scanMap(
  const geometry_msgs::Pose::Ptr& robot_pose,
  const std::string& scan_method,
  const bool& do_fill_map_scan)
{
#if DEBUG_EXECUTION_TIMES == 1
  ros::Time start = ros::Time::now();
#endif

  /*
   * Get the laser's pose: the map scan needs that one, not the pose of the
   * robot!
   */
  const geometry_msgs::Pose current_laser_pose =
    getCurrentLaserPose(*robot_pose);

  sensor_msgs::LaserScan::Ptr laser_scan_info =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  sensor_msgs::LaserScan::Ptr map_scan;


  /* Scan the map using the occupancy_grid_utils method */
  if (scan_method.compare("vanilla") == 0)
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }
  else if (scan_method.compare("ray_marching") == 0 ||
    scan_method.compare("bresenham")    == 0 ||
    scan_method.compare("cddt")         == 0)
  {
    /* Convert laser position to grid coordinates */
    float x = current_laser_pose.position.x / map_.info.resolution;
    float y = map_.info.height - 1 -
      current_laser_pose.position.y / map_.info.resolution;
    float a = extractYawFromPose(current_laser_pose);

    /*
     * How many rays?
     * int num_rays = numRaysFromAngleRange(latest_world_scan_->angle_min,
     * latest_world_scan_->angle_max, nrays_,
     * laser_scan_info->angle_max - laser_scan_info->angle_min);
     */

    double min_a = laser_scan_info->angle_min;
    double inc_a = laser_scan_info->angle_increment;

    map_scan = boost::make_shared<sensor_msgs::LaserScan>(*laser_scan_info);
    map_scan->ranges.clear();

    /*
     * The angular progression of scanning depends on how the laser is mounted
     * on the robot:
     * a. On the turtlebot the laser faces upwards;
     * b. On the rb1 the laser faces downwards
     */
    int sgn = 0;
    if (laser_z_orientation_.compare("upwards") == 0)
      sgn = -1;
    else if (laser_z_orientation_.compare("downwards") == 0)
      sgn = 1;
    else
    {
      ROS_ERROR("[CBGL] Please provide a valid value");
      ROS_ERROR("       for param laser_z_orientation");
    }

    /*
     * Iterate over all angles.
     * Calculate range for every angle (calc_range returns range in pixels).
     * The angles start counting negatively.
     * For details see
     * https://github.com/kctess5/range_libc/blob/deploy/docs/RangeLibcUsageandInformation.pdf
     */

    if (scan_method.compare("ray_marching") == 0)
    {
      for (unsigned int i = 0; i < nrays_; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          rm_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("cddt") == 0)
    {
      for (unsigned int i = 0; i < nrays_; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          cddt_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("bresenham") == 0)
    {
      for (unsigned int i = 0; i < nrays_; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          br_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
  }
  else  /* Default to vanilla */
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }

  map_scan->header.stamp = ros::Time::now();

#if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("scanMap() took %.2f ms", (ros::Time::now() - start).toSec() * 1000);
#endif

  return map_scan;
}

/*******************************************************************************
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
  sensor_msgs::LaserScan::Ptr
CBGL::scanMapPanoramic(
  const geometry_msgs::Pose::Ptr& robot_pose,
  const std::string& scan_method)
{
  const geometry_msgs::Pose current_laser_pose = *robot_pose;

  sensor_msgs::LaserScan::Ptr laser_scan_info =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  sensor_msgs::LaserScan::Ptr map_scan;


  /* Scan the map using the occupancy_grid_utils method */
  if (scan_method.compare("vanilla") == 0)
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }
  else if (scan_method.compare("ray_marching") == 0 ||
    scan_method.compare("bresenham")    == 0 ||
    scan_method.compare("cddt")         == 0)
  {
    /* Convert laser position to grid coordinates */
    float x = current_laser_pose.position.x / map_.info.resolution;
    float y = map_.info.height - 1 -
      current_laser_pose.position.y / map_.info.resolution;
    float a = extractYawFromPose(current_laser_pose);

    /* How many rays? */
    int num_rays = numRaysFromAngleRange(latest_world_scan_->angle_min,
      latest_world_scan_->angle_max, nrays_, 2*M_PI);

    /* printf("%d,%d\n", nrays_, num_rays); */


    double min_a = M_PI;
    double inc_a = laser_scan_info->angle_increment;

    map_scan = boost::make_shared<sensor_msgs::LaserScan>(*laser_scan_info);
    map_scan->ranges.clear();

    /*
     * The angular progression of scanning depends on how the laser is mounted
     * on the robot:
     * a. On the turtlebot the laser faces upwards;
     * b. On the rb1 the laser faces downwards
     */
    int sgn = 0;
    if (laser_z_orientation_.compare("upwards") == 0)
      sgn = -1;
    else if (laser_z_orientation_.compare("downwards") == 0)
      sgn = 1;
    else
    {
      ROS_ERROR("[CBGL] Please provide a valid value");
      ROS_ERROR("       for param laser_z_orientation");
    }

    /*
     * Iterate over all angles.
     * Calculate range for every angle (calc_range returns range in pixels).
     * The angles start counting negatively.
     * For details see
     * https://github.com/kctess5/range_libc/blob/deploy/docs/RangeLibcUsageandInformation.pdf
     */

    if (scan_method.compare("ray_marching") == 0)
    {
      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          rm_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("cddt") == 0)
    {

      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          cddt_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("bresenham") == 0)
    {
      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          br_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
  }
  else  /* Default to vanilla */
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }

  map_scan->header.stamp = ros::Time::now();

  return map_scan;
}

/*******************************************************************************
 * @brief Takes all pose hypotheses, computes their caer against the real scan,
 * and ranks them. Only p% of all poses are then considered for sm2
 */
  std::vector<geometry_msgs::Pose::Ptr>
CBGL::siftThroughCAERPanoramic(
  const std::vector<geometry_msgs::Pose::Ptr>& init_hypotheses)
{
  /* All da_*init_hypotheses.size() hypotheses */
  std::vector<geometry_msgs::Pose::Ptr> all_hypotheses;

  /* Compute all caers */
  std::vector<double> caers;
  std::vector<double> scan_map_times;
  sensor_msgs::LaserScan::Ptr sv_i;
  for (unsigned int i = 0; i < init_hypotheses.size(); i++)
  {
    /* Compute the virtual scan from pose hypothesis i */
    sv_i = scanMapPanoramic(init_hypotheses[i], map_scan_method_);

    /* This hypothesis' orientation */
    const double yaw_i = extractYawFromPose(*init_hypotheses[i]);
    double new_yaw_i = 0.0;
    tf::Quaternion q;

    /* Extract only ranges */
    const std::vector<float> r_i = sv_i->ranges;
    const size_t num_rays = r_i.size();

    const unsigned int shifter = round(static_cast<double>(num_rays) / da_);
    const unsigned int prefix = round((static_cast<double>(num_rays-nrays_))/2);

    std::vector<float> r_ik;
    geometry_msgs::Pose::Ptr pose_ik;
    for (unsigned int k = 0; k < da_; k++)
    {
      r_ik = r_i;
      std::rotate(r_ik.begin(), r_ik.begin()+k*shifter, r_ik.end());
      std::vector<float>::const_iterator a = r_ik.begin() + prefix;
      std::vector<float>::const_iterator b = a + nrays_;


      /* deez ranges */
      std::vector<float> r_ikf(a,b);

      /* This is the pose from which r_ikf was captured */
      pose_ik = boost::make_shared<geometry_msgs::Pose>(*init_hypotheses[i]);

      /* Orientation */
      new_yaw_i = yaw_i - 2*M_PI * static_cast<double>(k)/da_;
      wrapAngle(new_yaw_i);
      q.setRPY(0.0, 0.0, new_yaw_i); q.normalize();
      tf::quaternionTFToMsg(q, pose_ik->orientation);

      /* The full hypothesis */
      all_hypotheses.push_back(pose_ik);


      /* Compute CAER */
      double c = caer(latest_world_scan_->ranges, r_ikf);
      if (c > 0.0)
        caers.push_back(c);
      else  /* THIS IS NECESSARY */
        caers.push_back(std::numeric_limits<double>::max());

      /* printf("%f\n", caers.back()); */
    }
  }

  /*
   * for (unsigned int i = 0; i < all_hypotheses.size(); i=i+da_)
   * {
   * ROS_INFO("(%f,%f,%f)",
   * all_hypotheses[i]->position.x,
   * all_hypotheses[i]->position.y,
   * extractYawFromPose(*all_hypotheses[i]));
   * }

   * / * Publish ALL CAERS * /
   * geometry_msgs::PoseArray pa2;
   * pa2.header.stamp = ros::Time::now();
   * std::vector<geometry_msgs::Pose> pv2;
   * for (unsigned int i = 0; i < all_hypotheses.size(); i++)
   * {
   * pv2.push_back(*all_hypotheses[i]);
   * pv2[i].position.x = caers[i];
   * }
   * pa2.poses = pv2;
   * all_hypotheses_caer_publisher_.publish(pa2);


   * / * Publish ALL hypotheses * /
   * geometry_msgs::PoseArray pa1;
   * pa1.header.stamp = ros::Time::now();
   * std::vector<geometry_msgs::Pose> pv1;
   * for (unsigned int i = 0; i < all_hypotheses.size(); i++)
   * pv1.push_back(*all_hypotheses[i]);
   * pa1.poses = pv1;
   * all_hypotheses_publisher_.publish(pa1);
   */

  /* Sort all caers and get the original indices of all sorted caer values */
  std::vector<size_t> idx(caers.size());
  std::iota(idx.begin(), idx.end(), 0);

  std::stable_sort(
    idx.begin(),
    idx.end(),
    [&caers](size_t i1,size_t i2) {return caers[i1] < caers[i2];});


  /* The top p% of lowest caers in hypotheses */
  std::vector<geometry_msgs::Pose::Ptr> caer_best_particles;
  for (unsigned int i = 0; i < top_k_caers_; i++)
    caer_best_particles.push_back(all_hypotheses[idx[i]]);

  return caer_best_particles;
}

/*******************************************************************************
 * @brief User calls this service and cbgl functionality is executed
*/
  bool
CBGL::startSignalService(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  while (received_start_signal_)
    ros::Duration(1).sleep();

  if (!received_map_)
  {
    ROS_WARN("[CBGL] Map not received yet, returning ...");
    return false;
  }

  ROS_INFO("--------------------------------------------------------------------------");
  ROS_INFO("[CBGL] Initializing with uniform distribution over map");
  pf_init_model(pf_hyp_, (pf_init_model_fn_t)CBGL::uniformPoseGenerator,
    (void *)map_hyp_);

  /* The set of all newly-distributed hypotheses */
  pf_sample_set_t* set = pf_hyp_->sets + pf_hyp_->current_set;

  geometry_msgs::PoseArray::Ptr pose_cloud_msg =
    boost::make_shared<geometry_msgs::PoseArray>();
  pose_cloud_msg->header.stamp = ros::Time::now();
  pose_cloud_msg->header.frame_id = fixed_frame_id_;
  pose_cloud_msg->poses.resize(set->sample_count);

  for(int i = 0; i < set->sample_count; i++)
  {
    pose_cloud_msg->poses[i].position.x = set->samples[i].pose.v[0];
    pose_cloud_msg->poses[i].position.y = set->samples[i].pose.v[1];
    pose_cloud_msg->poses[i].position.z = 0;

    tf::Quaternion q;
    q.setRPY(0, 0, set->samples[i].pose.v[2]);
    q.normalize();
    tf::quaternionTFToMsg(q, pose_cloud_msg->poses[i].orientation);

    /*
     * ROS_ERROR("(%f,%f)",
     * pose_cloud_msg->poses[i].position.x,
     * pose_cloud_msg->poses[i].position.y);
     */
  }

  /*
   * Publish the cloud. This is not the full cloud considered by CBGL; it only
   * features dl * area poses at this point. The next step here is to rotate
   * all of them in 360 / da deg intervals in order to cover a whole cycle ...
   */
  if (publish_pose_sets_)
    all_hypotheses_publisher_.publish(*pose_cloud_msg);

  /* ... which is handled here. This used to be a callback and the name stuck. */
  poseCloudCallback(pose_cloud_msg);

  ROS_INFO("[CBGL] Global initialisation done!");
  ROS_INFO("----------------------------------");

  received_start_signal_ = true;
  return received_start_signal_;
}

/*******************************************************************************
 * @brief Pose disperser
 * @return [pf_vector_t] Uniformly random pose set over arg
 */
  pf_vector_t
CBGL::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;

  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("[CBGL] Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2*M_PI - M_PI;

    /* Check that it's a free cell */
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
  return p;
}

/*******************************************************************************
 * @brief Wraps an angle in the [-π, π] interval
 * @param[in,out] angle [double&] The angle to be expressed in [-π,π]
 * @return void
 */
  void
CBGL::wrapAngle(double& angle)
{
  angle = fmod(angle + 5*M_PI, 2*M_PI) - M_PI;
}

/*******************************************************************************
 * @brief Wraps a pose's orientation in the [-π,π] interval
 * @param[in,out] pose [geometry_msgs::Pose::Ptr&] The pose
 * whose input will be wrapped in [-π,π]
 * @return void
 */
  void
CBGL::wrapPoseOrientation(
  geometry_msgs::Pose::Ptr& pose)
{
  /* Extract yaw ... */
  double yaw = extractYawFromPose(*pose);

  /* ... and wrap between [-π, π] */
  wrapAngle(yaw);

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  tf::quaternionTFToMsg(q, pose->orientation);
}
