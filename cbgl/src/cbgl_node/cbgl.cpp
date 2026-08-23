/*
 * cbgl - [IROS'24] Globally localise your 2D LIDAR in a 2D map in no time
 *
 * Copyright (c) 2024 Alexandros PHILOTHEOU
 *
 * Licensed under the MIT License.
 * See LICENSE.MIT for details.
 */
#include <cbgl_node/cbgl.h>

#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

#define DEBUG_EXECUTION_TIMES 0

namespace
{

/** @brief The largest width or height this reads from a map image, and the
 *  largest number of cells such an image may declare.
 *  @details A PGM header states its own dimensions, and nothing in the file
 *  obliges them to describe the bytes that follow. Sizing a buffer from an
 *  unbounded header is how a mistyped digit becomes an allocation of gigabytes
 *  or, once two such numbers are multiplied in a narrow type, an index that
 *  walks outside every buffer in the function. Both are bounded here instead.
 *  A hundred thousand cells on a side is far past any occupancy grid a robot
 *  carries, and the cell ceiling is roughly a hundred times the largest grid
 *  this package has ever been driven over. */
const uint64_t kMaxPGMDimension = 100000;
const uint64_t kMaxPGMCells = 600000000;

/** @brief The largest map image this reads, in bytes. Two bytes a cell at the
 *  cell ceiling above, which is more than any occupancy grid a robot carries
 *  and small enough that refusing a larger one costs nobody anything. */
const uint64_t kMaxPGMBytes = 1200000000;

/** @brief Whether a double is finite, written so that fast maths cannot fold
 *  it away.
 *  @details This package compiles with `-Ofast`, which implies
 *  `-ffinite-math-only`, which tells the compiler that no infinity and no
 *  quiet NaN can occur. Under that promise `std::isfinite` is a constant true
 *  and the guards built on it vanish entirely, so a NaN read out of a map
 *  description reaches the arithmetic that places every hypothesis. The cell
 *  index it then produces is refused for every draw and the dispersal loop
 *  never ends. Comparing the bits instead asks a question the optimiser is not
 *  permitted to answer in advance.
 *  @remark Do not replace this with `std::isfinite`, however tempting: it is
 *  correct only in a build that does not set `-ffast-math` or `-Ofast`, and
 *  this one does. */
bool isFiniteUnderFastMaths(const double value)
{
  uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  const uint64_t exponent = (bits >> 52) & 0x7FF;
  return exponent != 0x7FF;
}

}  // namespace

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
  cachedFFTW3Plans_(false),
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
  map_yaw_(0.0),
  map_sin_yaw_(0.0),
  map_cos_yaw_(1.0),
  top_k_caers_(10)
{
  nh_private_.param<std::string>("pkg_name", PKG_NAME, "CBGL");
  ROS_INFO("[%s] Initialising ...", PKG_NAME.c_str());

  initParams();

  global_pose_publisher_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      output_pose_topic_, 1);

  execution_time_publisher_ =
    nh_.advertise<std_msgs::Duration>(
      ros::this_node::getName() + "/execution_time", 10);

  best_particle_publisher_ =
    nh_.advertise<std_msgs::UInt64>(
      ros::this_node::getName() + "/best_particle", 10);

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

  world_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/world_scan", 1);

  map_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/map_scan", 1);

  if (!map_topic_.empty())
  {
    if (!map_file_.empty())
      ROS_WARN(
        "[%s] Both map_topic and map_file are set; subscribing to "
        "map_topic and ignoring map_file", PKG_NAME.c_str());

    map_subscriber_ = nh_.subscribe(map_topic_, 1,
      &CBGL::mapCallback, this);
  }
  else if (!map_file_.empty())
    mapCallback(loadMapFromFile(map_file_));
  else
    throw std::runtime_error(
      "[" + PKG_NAME + "] One of map_topic or map_file is required");

  scan_subscriber_ = nh_.subscribe(scan_topic_, 1,
    &CBGL::scanCallback, this);

  global_localisation_service_ =
    nh_.advertiseService(
      ros::this_node::getName() + "/" + global_localisation_service_name_,
      &CBGL::startSignalService, this);

  ROS_INFO("[%s] Inititialised.",                      PKG_NAME.c_str());
  ROS_INFO("[%s] To execute global localization issue",PKG_NAME.c_str());
  ROS_INFO("%*s rosservice call %s", (int)(PKG_NAME.size()+2),"",
    (ros::this_node::getName()+"/"+global_localisation_service_name_).c_str());
}

/*******************************************************************************
 * @brief Destructor
 * @params void
 */
CBGL::~CBGL()
{
  if (cachedFFTW3Plans_ == true)
  {
    if (r2rp_)
    {
      fftw_destroy_plan(r2rp_);
      r2rp_ = nullptr;
    }

    if (c2rp_)
    {
      fftw_destroy_plan(c2rp_);
      c2rp_ = nullptr;
    }

    fftw_cleanup();
  }

  ROS_INFO("[%s] Destroyed", PKG_NAME.c_str());
}

/*****************************************************************************
 * @brief Broadcasts the estimated pose as the odom<--map transform
 */
void CBGL::broadcast_global_pose_tf(
  const geometry_msgs::Pose& pose)
{

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
    ROS_WARN("[%s] Could not get transform from %s to %s. Error: %s",
      PKG_NAME.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
  }

  tf::Transform base_to_map_tf;
  createTfFromXYTheta(pose.position.x, pose.position.y, extractYawFromPose(pose),
    base_to_map_tf);

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
  const size_t rays = std::min(sr.size(), sv.size());

  double c = 0;
  for (size_t i = 0; i < rays; i++)
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
  double* r2r_in;
  double* r2r_out;

  r2r_in = (double*) fftw_malloc(sz * sizeof(double));
  r2r_out = (double*) fftw_malloc(sz * sizeof(double));

  r2rp_ = fftw_plan_r2r_1d(sz, r2r_in, r2r_out, FFTW_R2HC, FFTW_MEASURE);

  fftw_complex* c2r_in;
  double* c2r_out;

  c2r_in = (fftw_complex*) fftw_malloc(sz * sizeof(fftw_complex));
  c2r_out = (double*) fftw_malloc(sz * sizeof(double));

  c2rp_ = fftw_plan_dft_c2r_1d(sz, c2r_in, c2r_out, FFTW_MEASURE);

  fftw_free(r2r_in);
  fftw_free(r2r_out);
  fftw_free(c2r_in);
  fftw_free(c2r_out);

  cachedFFTW3Plans_ = true;
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

  map->origin_yaw = extractYawFromPose(map_msg.info.origin);
  map->origin_sin_yaw = sin(map->origin_yaw);
  map->origin_cos_yaw = cos(map->origin_yaw);

  const double half_x = (map->size_x / 2) * map->scale;
  const double half_y = (map->size_y / 2) * map->scale;

  map->origin_x = map_msg.info.origin.position.x
    + map->origin_cos_yaw * half_x - map->origin_sin_yaw * half_y;
  map->origin_y = map_msg.info.origin.position.y
    + map->origin_sin_yaw * half_x + map->origin_cos_yaw * half_y;

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
    if (map_msg.data[i] == 100)
    {
      converted[ic+0] = 0;
      converted[ic+1] = 0;
      converted[ic+2] = 0;
      converted[ic+3] = 255;
    }

    if (map_msg.data[i] == 0)
    {
      converted[ic+0] = 254;
      converted[ic+1] = 254;
      converted[ic+2] = 254;
      converted[ic+3] = 255;
    }

    if (map_msg.data[i] == -1)
    {
      converted[ic+0] = 205;
      converted[ic+1] = 205;
      converted[ic+2] = 205;
      converted[ic+3] = 255;
    }
    ic += 4;
  }

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
  std::vector<unsigned char> converted(4*map_msg.data.size());
  convertMapToPNG(map_msg, converted.data());

  unsigned int width = map_msg.info.width;
  unsigned int height = map_msg.info.height;
  std::vector<uint8_t> png;
  unsigned error =
    lodepng::encode(png, converted.data(), width, height, LCT_RGBA, 8);
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

    tf::Transform icp_corrected_pose_tf = map_to_base_tf * f2b;

    tf::poseTFToMsg(icp_corrected_pose_tf, *icp_corrected_pose);

    wrapPoseOrientation(icp_corrected_pose);
  }
  else
  {
    ROS_ERROR("[%s] ICP has failed; falling back to amcl pose", PKG_NAME.c_str());
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
#if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_icp = ros::Time::now();
#endif

  geometry_msgs::Pose::Ptr icp_corrected_pose =
    boost::make_shared<geometry_msgs::Pose>(*amcl_pose_msg);

  sensor_msgs::LaserScan::Ptr map_scan = scanMap(icp_corrected_pose,
    map_scan_method_, false);

  std::tuple<double,double,double> origin;
  std::get<0>(origin) = 0.0;
  std::get<1>(origin) = 0.0;
  std::get<2>(origin) = 0.0;


  std::vector<double> sr = retypeScan(latest_world_scan);
  std::vector<double> sv = retypeScan(map_scan);

  std::vector< std::pair<double,double> > vp;
  FSM::Utils::scan2points(sv, origin, &vp);

  FSM::output_params op;
  std::tuple<double,double,double> diff;

  FSM::Match::fmtdbh(sr, origin, vp, r2rp_, c2rp_, ip_, &op, &diff);

  double dx = std::get<0>(diff);
  double dy = std::get<1>(diff);
  double dt = std::get<2>(diff);

  tf::Transform corr_ch_l;
  createTfFromXYTheta(dx,dy,dt, corr_ch_l);

  f2b_ = base_to_laser_ * corr_ch_l * laser_to_base_;
  *f2b = f2b_;
  output->x[0] = dx;
  output->x[1] = dy;
  output->x[2] = dt;
  output->valid = 1;

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
#if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_icp = ros::Time::now();
#endif

  geometry_msgs::Pose::Ptr icp_corrected_pose =
    boost::make_shared<geometry_msgs::Pose>(*amcl_pose_msg);

  LDP world_scan_ldp;
  laserScanToLDP(latest_world_scan, world_scan_ldp);

  sensor_msgs::LaserScan::Ptr map_scan = scanMap(icp_corrected_pose,
    map_scan_method_, false);

  LDP map_scan_ldp;
  laserScanToLDP(map_scan, map_scan_ldp);

  processScan(world_scan_ldp, map_scan_ldp, output, f2b);

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

    tf_listener_.lookupTransform(base_frame_id_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("[%s] Could not get initial transform from", PKG_NAME.c_str());
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
  tf::Transform map_to_base_tf;
  tf::poseMsgToTF(robot_pose, map_to_base_tf);

  tf::Transform map_to_laser_tf = map_to_base_tf * base_to_laser_;

  geometry_msgs::Pose laser_pose;
  tf::poseTFToMsg(map_to_laser_tf, laser_pose);

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
  if (nanInPose(pose_msg))
  {
    ROS_ERROR("[%s] pose contains nan's; aborting ...", PKG_NAME.c_str());
    return;
  }

  if (running_)
  {
    ROS_ERROR("[%s] Already running; aborting ...", PKG_NAME.c_str());
    return;
  }

  running_ = true;

  sensor_msgs::LaserScan::Ptr world_scan =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  geometry_msgs::Pose::Ptr global_pose;

  if (do_icp_)
    doICP(pose_msg, world_scan, output, f2b);
  if (do_fsm_)
    doFSM(pose_msg, world_scan, output, f2b);

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
    ROS_ERROR("[%s] base_frame_id", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("fixed_frame_id", fixed_frame_id_))
  {
    fixed_frame_id_ = "map";
    ROS_ERROR("[%s] fixed_frame_id", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("odom_frame_id", odom_frame_id_))
  {
    odom_frame_id_ = "odom";
    ROS_ERROR("[%s] odom_frame_id", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("tf_broadcast", tf_broadcast_))
  {
    tf_broadcast_ = false;
    ROS_ERROR("[%s] tf_broadcast", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("scan_topic", scan_topic_))
  {
    scan_topic_ = "/front_laser/scan";
    ROS_ERROR("[%s] scan_topic", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("map_topic", map_topic_))
  {
    map_topic_ = "/map";
    ROS_ERROR("[%s] map_topic", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("map_file", map_file_))
  {
    map_file_ = "";
    ROS_ERROR("[%s] map_file", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("output_pose_topic", output_pose_topic_))
  {
    output_pose_topic_ = "/initialpose";
    ROS_ERROR("[%s] output_pose_topic", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("dl", dl_))
  {
    dl_ = 40;
    ROS_ERROR("[%s] param dl", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("da", da_))
  {
    da_ = 32;
    ROS_ERROR("[%s] param da", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("top_k_caers", top_k_caers_))
  {
    top_k_caers_ = 10;
    ROS_ERROR("[%s] top_k_caers", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("map_scan_method", map_scan_method_))
  {
    map_scan_method_ = "vanilla";
    ROS_ERROR("[%s] map_scan_method", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("publish_pose_sets", publish_pose_sets_))
  {
    publish_pose_sets_ = false;
    ROS_ERROR("[%s] publish_pose_sets", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("global_localisation_service_name",
      global_localisation_service_name_))
  {
    global_localisation_service_name_ = "/global_localization";
    ROS_ERROR("[%s] global_localisation_service_name", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("laser_z_orientation", laser_z_orientation_))
  {
    laser_z_orientation_ = "upwards";
    ROS_ERROR("[%s] laser_z_orientation", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("do_undersample_scan", do_undersample_scan_))
  {
    do_undersample_scan_ = false;
    ROS_ERROR("[%s] do_undersample_scan", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("undersample_rate", undersample_rate_))
  {
    undersample_rate_ = 1;
    ROS_ERROR("[%s] undersample_rate", PKG_NAME.c_str());
  }
  if (laser_z_orientation_.compare("upwards") == 0)
    sgn_ = -1;
  else if (laser_z_orientation_.compare("downwards") == 0)
    sgn_ = 1;
  else
  {
    ROS_ERROR("[%s] Please provide a valid value", PKG_NAME.c_str());
    ROS_ERROR("       for param laser_z_orientation");
  }


  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
  {
    input_.max_angular_correction_deg = 45.0;
    ROS_ERROR("[%s] max_angular_correction_deg", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
  {
    input_.max_linear_correction = 0.50;
    ROS_ERROR("[%s] max_linear_correction", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
  {
    input_.max_iterations = 10;
    ROS_ERROR("[%s] max_iterations", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
  {
    input_.epsilon_xy = 0.000001;
    ROS_ERROR("[%s] epsilon_xy", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
  {
    input_.epsilon_theta = 0.000001;
    ROS_ERROR("[%s] epsilon_theta", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
  {
    input_.max_correspondence_dist = 0.3;
    ROS_ERROR("[%s] max_correspondence_dist", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("sigma", input_.sigma))
  {
    input_.sigma = 0.010;
    ROS_ERROR("[%s] sigma", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
  {
    input_.use_corr_tricks = 1;
    ROS_ERROR("[%s] use_corr_tricks", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("restart", input_.restart))
  {
    input_.restart = 0;
    ROS_ERROR("[%s] restart", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
  {
    input_.restart_threshold_mean_error = 0.01;
    ROS_ERROR("[%s] restart_threshold_mean_error", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
  {
    input_.restart_dt = 1.0;
    ROS_ERROR("[%s] restart_dt", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
  {
    input_.restart_dtheta = 0.1;
    ROS_ERROR("[%s] restart_dtheta", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
  {
    input_.clustering_threshold = 0.25;
    ROS_ERROR("[%s] clustering_threshold", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
  {
    input_.orientation_neighbourhood = 20;
    ROS_ERROR("[%s] orientation_neighbourhood", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
  {
    input_.use_point_to_line_distance = 1;
    ROS_ERROR("[%s] use_point_to_line_distance", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
  {
    input_.do_alpha_test = 0;
    ROS_ERROR("[%s] do_alpha_test", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
  {
    input_.do_alpha_test_thresholdDeg = 20.0;
    ROS_ERROR("[%s] do_alpha_test_thresholdDeg", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
  {
    input_.outliers_maxPerc = 0.90;
    ROS_ERROR("[%s] outliers_maxPerc", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
  {
    input_.outliers_adaptive_order = 0.7;
    ROS_ERROR("[%s] outliers_adaptive_order", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
  {
    input_.outliers_adaptive_mult = 2.0;
    ROS_ERROR("[%s] outliers_adaptive_mult", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
  {
    input_.do_visibility_test = 0;
    ROS_ERROR("[%s] do_visibility_test", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
  {
    input_.outliers_remove_doubles = 1;
    ROS_ERROR("[%s] outliers_remove_doubles", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
  {
    input_.do_compute_covariance = 0;
    ROS_ERROR("[%s] do_compute_covariance", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
  {
    input_.debug_verify_tricks = 0;
    ROS_ERROR("[%s] debug_verify_tricks", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
  {
    input_.use_ml_weights = 0;
    ROS_ERROR("[%s] use_ml_weights", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
  {
    input_.use_sigma_weights = 0;
    ROS_ERROR("[%s] use_sigma_weights", PKG_NAME.c_str());
  }

  if (!nh_private_.getParam ("gpm_theta_bin_size_deg", input_.gpm_theta_bin_size_deg))
  {
    input_.gpm_theta_bin_size_deg = 5;
    ROS_ERROR("[%s] gpm_theta_bin_size_deg", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam ("gpm_extend_range_deg", input_.gpm_extend_range_deg))
  {
    input_.gpm_extend_range_deg = 15;
    ROS_ERROR("[%s] gpm_extend_range_deg", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam ("gpm_interval", input_.gpm_interval))
  {
    input_.gpm_interval = 1;
    ROS_ERROR("[%s] gpm_interval", PKG_NAME.c_str());
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
    ROS_ERROR("[%s] do_icp", PKG_NAME.c_str());
  }
  if (!nh_private_.getParam("do_fsm", do_fsm_))
  {
    do_fsm_ = false;
    ROS_ERROR("[%s] do_fsm", PKG_NAME.c_str());
  }

  int int_param;
  if (!nh_private_.getParam ("num_iterations", int_param))
  {
    ROS_WARN("[%s] no num_iterations param found; resorting to defaults", PKG_NAME.c_str());
    ip_.num_iterations = 2;
  }
  else
    ip_.num_iterations = static_cast<unsigned int>(int_param);

  if (!nh_private_.getParam ("xy_bound", ip_.xy_bound))
  {
    ROS_WARN("[%s] no xy_bound param found; resorting to defaults", PKG_NAME.c_str());
    ip_.xy_bound = 0.2;
  }

  if (!nh_private_.getParam ("t_bound", ip_.t_bound))
  {
    ROS_WARN("[%s] no t_bound param found; resorting to defaults", PKG_NAME.c_str());
    ip_.t_bound = M_PI/4;
  }

  if (!nh_private_.getParam ("max_counter", int_param))
  {
    ROS_WARN("[%s] no max_counter param found; resorting to defaults", PKG_NAME.c_str());
    ip_.max_counter = 200;
  }
  else
    ip_.max_counter = static_cast<unsigned int>(int_param);

  if (!nh_private_.getParam ("min_magnification_size", int_param))
  {
    ROS_WARN("[%s] no min_magnification_size param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.min_magnification_size = 0;
  }
  else
    ip_.min_magnification_size = static_cast<unsigned int>(int_param);

  if (!nh_private_.getParam ("max_magnification_size", int_param))
  {
    ROS_WARN("[%s] no max_magnification_size param found; resorting to defaults",
      PKG_NAME.c_str());
    ip_.max_magnification_size = 3;
  }
  else
    ip_.max_magnification_size = static_cast<unsigned int>(int_param);

  if (!nh_private_.getParam ("max_recoveries", int_param))
  {
    ROS_WARN("[%s] no max_recoveries param found; resorting to defaults", PKG_NAME.c_str());
    ip_.max_recoveries = 10;
  }
  else
    ip_.max_recoveries = static_cast<unsigned int>(int_param);

  f2b_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  assert(ip_.num_iterations > 0);
  assert(ip_.xy_bound >= 0.0);
  assert(ip_.t_bound >= 0.0);
  assert(ip_.max_counter > 0);
  assert(ip_.max_magnification_size >= ip_.min_magnification_size);
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
  float max_range_rc = latest_world_scan_->range_max / map_res_;

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

  for (unsigned int i = 0; i < regions.size(); i++)
  {
    int begin = regions[i][0];
    int end = regions[i][regions[i].size()-1];

    regions[i].clear();

    for (unsigned int j = begin; j <= end; j++)
      regions[i].push_back(j);
  }

  int num_regions = regions.size();

  if (num_regions == 0)
    return ranges;

  int numel_last_region = regions[num_regions-1].size();

  if (regions[0][0] == 0 &&
    regions[num_regions-1][numel_last_region-1] == ranges.size()-1)
  {
    for (int i = 0; i < regions[0].size(); i++)
      regions[num_regions-1].push_back(regions[0][i]);

    regions.erase(regions.begin(), regions.begin()+1);
  }

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
 * @details Understands the map_server YAML format: `image`, `resolution`
 * and `origin` are required; `negate`, `occupied_thresh` and `free_thresh`
 * fall back to the map_server defaults when absent. A relative `image` path
 * resolves against the directory the YAML file itself lives in, not the
 * process' working directory. Only the PGM image format is decoded, in
 * both its `P2` and `P5` variants: the ROS 2 build of this package carries
 * no image decoder of its own, and the two builds must accept the same
 * maps, so PGM, the format `map_saver` writes by default, is the common
 * ground. Any missing required key, unreadable file, or malformed value
 * throws `std::runtime_error` naming the key and the file.
 */
  nav_msgs::OccupancyGrid
CBGL::loadMapFromFile(const std::string& filename) const
{
  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(filename);
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error(
      "[" + PKG_NAME + "] Could not read map file '" + filename + "': " +
      ex.what());
  }

  if (!yaml.IsMap())
    throw std::runtime_error(
      "[" + PKG_NAME + "] Map file '" + filename + "' is not a mapping of "
      "keys to values, so it is not a map description; pointing map_file at "
      "an image or at any other file does this");

  if (yaml["mode"])
  {
    std::string mode;
    try
    {
      mode = yaml["mode"].as<std::string>();
    }
    catch (const std::exception& ex)
    {
      throw std::runtime_error(
        "[" + PKG_NAME + "] Key 'mode' in '" + filename + "' is malformed: " +
        ex.what());
    }
    if (mode != "trinary")
      throw std::runtime_error(
        "[" + PKG_NAME + "] Key 'mode' in '" + filename + "' asks for '" +
        mode + "', and this reads 'trinary' only");
  }

  if (!yaml["image"])
    throw std::runtime_error(
      "[" + PKG_NAME + "] Map file '" + filename + "' is missing key 'image'");
  if (!yaml["resolution"])
    throw std::runtime_error(
      "[" + PKG_NAME + "] Map file '" + filename +
      "' is missing key 'resolution'");
  if (!yaml["origin"])
    throw std::runtime_error(
      "[" + PKG_NAME + "] Map file '" + filename + "' is missing key 'origin'");

  std::string image_key;
  double resolution = 0.0;
  std::vector<double> origin;
  double negate = 0.0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;

  try
  {
    image_key = yaml["image"].as<std::string>();
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error(
      "[" + PKG_NAME + "] Key 'image' in '" + filename + "' is malformed: " +
      ex.what());
  }
  try
  {
    resolution = yaml["resolution"].as<double>();
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error(
      "[" + PKG_NAME + "] Key 'resolution' in '" + filename +
      "' is malformed: " + ex.what());
  }
  if (!isFiniteUnderFastMaths(resolution) || resolution <= 0.0 ||
    resolution > 1000.0)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Key 'resolution' in '" + filename +
      "' must be a finite number greater than zero and no more than 1000, "
      "and it is " + std::to_string(resolution) + " metres a cell");
  try
  {
    origin = yaml["origin"].as<std::vector<double> >();
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error(
      "[" + PKG_NAME + "] Key 'origin' in '" + filename +
      "' is malformed: " + ex.what());
  }
  if (origin.size() != 3)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Key 'origin' in '" + filename +
      "' must have exactly three elements");
  for (const double& element : origin)
  {
    if (!isFiniteUnderFastMaths(element))
      throw std::runtime_error(
        "[" + PKG_NAME + "] Key 'origin' in '" + filename +
        "' must contain only finite numbers");
  }

  try
  {
    if (yaml["negate"])
    {
      try
      {
        negate = yaml["negate"].as<double>();
      }
      catch (const std::exception&)
      {
        negate = yaml["negate"].as<bool>() ? 1.0 : 0.0;
      }
    }
    if (yaml["occupied_thresh"])
      occupied_thresh = yaml["occupied_thresh"].as<double>();
    if (yaml["free_thresh"])
      free_thresh = yaml["free_thresh"].as<double>();
  }
  catch (const std::exception& ex)
  {
    throw std::runtime_error(
      "[" + PKG_NAME + "] An optional key in '" + filename +
      "' is malformed: " + ex.what());
  }

  std::string image_path = image_key;
  if (!image_path.empty() && image_path.front() != '/')
  {
    const size_t slash = filename.find_last_of('/');
    const std::string dir =
      (slash == std::string::npos) ? "." : filename.substr(0, slash);
    image_path = dir + "/" + image_key;
  }

  unsigned int width = 0;
  unsigned int height = 0;
  std::vector<unsigned char> pixels;
  readPGM(image_path, &width, &height, &pixels);

  nav_msgs::OccupancyGrid map_msg;
  map_msg.header.frame_id = fixed_frame_id_;
  map_msg.info.resolution = resolution;
  map_msg.info.width = width;
  map_msg.info.height = height;
  map_msg.info.origin.position.x = origin[0];
  map_msg.info.origin.position.y = origin[1];
  map_msg.info.origin.position.z = 0.0;

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, origin[2]);
  q.normalize();
  tf::quaternionTFToMsg(q, map_msg.info.origin.orientation);

  map_msg.data.resize(width * height);
  for (unsigned int row = 0; row < height; row++)
  {
    const unsigned int src_row = height - 1 - row;
    for (unsigned int col = 0; col < width; col++)
    {
      const unsigned char value = pixels[src_row * width + col];
      const double shade = (negate != 0.0) ?
        value / 255.0 : (255 - value) / 255.0;

      if (shade > occupied_thresh)
        map_msg.data[row * width + col] = 100;
      else if (shade < free_thresh)
        map_msg.data[row * width + col] = 0;
      else
        map_msg.data[row * width + col] = -1;
    }
  }

  return map_msg;
}

/*******************************************************************************
 * @brief Stores the map upon receipt. (The map does not change through time)
 * @param[in] map_msg [const nav_msgs::OccupancyGrid] The map
 * @return void
 */
  void
CBGL::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
  if (map_hyp_ != NULL)
  {
    map_free(map_hyp_);
    map_hyp_ = NULL;
  }

  if (pf_hyp_ != NULL)
  {
    pf_free(pf_hyp_);
    pf_hyp_ = NULL;
  }

  map_hyp_ = convertMap(map_msg);

  double area = freeArea(map_hyp_);

  unsigned int min_particles_ = dl_ * area;
  unsigned int max_particles_ = dl_ * area;

  pf_hyp_ = pf_alloc(min_particles_, max_particles_, 0.001, 0.1,
    (pf_init_model_fn_t)CBGL::uniformPoseGenerator, (void *)map_hyp_);
  pf_hyp_->pop_err = 0.01;
  pf_hyp_->pop_z = 0.99;

  pf_init(pf_hyp_, pf_vector_zero(), pf_matrix_zero());

  map_ = map_msg;
  map_res_ = map_.info.resolution;
  map_hgt_ = map_.info.height;
  map_yaw_ = extractYawFromPose(map_.info.origin);
  map_sin_yaw_ = sin(map_yaw_);
  map_cos_yaw_ = cos(map_yaw_);

  std::vector<unsigned char> converted(4*map_msg.data.size());
  convertMapToPNG(map_msg, converted.data());
  omap_ = ranges::OMap(converted.data(),
    map_msg.info.width, map_msg.info.height);

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
  const double span = num_rays * (angle_max - angle_min) / (num_rays - 1);
  double v = num_rays * new_range / span;
  int v_int = static_cast<int>(v);

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
  dispersed_particles_.clear();

  for (int i = 0; i < pose_cloud_msg->poses.size(); i++)
  {
    geometry_msgs::Pose::Ptr pose_i =
      boost::make_shared<geometry_msgs::Pose>(pose_cloud_msg->poses[i]);

    dispersed_particles_.push_back(pose_i);
  }

  ROS_INFO("[%s] There are %zux%d particles in total",
    PKG_NAME.c_str(), dispersed_particles_.size(), da_);

  received_pose_cloud_ = true;
  if (received_scan_ && received_map_ && received_start_signal_)
    processPoseCloud();
}

/*******************************************************************************
 * @brief
 * @return void
 */
/**
 * @details The per candidate result and its transform are set before the call
 * that fills them rather than after. `handleInputPose` writes into neither
 * where no refinement is selected, and a valid flag left as whatever the stack
 * held then decided whether to apply a transform that was never set either.
 * A refusal and the identity are what "nothing refined this" means, and they
 * are what the two gates further down read. Setting `do_icp` and `do_fsm` both
 * false is what `params_cbgl.yaml` has always said selects no refinement, and
 * this is what makes that true.
 * @remark Every field of the result is set, including the three the answer
 * would arrive in, so that nothing later read is read from the stack whether
 * or not this build ever reads it today.
 */
void CBGL::processPoseCloud()
{
  ros::Time start = ros::Time::now();

  bool cond = received_scan_ &&
    received_pose_cloud_     &&
    received_start_signal_   &&
    received_map_;

  if (!cond)
    return;

  initRangeLibRayCasters();

  if (publish_pose_sets_)
  {
    geometry_msgs::PoseArray pa1;
    pa1.header.stamp = latest_world_scan_->header.stamp;
    std::vector<geometry_msgs::Pose> pv1;
    for (unsigned int i = 0; i < dispersed_particles_.size(); i++)
      pv1.push_back(*dispersed_particles_[i]);
    pa1.poses = pv1;
    all_hypotheses_publisher_.publish(pa1);
  }

  std::vector<geometry_msgs::Pose::Ptr> caer_best_particles =
    siftThroughCAERPanoramic(dispersed_particles_);

  ROS_INFO("[%s] I will consider only %zu hypotheses", PKG_NAME.c_str(),
    caer_best_particles.size());

  if (publish_pose_sets_)
  {
    geometry_msgs::PoseArray pa2;
    pa2.header.stamp = latest_world_scan_->header.stamp;
    std::vector<geometry_msgs::Pose> pv2;
    for (unsigned int i = 0; i < caer_best_particles.size(); i++)
      pv2.push_back(*caer_best_particles[i]);
    pa2.poses = pv2;
    top_caer_hypotheses_publisher_.publish(pa2);
  }

  std::vector<sm_result> outputs;
  std::vector<tf::Transform> f2bs;
  double score = -1.0;
  double score_i = -1.0;
  int score_idx = -1;
  for (int i = 0; i < caer_best_particles.size(); i++)
  {
    ROS_INFO(
      "[%s] Processing hypothesis %d: (x,y,t) = (%f,%f,%f)", PKG_NAME.c_str(), i,
      caer_best_particles[i]->position.x,
      caer_best_particles[i]->position.y,
      extractYawFromPose(*caer_best_particles[i]));

    sm_result output;
    output.valid = 0;
    output.iterations = 0;
    output.nvalid = 0;
    output.error = 0.0;
    output.x[0] = 0.0;
    output.x[1] = 0.0;
    output.x[2] = 0.0;
    output.cov_x_m = 0;
    output.dx_dy1_m = 0;
    output.dx_dy2_m = 0;

    tf::Transform f2b;
    f2b.setIdentity();

    handleInputPose(caer_best_particles[i], &output, &f2b);

    outputs.push_back(output);
    f2bs.push_back(f2b);


    bool icps_score = false;
    if (icps_score)
      score_i = output.valid * output.nvalid / fabs(output.error);
    else
    {
      geometry_msgs::Pose::Ptr corrected_pose =
        boost::make_shared<geometry_msgs::Pose>(*caer_best_particles[i]);
      correctICPPose(corrected_pose, f2b);

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

  ROS_INFO("[%s] ---------------------",   PKG_NAME.c_str());
  ROS_INFO("[%s] Best hypothesis id = %d", PKG_NAME.c_str(), score_idx);
  ROS_INFO("[%s] ---------------------",   PKG_NAME.c_str());

  if (score_idx < 0)
  {
    ROS_ERROR("[%s] Every hypothesis was refused; no pose is published",
      PKG_NAME.c_str());
    return;
  }

  geometry_msgs::Pose::Ptr global_pose =
    boost::make_shared<geometry_msgs::Pose>(*caer_best_particles[score_idx]);

  if (outputs[score_idx].valid == 1)
  {
    correctICPPose(global_pose, f2bs[score_idx]);

    ROS_INFO("[%s] Pose found: (%f,%f,%f)",
      PKG_NAME.c_str(),
      global_pose->position.x,
      global_pose->position.y,
      extractYawFromPose(*global_pose));
    ROS_INFO("--------------------------------------------------------------------------");
  }
  else
    ROS_ERROR("[%s] No valid pose found", PKG_NAME.c_str());

  measureExecutionTime(start, ros::Time::now());

  geometry_msgs::PoseWithCovarianceStamped global_pose_wcs;
  global_pose_wcs.pose.pose = *global_pose;

  boost::array<double,36> covar;
  covar[6*0+0] = initial_cov_xx_;
  covar[6*1+1] = initial_cov_yy_;
  covar[6*5+5] = initial_cov_aa_;
  global_pose_wcs.pose.covariance =  covar;
  global_pose_wcs.header.stamp = latest_world_scan_->header.stamp;
  global_pose_wcs.header.frame_id = fixed_frame_id_;
  global_pose_publisher_.publish(global_pose_wcs);

  if (tf_broadcast_)
    broadcast_global_pose_tf(global_pose_wcs.pose.pose);

  std_msgs::UInt64 np_msg;
  if (outputs[score_idx].valid == 1)
    np_msg.data = score_idx;
  else
    np_msg.data = -1;
  best_particle_publisher_.publish(np_msg);

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

  sm_icp(&input_, &output_);

  if (output_.valid)
  {
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    f2b_ = base_to_laser_ * corr_ch_l * laser_to_base_;
  }
  else
  {
    f2b_.setIdentity();
    ROS_WARN("[%s] Error in scan matching", PKG_NAME.c_str());
  }

  *output = output_;
  *f2b = f2b_;

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("[%s] Scan matcher total duration: %.1f ms", PKG_NAME.c_str(), dur);
}

/*******************************************************************************
 * @details Supports the two PGM variants, `P2` (ASCII) and `P5` (binary).
 * Comments starting with `#` are skipped wherever whitespace is allowed, as
 * the format permits. A sample is rescaled to the [0,255] range when the
 * file's own maximum value differs from 255, so that callers can always
 * treat the output as an ordinary 8-bit greyscale image. Rows are returned
 * in the order they appear in the file, i.e. the first row decoded is the
 * top of the image; flipping rows so that the map's origin is at the
 * bottom is the caller's job, not this one's.
 */
  void
CBGL::readPGM(
  const std::string& filename,
  unsigned int* width,
  unsigned int* height,
  std::vector<unsigned char>* pixels) const
{
  struct stat image_stat;
  if (stat(filename.c_str(), &image_stat) != 0)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Could not read image file '" + filename + "'");
  if (!S_ISREG(image_stat.st_mode))
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' is not an ordinary "
      "file, and this reads a PGM from a file rather than from a device or a "
      "pipe");
  const uint64_t image_bytes = static_cast<uint64_t>(image_stat.st_size);
  if (image_bytes > kMaxPGMBytes)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' is " +
      std::to_string(image_bytes) + " bytes, above the largest this reads, " +
      std::to_string(kMaxPGMBytes));

  std::ifstream file(filename.c_str(), std::ios::binary);
  if (!file)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Could not open image file '" + filename + "'");

  auto skip_ignorable = [&file]()
  {
    for (;;)
    {
      const int c = file.peek();
      if (c == '#')
      {
        std::string line;
        std::getline(file, line);
      }
      else if (std::isspace(c))
        file.get();
      else
        break;
    }
  };

  auto next_token = [&file, &skip_ignorable]() -> std::string
  {
    skip_ignorable();
    std::string token;
    file >> token;
    return token;
  };

  const std::string magic = next_token();
  if (magic != "P2" && magic != "P5")
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename +
      "' is not a PGM file of either the 'P2' or 'P5' variant");

  auto header_number = [&filename, this](const std::string& token,
    const std::string& what) -> uint64_t
  {
    if (token.empty() || token.find_first_not_of("0123456789") !=
      std::string::npos)
      throw std::runtime_error(
        "[" + PKG_NAME + "] Image file '" + filename + "' declares a " + what +
        " of '" + token + "', which is not a whole number");

    uint64_t value = 0;
    for (size_t i = 0; i < token.size(); i++)
    {
      value = value * 10 + static_cast<uint64_t>(token[i] - '0');
      if (value > kMaxPGMDimension)
        throw std::runtime_error(
          "[" + PKG_NAME + "] Image file '" + filename + "' declares a " +
          what + " above the largest this reads, " +
          std::to_string(kMaxPGMDimension));
    }
    return value;
  };

  const uint64_t w64 = header_number(next_token(), "width");
  const uint64_t h64 = header_number(next_token(), "height");
  const uint64_t maxval64 = header_number(next_token(), "maxval");

  if (w64 == 0 || h64 == 0)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' declares a size of " +
      std::to_string(w64) + " by " + std::to_string(h64) +
      ", and an image with no cell describes no map");
  if (maxval64 == 0 || maxval64 > 65535)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' declares a maxval of " +
      std::to_string(maxval64) + ", and a PGM's is between 1 and 65535");

  const uint64_t cells = w64 * h64;
  if (cells > kMaxPGMCells)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' declares " +
      std::to_string(cells) + " cells, above the largest this reads, " +
      std::to_string(kMaxPGMCells));

  const uint64_t bytes_left = image_bytes -
    std::min<uint64_t>(image_bytes, static_cast<uint64_t>(file.tellg()));
  const uint64_t bytes_needed = (magic == "P5")
    ? cells * ((maxval64 < 256) ? 1 : 2) : cells * 2;
  if (bytes_needed > bytes_left)
    throw std::runtime_error(
      "[" + PKG_NAME + "] Image file '" + filename + "' declares " +
      std::to_string(cells) + " cells, which need at least " +
      std::to_string(bytes_needed) + " bytes, and carries " +
      std::to_string(bytes_left) + " after its header");

  const unsigned int w = static_cast<unsigned int>(w64);
  const unsigned int h = static_cast<unsigned int>(h64);
  const unsigned int maxval = static_cast<unsigned int>(maxval64);

  std::vector<unsigned int> raw(static_cast<size_t>(cells));

  if (magic == "P2")
  {
    for (unsigned int i = 0; i < w * h; i++)
    {
      const std::string sample = next_token();
      if (sample.empty())
        throw std::runtime_error(
          "[" + PKG_NAME + "] Image file '" + filename +
          "' ends before its declared size");
      if (sample.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error(
          "[" + PKG_NAME + "] Image file '" + filename + "' carries '" +
          sample + "' where a sample belongs, which is not a whole number");
      raw[i] = std::stoul(sample);
      if (raw[i] > maxval)
        throw std::runtime_error(
          "[" + PKG_NAME + "] Image file '" + filename + "' has a sample of "
          + std::to_string(raw[i]) + " above its declared maxval of " +
          std::to_string(maxval));
    }
  }
  else  /* P5 */
  {
    const int separator = file.get();
    if (separator == EOF || !std::isspace(separator))
      throw std::runtime_error(
        "[" + PKG_NAME + "] Image file '" + filename + "' does not separate "
        "its header from its data with a whitespace character");
    if (separator == '\r' && file.peek() == '\n')
      file.get();

    const unsigned int bytes_per_sample = (maxval < 256) ? 1 : 2;
    std::vector<unsigned char> buf(w * h * bytes_per_sample);
    file.read(reinterpret_cast<char*>(buf.data()), buf.size());
    if (!file)
      throw std::runtime_error(
        "[" + PKG_NAME + "] Image file '" + filename +
        "' ends before its declared size");

    for (unsigned int i = 0; i < w * h; i++)
    {
      if (bytes_per_sample == 1)
        raw[i] = buf[i];
      else
        raw[i] = (static_cast<unsigned int>(buf[2*i]) << 8) | buf[2*i+1];

      if (raw[i] > maxval)
        throw std::runtime_error(
          "[" + PKG_NAME + "] Image file '" + filename + "' has a sample of "
          + std::to_string(raw[i]) + " above its declared maxval of " +
          std::to_string(maxval));
    }
  }

  pixels->resize(w * h);
  for (unsigned int i = 0; i < w * h; i++)
    (*pixels)[i] = static_cast<unsigned char>(raw[i] * 255.0 / maxval + 0.5);

  *width = w;
  *height = h;
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
  if (scan_msg->ranges.empty())
  {
    ROS_WARN("[%s] A scan carrying no ranges was received and dropped",
      PKG_NAME.c_str());
    return;
  }

  if (!std::isfinite(scan_msg->ranges[0]))  /* premature scan */
    return;

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

    s_->ranges = uranges;
    s_->angle_increment = undersample_rate_ * scan_msg->angle_increment;
  }

  nrays_ = s_->ranges.size();
  angle_inc_ = s_->angle_increment;
  angle_min_ = s_->angle_min;
  input_.min_reading = s_->range_min;
  input_.max_reading = s_->range_max;


  latest_world_scan_ =
    boost::make_shared<sensor_msgs::LaserScan>(*s_);

  for (unsigned int i = 0; i < s_->ranges.size(); i++)
  {
    if (!std::isfinite(s_->ranges[i]))
      latest_world_scan_->ranges[i] = 0;
  }
  double latest_world_scan_min_range_now =
    *min_element(s_->ranges.begin(), s_->ranges.end());

  if (latest_world_scan_min_range_now <= latest_world_scan_->range_min)
    latest_world_scan_->ranges = interpolateRanges(latest_world_scan_->ranges,
      latest_world_scan_->range_min);


  if (!received_scan_)
  {
    if (!getBaseToLaserTf(s_->header.frame_id))
    {
      ROS_WARN("[%s] Skipping scan", PKG_NAME.c_str());
      return;
    }

    if (do_fsm_)
      cacheFFTW3Plans(s_->ranges.size());
  }

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

  const geometry_msgs::Pose current_laser_pose =
    getCurrentLaserPose(*robot_pose);

  sensor_msgs::LaserScan::Ptr laser_scan_info =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  sensor_msgs::LaserScan::Ptr map_scan;


  if (scan_method.compare("vanilla") == 0)
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }
  else if (scan_method.compare("ray_marching") == 0 ||
    scan_method.compare("bresenham")    == 0 ||
    scan_method.compare("cddt")         == 0)
  {
    const double dx =
      current_laser_pose.position.x - map_.info.origin.position.x;
    const double dy =
      current_laser_pose.position.y - map_.info.origin.position.y;

    float x = (map_cos_yaw_ * dx + map_sin_yaw_ * dy) / map_.info.resolution;
    float y = map_.info.height - 1 -
      (-map_sin_yaw_ * dx + map_cos_yaw_ * dy) / map_.info.resolution;
    float a = extractYawFromPose(current_laser_pose) - map_yaw_;

    double min_a = laser_scan_info->angle_min;
    double inc_a = laser_scan_info->angle_increment;

    map_scan = boost::make_shared<sensor_msgs::LaserScan>(*laser_scan_info);
    map_scan->ranges.clear();

    int sgn = 0;
    if (laser_z_orientation_.compare("upwards") == 0)
      sgn = -1;
    else if (laser_z_orientation_.compare("downwards") == 0)
      sgn = 1;
    else
    {
      ROS_ERROR("[%s] Please provide a valid value", PKG_NAME.c_str());
      ROS_ERROR("       for param laser_z_orientation");
    }

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


  if (scan_method.compare("vanilla") == 0)
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
      *laser_scan_info);
  }
  else if (scan_method.compare("ray_marching") == 0 ||
    scan_method.compare("bresenham")    == 0 ||
    scan_method.compare("cddt")         == 0)
  {
    const double dx =
      current_laser_pose.position.x - map_.info.origin.position.x;
    const double dy =
      current_laser_pose.position.y - map_.info.origin.position.y;

    float x = (map_cos_yaw_ * dx + map_sin_yaw_ * dy) / map_.info.resolution;
    float y = map_.info.height - 1 -
      (-map_sin_yaw_ * dx + map_cos_yaw_ * dy) / map_.info.resolution;
    float a = extractYawFromPose(current_laser_pose) - map_yaw_;

    int num_rays = numRaysFromAngleRange(latest_world_scan_->angle_min,
      latest_world_scan_->angle_max, nrays_, 2*M_PI);


    double min_a = M_PI;
    double inc_a = laser_scan_info->angle_increment;

    map_scan = boost::make_shared<sensor_msgs::LaserScan>(*laser_scan_info);
    map_scan->ranges.clear();

    int sgn = 0;
    if (laser_z_orientation_.compare("upwards") == 0)
      sgn = -1;
    else if (laser_z_orientation_.compare("downwards") == 0)
      sgn = 1;
    else
    {
      ROS_ERROR("[%s] Please provide a valid value", PKG_NAME.c_str());
      ROS_ERROR("       for param laser_z_orientation");
    }

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
  std::vector<geometry_msgs::Pose::Ptr> all_hypotheses;

  std::vector<double> caers;
  std::vector<double> scan_map_times;
  sensor_msgs::LaserScan::Ptr sv_i;
  for (unsigned int i = 0; i < init_hypotheses.size(); i++)
  {
    sv_i = scanMapPanoramic(init_hypotheses[i], map_scan_method_);

    const double yaw_i = extractYawFromPose(*init_hypotheses[i]);
    double new_yaw_i = 0.0;
    tf::Quaternion q;

    const std::vector<float> r_i = sv_i->ranges;
    const size_t num_rays = r_i.size();


    if (num_rays < nrays_)
    {
      ROS_ERROR("[%s] A cast of %zu rays cannot serve a scan of %u; skipping "
        "hypothesis %u", PKG_NAME.c_str(), num_rays, nrays_, i);
      continue;
    }

    const unsigned int prefix = round(
      (static_cast<double>(num_rays) - static_cast<double>(nrays_)) / 2);

    std::vector<float> r_ik;
    geometry_msgs::Pose::Ptr pose_ik;
    for (unsigned int k = 0; k < da_; k++)
    {
      const unsigned int shift = round(
        static_cast<double>(k) * num_rays / da_);

      r_ik = r_i;
      std::rotate(r_ik.begin(), r_ik.begin()+shift, r_ik.end());
      std::vector<float>::const_iterator a = r_ik.begin() + prefix;
      std::vector<float>::const_iterator b = a + nrays_;


      std::vector<float> r_ikf(a,b);

      pose_ik = boost::make_shared<geometry_msgs::Pose>(*init_hypotheses[i]);

      new_yaw_i = yaw_i +
        2*M_PI * static_cast<double>(shift)/static_cast<double>(num_rays);
      wrapAngle(new_yaw_i);
      q.setRPY(0.0, 0.0, new_yaw_i); q.normalize();
      tf::quaternionTFToMsg(q, pose_ik->orientation);

      all_hypotheses.push_back(pose_ik);


      double c = caer(latest_world_scan_->ranges, r_ikf);
      if (c > 0.0)
        caers.push_back(c);
      else  /* THIS IS NECESSARY */
        caers.push_back(std::numeric_limits<double>::max());

    }
  }

  std::vector<size_t> idx(caers.size());
  std::iota(idx.begin(), idx.end(), 0);

  const unsigned int kept =
    std::min(static_cast<size_t>(top_k_caers_), idx.size());

  std::nth_element(
    idx.begin(),
    idx.begin() + kept,
    idx.end(),
    [&caers](size_t i1,size_t i2) {return caers[i1] < caers[i2];});


  std::vector<geometry_msgs::Pose::Ptr> caer_best_particles;
  for (unsigned int i = 0; i < kept; i++)
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
    ROS_WARN("[%s] Map not received yet, returning ...", PKG_NAME.c_str());
    return false;
  }

  ROS_INFO("--------------------------------------------------------------------------");
  ROS_INFO("[%s] Initializing with uniform distribution over map", PKG_NAME.c_str());
  pf_init_model(pf_hyp_, (pf_init_model_fn_t)CBGL::uniformPoseGenerator,
    (void *)map_hyp_);

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

  }

  if (publish_pose_sets_)
    all_hypotheses_publisher_.publish(*pose_cloud_msg);

  poseCloudCallback(pose_cloud_msg);

  ROS_INFO("[%s] Global initialisation done!", PKG_NAME.c_str());
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

  const double half_x = (map->size_x * map->scale)/2.0;
  const double half_y = (map->size_y * map->scale)/2.0;

  pf_vector_t p;

  for(;;)
  {
    const double offset_x = -half_x + drand48() * 2*half_x;
    const double offset_y = -half_y + drand48() * 2*half_y;

    p.v[0] = map->origin_x
      + map->origin_cos_yaw * offset_x - map->origin_sin_yaw * offset_y;
    p.v[1] = map->origin_y
      + map->origin_sin_yaw * offset_x + map->origin_cos_yaw * offset_y;
    p.v[2] = drand48() * 2*M_PI - M_PI;

    int i,j;
    i = MAP_GXWX(map, p.v[0], p.v[1]);
    j = MAP_GYWY(map, p.v[0], p.v[1]);
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
  double yaw = extractYawFromPose(*pose);

  wrapAngle(yaw);

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  tf::quaternionTFToMsg(q, pose->orientation);
}
