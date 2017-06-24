/*
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/*
 * Copyright (c) 2017, Beijing SmartConn Tech. Co.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Author: Brian Gerkey */
/* Modified by: Charles DuHadway */
/* Modified by: Wei Ren */

#include "gmapping_enhanced/slam_gmapping.h"

#include <cmath>
#include <queue>
#include <time.h>

#include <ros/console.h>
#include <nav_msgs/MapMetaData.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;

// Compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamGMapping::SlamGMapping():
    m_privateNodeHandle("~"),
    m_mapToOdomTf(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))),
    m_samplingSeed(time(nullptr)),
    m_cntLaserScans(0),
    m_gotFirstLaserScan(false),
    m_gotMap(false),
    m_lastMapUpdateTime(0, 0)
{
    Init();
}

SlamGMapping::SlamGMapping(ros::NodeHandle& nh, ros::NodeHandle& pnh):
    m_nodeHandle(nh), 
    m_privateNodeHandle(pnh),
    m_mapToOdomTf(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))),
    m_samplingSeed(time(nullptr)),
    m_cntLaserScans(0),
    m_gotFirstLaserScan(false),
    m_gotMap(false),
    m_lastMapUpdateTime(0, 0)
{
    Init();
}

SlamGMapping::SlamGMapping(long unsigned int seed, long unsigned int maxDurationBuffer):
    m_privateNodeHandle("~"),
    m_tfListener(ros::Duration(maxDurationBuffer)),
    m_mapToOdomTf(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Point(0, 0, 0))),
    m_samplingSeed(seed),
    m_cntLaserScans(0),
    m_gotFirstLaserScan(false),
    m_gotMap(false),
    m_lastMapUpdateTime(0, 0)
{
    Init();
}

SlamGMapping::~SlamGMapping()
{
    if (m_tfTransformPublishThread)
    {
        m_tfTransformPublishThread->join();
        m_tfTransformPublishThread.reset();
    }
}

void SlamGMapping::Init()
{
    // The GMapping library is pretty chatty.
    m_gsp.reset(new GMapping::GridSlamProcessor());
    ROS_ASSERT(m_gsp);

    // Parameters used by our GMapping wrapper:
    m_privateNodeHandle.param("throttle_scans", m_throttleScans, 1);
    m_privateNodeHandle.param("base_frame", m_baseFrame, string("base_link"));
    m_privateNodeHandle.param("map_frame", m_mapFrame, string("map"));
    m_privateNodeHandle.param("odom_frame", m_odomFrame, string("odom"));
    
    double tmp = 0.0;
    m_privateNodeHandle.param("map_update_interval", tmp, 5.0);
    m_mapUpdateInterval.fromSec(tmp);

    m_privateNodeHandle.param("transform_publish_period", m_tfTransformPublishPeriod, 0.05);
    m_privateNodeHandle.param("tf_delay", m_tfDelay, m_tfTransformPublishPeriod);
    m_privateNodeHandle.param("occ_thresh", m_mapOccupancyThreshold, 0.25);
    
    // Parameters used by the openslam_gmapping library:
    // Preliminary default, will be set in InitGMapping() when the first laser scan is received.
    m_maxRange = 0.0;
    m_maxURange = 0.0;
    m_privateNodeHandle.param("sigma", m_sigma, 0.05);
    m_privateNodeHandle.param("kernelSize", m_kernelSize, 1);
    m_privateNodeHandle.param("lstep", m_lstep, 0.05);
    m_privateNodeHandle.param("astep", m_astep, 0.05);
    m_privateNodeHandle.param("iterations", m_iterations, 5);
    m_privateNodeHandle.param("lsigma", m_lsigma, 0.075);
    m_privateNodeHandle.param("ogain", m_ogain, 3.0);
    m_privateNodeHandle.param("lskip", m_lskip, 0);
    m_privateNodeHandle.param("minimumScore", m_minimumScore);
    m_privateNodeHandle.param("llsamplerange", m_llSampleRange, 0.01);
    m_privateNodeHandle.param("llsamplestep", m_llSampleStep, 0.01);
    m_privateNodeHandle.param("lasamplerange", m_laSampleRange, 0.005);
    m_privateNodeHandle.param("lasamplestep", m_laSampleStep, 0.005);
    m_privateNodeHandle.param("srr", m_srr, 0.1);
    m_privateNodeHandle.param("srt", m_srt, 0.2);
    m_privateNodeHandle.param("str", m_str, 0.1);
    m_privateNodeHandle.param("stt", m_stt, 0.2);
    m_privateNodeHandle.param("linearUpdate", m_linearUpdate, 1.0);
    m_privateNodeHandle.param("angularUpdate", m_angularUpdate, 0.5);
    m_privateNodeHandle.param("temporalUpdate", m_temporalUpdate, -1.0);
    m_privateNodeHandle.param("resampleThreshold", m_resampleThreshold, 0.5);
    m_privateNodeHandle.param("particles", m_cntParticles, 30);
    m_privateNodeHandle.param("xmin", m_initMapXMin, -100.0);
    m_privateNodeHandle.param("ymin", m_initMapYMin, -100.0);
    m_privateNodeHandle.param("xmax", m_initMapXMax, 100.0);
    m_privateNodeHandle.param("ymax", m_initMapYMax, 100.0);
    m_privateNodeHandle.param("delta", m_mapResolution, 0.05);
}

void SlamGMapping::StartLiveSlam()
{
    // Create the publishers.
    m_privateEntropyPublisher = m_privateNodeHandle.advertise<std_msgs::Float64>("entropy", 1, true);
    m_mapMetaDataPublisher = m_nodeHandle.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    m_mapPublisher = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    
    // Create the subscribers.
    m_laserScanSubscriber.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(m_nodeHandle, "scan", 5));
    m_laserScanTfFilter.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(
        *m_laserScanSubscriber, m_tfListener, m_odomFrame, 5));
    m_laserScanTfFilter->registerCallback(bind(&SlamGMapping::LaserCallback, this, _1));

    // Create the service.
    m_getMapService = m_nodeHandle.advertiseService("dynamic_map", &SlamGMapping::MapCallback, this);

    // Start the thread for publishing the map-to-odom transform.
    m_tfTransformPublishThread.reset(new thread(
        bind(&SlamGMapping::PublishTfTransformLoop, this, m_tfTransformPublishPeriod)));
}

void SlamGMapping::StartReplay(const string& bagFileName, const string& scanTopic)
{
    // Create the publishers.
    m_privateEntropyPublisher = m_privateNodeHandle.advertise<std_msgs::Float64>("entropy", 1, true);
    m_mapMetaDataPublisher = m_nodeHandle.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
    m_mapPublisher = m_nodeHandle.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    
    // Create the service.
    m_getMapService = m_nodeHandle.advertiseService("dynamic_map", &SlamGMapping::MapCallback, this);
    
    // Open the bag file.
    rosbag::Bag bag;
    bag.open(bagFileName, rosbag::bagmode::Read);
    
    // Create a view of the bag file which only contains the TF transforms and the laser scan messages.
    vector<string> topics{string("/tf"), scanTopic};
    rosbag::View tfAndScanView(bag, rosbag::TopicQuery(topics));

    // Store up to 5 laser scan messages. If the oldest message cannot be processed right away, 
    // also store the error message.
    queue<pair<sensor_msgs::LaserScan::ConstPtr, string> > msgQueue;
    for (rosbag::MessageInstance const msg : tfAndScanView)
    {
        tf::tfMessage::ConstPtr tfMsg = msg.instantiate<tf::tfMessage>();
        if (tfMsg != nullptr) 
        {
            for (size_t transformIndex = 0; transformIndex < tfMsg->transforms.size(); ++transformIndex)
            {
                geometry_msgs::TransformStamped transformStamped;
                tf::StampedTransform stampedTf;
                transformStamped = tfMsg->transforms[transformIndex];
                tf::transformStampedMsgToTF(transformStamped, stampedTf);
                m_tfListener.setTransform(stampedTf);
            }
        }

        sensor_msgs::LaserScan::ConstPtr scanMsg = msg.instantiate<sensor_msgs::LaserScan>();
        if (scanMsg != nullptr) 
        {
            // Ignore un-timestamped laser scan.
            if (!((ros::Time(scanMsg->header.stamp)).is_zero()))
            {
                msgQueue.push(make_pair(scanMsg, string("")));
            }

            // Just like in live processing, only process the latest 5 laser scans.
            if (msgQueue.size() > 5) 
            {
                ROS_WARN("Dropping the oldest scan with an possible error message: %s", 
                    msgQueue.front().second.c_str());
                msgQueue.pop();
            }
        }

        // Only process a scan if it has the corresponding TF transform.
        while (!msgQueue.empty())
        {
            try
            {
                // Check if the corresponding odom-to-laser TF transform is ready. 
                // If not, lookupTransform will raise an exception.
                tf::StampedTransform stampedTf;
                m_tfListener.lookupTransform(
                    msgQueue.front().first->header.frame_id, 
                    m_odomFrame, 
                    msgQueue.front().first->header.stamp, 
                    stampedTf);
                
                // Invoke the callback to process the laser scan message and do SLAM.
                this->LaserCallback(msgQueue.front().first);
                msgQueue.pop();
            }
            catch (tf2::TransformException& e)
            {
                // Store the error to display it later if we drop the oldest laser scan when 
                // the message queue accumulates more than 5 laser scan messages.
                msgQueue.front().second = string(e.what());
                break;
            }
        }
    }

    // Close the bag file.
    bag.close();
}

void SlamGMapping::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    m_cntLaserScans++;
    if ((m_cntLaserScans % m_throttleScans) != 0)
    {
        return;
    }

    // We can't initialize the mapper until we've got the first scan.
    if (!m_gotFirstLaserScan)
    {
        if (!InitGMapping(*scan))
        {    
            return;
        }
        
        m_gotFirstLaserScan = true;
    }

    GMapping::OrientedPoint odomPose;
    if (AddScan(*scan, odomPose))
    {
        ROS_DEBUG("scan added and processed");

        // Get the centered laser beam pose in the map frame.
        GMapping::OrientedPoint mapPose = m_gsp->getParticles()[m_gsp->getBestParticleIndex()].pose;
        ROS_DEBUG("Best scan pose in the map frame: x = %.3f, y = %.3f, theta = %.3f", mapPose.x, mapPose.y, mapPose.theta);
        ROS_DEBUG("Scan pose in the odom frame: x = %.3f, y = %.3f, theta = %.3f", odomPose.x, odomPose.y, odomPose.theta);
        ROS_DEBUG("correction: dx = %.3f, dy = %.3f, dtheta = %.3f", 
            mapPose.x - odomPose.x, mapPose.y - odomPose.y, mapPose.theta - odomPose.theta);

        // Calculate the map-to-odom TF transform.
        tf::Transform mapToLaserTf(tf::createQuaternionFromRPY(0, 0, mapPose.theta), tf::Vector3(mapPose.x, mapPose.y, 0.0));
        tf::Transform odomToLaserTf(tf::createQuaternionFromRPY(0, 0, odomPose.theta), tf::Vector3(odomPose.x, odomPose.y, 0.0));

        {
            unique_lock<mutex> mapToOdomTfLock(m_mapToOdomMutex);
            m_mapToOdomTf = mapToLaserTf * odomToLaserTf.inverse();
        }

        // Update the grid map periodically.
        if (!m_gotMap || (scan->header.stamp - m_lastMapUpdateTime > m_mapUpdateInterval))
        {
            UpdateMap(*scan);
            m_lastMapUpdateTime = scan->header.stamp;
            ROS_DEBUG("Updated the map");
        }
    } 
    else
    {
        // This case occurs a lot, so set the log message level to DEBUG.
        ROS_DEBUG("Cannot process scan");
    }
}

bool SlamGMapping::InitGMapping(const sensor_msgs::LaserScan& scan)
{
    // Get the laser frame ID.
    m_laserFrame = scan.header.frame_id;
    
    // Get the laser's pose in the base frame.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laserPose;
    ident.setIdentity();
    ident.frame_id_ = m_laserFrame;
    ident.stamp_ = scan.header.stamp;
    try
    {
        m_tfListener.transformPose(m_baseFrame, ident, laserPose);
    }
    catch (tf::TransformException e)
    {
        ROS_WARN("Failed to compute laser's pose in the base frame, aborting initialization (%s)", e.what());
        return false;
    }

    // Create a point 1m above the laser position in the odom fram 
    // and then transform it into the laser frame.
    tf::Vector3 v;
    v.setValue(0.0, 0.0, 1.0 + laserPose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan.header.stamp, m_baseFrame);
    try
    {
        m_tfListener.transformPoint(m_laserFrame, up, up);
        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException& e)
    {
        ROS_WARN("Unable to determine orientation of laser: %s", e.what());
        return false;
    }
    
    // The Gmapping library doesn't take into account roll or pitch, so check if the LiDAR aligns with 
    // the 2D plane. If not, fails the initialization.
    if (abs(abs(up.z()) - 1.0) > 0.001)
    {
        ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
            up.z());
        return false;
    }

    // Read the two private parameters about the laser ranges. If they are not given as private 
    // parameters, read them from the first laser scan data.
    m_privateNodeHandle.param("maxRange", m_maxRange, scan.range_max - 0.01);
    m_privateNodeHandle.param("maxUrange", m_maxURange, m_maxRange);
    
    // Get the number of beams in one laser scan.
    m_cntLaserBeams = scan.ranges.size();

    // Get the angle of the centered beam in one laser scan, determine the orientation of the LiDAR 
    // and if the scan is done in a counterclockwise way. Note that if not, we say that the scan 
    // is done reversely.
    double centeredAngle = (scan.angle_min + scan.angle_max)/2.0;
    if (up.z() > 0.0)
    {
        ROS_INFO("Laser is mounted upwards.");
        
        m_doReverseRange = (scan.angle_min > scan.angle_max);
        m_centeredLaserPose = tf::Stamped<tf::Pose>(
            tf::Transform(tf::createQuaternionFromRPY(0.0, 0.0, centeredAngle), tf::Vector3(0.0, 0.0, 0.0)), 
            ros::Time::now(), 
            m_laserFrame);
    }
    else
    {
        ROS_INFO("Laser is mounted upside down.");
        
        m_doReverseRange = (scan.angle_min < scan.angle_max);
        m_centeredLaserPose = tf::Stamped<tf::Pose>(
            tf::Transform(tf::createQuaternionFromRPY(M_PI, 0.0, -centeredAngle), tf::Vector3(0.0, 0.0, 0.0)), 
            ros::Time::now(), 
            m_laserFrame);
    }

    // Compute the angles of the laser from -x to x, basically symmetric and in an increasing order.
    m_laserBeamAngles.resize(scan.ranges.size());
    // Make sure angles are started so that they are centered at centeredAngle.
    double theta = -abs(scan.angle_min - scan.angle_max)/2.0;
    for (size_t angleIndex = 0; angleIndex < scan.ranges.size(); ++angleIndex)
    {
        m_laserBeamAngles[angleIndex] = theta;
        theta += abs(scan.angle_increment);
    }

    ROS_DEBUG("Laser angles in laser-frame: min: %.3f, max: %.3f, inc: %.3f", 
        scan.angle_min, scan.angle_max, scan.angle_increment);
    ROS_DEBUG("Laser angles in top-down counterclockwise centered laser-frame: min: %.3f, max: %.3f, inc: %.3f", 
        m_laserBeamAngles.front(), m_laserBeamAngles.back(), abs(scan.angle_increment));

    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll reverse the order of ranges before
    // feeding each scan to GMapping.
    GMapping::OrientedPoint beamOriginPose(0.0, 0.0, 0.0);  
    m_gspLaser.reset(new GMapping::RangeSensor(
        "FLASER",        // The laser must be called "FLASER".
        m_cntLaserBeams,
        abs(scan.angle_increment),
        beamOriginPose,  // The pose of the beam origin relative to the center of the LiDAR
        0.0,             // "Span = 0" indicates a line-like beam, which is obviously true for laser.
        m_maxRange));
    ROS_ASSERT(m_gspLaser);

    // Insert the laser sensor into the sensor map of the GMapping library.
    GMapping::SensorMap smap;
    smap.insert(make_pair(m_gspLaser->getName(), m_gspLaser.get()));
    m_gsp->setSensorMap(smap);

    /// TODO: Expose an initial pose as a private parameter.
    // Note that although the initial pose is in the odom frame, it is identical to the pose in the 
    // map frame since the map frame superposes the odom frame at the beginning.
    GMapping::OrientedPoint initialPose;
    if (!GetCenteredLaserPoseInOdom(initialPose, scan.header.stamp))
    {
        ROS_WARN("Unable to determine inital pose of laser! The starting point of the trajectory will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    // Set the GMapping parameters.
    m_gsp->setMatchingParameters(
        m_maxURange, 
        m_maxRange, 
        m_sigma,
        m_kernelSize, 
        m_lstep, 
        m_astep, 
        m_iterations,
        m_lsigma, 
        m_ogain, 
        m_lskip);
    m_gsp->setMotionModelParameters(
        m_srr, 
        m_srt, 
        m_str, 
        m_stt);
    m_gsp->setUpdateDistances(
        m_linearUpdate, 
        m_angularUpdate, 
        m_resampleThreshold);
    m_gsp->setUpdatePeriod(m_temporalUpdate);
    m_gsp->setgenerateMap(false);

    m_gsp->GridSlamProcessor::init(
        m_cntParticles, 
        m_initMapXMin, 
        m_initMapYMin, 
        m_initMapXMax, 
        m_initMapYMax,
        m_mapResolution, 
        initialPose);
    m_gsp->setllsamplerange(m_llSampleRange);
    m_gsp->setllsamplestep(m_llSampleStep);
    /// TODO: Check these calls; in the Gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange. It was probably a typo, but who knows.
    m_gsp->setlasamplerange(m_laSampleRange);
    m_gsp->setlasamplestep(m_laSampleStep);
    m_gsp->setminimumScore(m_minimumScore);
    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, m_samplingSeed);

    ROS_INFO("GMapping initialization complete");

    return true;
}

bool SlamGMapping::GetCenteredLaserPoseInOdom(GMapping::OrientedPoint& odomPose, const ros::Time& t)
{
    // Get the pose of the centered laser at the right time.
    m_centeredLaserPose.stamp_ = t;
    
    // Get the centered laser's pose in the odom frame.
    tf::Stamped<tf::Transform> centeredLaserPoseInOdom;
    try
    {
        m_tfListener.transformPose(m_odomFrame, m_centeredLaserPose, centeredLaserPoseInOdom);
    }
    catch (tf::TransformException e)
    {
        ROS_WARN("Failed to compute the centered laser pose in the odom frame, skipping scan (%s)", 
            e.what());
        return false;
    }

    // The GMapping library can only do SLAM on a 2D plane, so it only needs a 2D pose.
    double yaw = tf::getYaw(centeredLaserPoseInOdom.getRotation());
    odomPose = GMapping::OrientedPoint(
        centeredLaserPoseInOdom.getOrigin().x(),
        centeredLaserPoseInOdom.getOrigin().y(),
        yaw);

    return true;
}

bool SlamGMapping::AddScan(const sensor_msgs::LaserScan& scan, GMapping::OrientedPoint& odomPose)
{
    if (!GetCenteredLaserPoseInOdom(odomPose, scan.header.stamp))
    {
        return false;
    }
    
    if (m_cntLaserBeams != scan.ranges.size())
    {
        return false;
    }

    // GMapping wants an array of doubles. Note that when we instantiate a laser reading, it deep copies rangesInDouble 
    // in RangeReading constructor, so we don't need to keep around our dynamic array after that.
    unique_ptr<double[]> rangesInDouble(new double[scan.ranges.size()]);
    
    // If the angle increment is negative, i.e., the laser scan is clockwise, 
    // we have to reverse the order of the readings.
    if (m_doReverseRange)
    {
        ROS_DEBUG("Reversing the scan");
        size_t cntRanges = scan.ranges.size();
        for (size_t rangeIndex = 0; rangeIndex < cntRanges; ++rangeIndex)
        {
            // Must filter out short readings, because the GMapping library won't.
            if (scan.ranges[cntRanges - rangeIndex - 1] < scan.range_min)
            {
                rangesInDouble[rangeIndex] = static_cast<double>(scan.range_max);
            }
            else
            {
                rangesInDouble[rangeIndex] = static_cast<double>(scan.ranges[cntRanges - rangeIndex - 1]);
            }
        }
    } 
    else 
    {
        for (size_t rangeIndex = 0; rangeIndex < scan.ranges.size(); ++rangeIndex)
        {
            // Must filter out short readings, because the GMapping library won't.
            if (scan.ranges[rangeIndex] < scan.range_min)
            {
                rangesInDouble[rangeIndex] = static_cast<double>(scan.range_max);
            }
            else
            {
                rangesInDouble[rangeIndex] = static_cast<double>(scan.ranges[rangeIndex]);
            }
        }
    }

    GMapping::RangeReading reading(
        scan.ranges.size(),
        rangesInDouble.get(),
        m_gspLaser.get(),
        scan.header.stamp.toSec());

    reading.setPose(odomPose);

    /*
    ROS_DEBUG("Scan pose in the odom frame at time = %.3f: x = %.3f, y = %.3f, theta = %.3f",
              scan.header.stamp.toSec(),
              odom.x,
              odom.y,
              odom.theta);
              */
    ROS_DEBUG("processing scan");

    return m_gsp->processScan(reading);
}

void SlamGMapping::UpdateMap(const sensor_msgs::LaserScan& scan)
{
    unique_lock<mutex> mapLock(m_mapMutex);
    ROS_DEBUG("Updating map");
    
    // Calculate and publish the entropy of the robot's pose.
    std_msgs::Float64 entropy;
    entropy.data = ComputePoseEntropy();
    if (entropy.data > 0.0)
    {
        m_privateEntropyPublisher.publish(entropy);
    }

    // Initialize the map metadata when we firstly create map.
    if (!m_gotMap) {
        m_map.map.info.resolution = m_mapResolution;
        m_map.map.info.origin.position.x = 0.0;
        m_map.map.info.origin.position.y = 0.0;
        m_map.map.info.origin.position.z = 0.0;
        m_map.map.info.origin.orientation.x = 0.0;
        m_map.map.info.origin.orientation.y = 0.0;
        m_map.map.info.origin.orientation.z = 0.0;
        m_map.map.info.origin.orientation.w = 1.0;
    } 

    // Initialize the map which will be updated by the scans.
    GMapping::Point center;
    center.x = (m_initMapXMin + m_initMapXMax)/2.0;
    center.y = (m_initMapYMin + m_initMapYMax)/2.0;
    GMapping::ScanMatcherMap smap(
        center, 
        m_initMapXMin, 
        m_initMapYMin, 
        m_initMapXMax, 
        m_initMapYMax, 
        m_mapResolution);
   
    // Create the scan matcher which will be used to update the map.
    GMapping::ScanMatcher matcher;
    matcher.setLaserParameters(
        scan.ranges.size(), 
        &(m_laserBeamAngles[0]),
        m_gspLaser->getPose());
    matcher.setlaserMaxRange(m_maxRange);
    matcher.setusableRange(m_maxURange);
    matcher.setgenerateMap(true);

    // Select the best particle and assume its trajectory poses as the robot poses 
    // to update the map with the associated laser scans.
    GMapping::GridSlamProcessor::Particle bestParticle =
        m_gsp->getParticles()[m_gsp->getBestParticleIndex()];
    ROS_DEBUG("Trajectory tree of the best particle:");
    for (auto trajectoryNode = bestParticle.node;
        trajectoryNode != nullptr;
        trajectoryNode = trajectoryNode->parent)
    {
        ROS_DEBUG("x = %.3f, y = %.3f, theta = %.3f",
            trajectoryNode->pose.x,
            trajectoryNode->pose.y,
            trajectoryNode->pose.theta);
       
        if (!trajectoryNode->reading)
        {
            ROS_DEBUG("Reading is NULL");
            continue;
        }

        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, trajectoryNode->pose, &((*trajectoryNode->reading)[0]));
        matcher.registerScan(smap, trajectoryNode->pose, &((*trajectoryNode->reading)[0]));
    }

    // The map may have been expanded, so adjust the map metadata accordingly.
    if (m_map.map.info.width != (unsigned int) smap.getMapSizeX() || 
        m_map.map.info.height != (unsigned int) smap.getMapSizeY()) 
    {
        // Note that due to the discretization and the hierachical 2D storage, some results of ScanMatcherMap::getSize() 
        // may be different from the parameters given to the constructor. Specifically,
        // (1) xmin and ymin should be the same.
        // (2) xmax and ymax returned by ScanMatcherMap::getSize() may be larger than the values given to the constructor.
        GMapping::Point bottomLeftPoint = smap.map2world(GMapping::IntPoint(0, 0));
        // Note that ScanMatcherMap::getSize() assumes (smap.getMapSizeX() - 1, smap.getMapSizeY() - 1) as the top right 
        // boundary point of the grid map, but we think that the correct value should be (smap.getMapSizeX(), smap.getMapSizeY()).
        GMapping::Point topRightPoint = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        m_initMapXMin = bottomLeftPoint.x; 
        m_initMapYMin = bottomLeftPoint.y;
        m_initMapXMax = topRightPoint.x; 
        m_initMapYMax = topRightPoint.y;
        
        ROS_DEBUG("map size is now %dx%d cells (%f, %f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
            m_initMapXMin, m_initMapYMin, m_initMapXMax, m_initMapYMax);

        m_map.map.info.width = smap.getMapSizeX();
        m_map.map.info.height = smap.getMapSizeY();
        // This position is the real-world pose of the cell (0, 0) in the map.
        m_map.map.info.origin.position.x = m_initMapXMin;
        m_map.map.info.origin.position.y = m_initMapYMin;
        m_map.map.data.resize(m_map.map.info.width * m_map.map.info.height);

        ROS_DEBUG("map origin: (%f, %f)", m_map.map.info.origin.position.x, m_map.map.info.origin.position.y);
    }

    // Set the occupancy value of each cell.
    for (int x = 0; x < smap.getMapSizeX(); ++x)
    {
        for (int y = 0; y < smap.getMapSizeY(); ++y)
        {
            GMapping::IntPoint p(x, y);
            // The occupancy value of a cell in the ScanMatcherMap is essentially a probability in the range [0, 1] 
            // if the cell has ever been "visited" by a laser beam.
            double occProb = smap.cell(p);
            assert(occProb <= 1.0);
            
            // The occupancy value of a cell in our map is in the range [0, 100] if the cell state is known, and 
            // -1 otherwise.
            if (occProb < 0)
            {
                // The cell has never been "visited" by a laser beam so its state is unknown.
                m_map.map.data[MAP_IDX(m_map.map.info.width, x, y)] = -1;
            }
            else if (occProb > m_mapOccupancyThreshold)
            {
                /// TODO: Multiply the exact occupancy probability by 100.0 to show the confidence of occupancy.
                /// m_map.map.data[MAP_IDX(m_map.map.info.width, x, y)] = static_cast<int>(round(occProb*100.0));
                m_map.map.data[MAP_IDX(m_map.map.info.width, x, y)] = 100;
            }
            else
            {
                m_map.map.data[MAP_IDX(m_map.map.info.width, x, y)] = 0;
            }
        }
    }

    m_gotMap = true;

    // Set the header information on the map.
    m_map.map.header.stamp = ros::Time::now();
    m_map.map.header.frame_id = m_tfListener.resolve(m_mapFrame);

    // Publish the map meta data and the map.
    m_mapMetaDataPublisher.publish(m_map.map.info);
    m_mapPublisher.publish(m_map.map);
}

double SlamGMapping::ComputePoseEntropy()
{
    double totalWeight = 0.0;
    for (auto it = m_gsp->getParticles().begin(); it != m_gsp->getParticles().end(); ++it)
    {
        totalWeight += it->weight;
    }

    double entropy = 0.0;
    for (auto it = m_gsp->getParticles().begin(); it != m_gsp->getParticles().end(); ++it)
    {
        if ((it->weight)/totalWeight > 0.0)
        {
            entropy += ((it->weight)/totalWeight)*log((it->weight)/totalWeight);
        }
    }

    return -entropy;
}

bool SlamGMapping::MapCallback(
    nav_msgs::GetMap::Request& request, 
    nav_msgs::GetMap::Response& response)
{
    unique_lock<mutex> mapLock(m_mapMutex);
    if (m_gotMap && m_map.map.info.width && m_map.map.info.height)
    {
        response = m_map;
        return true;
    }
    else
    {
        ROS_ERROR("m_gotMap is %d or map is empty (width = %d, height = %d)", 
            m_gotMap, m_map.map.info.width, m_map.map.info.height);
        return false;
    }
}

void SlamGMapping::PublishTfTransformLoop(double tfTransformPublishPeriod)
{
    if (tfTransformPublishPeriod == 0.0)
    {
        return;
    }

    ros::Rate tfPublishRate(1.0/tfTransformPublishPeriod);
    while (ros::ok())
    {
        PublishTfTransform();
        tfPublishRate.sleep();
    }
}

void SlamGMapping::PublishTfTransform()
{
    unique_lock<mutex> mapToOdomTfLock(m_mapToOdomMutex);

    ros::Time tfExpiration = ros::Time::now() + ros::Duration(m_tfDelay);
    m_tfBroadcaster.sendTransform(tf::StampedTransform(m_mapToOdomTf, tfExpiration, m_mapFrame, m_odomFrame));
}