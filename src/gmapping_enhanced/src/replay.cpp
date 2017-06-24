/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

/* Modified by: Wei Ren */

#include "gmapping_enhanced/slam_gmapping.h"

#include <string>

#include <boost/program_options.hpp>
#include <ros/ros.h>

using namespace std;
namespace po = boost::program_options; 

int main(int argc, char** argv)
{
    // Define and parse the program options 
    po::options_description desc("Options"); 
    desc.add_options() 
        ("help", "Print help messages") 
        ("scan_topic",  po::value<std::string>()->default_value("/scan"), "topic that contains the laserScan in the rosbag")
        ("bag_filename", po::value<std::string>()->required(), "ros bag filename") 
        ("seed", po::value<unsigned long int>()->default_value(0), "seed")
        ("max_duration_buffer", po::value<unsigned long int>()->default_value(99999), "max tf buffer duration")
        ("on_done", po::value<std::string>(), "command to execute when done");
    
    po::variables_map vm; 
    try 
    { 
        po::store(po::parse_command_line(argc, argv, desc), vm); // can throw 
        
        // Check if "--help" option is specified
        if (vm.count("help")) 
        { 
            ROS_INFO_STREAM("Basic Command Line Parameter App" << endl 
                << desc << endl);
            
            return 0; 
        } 
        
        // May throw an error, so do it after help in case that there are some unexpected problems.
        po::notify(vm);
    } 
    catch(po::error& e) 
    { 
        ROS_ERROR_STREAM(e.what() << endl << endl
            << desc << endl);

        return -1; 
    } 
    
    string bagFilename = vm["bag_filename"].as<string>();
    string scanTopic = vm["scan_topic"].as<string>();
    unsigned long int seed = vm["seed"].as<unsigned long int>();
    unsigned long int maxDurationBuffer = vm["max_duration_buffer"].as<unsigned long int>();
    
    ros::init(argc, argv, "gmapping_enhanced");
    SlamGMapping sgm(seed, maxDurationBuffer);
    ROS_INFO("Replay starting.");
    sgm.StartReplay(bagFilename, scanTopic);
    ROS_INFO("Replay stopped.");

    if (vm.count("on_done"))
    {
        // Run the "on_done" command and then exit.
        system(vm["on_done"].as<std::string>().c_str());
    }
    else
    {
        // Wait here so user can save the map.
        ros::spin(); 
    }

    return 0;
}