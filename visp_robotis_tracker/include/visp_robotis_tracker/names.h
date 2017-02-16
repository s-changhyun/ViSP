/*
 * names.h
 *
 *  Created on: Jun 22, 2016
 *      Author: robotis
 */

#ifndef SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NAMES_H_
#define SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NAMES_H_

# include <string>

namespace visp_robotis_tracker
{
extern std::string camera_info_topic;
extern std::string image_topic;
extern std::string moving_edge_sites_topic;
extern std::string klt_points_topic;
extern std::string status_topic;

extern std::string object_position_topic;
extern std::string object_position_covariance_topic;
extern std::string code_message_topic;
extern std::string init_service;

extern std::string tracker_ref_frame;
extern std::string tracker_config_file;
}

#endif /* SRC_VISP_ROBOTIS_TRACKER_INCLUDE_VISP_ROBOTIS_TRACKER_NAMES_H_ */
