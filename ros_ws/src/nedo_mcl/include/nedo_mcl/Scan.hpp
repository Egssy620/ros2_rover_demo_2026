// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef NEDO_MCL__SCAN_HPP_
#define NEDO_MCL__SCAN_HPP_

#include <iostream>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace nedo_mcl
{

class Scan
{
	public:
	int seq_;
	std::map<std::string, geometry_msgs::msg::PoseStamped> markers_ ;

	Scan & operator=(const Scan & s);
};

}  // namespace nedo_mcl

#endif	// NEDO_MCL__SCAN_HPP_
