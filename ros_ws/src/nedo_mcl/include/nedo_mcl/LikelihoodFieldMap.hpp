// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef NEDO_MCL__LIKELIFOODFIELDMAP_HPP_
#define NEDO_MCL__LIKELIFOODFIELDMAP_HPP_

#include "nedo_mcl/Pose.hpp"
#include "nedo_mcl/Scan.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <utility>
#include <vector>

namespace nedo_mcl
{
class LikelihoodFieldMap
{
	public:
	LikelihoodFieldMap(const std::map<std::string, geometry_msgs::msg::PoseStamped> &map);
	~LikelihoodFieldMap();

	void setLikelihood(double range);
	uint8_t likelihood(std::string marker_id);

	void drawFreePoses(int num, std::vector<Pose> & result);
	std::map<std::string, geometry_msgs::msg::PoseStamped> world_map_;
	private:

	void normalize(void);
};

}  // namespace nedo_mcl

#endif	// NEDO_MCL__LIKELIFOODFIELDMAP_HPP_
