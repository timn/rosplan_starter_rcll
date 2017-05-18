/***************************************************************************
 *  rosplan_starter_rcll.cpp - Start planning in production phase
 *
 *  Created: Wed May 17 15:40:49 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/* Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <rcll_ros_msgs/GameState.h>
#include <std_msgs/String.h>

#include <map>
#include <string>

#define GET_CONFIG(privn, n, path, var, default_value)	  \
	if (! privn.getParam(path, var)) { \
		if (! n.getParam(path, var)) { \
			var = default_value; \
		} \
	}

class ROSPlanStarterRCLL {
 public:
	ROSPlanStarterRCLL(ros::NodeHandle &n)
		: n(n)
	{
		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "delay", cfg_delay_, 2.0);

		pub_planning_cmd_ =
			n.advertise<std_msgs::String>("kcl_rosplan/planning_commands", 10, true);

		started_planning_ = false;

		ROS_INFO("[RP-StarterRCLL] Waiting for PRODUCTION phase to start planning");
		sub_game_state_ = n.subscribe("rcll/game_state", 10,
		                              &ROSPlanStarterRCLL::game_state_cb, this);
	}

	void
	cb_start_timer(const ros::WallTimerEvent& event)
	{
		start_timer_.stop();

		ROS_INFO("[RP-StarterRCLL] *** Starting Planner ***");
		std_msgs::String msg;
		msg.data = "plan";
		pub_planning_cmd_.publish(msg);				

		shutdown_timer_ =
			n.createWallTimer(ros::WallDuration(5.0), boost::bind(ros::shutdown));
	}
	
	void
	game_state_cb(const rcll_ros_msgs::GameState::ConstPtr& msg)
	{
		if (started_planning_)  return;

		if (msg->state == rcll_ros_msgs::GameState::STATE_RUNNING &&
		    msg->phase == rcll_ros_msgs::GameState::PHASE_PRODUCTION)
		{
			if (pub_planning_cmd_.getNumSubscribers() >= 0) {
				ROS_INFO("[RP-StarterRCLL] Detected PRODUCTION phase, starting in %.1f sec", cfg_delay_);
				started_planning_ = true;
				start_timer_ =
					n.createWallTimer(ros::WallDuration(cfg_delay_), &ROSPlanStarterRCLL::cb_start_timer, this);
			} else {
				ROS_WARN_THROTTLE(15, "Production phase running, but planning system not loaded");
			}
		}
	}

 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_game_state_;
	ros::Publisher     pub_planning_cmd_;
	ros::WallTimer     shutdown_timer_;
	ros::WallTimer     start_timer_;

	float cfg_delay_;
	
	bool               started_planning_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_starter_rcll");

	ros::NodeHandle n;

	ROSPlanStarterRCLL rosplan_starter_rcll(n);
	
  ros::spin();
  
	return 0;
}
