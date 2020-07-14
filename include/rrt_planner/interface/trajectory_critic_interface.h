/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _TRAJECTORY_CRITIC_INTERFACE_H_
#define _TRAJECTORY_CRITIC_INTERFACE_H_

#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace rrt_planner
{
class TrajectoryCriticInterface
{
  public:
    double COST_INVALID = -std::numeric_limits<double>::infinity();
  public:
    virtual bool prepare() = 0;
    virtual double scoreTrajectory(const std::vector<geometry_msgs::PoseStamped> &traj) = 0;

    virtual ~TrajectoryCriticInterface(){}
    TrajectoryCriticInterface(){}
};

} // namespace rrt_planner

#endif