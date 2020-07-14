/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _LINE_CRITIC_INTERFACE_H_
#define _LINE_CRITIC_INTERFACE_H_

#include <geometry_msgs/PoseStamped.h>

namespace rrt_planner
{
class LineCriticInterface
{
  public:
    double COST_INVALID = -std::numeric_limits<double>::infinity();
  public:
    virtual bool prepare() = 0;
    virtual double scorePose(const geometry_msgs::PoseStamped &p) = 0;
    virtual double scoreLine(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2) = 0;

    virtual ~LineCriticInterface(){}
    LineCriticInterface(){}
};

} // namespace rrt_planner

#endif