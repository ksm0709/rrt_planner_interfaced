//TODO Prune Plan, Smoother
/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _PLAN_TOOL_H_
#define _PLAN_TOOL_H_

#include <vector>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>

#include <kt_tools/geometry_tool.h>

namespace rrt_planner
{
    static bool PrunePlan(const geometry_msgs::PoseStamped& robot_pose, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if( !plan.size() ) 
            return false;

        if( plan[0].header.frame_id != robot_pose.header.frame_id ) 
            return false;
        
        Eigen::Quaterniond rot;
        Eigen::Vector3d heading_vec, plan_vec;
        tf::quaternionMsgToEigen(robot_pose.pose.orientation, rot);

        heading_vec = rot * Eigen::Vector3d(1,0,0);

        while( plan.size() )
        {
            double dist = kt_tools::distance( plan[0], robot_pose );

            plan_vec << plan[0].pose.position.x - robot_pose.pose.position.x,
                        plan[0].pose.position.y - robot_pose.pose.position.y,
                        plan[0].pose.position.z - robot_pose.pose.position.z;

            double angle = abs(acos(heading_vec.dot(plan_vec)/dist));

            if( angle <= M_PI/2 || dist < 0.3 )
            {
                break;
            }    

            plan.erase(plan.begin());
        }

        return true;
    }
}

#endif