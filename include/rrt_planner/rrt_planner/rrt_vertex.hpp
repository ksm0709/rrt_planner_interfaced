/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _RRT_VERTEX_H_
#define _RRT_VERTEX_H_

#include <pcl/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <list>

#define IDX_NONE    -2
#define IDX_ROOT    0

namespace rrt_planner
{
typedef pcl::PointXYZ RRTPoint;

class RRTVertex
{
  public:
    typedef std::shared_ptr<RRTVertex> Ptr;

    RRTPoint pos;
    int idx, parent_idx;
    double cost;
    int num_child;

  public:
    RRTVertex()
        : pos(0, 0, 0), idx(IDX_NONE), parent_idx(IDX_NONE), cost(0), num_child(0)
    {
    }
    RRTVertex(double x, double y, double z, int idx_=IDX_NONE, int parent_idx_=IDX_NONE)
        : pos(x, y, z), idx(idx_), parent_idx(parent_idx_), cost(0), num_child(0)
    {
    }

    void set_pos(double x, double y, double z)
    {
        pos.x = x;
        pos.y = y;
        pos.z = z;
    }
    
    void add_child(){ num_child++; }
    void remove_child(){ if(num_child > 0) num_child--; }
    void clear_child(){ num_child = 0; }

    bool is_leaf()
    {
        return num_child == 0;
    }

    double length() { return sqrt(pow(pos.x, 2) + pow(pos.y, 2) + pow(pos.z, 2)); }

    double distance(const RRTVertex &v) const
    {
        return sqrt(pow(pos.x - v.pos.x, 2) + pow(pos.y - v.pos.y, 2) + pow(pos.z - v.pos.z, 2));
    }

    bool operator==(const RRTVertex &v)
    {
        return (v.pos.x == pos.x && v.pos.y == pos.y && v.pos.z == pos.z && idx == v.idx && parent_idx == v.parent_idx);
    }
    bool operator!=(const RRTVertex &v)
    {
        return !(v.pos.x == pos.x && v.pos.y == pos.y && v.pos.z == pos.z && idx == v.idx && parent_idx == v.parent_idx);
    }


    void toPoseStamped(std::string frame_id, ros::Time stamp, geometry_msgs::PoseStamped& p)
    {
        p.header.frame_id = frame_id;
        p.header.stamp = stamp;
        p.pose.position.x = pos.x;
        p.pose.position.y = pos.y;
        p.pose.position.z = pos.z;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;
        p.pose.orientation.w = 1;
    }
};
} // namespace rrt_planner

#endif