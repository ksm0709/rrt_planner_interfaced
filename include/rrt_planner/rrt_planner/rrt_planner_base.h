/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _RRT_PLANNER_BASE_H_
#define _RRT_PLANNER_BASE_H_

#include <cmath>
#include <vector>
#include <utility>
#include <iostream>
#include <string>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
//#include <tf/transform_listener.h>
#include <kt_tools/tf_interface.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>


#include <kt_tools/coord_hash.h>
#include <kt_tools/geometry_tool.h>

#include "rrt_planner/rrt_planner/rrt_vertex.hpp"
#include "rrt_planner/interface/line_critic_interface.h"

/*
   x
   |
   |
   @----- y
  z

*/

namespace rrt_planner
{
    typedef pcl::octree::OctreePointCloudSearch<RRTPoint> OctreeType;
    typedef pcl::PointCloud<RRTPoint> PointCloudType;
    typedef std::vector<RRTVertex> RRTTreeType;
    typedef std::vector<geometry_msgs::PoseStamped> PlanType;
    typedef std::shared_ptr<PlanType> PlanTypePtr;

    class RRTPlannerBase
    {
      
    protected:
      // data structure
      boost::shared_ptr<RRTTreeType> tree;
      boost::shared_ptr<OctreeType> octree;
      boost::shared_ptr<PointCloudType> cloud;
      kt_tools::CoordHash<RRTPoint, int> hash;

      // critic
      std::vector<LineCriticInterface*> critic_;

      // parameters
      RRTPoint map_size;
      double sampling_radius = 0;
      double resolution;
      std::string frame_id, base_frame_id;
      bool two_dimension;
      double hx, hy, hz;
      double offset_x, offset_y, offset_z;

    public:
      double COST_INVALID = -std::numeric_limits<double>::infinity();

      
    public:
        RRTPlannerBase(){}
        virtual ~RRTPlannerBase(){}

        void initialize(std::string frame_id_,
                        std::string base_frame_id_,
                        double resolution_,
                        bool two_dimension_,
                        double range,
                        double mapsize_x,
                        double mapsize_y,
                        double mapsize_z);

        void addCritic(LineCriticInterface* critic_new);
        bool makeTree(double x, double y, double z);
        void addVertexToTree(RRTVertex& v, int parent_idx, double cost);
        void reconnect(RRTVertex& v, int new_parent, double new_cost);
        void clear();

        void setMapSize(double x, double y, double z, bool two_dimension_);
        void setOrigin(double x, double y, double z);
        void setRange(double r, bool two_dimension_);
        double fitResolution(double val);
        void fitResolution(RRTVertex& v);

        RRTVertex& getVertex(RRTPoint p);
        RRTVertex& getVertex(int idx);
        RRTVertex::Ptr getVertexPtr(int idx);
        RRTVertex getRandomVertex();  
        int getNearestVertex(const RRTVertex &v);
        double getNearestDist(const RRTVertex &v);
        double getEdgeLength(const RRTVertex &v1, const RRTVertex &v2);
        void getNearVertexs(const RRTVertex& v, double search_radius, std::vector<int>& idxs);
        void getNearKVertexs(const RRTVertex& v, int k, std::vector<int>& idxs);

        double getCost(const geometry_msgs::PoseStamped& p);
		double getCost(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
        double getCost(const RRTVertex& v1, const RRTVertex& v2);
        double getCost(const int v1_idx, const int v2_idx);
        bool isValid(const RRTVertex &v1, const RRTVertex &v2);
        bool isValid(const double cost); 
        void getLeafVertexs(std::vector<int>& list);
        void getAncestorChecker(int idx, std::vector<bool>& list);

        geometry_msgs::Quaternion getQuaternion(const RRTPoint& p1, const RRTPoint& p2);
        geometry_msgs::Quaternion getQuaternion(const RRTVertex& v1, const RRTVertex& v2);

        // edge = 1, out of edge = -1, inside edge = 0
        bool checkInBound(const RRTVertex& v);
        bool checkInBound(double x, double y, double z);
        bool checkInBound(const geometry_msgs::PoseStamped& p);
        bool isOccupied(const RRTPoint& p);

      public:

        virtual void steer(const RRTVertex& v_nearest, RRTVertex& v_rand) = 0;
        virtual bool connect(const RRTVertex &v_nearest, RRTVertex &v_new, const std::vector<int>& v_near_list) = 0;
        virtual void rewire(const RRTVertex& v_new, const std::vector<int>& v_near_list) = 0;
        virtual int findPath(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal) = 0;
        virtual bool getPath(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             const int goal_idx,
                             std::vector<geometry_msgs::PoseStamped> &path) = 0;
    };
}
#endif
