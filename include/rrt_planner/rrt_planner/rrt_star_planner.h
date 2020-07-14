/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#ifndef _RRT_STAR_PLANNER_H_
#define _RRT_STAR_PLANNER_H_

//TODO replace with Kalman Smoother
//#include <ecl/geometry.hpp>
//#include <ecl/containers.hpp>

#include <kt_tools/visualize_tool.h>
#include <kt_tools/geometry_tool.h>

#include "rrt_planner/rrt_planner/rrt_planner_base.h"
#include "rrt_planner/interface/line_critic_interface.h"
#include "rrt_planner/tool/plan_tool.hpp"

namespace rrt_planner
{
class RRTStarPlanner : public RRTPlannerBase
{

  public:
    enum
    {
      EXCEPTION_FAILED = -1,
      EXCEPTION_SKIP = -2
    };

  public: 
  
    RRTStarPlanner() : kt_planner::RRTPlannerBase(),
        initialized(false)
    {
    }
    ~RRTStarPlanner() { clear(); }

    // init planner
    void init(kt_tools::tfInterface *tf_listener_,
              std::string frame_id_,
              std::string base_frame_id_,
              double resolution_,
              double max_point_,
              double time_limit_,
              double step_size_,
              double search_radius,
              double goal_radius_,
              double range_,
              double mapsize_x_,
              double mapsize_y_,
              double mapsize_z_,
              bool pub_tree_,
              bool pub_boundary_,
              bool pub_localgoal_,
              bool fit_resolution_,
              bool use_global_plan_,
              bool two_dimension_);

    // init planner ( init rrt tree, set variables )
    void plannerInit(PlanTypePtr initial_plan);

    // confirm planner is initialized
    bool isInitialized() { return initialized; }

    // for goal distance critic interfacing
    geometry_msgs::PoseStamped* getLocalGoalPtr(){ return &goal_; }

    // find front goal using initial_plan
    geometry_msgs::PoseStamped &findGoal();
    geometry_msgs::PoseStamped &getPlanPose(int idx);

    // add initial vertexs ( nn of start ~ front goal ) to tree
    void initTree();

    // wrapper function
    int findPath(const geometry_msgs::PoseStamped &start);

    // random rewire
    void rewireRand();

    // get path
    bool getPathIdx(const geometry_msgs::PoseStamped &current_pose,
                    const int goal_idx,
                    std::vector<geometry_msgs::PoseStamped> &path);

    // utility function
    void draw();
    
    double getDistToGoal(const geometry_msgs::PoseStamped& pose);

    bool isUpdateNeed();

	void updateRequest(){update_request = true;}

  protected:
    // virtual function body
    void steer(const RRTVertex &v_nearest, RRTVertex &v_rand);
    bool connect(const RRTVertex &v_nearest, RRTVertex &v_new, const std::vector<int> &v_near_list);
    void rewire(const RRTVertex &v_new, const std::vector<int> &v_near_list);
    int findPath(const geometry_msgs::PoseStamped &start,
                 const geometry_msgs::PoseStamped &goal);
    bool getPath(const geometry_msgs::PoseStamped &start,
                 const geometry_msgs::PoseStamped &goal,
                 const int goal_idx,
                 std::vector<geometry_msgs::PoseStamped> &path);

    // before planning, set() should be called.
    bool initialized, use_global_plan, update_request;
    bool pub_tree, pub_boundary, pub_localgoal, fit_resolution;

    double dist_to_goal;
    geometry_msgs::PoseStamped start_, goal_;
    std::vector<geometry_msgs::PoseStamped> latest_path;
	int latest_path_size;

    boost::shared_ptr<OctreeType> octree_initial_plan;
    boost::shared_ptr<PointCloudType> cloud_initial_plan;

    PlanTypePtr initial_plan_;
    int initial_plan_front;

    // parameters
    double time_limit, step_size, search_radius, max_point, goal_radius;

    // util
    kt_tools::VisMarkers viz;

	kt_tools::tfInterface* tf_listener;
};
} // namespace rrt_planner

#endif
