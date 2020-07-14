/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#include <rrt_planner/rrt_planner/rrt_star_planner.h>

namespace rrt_planner
{

void RRTStarPlanner::init(kt_tools::tfInterface* tf_listener_,
                          std::string frame_id_ = "map",
                          std::string base_frame_id_ = "base_link",
                          double resolution_ = 0.05,
                          double max_point_ = 1000,
                          double time_limit_ = 1.0,
                          double step_size_ = 0.1,
                          double search_radius_ = 0.2,
                          double goal_radius_ = 0.3,
                          double range_ = 3.0,
                          double mapsize_x_ = 0.0,
                          double mapsize_y_ = 0.0,
                          double mapsize_z_ = 0.0,
                          bool pub_tree_ = true,
                          bool pub_boundary_ = true,
                          bool pub_localgoal_ = true,
                          bool fit_resolution_ = false,
                          bool use_global_plan_ = false,
                          bool two_dimension_ = true)
{
    initialize(frame_id_, base_frame_id_, resolution_, two_dimension_, range_ ,mapsize_x_ ,mapsize_y_ ,mapsize_z_);

    viz.init("rrt_tree", 10, &RRTStarPlanner::draw, this);

    tf_listener = tf_listener_;
    max_point = max_point_;
    time_limit = time_limit_;
    step_size = step_size_;
    search_radius = search_radius_;
    goal_radius = goal_radius_;
    pub_tree = pub_tree_;
    pub_boundary = pub_boundary_;
    pub_localgoal = pub_localgoal_;
    fit_resolution = fit_resolution_;
    use_global_plan = use_global_plan_;

	latest_path_size = 9999;
}

void RRTStarPlanner::plannerInit(PlanTypePtr initial_plan)
{
    // set new initial plan
    initial_plan_ = initial_plan;
    initial_plan_front = 0;
    goal_ = getPlanPose( initial_plan_front );
	latest_path.assign(initial_plan->begin(), initial_plan->end());

    //make octree of initial plan -> for nn search
    if (!octree_initial_plan)
        octree_initial_plan = boost::make_shared<OctreeType>(resolution);
    else
        octree_initial_plan->deleteTree();

    if (!cloud_initial_plan)
    {
        cloud_initial_plan = boost::make_shared<PointCloudType>();
        octree_initial_plan->setInputCloud(cloud_initial_plan);
    }
    else
        cloud_initial_plan->clear();

    for (const auto &p : *initial_plan_)
    {
        octree_initial_plan->addPointToCloud(
            pcl::PointXYZ(p.pose.position.x, p.pose.position.y, p.pose.position.z),
            cloud_initial_plan);
    }

    // set flag
    initialized = true;
}

geometry_msgs::PoseStamped &RRTStarPlanner::findGoal()
{
    if (!checkInBound( getPlanPose(initial_plan_front) ))
    {
        // if previous local goal is out of bound, move initial_plan_front to first inbound point index
        // - inbound point : global plan pose which is located inside the sampling radius
        for (int i = initial_plan_front; i < initial_plan_->size(); i++)
        {
            geometry_msgs::PoseStamped &p = getPlanPose(i);

            if (checkInBound(p))
            {
                initial_plan_front = i;
                break;
            }
        }
    }

    // find local goal from previous inbound point
    for (int i = initial_plan_front; i < initial_plan_->size(); i++)
    {
        geometry_msgs::PoseStamped &p = getPlanPose(i);

        if (!checkInBound(p))
        {
            initial_plan_front = i - 1;
            return getPlanPose(initial_plan_front);
        }
    }

    // if there is no point that exceed boundary, return last pose of initial_plan
    initial_plan_front = initial_plan_->size() - 1;
    return getPlanPose(initial_plan_front);
}

void RRTStarPlanner::initTree()
{
    int parent = IDX_ROOT;

    // find nn of start pose
    std::vector<int> idxs;
    std::vector<float> dist;
    pcl::PointXYZ point_start(start_.pose.position.x, start_.pose.position.y, start_.pose.position.z);
    octree_initial_plan->nearestKSearch(point_start, 1, idxs, dist);

    if (!idxs.size())
        return;

    assert(idxs[0] >= 0 && idxs[0] < initial_plan_->size());

    for (int i = idxs[0]; i < initial_plan_front; i++)
    {
        geometry_msgs::PoseStamped &p = getPlanPose(i);

        if (checkInBound(p))
        {
            RRTVertex v_new(p.pose.position.x,
                            p.pose.position.y,
                            p.pose.position.z,
                            parent + 1,
                            parent);

            //if( fit_resolution )
            fitResolution(v_new);

            double line_cost = getCost(getVertex(parent), v_new);

            if (isValid(line_cost))
            {
                addVertexToTree(v_new, parent, getVertex(parent).cost + line_cost);
                parent++;
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
}
bool RRTStarPlanner::getPath(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             const int goal_idx,
                             std::vector<geometry_msgs::PoseStamped> &path)
{
    assert(goal_idx >= IDX_ROOT && goal_idx < tree->size());
    RRTVertex::Ptr vc, vp;


    //TODO replace with Kalman Smoother

    std::vector<geometry_msgs::PoseStamped> waypoint;
    geometry_msgs::PoseStamped tmp;
    int wp_size = 0;

    // when sub goal(initial plan front) is same with global goal
    if( initial_plan_->size() - 1 == initial_plan_front )
    {
        waypoint.push_back(goal);
    }

	// Get waypoints from RRT Tree
    int idx = goal_idx;
    while (true)
    {
        RRTVertex &v = getVertex(idx);

        v.toPoseStamped(frame_id, ros::Time::now(), tmp);
        waypoint.push_back(tmp);

        if (idx == IDX_ROOT)
            break;
        else
            idx = v.parent_idx;
    }

	std::reverse(waypoint.begin(), waypoint.end());
	wp_size = waypoint.size();

	if (wp_size < 2)
	{
		ROS_WARN("[RRT] waypoint size < 2");
		return false;
	}


	int idx_end = wp_size;
	for( int i = 0; i < wp_size-1 ; i++ )
	{
		// get orientation
        waypoint[i].pose.orientation = kt_tools::getQuaternion(waypoint[i].pose.position, waypoint[i + 1].pose.position);

		// filter collision 	
		if( getCost( waypoint[i], waypoint[i+1] ) < 0 )
		{
			idx_end = i; 
			break;
		}
	}

	if( idx_end == 0 ) 
	{
		ROS_WARN("[RRT] plan is invalid ...");
		return false;
	}

/*  
	//Smoothing
	ecl::Array<double> x_set(wp_size), y_set(wp_size), yaw_set(wp_size), t_set(wp_size);
	ecl::CubicSpline x_spline, y_spline, yaw_spline;

	for (int i = 0; i < idx_end; i++)
	{
		x_set[i] = waypoint[i].pose.position.x;
		y_set[i] = waypoint[i].pose.position.y;
		yaw_set[i] = tf::getYaw(waypoint[i].pose.orientation);
		t_set[i] = i;
	}

	try
	{
		x_spline = ecl::CubicSpline::Natural(t_set, x_set);
		y_spline = ecl::CubicSpline::Natural(t_set, y_set);
		yaw_spline = ecl::CubicSpline::Natural(t_set, yaw_set);
	}
	catch (ecl::StandardException &ex)
	{
		ROS_WARN("[RRT] Failed to smoothing path");
		return false;
	}

	// Make Path
	
	// clear old path
    path.clear();

	// get path size after collision filtering & smoothing
	int path_size = (idx_end - 1) * 10 + 1;
	path.resize(path_size);

	path[0].header.frame_id = frame_id;
	path[0].pose.position.x = x_spline(0);
	path[0].pose.position.y = y_spline(0);
	path[0].pose.position.z = 0;
	path[0].pose.orientation = start.pose.orientation;
	
	for (int i = 1; i < path_size; i++)
	{
		path[i].header.frame_id = frame_id;
		path[i].pose.position.x = x_spline((double)i / 10.0);
		path[i].pose.position.y = y_spline((double)i / 10.0);
		path[i].pose.position.z = 0;
		path[i].pose.orientation = kt_tools::getQuaternion(path[i-1].pose.position, path[i].pose.position);
	}

	// set last pose orientation as goal orientation
	path[path_size-1].pose.orientation = goal.pose.orientation;
	*/

	path.clear();
	path.assign( waypoint.begin(), waypoint.end() );
	path[path.size() - 1].pose.orientation = goal.pose.orientation;

    latest_path.clear();
    latest_path.assign(path.begin(),path.end());
	latest_path_size = path.size();

    return true;
}

int RRTStarPlanner::findPath(const geometry_msgs::PoseStamped& start)
{
    // if update is not needed, skip
    if( !isUpdateNeed() )
    {
        return EXCEPTION_SKIP;        
    }

    // make new tree from start pose
    if (!makeTree(start.pose.position.x, start.pose.position.y, start.pose.position.z))
    {
        ROS_ERROR("[RRT] Making tree failed!");
        return EXCEPTION_FAILED;
    }

    start_ = start;
    goal_ = findGoal();

    if (use_global_plan)
        initTree();

    return findPath(start_, goal_);
}

int RRTStarPlanner::findPath(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal)
{
    std::vector<int> v_near_list;
    
    // start to draw tree
    viz.start();

    double start_time = ros::Time::now().toSec();

    // find path( rrt* )
    while (ros::Time::now().toSec() - start_time <= time_limit && max_point > tree->size())
    {
        // sampling
        RRTVertex v_new = getRandomVertex();
        const RRTVertex &v_nearest = getVertex(getNearestVertex(v_new));

        // steer
        steer(v_nearest, v_new);

        // fit vertex coord to resolution
        if (fit_resolution)
            fitResolution(v_new);

        // connect & rewire
        if (isValid(v_nearest, v_new))
        {
            // get near vertexs
            getNearVertexs(v_new, search_radius, v_near_list);

            // find best near vertex and connect
            if (!connect(v_nearest, v_new, v_near_list))
            {
                ROS_WARN("[RRT] Could not find vertex in the RRT tree");
                continue;
            }

            // rewiring
            rewire(v_new, v_near_list);
        }
    }

    // global rewiring from root
    for (const auto &v : *tree)
    {
        getNearVertexs(v, search_radius, v_near_list);
        rewire(v, v_near_list);
    }

    // stop to draw tree
    viz.stop();

    // find best candidate near goal
    std::vector<int> list;

    RRTVertex v_goal(goal.pose.position.x,
                     goal.pose.position.y,
                     goal.pose.position.z);

    // first candidate : vertexs inside goal_radius
    getNearVertexs(v_goal, goal_radius, list);

    // failed to find first candidate
    if (list.empty())
    {
        ROS_WARN("[RRT] Failed to find valid path!");
		return EXCEPTION_FAILED;		
    }

    // Find best candidate and return its index
    int best_idx = list[0];
    for (const auto &idx : list)
    {
        if (getVertex(idx).cost < getVertex(best_idx).cost)
        {
            best_idx = idx;
        }
    }

    return best_idx;
}

void RRTStarPlanner::steer(const RRTVertex &v_nearest, RRTVertex &v_rand)
{
    std::uniform_real_distribution<> rot(-M_PI / 4, M_PI / 4);
    std::random_device rd;
    std::mt19937 gen(rd());
    Eigen::Vector3d dir_vec;

    if (v_nearest.idx == IDX_ROOT)
    {
        // get start pose rotation
        //Eigen::Quaterniond quat;
        //tf::quaternionMsgToEigen(start_.pose.orientation, quat);

        // get start direction vector
        dir_vec << v_rand.pos.x - v_nearest.pos.x,
            v_rand.pos.y - v_nearest.pos.y,
            v_rand.pos.z - v_nearest.pos.z;
    }
    else
    {
        const RRTVertex &vp = getVertex(v_nearest.parent_idx);

        // get parent to child direction vector
        dir_vec << v_nearest.pos.x - vp.pos.x,
            v_nearest.pos.y - vp.pos.y,
            v_nearest.pos.z - vp.pos.z;
    }

    dir_vec.normalize();

    // get random rotation matrix( -30~30 degree )
    Eigen::Matrix3d m;
    if (two_dimension)
    {
        m = Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitZ());
    }
    else
    {
        m = Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rot(gen), Eigen::Vector3d::UnitY());
    }

    // calc next step vector (rotated normal vector)
    Eigen::Vector3d step_vec = step_size * m * dir_vec;

    // update position of new vertex
    v_rand.pos.x = v_nearest.pos.x + step_vec.x();
    v_rand.pos.y = v_nearest.pos.y + step_vec.y();
    v_rand.pos.z = v_nearest.pos.z + step_vec.z();
}

bool RRTStarPlanner::connect(const RRTVertex &v_nearest, RRTVertex &v_new, const std::vector<int> &v_near_list)
{
    if (!isOccupied(v_new.pos))
    {
        int v_min_idx = v_nearest.idx;
        double c_min = v_nearest.cost + getCost(v_nearest, v_new);

        // find best connection
        for (const int &idx : v_near_list)
        {
            const RRTVertex &v_near = getVertex(idx);
            double line_cost = getCost(v_near, v_new);
            double c_new = v_near.cost + line_cost;

            if (isValid(line_cost) && c_min > c_new)
            {
                v_min_idx = idx;
                c_min = c_new;
            }
        }

        // connect , add vertex
        addVertexToTree(v_new, v_min_idx, c_min);
    }
    else
    {
        const RRTVertex &v_new_old = getVertex(v_new.pos);

        if (v_new_old.idx == IDX_ROOT)
            return false;

        v_new.idx = v_new_old.idx;
        v_new.parent_idx = v_new_old.parent_idx;
        v_new.cost = v_new_old.cost;
        v_new.num_child = v_new_old.num_child;
    }

    return true;
}

void RRTStarPlanner::rewire(const RRTVertex &v_new, const std::vector<int> &v_near_list)
{
    // rewire
    std::vector<bool> ancestor_v_new;

    getAncestorChecker(v_new.idx, ancestor_v_new);

    for (const int &idx : v_near_list)
    {
        if (!ancestor_v_new[idx])
        {
            RRTVertex &v_near = getVertex(idx);
			RRTVertex &v_near_parent = getVertex(v_near.parent_idx);

			Eigen::Vector3d near_to_new, near_to_parent;

			near_to_new << v_new.pos.x - v_near.pos.x,
						   v_new.pos.y - v_near.pos.y,
						   v_new.pos.z - v_near.pos.z;

			near_to_parent << v_near_parent.pos.x - v_near.pos.x,
						   	  v_near_parent.pos.y - v_near.pos.y,
							  v_near_parent.pos.z - v_near.pos.z;

			double theta = acos( near_to_new.dot( near_to_parent ) / (near_to_new.norm() * near_to_parent.norm()) );

			if( abs(theta) > M_PI / 4 )
				continue; 	

            double line_cost = getCost(v_new, v_near);
            double c_new = v_new.cost + line_cost;

            if ( isValid(line_cost) && v_near.cost > c_new)
            {
                reconnect(v_near, v_new.idx, c_new);
            }
        }
    }
}

void RRTStarPlanner::rewireRand()
{
    std::vector<int> v_near_list;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> idx(IDX_ROOT + 1, tree->size());

    const RRTVertex &v = getVertex(idx(gen));

    getNearVertexs(v, search_radius, v_near_list);

    rewire(v, v_near_list);
}

bool RRTStarPlanner::getPathIdx(const geometry_msgs::PoseStamped &current_pose,
                                      const int goal_idx,
                                      std::vector<geometry_msgs::PoseStamped> &path)
{
    return getPath(current_pose, goal_, goal_idx, path);
}
geometry_msgs::PoseStamped &RRTStarPlanner::getPlanPose(int idx)
{
    assert(idx >= 0 && idx <= initial_plan_->size());
    return (*initial_plan_)[idx];
}

double RRTStarPlanner::getDistToGoal(const geometry_msgs::PoseStamped& pose)
{
    return kt_tools::distance(pose, goal_);
}

void RRTStarPlanner::draw()
{
    if(tree->size() < 2) return;

    viz.clear();
    //viz.removeAll();

    if (pub_tree)
    {
        std::vector<geometry_msgs::Point> p_list;
        geometry_msgs::Point p1, p2;

        for (const auto &v : *tree)
            if (v.parent_idx != IDX_NONE)
            {
                const RRTVertex &vp = getVertex(v.parent_idx);
 
                p1.x = vp.pos.x;
                p1.y = vp.pos.y;
                p1.z = vp.pos.z;
                p2.x = v.pos.x;
                p2.y = v.pos.y;
                p2.z = v.pos.z;

                p_list.push_back(p1);
                p_list.push_back(p2);
            }

        viz.addLineList(p_list, frame_id, "tree", 0.02, kt_tools::Color::Orange(), 0.8);
    }

    if (pub_localgoal)
        viz.addSphere(goal_.pose, frame_id, "local_goal", 0.1, kt_tools::Color::Green(), 1.0);

    if (pub_boundary)
    {
        if (sampling_radius)
            viz.addCircle(offset_x, offset_y, offset_z, sampling_radius, frame_id, "sampling_boundary", 0.01);
        else
            viz.addRectangle(offset_x, offset_y, offset_z, map_size.x, map_size.y, frame_id, "sampling_boundary", 0.01);
    }

    viz.draw();
}

bool RRTStarPlanner::isUpdateNeed()
{
	static double last_update_time= ros::Time::now().toSec();
	double current_time = ros::Time::now().toSec();

	if( update_request )
	{
		last_update_time = ros::Time::now().toSec();
		update_request = false;
        return true;
	}

    geometry_msgs::PoseStamped current_pose;

    // get current robot pose
    if( !kt_tools::getRobotPose(current_pose, tf_listener, frame_id, base_frame_id))
    {
        ROS_WARN("[RRT] Could not get robot pose");
        return false;
    }  

    // prune plan (except last pose)
    for( int i = 0; i < latest_path.size()-1; i++ )
        if( kt_tools::distance(current_pose, latest_path[i]) < 0.2 )
        {
            latest_path.erase(latest_path.begin(), latest_path.begin() + i);
        }

    // check latest planned path is blocked
    for(const auto& p : latest_path)
    {
        if( getCost(p) < 0 )
        {
            ROS_WARN("[RRT] Path Blocked!");
			last_update_time= ros::Time::now().toSec();
            return true;
        }
    }

	// goal is close enough OR plan has been consumed over 50%
	bool consumed_plan_enough = kt_tools::distance(current_pose, goal_) < 0.2 || ((float)latest_path.size()/(float)latest_path_size) <= 0.50;
	bool is_final_goal = initial_plan_front == initial_plan_->size() - 1;
	bool is_time_over = ros::Time::now().toSec() - last_update_time > 5.0;

    // check refresh time & goal arrival
    if( (consumed_plan_enough && !is_final_goal) || is_time_over )
	{
		last_update_time = ros::Time::now().toSec();
        return true;
	}

	return false;
}


} // namespace rrt_planner
