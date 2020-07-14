/**
 * @author Taeho Kang (ksm07091@gmail.com)
 * @date 2018-12-05
 */
#include <rrt_planner/rrt_planner/rrt_planner_base.h>

namespace rrt_planner
{

void RRTPlannerBase::initialize(std::string frame_id_,
                                std::string base_frame_id_,
                                double resolution_,
                                bool two_dimension_,
                                double range,
                                double mapsize_x,
                                double mapsize_y,
                                double mapsize_z)
{
    frame_id = frame_id_;
    base_frame_id = base_frame_id_;
    resolution = resolution_;
    two_dimension = two_dimension_;

    if (range)
    {
        setRange(range, two_dimension);
    }
    else
    {
        assert(mapsize_x + mapsize_y + mapsize_z != 0);
        setMapSize(mapsize_x, mapsize_y, mapsize_z, two_dimension);
    }
}

void RRTPlannerBase::addCritic(LineCriticInterface *critic_new)
{
    critic_.push_back(critic_new);
}

void RRTPlannerBase::clear()
{
    if (!cloud)
        cloud = boost::make_shared<PointCloudType>();
    else
        cloud->clear();

    if (!octree)
        octree = boost::make_shared<OctreeType>(resolution);
    else
        octree->deleteTree();

    if (!tree)
        tree = boost::make_shared<RRTTreeType>();
    else
        tree->clear();

    octree->setInputCloud(cloud);

    hash.table().clear();
}

bool RRTPlannerBase::makeTree(double x, double y, double z)
{
    // clear old tree
    clear();

    // set root vertex
    RRTVertex v_root(x, y, z, IDX_ROOT, IDX_NONE);

    // save offset of coordinate
    setOrigin(x, y, z);

    // add root vertex to new tree
    addVertexToTree(v_root, IDX_NONE, getCost(v_root, v_root));

    // prepare critics
    for (auto c : critic_)
        if (!c->prepare())
        {
            ROS_WARN("Preparing CriticInterface failed!");
            return false;
        }

    return true;
}

void RRTPlannerBase::setMapSize(double x, double y, double z, bool two_dimension_)
{
    two_dimension = two_dimension_;

    map_size.x = x;
    map_size.y = y;
    hx = map_size.x / 2.0;
    hy = map_size.y / 2.0;
    sampling_radius = 0;

    if (two_dimension)
    {
        map_size.z = 0;
        hz = 0;
    }
    else
    {
        map_size.z = z;
        hz = map_size.z / 2.0;
    }
}

void RRTPlannerBase::setOrigin(double x, double y, double z)
{
    offset_x = x;
    offset_y = y;
    offset_z = z;
}

void RRTPlannerBase::setRange(double r, bool two_dimension_)
{
    assert(r > 0);
    two_dimension = two_dimension_;
    sampling_radius = r;
}

double RRTPlannerBase::getEdgeLength(const RRTVertex &v1, const RRTVertex &v2)
{
    return v1.distance(v2);
}

void RRTPlannerBase::addVertexToTree(RRTVertex &v, int parent_idx, double cost)
{
    v.parent_idx = parent_idx;
    v.idx = tree->size();
    v.cost = cost;
    v.clear_child();

    tree->push_back(v);
    octree->addPointToCloud(v.pos, cloud);

    if (v.parent_idx >= IDX_ROOT)
    {
        RRTVertex &vp = getVertex(v.parent_idx);
        vp.add_child();
    }

    hash[v.pos] = v.idx;
}
void RRTPlannerBase::reconnect(RRTVertex &v, int new_parent, double new_cost)
{
    assert((new_parent >= IDX_ROOT && new_parent < tree->size(), "parent vertex is not exist"));

    getVertex(v.parent_idx).remove_child();

    v.parent_idx = new_parent;
    v.cost = new_cost;
}

RRTVertex &RRTPlannerBase::getVertex(RRTPoint p)
{
    auto it = hash.table().find(p);

    if (it == hash.table().end())
        return getVertex(IDX_ROOT);

    return getVertex(it->second);
}

RRTVertex &RRTPlannerBase::getVertex(int idx)
{
    assert((tree->size() > idx && idx >= 0, "Undefined tree index"));

    return (*tree)[idx];
}
RRTVertex::Ptr RRTPlannerBase::getVertexPtr(int idx)
{
    assert((tree->size() > idx && idx >= 0, "Undefined tree index"));

    return std::make_shared<RRTVertex>((*tree)[idx]);
}

RRTVertex RRTPlannerBase::getRandomVertex()
{
    double xr, yr, zr;
    std::random_device rd;
    std::mt19937 gen(rd());

    do
    {
        // Range(Round,Sphere) type Sampling
        if (sampling_radius)
        {
            hx = sampling_radius;
            std::uniform_real_distribution<> x(-hx, hx);
            xr = x(gen);

            hy = sqrt(sampling_radius * sampling_radius - xr * xr);
            std::uniform_real_distribution<> y(-hy, hy);
            yr = y(gen);

            if (!two_dimension)
            {
                hz = sqrt(sampling_radius * sampling_radius - xr * xr - yr * yr);
                std::uniform_real_distribution<> z(-hz, hz);
                zr = z(gen);
            }
            else
            {
                zr = 0;
            }
        }
        else // Box,Square type Sampling
        {
            assert((hx && hy, "Invalid map size"));

            std::uniform_real_distribution<> x(-hx, hx);
            xr = x(gen);

            std::uniform_real_distribution<> y(-hy, hy);
            yr = y(gen);

            if (!two_dimension)
            {
                std::uniform_real_distribution<> z(-hz, hz);
                zr = z(gen);
            }
            else
            {
                zr = 0;
            }
        }
        // resampling if sampled point is origin
    } while (!xr && !yr && !zr);

    RRTVertex v_rand(offset_x + xr, offset_y + yr, offset_z + zr);
    return v_rand;
}

int RRTPlannerBase::getNearestVertex(const RRTVertex &v)
{
    std::vector<int> idxs;
    std::vector<float> dist;

    octree->nearestKSearch(v.pos, 1, idxs, dist);

    return idxs[0];
}

double RRTPlannerBase::getNearestDist(const RRTVertex &v)
{
    std::vector<int> idxs;
    std::vector<float> dist;

    octree->nearestKSearch(v.pos, 1, idxs, dist);

    return (double)dist[0];
}

void RRTPlannerBase::getNearVertexs(const RRTVertex &v, double search_radius, std::vector<int> &idxs)
{
    idxs.clear();
    std::vector<float> dist;

    octree->radiusSearch(v.pos, search_radius, idxs, dist);
}
void RRTPlannerBase::getNearKVertexs(const RRTVertex& v, int k, std::vector<int>& idxs)
{
    std::vector<float> dist;

    octree->nearestKSearch(v.pos, k, idxs, dist);
}

bool RRTPlannerBase::isValid(const double cost)
{
    return cost > 0;
}

bool RRTPlannerBase::isValid(const RRTVertex &v1,
                             const RRTVertex &v2)
{
    return getCost(v1, v2) != COST_INVALID;
}

double RRTPlannerBase::getCost(const int v1_idx, const int v2_idx)
{
    return getCost(getVertex(v1_idx), getVertex(v2_idx));
}

double RRTPlannerBase::getCost(const rrt_planner::RRTVertex &v1,
                               const rrt_planner::RRTVertex &v2)
{
    //v1 --> v2
    //set positions

    if (!checkInBound(v2))
        return RRTPlannerBase::COST_INVALID;

    //set orientation of line v1 --> v2
    geometry_msgs::Quaternion q = getQuaternion(v1, v2);
    geometry_msgs::PoseStamped p1, p2;

    p1.header.frame_id = frame_id;
    p2.header.frame_id = frame_id;

    p1.pose.position.x = v1.pos.x;
    p1.pose.position.y = v1.pos.y;
    p1.pose.position.z = v1.pos.z;
    p2.pose.position.x = v2.pos.x;
    p2.pose.position.y = v2.pos.y;
    p2.pose.position.z = v2.pos.z;

    p1.pose.orientation = q;
    p2.pose.orientation = q;

    double cost = 0;
    for( auto c : critic_)
    {
        double cc = c->scoreLine(p1, p2);  

        if( cc < 0 )
            return RRTPlannerBase::COST_INVALID;

        cost += cc; 
    }

    return cost;
}

double RRTPlannerBase::getCost(const geometry_msgs::PoseStamped& p)
{
    if (!checkInBound(RRTVertex(p.pose.position.x, p.pose.position.y, p.pose.position.z)))
    {
        ROS_WARN("Given pose is not in rrt sampling bondary");
        return RRTPlannerBase::COST_INVALID;
    }
    double cost = 0;
    for( auto c : critic_)
    {
        double cc = c->scorePose(p);

        if( cc < 0 )
            return RRTPlannerBase::COST_INVALID;

        cost += cc;
    }
   
    return cost;
}

double RRTPlannerBase::getCost(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
	if (!checkInBound(RRTVertex(p1.pose.position.x, p1.pose.position.y, p1.pose.position.z)) ||
		!checkInBound(RRTVertex(p2.pose.position.x, p2.pose.position.y, p2.pose.position.z)))
    {
        ROS_WARN("Given pose is not in rrt sampling bondary");
        return RRTPlannerBase::COST_INVALID;
    }

    double cost = 0;
    for( auto c : critic_)
    {
        double cc = c->scoreLine(p1,p2);

        if( cc < 0 )
            return RRTPlannerBase::COST_INVALID;

        cost += cc;
    }
   
    return cost;

}

geometry_msgs::Quaternion RRTPlannerBase::getQuaternion(const RRTPoint &p1, const RRTPoint &p2)
{
    return kt_tools::getQuaternion(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z);
}

geometry_msgs::Quaternion RRTPlannerBase::getQuaternion(const RRTVertex &v1, const RRTVertex &v2)
{
    return kt_tools::getQuaternion(v1.pos.x, v1.pos.y, v1.pos.z, v2.pos.x, v2.pos.y, v2.pos.z);
}


double RRTPlannerBase::fitResolution(double val)
{
    assert((resolution > 0, "Resolution should be set"));

    return (double)trunc(val / resolution) * resolution;
}

void RRTPlannerBase::fitResolution(RRTVertex &v)
{
    v.pos.x = fitResolution(v.pos.x);
    v.pos.y = fitResolution(v.pos.y);
    v.pos.z = fitResolution(v.pos.z);
}

bool RRTPlannerBase::checkInBound(const RRTVertex &v)
{
    checkInBound(v.pos.x, v.pos.y, v.pos.z);
}

bool RRTPlannerBase::checkInBound(double x, double y, double z)
{
    if (two_dimension)
    {
        if (sampling_radius)
            return (pow(x - offset_x, 2) + pow(y - offset_y, 2)) < sampling_radius * sampling_radius;

        return !(x > offset_x + hx || x < offset_x - hx ||
                 y > offset_y + hy || y < offset_y - hy);
    }
    else
    {
        if (sampling_radius)
            return (pow(x - offset_x, 2) + pow(y - offset_y, 2) + pow(z - offset_z, 2)) < sampling_radius * sampling_radius;

        return !(x > offset_x + hx || x < offset_x - hx ||
                 y > offset_y + hy || y < offset_y - hy ||
                 z > offset_z + hz || z < offset_z - hz);
    }
}

bool RRTPlannerBase::checkInBound(const geometry_msgs::PoseStamped &p)
{
    return checkInBound(p.pose.position.x, p.pose.position.y, p.pose.position.z);
}

void RRTPlannerBase::getLeafVertexs(std::vector<int> &list)
{
    list.clear();

    for (auto &v : *tree)
        if (v.is_leaf())
            list.push_back(v.idx);
}

void RRTPlannerBase::getAncestorChecker(int idx, std::vector<bool> &list)
{
    assert(idx >= IDX_ROOT && idx < tree->size());

    list.resize(tree->size());
    list.clear();

    int idx_ = idx;

    list[idx_] = true;

    while (idx_ != IDX_ROOT)
    {
        idx_ = getVertex(idx_).parent_idx;
        list[idx_] = true;
    }
}

bool RRTPlannerBase::isOccupied(const RRTPoint &p)
{
    bool octree_result = octree->isVoxelOccupiedAtPoint(p);
    bool hash_result;

    auto it = hash.table().find(p);

    if (it == hash.table().end())
        hash_result = false;
    else
        hash_result = true;

    return octree_result & hash_result;
}

} // namespace rrt_planner
