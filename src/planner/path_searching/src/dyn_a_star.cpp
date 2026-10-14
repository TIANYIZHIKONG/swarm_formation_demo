#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    /* ------------------------------------------------------------------
       Bounded "step out of an occupied cell" loops.
       The original loops did
         do { pt += (pt - other).normalized() * step; }
         while (checkOccupancy(...));
       and returned false as soon as the point left the A* pool. When the start
       point is reported as occupied for a reason that stepping cannot fix -- e.g.
       it lies outside the map, where getInflateOccupancy() returns -1 (occupied)
       -- the loop simply ran away from the target (measured live: 111 iterations
       = 11.1 m) until it left the pool, and the whole A* call failed:
         [ERROR] Ran out of pool, index=508 -1 22
         [ERROR] Unable to handle the initial or end point, force return!
       The loops are now bounded, and if they cannot stay inside the pool they keep
       the last usable cell instead of failing the search: AstarSearch() never
       checks the occupancy of the start cell itself (only of its neighbours), so
       handing back an index always beats returning false here.
       ------------------------------------------------------------------ */
    constexpr int max_escape_steps = 50; // 50 x 0.1 m = 5 m, far more than any inflation band

    const Vector3d start_to_end = start_pt - end_pt;
    if (checkOccupancy(Index2Coord(start_idx)) && start_to_end.norm() > 1e-6)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        const Vector3d escape_dir = start_to_end.normalized();
        Vector3i tmp_idx;
        bool escaped = false;
        for (int i = 0; i < max_escape_steps && !escaped; ++i)
        {
            const Vector3d next_pt = start_pt + escape_dir * step_size_;
            if (!Coord2Index(next_pt, tmp_idx))
                break; // would leave the pool: keep the last usable cell
            start_pt = next_pt;
            start_idx = tmp_idx;
            escaped = !checkOccupancy(Index2Coord(start_idx));
        }
        if (!escaped)
            ROS_WARN("A*: start point [%f, %f, %f] is still inside an obstacle after %d escape steps, "
                     "planning from the current cell.",
                     start_pt(0), start_pt(1), start_pt(2), max_escape_steps);
    }

    const Vector3d end_to_start = end_pt - start_pt;
    if (checkOccupancy(Index2Coord(end_idx)) && end_to_start.norm() > 1e-6)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        const Vector3d escape_dir = end_to_start.normalized();
        Vector3i tmp_idx;
        bool escaped = false;
        for (int i = 0; i < max_escape_steps && !escaped; ++i)
        {
            const Vector3d next_pt = end_pt + escape_dir * step_size_;
            if (!Coord2Index(next_pt, tmp_idx))
                break; // would leave the pool: keep the last usable cell
            end_pt = next_pt;
            end_idx = tmp_idx;
            escaped = !checkOccupancy(Index2Coord(end_idx));
        }
        if (!escaped)
            ROS_WARN("A*: end point [%f, %f, %f] is still inside an obstacle after %d escape steps, "
                     "planning to the current cell.",
                     end_pt(0), end_pt(1), end_pt(2), max_escape_steps);
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt, bool use_esdf_check)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;
    
    /* A failed search must not hand back the path of the previous, successful
       one: getPath() would then return stale coordinates (re-projected through
       the *new* center_) which the caller cannot distinguish from a real path. */
    gridPath_.clear();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;

    /* ------------------------------------------------------------------
       Keep the requested start / end points inside the map.
       Outside the map GridMap::getInflateOccupancy() returns -1 ("out of map"),
       and checkOccupancy() casts it to bool -> true, i.e. an out-of-map point is
       reported as being INSIDE an obstacle. The start-point escape loop in
       ConvertToIndexAndAdjustStartEndPoints() then walks the point further and
       further away from the target until it leaves the A* pool:
         [ERROR] Ran out of pool, index=508 -1 22
         [ERROR] Unable to handle the initial or end point, force return!
       -> A* returns an empty path and the caller rejects the whole plan. This is
       exactly what was observed live: the drone had been sent to a goal outside
       the 40 x 30 m map (y = -16.1), and the very next goal could not be planned
       any more (the empty path then even killed the planner node).
       Clamping gives the search a usable in-map cell instead, so a drone that
       ended up at / slightly beyond the map edge can still be planned back.
       ------------------------------------------------------------------ */
    if (grid_map_)
    {
      const double clamp_margin = 2.0 * step_size;
      if (!grid_map_->isInMap(start_pt))
      {
        const Vector3d clamped = grid_map_->clampToMapBounds(start_pt, clamp_margin);
        ROS_WARN("A*: start point [%f, %f, %f] is outside the map, clamped to [%f, %f, %f]",
                 start_pt(0), start_pt(1), start_pt(2), clamped(0), clamped(1), clamped(2));
        start_pt = clamped;
      }
      if (!grid_map_->isInMap(end_pt))
      {
        const Vector3d clamped = grid_map_->clampToMapBounds(end_pt, clamp_margin);
        ROS_WARN("A*: end point [%f, %f, %f] is outside the map, clamped to [%f, %f, %f]",
                 end_pt(0), end_pt(1), end_pt(2), clamped(0), clamped(1), clamped(2));
        end_pt = clamped;
      }
    }

    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            ros::Time time_2 = ros::Time::now();
            printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if(use_esdf_check){
                        if (checkOccupancy_esdf(Index2Coord(neighborPtr->index)))
                            continue;
                    } else {
                        if (checkOccupancy(Index2Coord(neighborPtr->index)))
                            continue;
                    }
                    
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

vector<Vector3d> AStar::astarSearchAndGetSimplePath(const double step_size, Vector3d start_pt, Vector3d end_pt){
    /* ------------------------------------------------------------------
       Reject invalid requests up-front. Right after a collision the odometry
       (hence the FSM's start/target points) can be NaN / Inf, and a NaN start
       makes every sampling loop further down undefined behaviour (NaN -> int
       index conversion, division by zero, ...).
       ------------------------------------------------------------------ */
    if (!start_pt.allFinite() || !end_pt.allFinite() || !(step_size > 1e-6))
    {
        ROS_ERROR("astarSearchAndGetSimplePath: invalid request, start=[%f %f %f] end=[%f %f %f] step=%f",
                  start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2), step_size);
        return vector<Vector3d>();
    }

    /* 1st attempt: with the ESDF (0.2 m) clearance check. */
    bool ok = AstarSearch(step_size, start_pt, end_pt, true);
    vector<Vector3d> path = getPath(); // empty when the search failed (gridPath_ is cleared on entry)
    bool is_show_debug = false;

    /* The ESDF-checked search refuses to route through the 0.2 m inflation band,
       so close to an obstacle it either fails outright, or succeeds only after
       the start point was pushed > 0.5 m away from the requested one. Retry with
       the plain occupancy check: this is the original "I don't know why, but
       only try A* again" workaround, now driven by the return value instead of
       by reading path[0] of a possibly EMPTY vector (out-of-bounds -> UB). */
    if (!ok || path.empty() || (path[0] - start_pt).norm() > 0.5)
    {
        ROS_WARN("A* with esdf check usable=%d, path_size=%zu -> retry without the esdf check",
                 (int)ok, path.size());
        if (AstarSearch(step_size, start_pt, end_pt, false))
            path = getPath();
    }

    if (path.empty())
    {
        ROS_ERROR("astarSearchAndGetSimplePath: A* found no path, start=[%f %f %f] end=[%f %f %f]",
                  start_pt(0), start_pt(1), start_pt(2), end_pt(0), end_pt(1), end_pt(2));
        return path; // empty -> caller rejects it and the FSM retries on the next cycle
    }
    
    // generate the simple path
    vector<Vector3d> simple_path;
    int size = path.size();
    if (size <= 2){
        ROS_WARN("the path only have two points");
        return path;
    }
        
    int end_idx   = 1;
    Vector3d cut_start = path[0];
    simple_path.push_back(cut_start);
    
    bool finish = false;
    int  guard  = 0;
    /* ================= HARD TERMINATION GUARANTEE =================
       This loop shortens the "safe" segment every time it meets a blocked
       sample. When the LAST waypoint cannot be connected from path[size-2] --
       which is the normal situation when the goal sits inside / next to the
       obstacle band, or outside the region where the ESDF has been evaluated --
       the loop used to re-enter with EXACTLY the same state
       (end_idx == size-1, cut_start == path[size-2]) and spin forever.
       Because the planner is a single-threaded ros::spin() node, that froze the
       whole process for good: no more /planning/trajectory, no more broadcast,
       the drone silently stopped taking part in the formation. It was observed
       live with these two messages being the LAST ever printed by the node:
         "Failed in A star path searching !!! 0.2 seconds time limit exceeded."
         "I don't know why, but only try A* again"
       The round budget plus "always accept the terminal point" below remove it.
       ============================================================= */
    const int max_rounds = size + 2;
    while (!finish && guard++ < max_rounds) {
        for (int i = end_idx; i < size; i++){
            bool is_safe = true;
            Vector3d check_pt = path[i];
            int check_num = ceil((check_pt - cut_start).norm() / 0.01);
            /* Guard the sampling count: 0 would make 1.0/check_num == Inf and
               turn every sample into NaN (NaN -> undefined int index conversion),
               and a garbage cut_start could demand billions of samples, which is
               a hang of its own. 1e4 samples at 1 cm == 100 m, longer than any
               segment A* can return here. */
            if (check_num < 1)     check_num = 1;
            if (check_num > 10000) check_num = 10000;
            // check collision
            for (int j=0; j<=check_num; j++){
                double alpha = double(1.0 / check_num) * j;
                Vector3d check_safe_pt = (1 - alpha) * cut_start + alpha * check_pt;
                if (checkOccupancy_esdf(check_safe_pt)){
                    is_safe = false;
                    break;
                }
            }
            
            if (is_safe && i == (size -1)){
                finish = true;
                simple_path.push_back(check_pt);
            }

            if (is_safe){
                continue;
            }
            else{
                end_idx = i;
                cut_start = path[end_idx-1];
                simple_path.push_back(cut_start);
            }
        }

        if (!finish)
        {
            /* The scan reached the last waypoint and it could not be connected
               from path[size-2] (or the round budget ran out). Re-scanning would
               reproduce the exact same state, so accept the terminal point and
               stop: keeping the drone planning -- and letting the back-end
               optimizer / the next replan cycle deal with the last segment -- is
               strictly better than freezing the node forever as before. */
            if ((simple_path.back() - path[size - 1]).norm() > 1e-9)
                simple_path.push_back(path[size - 1]);
            ROS_WARN("A* simple path: the final segment could not be cleared "
                     "(path_size=%d, round=%d), keeping the terminal waypoint.",
                     size, guard);
            finish = true;
        }
    }

    // debug
    if (is_show_debug){
        cout << "[simple A* path] : --------- " << endl;
        int n1 = simple_path.size();
        cout << "simple A* path size : " << n1 << endl;
        for (int i=0; i<n1; i++)
            cout << simple_path[i].transpose() << endl;
    }
    

    // check the near points and delete it
    bool near_flag;
    do
    {
        near_flag = false;
        if (simple_path.size() <=2){
            near_flag = false;
            break;
        }

        int num_same_check = simple_path.size();
        for (int i=0; i<num_same_check-1; i++){
            double len = (simple_path[i+1] - simple_path[i]).norm();
            if (len < 0.3){
                simple_path.erase(simple_path.begin()+i+1);
                near_flag = true;
                break;
            }
        }
        
    } while (near_flag);

    // debug
    if (is_show_debug){
        cout << "[delete simple path] : --------- " << endl;
        int n2 = simple_path.size();
        cout << "delete simple path size : " << n2 << endl;
        for (int i=0; i<n2; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    // check the path and add a point if two of them are too far away
    bool too_long_flag;
    const double length_threshold = 3;
    int debug_num = 0;
    do
    {
        debug_num ++;
        too_long_flag = false;
        int num = simple_path.size();
        for (int i=0; i<num-1; i++){
            double leng = (simple_path[i+1] - simple_path[i]).norm();
            if (leng > length_threshold){
                Vector3d insert_point = (simple_path[i+1] + simple_path[i]) / 2;
                simple_path.insert(simple_path.begin()+i+1 ,insert_point);
                too_long_flag = true;
                break;
            }
        }
    } while (too_long_flag && debug_num < 10);

    // debug
    if (is_show_debug){
        cout << "[final simple path] : --------- " << endl;
        int n3 = simple_path.size();
        cout << "final simple path size : " << n3 << endl;
        for (int i=0; i<n3; i++)
            cout << simple_path[i].transpose() << endl;
    }
    
    return simple_path;    
}
