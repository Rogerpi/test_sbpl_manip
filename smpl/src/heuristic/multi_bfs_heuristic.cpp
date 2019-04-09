#include <smpl/heuristic/multi_bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/intrusive_heap.h>
#include <smpl/grid.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace sbpl {
namespace motion {

static const char* LOG = "heuristic.multi_bfs";

MultiBfsHeuristic::~MultiBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiBfsHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{

    ROS_INFO_STREAM("MULTI BFS HEURISTIC");
    ROS_INFO_STREAM("GRID RESOLUTION: "<<grid->resolution());
     if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;

    ros::NodeHandle nh;
    fk_pub_debug = nh.advertise< visualization_msgs::Marker >("fk_goal", 10);


    m_pp = space->getExtension<PoseProjectionExtension>();
    if (m_pp) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_manip = dynamic_cast<ManipLattice*> (space);
    syncGridAndBfs();
    return true;
}

void MultiBfsHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void MultiBfsHeuristic::setBaseInflationRadius(double radius)
{
    m_base_inflation_radius = radius;
}

void MultiBfsHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void MultiBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    ROS_WARN_STREAM("MBFS UpdateGoal. argument not used, got it from planningSpace (ActionSpace attribute)");


    GoalConstraint goal1 =  planningSpace()->goal();
    GoalConstraint goal2 = planningSpace()->goal2();


    int gx, gy, gz, base_gx, base_gy, base_gz;
    int gx2, gy2, gz2;

    int gxb1, gxb2, gyb1, gyb2, gzb1, gzb2;
    grid()->worldToGrid(
            goal1.tgt_off_pose[0], goal1.tgt_off_pose[1], goal1.tgt_off_pose[2],
            gx, gy, gz);

    grid()->worldToGrid(
            goal2.tgt_off_pose[0], goal2.tgt_off_pose[1], goal2.tgt_off_pose[2],
            gx2, gy2, gz2);

    m_goal_x = gx;
    m_goal_y = gy;
    m_goal_z = gz;

    m_goal_x2 = gx2;
    m_goal_y2 = gy2;
    m_goal_z2 = gz2;

    //SMPL_ERROR_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);
    
    if(goal.angles.empty())
    {

        int listSize = ((double)(1.8/grid()->resolution()))+1;
        std::vector<int> inputGoals((listSize*(listSize/2)*(listSize/3))*3);
        std::vector<Eigen::Vector3d> centers;
        SMPL_ERROR_STREAM("Goal Region Size "<<listSize<<" resolution "<<grid()->resolution()<<" total size "<<inputGoals.size());
        int idx = 0;
        int xSign = 1,ySign = 1, zSign = 1;
        for(int i=0;i<listSize;i++)
        {
            for(int j=0;j<listSize/2;j++)
               for(int k=0;k<listSize/3;k++)
                {   
                    if(i>listSize/2)
                        xSign = -(i-std::ceil(listSize/2));
                    else
                        xSign = i;
                    if(j>listSize/4)
                        ySign = -(j-std::ceil(listSize/4));
                    else
                        ySign = j;
                    if(k>listSize/4)
                        zSign = -(k-std::ceil(listSize/4));
                    else
                        zSign = k;
                    inputGoals[idx]  = (gx+gx2)/2 + xSign;
                    inputGoals[idx+1] = (gy+gy2)/2 + ySign; //+ abs(gy-gy2)*0.25
                    inputGoals[idx+2] = (gz+gz2)/2 + zSign;
                    
                    Eigen::Vector3d p;
                    grid()->gridToWorld( inputGoals[idx],  inputGoals[idx+1],  inputGoals[idx+2], p.x(), p.y(), p.z());
                    centers.push_back(p);
                    idx+=3; 
                }
        }

        /*

        double x_offset = -0.75;
        double z_offset = 0.372;
        double y_offset = 0.0;

        grid()->worldToGrid(
                goal1.tgt_off_pose[0]+x_offset, goal1.tgt_off_pose[1]+y_offset, goal1.tgt_off_pose[2]+z_offset,
                gxb1, gyb1, gzb1);

        grid()->worldToGrid(
                goal2.tgt_off_pose[0]+x_offset, goal2.tgt_off_pose[1]+y_offset, goal2.tgt_off_pose[2]+z_offset,
                gxb2, gyb2, gzb2);

        int x_eca_max = (int) (0.2/grid()->resolution())+1;
        int y_eca_max = (int)(0.1/grid()->resolution())+1;
        int z_eca_max = (int)(0.6/grid()->resolution())+1;
        int x_eca_min = (int)(-0.6/grid()->resolution());
        int y_eca_min = (int)(-0.73/grid()->resolution());
        int z_eca_min = (int)(-0.35/grid()->resolution());

        int x_r5m_max = (int) (0.45/grid()->resolution())+1;
        int y_r5m_max = (int)(0.76/grid()->resolution())+1;
        int z_r5m_max = (int)(0.7/grid()->resolution())+1;
        int x_r5m_min = (int)(-0.75/grid()->resolution());
        int y_r5m_min = (int)(-0.4/grid()->resolution());
        int z_r5m_min = (int)(-0.6/grid()->resolution());

        std::vector<int> inputGoalsECA, inputGoalsR5M, inputGoalsInt;
        std::vector<Eigen::Vector3d> centersECA, centersR5M, centersInt;

        int idx = 0;
        for(int i=x_eca_min;i<x_eca_max;i++)
            for(int j=y_eca_min;j<y_eca_max;j++)
                for(int k=z_eca_min;k<z_eca_max;k++)
                {
                    if (sqrt(pow(i*grid()->resolution(),2)+pow(j*grid()->resolution(),2)+pow(k*grid()->resolution(),2))>0.7)
                        continue; //discard some corners
                    inputGoalsECA.push_back(gxb1 + i);
                    inputGoalsECA.push_back(gyb1 + j);
                    inputGoalsECA.push_back(gzb1 + k);

                    Eigen::Vector3d p;
                    grid()->gridToWorld( inputGoalsECA[idx],  inputGoalsECA[idx+1],  inputGoalsECA[idx+2], p.x(), p.y(), p.z());
                    centersECA.push_back(p);
                    idx+=3;
                    
                }
        idx = 0;
        for(int i=x_r5m_min;i<x_r5m_max;i++)
            for(int j=y_r5m_min;j<y_r5m_max;j++)
                for(int k=z_r5m_min;k<z_r5m_max;k++)
                {
                    if (sqrt(pow(i*grid()->resolution(),2)+pow(j*grid()->resolution(),2)+pow(k*grid()->resolution(),2))>0.66)
                        continue; //discard some corners
                    inputGoalsR5M.push_back(gxb2 + i);
                    inputGoalsR5M.push_back(gyb2 + j);
                    inputGoalsR5M.push_back(gzb2 + k);

                    Eigen::Vector3d p;
                    grid()->gridToWorld( inputGoalsR5M[idx],  inputGoalsR5M[idx+1],  inputGoalsR5M[idx+2], p.x(), p.y(), p.z());
                    centersR5M.push_back(p);
                    idx+=3;

                }

        for( int i = 0; i<centersECA.size(); i++) {
            auto c_ECA = centersECA[i];
            for (auto c_R5M : centersR5M) {
                double dist = (c_ECA - c_R5M).norm();
                if (dist > grid()->resolution() / 2)
                    continue;
                centersInt.push_back(Eigen::Vector3d(c_ECA));
                inputGoalsInt.push_back(inputGoalsECA[i*3]);
                inputGoalsInt.push_back(inputGoalsECA[i*3+1]);
                inputGoalsInt.push_back(inputGoalsECA[i*3+2]);
            }
        }



        visual::Color colorECA, colorR5M, color;
        colorECA.r = 0.0f;
        colorECA.g = 1.0f;
        colorECA.b = 0.0f;
        colorECA.a = 0.25f;

        colorR5M.r = 0.0f;
        colorR5M.g = 0.0f;
        colorR5M.b = 1.0f;
        colorR5M.a = 0.25f;

        color.r = 238.0f / 255.0f;
        color.g = 100.0f / 255.0f;
        color.b = 149.0f / 255.0f;
        color.a = 0.4f;

        SV_SHOW_INFO (visual::MakeCubesMarker(
                centersInt,
                grid()->resolution(),
                color,
                grid()->getReferenceFrame(),
                "bfs_base_goals_int"));

        SV_SHOW_INFO (visual::MakeCubesMarker(
                centersECA,
                grid()->resolution(),
                colorECA,
                grid()->getReferenceFrame(),
                "bfs_base_goals_eca"));

        SV_SHOW_INFO (visual::MakeCubesMarker(
                centersR5M,
                grid()->resolution(),
                colorR5M,
                grid()->getReferenceFrame(),
                "bfs_base_goals_r5m"));
*/


        visual::Color color;

        color.r = 238.0f / 255.0f;
        color.g = 100.0f / 255.0f;
        color.b = 149.0f / 255.0f;
        color.a = 0.4f;

        SV_SHOW_INFO (visual::MakeCubesMarker(
                centers,
                grid()->resolution(),
                color,
                grid()->getReferenceFrame(),
                "bfs_base_goals"));
        /*
        visual::Color color;

        color.r = 238.0f / 255.0f;
        color.g = 100.0f / 255.0f;
        color.b = 149.0f / 255.0f;
        color.a = 0.4f;

        std::vector<Eigen::Vector3d> grid_centers;
        for(int i=-10;i<10;i++)
        {
            for(int j=-10;j<10;j++)
            {
                for(int k=1;k<6;k++)
                {
                    Eigen::Vector3d p;
                    grid()->gridToWorld( gx+i,  gy+j,  gz+k, p.x(), p.y(), p.z());
                    
                    grid_centers.push_back(p);
                }
            }   
        }
        SV_SHOW_INFO (visual::MakeCubesMarker(
                grid_centers,
                grid()->resolution(),
                color,
                grid()->getReferenceFrame(),
                "Grid"));

        */

        //int numGoals = m_bfs[GroupType::BASE]->run < std::vector<int>::iterator >(inputGoalsInt.begin(),inputGoalsInt.end());
        int numGoals = m_bfs[GroupType::BASE]->run < std::vector<int>::iterator >(inputGoals.begin(),inputGoals.end());
        if (!numGoals)
        {
            SMPL_ERROR_NAMED(LOG, "Grid Base Heuristic goal is out of BFS bounds");
        }
        else
        {
            SMPL_ERROR_STREAM("Number of base goal added "<<numGoals);
        }
    }
    else
    {
        goal_config = goal.angles;
        goal_config2 = goal2.angles;
        /*SMPL_ERROR_STREAM("Goal config is "<<goal_config[0]<<","<<goal_config[1]<<","
            <<goal_config[2]<<","<<goal_config[3]<<","<<goal_config[4]<<","<<goal_config[5]<<","
            <<goal_config[6]<<","<<goal_config[7]);*/
        grid()->worldToGrid(
                goal.angles[0], goal.angles[1], 5,//goal.angles[2],
                base_gx, base_gy, base_gz);
        ROS_WARN_STREAM("WHY HARDCODED 5--- TRA!");

        SMPL_ERROR_STREAM("Base Goal :"<<goal.angles[0]<<","<<goal.angles[1]<<","<<5//goal.angles[2]
            <<","<<base_gx<<","<<base_gy<<","<<base_gz);
        

        if (!m_bfs[GroupType::BASE]->inBounds(base_gx, base_gy, base_gz))
        {
            SMPL_ERROR_NAMED(LOG, "Base Heuristic goal is out of BFS bounds");
        }

        m_bfs[GroupType::BASE]->run(base_gx,base_gy,base_gz);
    }


    if (!m_bfs[GroupType::ARM]->inBounds(gx, gy, gz))
    {
        SMPL_ERROR_NAMED(LOG, "Arm Heuristic goal is out of BFS bounds");
    }
    
    m_bfs[GroupType::ARM]->run(gx, gy, gz);


    if (!m_bfs[GroupType::R5M]->inBounds(gx2, gy2, gz2))
    {
                SMPL_ERROR_NAMED(LOG, "Arm Heuristic goal is out of BFS bounds");
    }

    m_bfs[GroupType::R5M]->run(gx2, gy2, gz2);


    if (!m_pp) {
        return ;
    }
}

double MultiBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    ROS_ERROR_STREAM("Called getMetricStart distance not made for dual. enter to continue");

    int start_id = planningSpace()->getStartStateID();
    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    ROS_ERROR_STREAM("Called getMetricGoal distance not made for dual");
    //std::getchar();
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs[GroupType::ARM]->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs[GroupType::ARM]->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z, GroupType planning_group)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs[planning_group]->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs[planning_group]->getDistance(gx, gy, gz) * grid()->resolution();
    }

}

Extension* MultiBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int MultiBfsHeuristic::GetGoalHeuristic(int state_id)
{
    ROS_WARN_STREAM("Called getgoalheuristic... i.e getgoalheuristic of the arm. enter to continue");
    std::getchar();
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    return getBfsCostToGoal(*m_bfs[GroupType::ARM], dp.x(), dp.y(), dp.z());
}


int MultiBfsHeuristic::GetGoalHeuristic(int state_id, int planning_group, int base_heuristic_idx)
{

    if (!m_pp) {
        return 0;
    }

    std::string ee_link;
    Eigen::Vector3d p;
    Eigen::Affine3d pose;
    double penalty = 0;
    if(planning_group==GroupType::BASE)
    {
        //ROS_INFO_STREAM("MBFS HEUR: BEFORE PROJECT TO BASE POINT!!!");
        if (state_id==0)
            return 0;

        else if (!m_pp->projectToBasePoint(state_id, p)) {
            return 0;
        }
        ee_link = "AUV_roll_link";

        //penalty = (GetGoalHeuristic(state_id,GroupType::ARM,base_heuristic_idx) +  GetGoalHeuristic(state_id,GroupType::R5M,base_heuristic_idx))/2;
    }
    else if(planning_group==GroupType::ARM)
    {
        //ROS_INFO_STREAM("MBFS HEUR: BEFORE PROJECT TO POINT ECA MBFS!!!");
        ee_link = "ECA_Jaw";
        if (!m_pp->projectToPose(state_id, ee_link, pose)) {
            return 0;
        }
        p = pose.translation();

        GoalConstraint goal =  planningSpace()->goal();

        goal.pose; // roll pitch yaw



        Eigen::Matrix3d rot = pose.rotation();


        auto ea = rot.eulerAngles(2,1,0);




        ROS_INFO_STREAM(goal.pose[3]<< " " << ea[0]<< " " << goal.pose[4]<< " " << ea[1]<< " " << goal.pose[5]<< " " << ea[2]);
        //std::getchar();

        ROS_INFO_STREAM( std::abs(ea[0] - goal.pose[3])<<" "<<std::abs(ea[2] - goal.pose[4]) <<" "<< std::abs(ea[3] - goal.pose[5]))  ;
        //penalty = std::abs(ea[0] - goal.pose[3])*100 + std::abs(ea[2] - goal.pose[4])*100 + std::abs(ea[3] - goal.pose[5])*100  ;
        ROS_INFO_STREAM("penalty: "<<penalty);





    }
    else if(planning_group == GroupType::R5M) {
        //ROS_INFO_STREAM("MBFS HEUR: BEFORE PROJECT TO POINT R5M MBFS!!!");
        ee_link = "R5M_Jaw";
        if (!m_pp->projectToPose(state_id, ee_link, pose)) {
            return 0;

        }

        p = pose.translation();

        GoalConstraint goal =  planningSpace()->goal2();

        goal.pose; // roll pitch yaw



        Eigen::Matrix3d rot = pose.rotation();


        auto ea = rot.eulerAngles(2,1,0);



        ROS_INFO_STREAM(goal.pose[3]<< " " << ea[0]<< " " << goal.pose[4]<< " " << ea[1]<< " " << goal.pose[5]<< " " << ea[2]);
        //std::getchar();

        ROS_INFO_STREAM( std::abs(ea[0] - goal.pose[3])<<" "<<std::abs(ea[2] - goal.pose[4]) <<" "<< std::abs(ea[3] - goal.pose[5]))  ;
        //penalty = std::abs(ea[0] - goal.pose[3])*100 + std::abs(ea[2] - goal.pose[4])*100 + std::abs(ea[3] - goal.pose[5])*100  ;
        ROS_INFO_STREAM("penalty: "<<penalty);
    }
    else{
        ROS_ERROR_STREAM("No group identified. Setting BASE as group");
        return GetGoalHeuristic(state_id,GroupType::BASE,base_heuristic_idx); //does it make any sense?
    }
    
    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    SMPL_INFO_STREAM("Getting distance heursitic for group "<<planning_group);
    SMPL_INFO_STREAM("get heursitic for grid point "<<dp.x()<<","<<dp.y()<<","<<dp.z());
    SMPL_INFO_STREAM("get heursitic for world point "<<p.x()<<","<<p.y()<<","<<p.z());

    //ROS_INFO_STREAM("call getbfscosttoGoal for planning group: "<<planning_group);
   double cost  = getBfsCostToGoal(*m_bfs[planning_group], dp.x(), dp.y(), dp.z()) + penalty;

   /* WAS done to make sure FK for r5m was correct
   visualization_msgs::Marker marker;
   marker.header.frame_id = "/world";
   marker.header.stamp = ros::Time();
   marker.ns = std::to_string(planning_group);
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    fk_pub_debug.publish(marker);
    */





    //if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), ee_link, p)) {
    //    return 0.0;
    //}
    //ROS_INFO_STREAM("after project to point multi_");

    //int sx, sy, sz;
    //grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);



SMPL_INFO_STREAM("Total final heuristic Cost is "<<cost-penalty<<" penalty: "<<penalty);

SMPL_INFO_STREAM("====================================================================");
        
   return cost;
}

int MultiBfsHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("MultiBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    ROS_ERROR_STREAM("Get from to heuristic not made for dual. enter to continue");

    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("MultiBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto MultiBfsHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Eigen::Vector3d> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_bfs[GroupType::BASE]->isWall(x, y, z)) {
            Eigen::Vector3d p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_ERROR_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;
    return visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");

}

auto MultiBfsHeuristic::getValuesVisualization() -> visual::Marker
{

    if (m_goal_x < 0 || m_goal_y < 0 || m_goal_z < 0) {
        return visual::MakeEmptyMarker();
    }

    if (m_bfs[GroupType::ARM]->isWall(m_goal_x, m_goal_y, m_goal_z)) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID(),GroupType::ARM,0);
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    const size_t max_points = 4 * 4096;

    std::vector<Eigen::Vector3d> points;
    std::vector<visual::Color> colors;

    struct CostCell
    {
        int x, y, z, g;
    };
    std::queue<CostCell> cells;
    Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
    visited(m_goal_x, m_goal_y, m_goal_z) = true;
    cells.push({m_goal_x, m_goal_y, m_goal_z, 0});
    while (!cells.empty()) {
        CostCell c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            double cost_pct = (double)c.g / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);

            Eigen::Vector3d p;
            grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            if (std::fabs(p.z() -  3.73) < 1) {

                            points.push_back(p);

            colors.push_back(color);
            } 
        }

//        visited(c.x, c.y, c.z) = true;

        const int d = m_cost_per_cell * m_bfs[GroupType::ARM]->getDistance(c.x, c.y, c.z);

        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            int sx = c.x + dx;
            int sy = c.y + dy;
            int sz = c.z + dz;

            // check if neighbor is valid
            if (!m_bfs[GroupType::ARM]->inBounds(sx, sy, sz) || m_bfs[GroupType::ARM]->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_bfs[GroupType::ARM]->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }
     return visual::MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs_values");
}

    auto MultiBfsHeuristic::getValuesVisualization2() -> visual::Marker
    {
        if (m_goal_x2 < 0 || m_goal_y2 < 0 || m_goal_z2 < 0) {
            return visual::MakeEmptyMarker();
        }

        if (m_bfs[GroupType::R5M]->isWall(m_goal_x2, m_goal_y2, m_goal_z2)) {
            return visual::MakeEmptyMarker();
        }

        // hopefully this doesn't screw anything up too badly...this will flush the
        // bfs to a little past the start, but this would be done by the search
        // hereafter anyway
        int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID(),GroupType::R5M,0);
        if (start_heur == Infinity) {
            return visual::MakeEmptyMarker();
        }

                SMPL_INFO("Start cell heuristic: %d", start_heur);

        const int max_cost = (int)(1.1 * start_heur);

                SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

        // ...and this will also flush the bfs...

        const size_t max_points = 4 * 4096;

        std::vector<Eigen::Vector3d> points;
        std::vector<visual::Color> colors;

        struct CostCell
        {
            int x, y, z, g;
        };
        std::queue<CostCell> cells;
        Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
        visited(m_goal_x2, m_goal_y2, m_goal_z2) = true;
        cells.push({m_goal_x2, m_goal_y2, m_goal_z2, 0});
        while (!cells.empty()) {
            CostCell c = cells.front();
            cells.pop();

            if (c.g > max_cost || points.size() >= max_points) {
                break;
            }

            {
                double cost_pct = (double)c.g / (double)max_cost;

                visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

                auto clamp = [](double d, double lo, double hi) {
                    if (d < lo) {
                        return lo;
                    } else if (d > hi) {
                        return hi;
                    } else {
                        return d;
                    }
                };

                color.r = clamp(color.r, 0.0f, 1.0f);
                color.g = clamp(color.g, 0.0f, 1.0f);
                color.b = clamp(color.b, 0.0f, 1.0f);

                Eigen::Vector3d p;
                grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
                if (std::fabs(p.z() -  3.73) < 1) {

                    points.push_back(p);

                    colors.push_back(color);
                }
            }

//        visited(c.x, c.y, c.z) = true;

            const int d = m_cost_per_cell * m_bfs[GroupType::R5M]->getDistance(c.x, c.y, c.z);

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (!(dx | dy | dz)) {
                            continue;
                        }

                        int sx = c.x + dx;
                        int sy = c.y + dy;
                        int sz = c.z + dz;

                        // check if neighbor is valid
                        if (!m_bfs[GroupType::R5M]->inBounds(sx, sy, sz) || m_bfs[GroupType::R5M]->isWall(sx, sy, sz)) {
                            continue;
                        }

                        // check if cost can be improved
                        if (visited(sx, sy, sz)) {
                            continue;
                        }

                        visited(sx, sy, sz) = true;

                        int dd = m_cost_per_cell * m_bfs[GroupType::R5M]->getDistance(sx, sy, sz);
                        cells.push({sx, sy, sz, dd});
                    }
                }
            }
        }

        return visual::MakeCubesMarker(
                std::move(points),
                0.5 * grid()->resolution(),
                std::move(colors),
                grid()->getReferenceFrame(),
                "bfs_values2");
    }
void MultiBfsHeuristic::syncGridAndBfs()
{

    ROS_ERROR_STREAM("check SYNC GRID AND BFS. enter to  continue");

    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    std::unique_ptr<BFS_3D> temp_base, temp_arm, temp_r5m;
    temp_base.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp_base));
    temp_arm.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp_arm));
    temp_r5m.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp_r5m));

    const int cell_count = xc * yc * zc;
    int wall_count = 0, base_wall_count = 0;
    for (int x = 0; x < xc; ++x) {
        for (int y = 0; y < yc; ++y) {
            for (int z = 0; z < zc; ++z) {
                const double radius = m_inflation_radius;
                if (grid()->getDistance(x, y, z) <= radius) {
                    m_bfs[GroupType::ARM]->setWall(x, y, z);
                    m_bfs[GroupType::R5M]->setWall(x, y, z);
                    ++wall_count;
                }

                if(grid()->getDistance(x, y, z) <= m_base_inflation_radius)
                {
                    m_bfs[GroupType::BASE]->setWall(x,y,z);
                    ++base_wall_count;
                }
            }
        }
    }

    SMPL_ERROR_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
    SMPL_ERROR_NAMED(LOG, "%d/%d (%0.3f%%) walls in the base bfs heuristic of radius (%0.3f)", base_wall_count, cell_count, 100.0 * (double)base_wall_count / cell_count, m_base_inflation_radius);
}

int MultiBfsHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    //ROS_WARN_STREAM("USing get BFS Cost To Goal. Not modified, but shouldnt I guess");
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else 
    {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

} // namespace motion
} // namespace sbpl
