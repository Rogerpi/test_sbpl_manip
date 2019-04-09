////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#include <smpl/graph/manip_lattice_action_space.h>

// standard includes
#include <limits>
#include <numeric>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>


namespace sbpl {
namespace motion {

bool ManipLatticeActionSpace::init(ManipLattice* space)
{
    if (!ActionSpace::init(space)) {
        return false;
    }

    // NOTE: other default thresholds will be set in readParameters, with
    // default values specified in PlanningParams
    m_mprim_thresh[MotionPrimitive::Type::LONG_DISTANCE] =
            std::numeric_limits<double>::infinity();

    clear();

    RobotModel* robot = planningSpace()->robot();

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();

    if (!m_fk_iface) {
        SMPL_WARN("Manip Lattice Action Set requires Forward Kinematics Interface");
    }

    if (!m_ik_iface) {
        SMPL_WARN("Manip Lattice Action Set recommends Inverse Kinematics Interface");
    }

    useMultipleIkSolutions(false);
    useLongAndShortPrims(false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
    useAmp(MotionPrimitive::SNAP_TO_RPY, true);
    useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, true);
    useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ, 0.4);
    ampThresh(MotionPrimitive::SNAP_TO_RPY, 0.4);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, 0.4);
    ampThresh(MotionPrimitive::SHORT_DISTANCE, 0.4);
    motion_plan_request_type_ = 0;
    return true;
}

/// \brief Load motion primitives from file.
///
/// Read in the discrete variable deltas for each motion. If short distance
/// motion primitives are included in the file, they are also enabled.
///
/// Action Set File Format
///
/// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
/// dv11         dv12        ... dv1m
/// ...
/// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
/// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
/// ...
/// dvi1         dvi2        ... dvim
bool ManipLatticeActionSpace::load(const std::string& action_filename)
{
    FILE* fCfg = fopen(action_filename.c_str(), "r");

    if (!fCfg) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    char sTemp[1024] = { 0 };
    int nrows = 0;
    int ncols = 0;
    int short_mprims = 0;

    // read and check header
    if (fscanf(fCfg, "%1023s", sTemp) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        SMPL_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // read number of actions
    if (fscanf(fCfg, "%d", &nrows) < 1) {
        SMPL_ERROR("Parsed string has length < 1..");
        return false;
    }

    // read length of joint array
    if (fscanf(fCfg, "%d", &ncols) < 1) {
        SMPL_ERROR("Parsed string has length < 1...");
        return false;
    }

    // read number of short distance motion primitives
    if (fscanf(fCfg, "%d", &short_mprims) < 1) {
        SMPL_ERROR("Parsed string has length < 1....");
        return false;
    }

    if (short_mprims == nrows) {
        SMPL_WARN("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }
 
    std::vector<double> mprim(ncols-2, 0);

    bool have_short_dist_mprims = short_mprims > 0;
    /*if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }*/

    ManipLattice* lattice =  static_cast<ManipLattice*>(planningSpace());

    ROS_INFO_STREAM("SIZE 1: "<<m_mprims.size());
    for (int i = 0; i < nrows; ++i) {

        // read joint delta
        for (int j = 0; j < ncols-2; ++j) {
            double d;
            if (fscanf(fCfg, "%lf", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1");
                return false;
            }
            if (feof(fCfg)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }

            mprim[j] = d * lattice->resolutions()[j];
            SMPL_DEBUG("Got %0.3f deg -> %0.3f rad", d, mprim[j]);
        }

        int group;
        if(fscanf(fCfg, "%i", &group)<1)
        {
            SMPL_ERROR("No planning group defiend for this MP!");
        }

        double weight;
        if(fscanf(fCfg, "%lf", &weight)<1)
        {
            SMPL_ERROR("No weight defiend for this MP!");
        }
        if (i < (nrows - short_mprims)) {
            addMotionPrim(mprim, group, weight, false);
        } else {
            addMotionPrim(mprim, group, weight, true);
        }
    }
    fclose(fCfg);
    return true;
}

/// \brief Add a long or short distance motion primitive to the action set
/// \param mprim The angle delta for each joint, in radians
/// \param short_dist true = short distance; false = long distance
/// \param add_converse Whether to add the negative of this motion primitive
///     to the action set
void ManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim, int group, double weight, 
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }
    m.action.push_back(mprim);
    m.group = sbpl::motion::GroupType(group);
    m.weight = weight;
    m_mprims.push_back(m);
    if (add_converse) {
      /*
        m2.action.push_back(mprim);
        m2.group = sbpl::motion::GroupType(group);
        m2.weight = weight;*/
        for (RobotState& state : m.action) {
            for (size_t i = 0; i < state.size(); ++i) {
                state[i] *= -1.0;
            }

        }
        m_mprims.push_back(m);

    }

}

/// \brief Remove long and short motion primitives and disable adaptive motions.
///
/// Thresholds for short distance and adaptive motions are retained
void ManipLatticeActionSpace::clear()
{
    m_mprims.clear();
    
    // add all amps to the motion primitive set
    MotionPrimitive mprim;


    std::vector<sbpl::motion::GroupType> groups({sbpl::motion::GroupType::ARM,sbpl::motion::GroupType::R5M});

    for (const auto gtype:groups) {
        mprim.type = MotionPrimitive::SNAP_TO_RPY;
        mprim.action.clear();

        mprim.group = gtype;
        mprim.weight = 0.5;
        m_mprims.push_back(mprim);

        mprim.type = MotionPrimitive::SNAP_TO_XYZ;
        mprim.action.clear();
        mprim.group = gtype;
        mprim.weight = 0.5;
        m_mprims.push_back(mprim);

        mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
        mprim.action.clear();
        mprim.group = gtype;
        mprim.weight = 0.5;
        m_mprims.push_back(mprim);

    }


    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = (i == MotionPrimitive::Type::LONG_DISTANCE);
    }
}

int ManipLatticeActionSpace::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ManipLatticeActionSpace::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type) const
{
    return m_mprim_enabled[type];
}

bool ManipLatticeActionSpace::useMultipleIkSolutions() const
{
    return m_use_multiple_ik_solutions;
}

bool ManipLatticeActionSpace::useLongAndShortPrims() const
{
    return m_use_long_and_short_dist_mprims;
}

double ManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type) const
{
    return m_mprim_thresh[type];
}

void ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type, bool enable)
{
    m_mprim_enabled[type] = enable;
}

void ManipLatticeActionSpace::useMultipleIkSolutions(bool enable)
{
    m_use_multiple_ik_solutions = enable;
}

void ManipLatticeActionSpace::useLongAndShortPrims(bool enable)
{
    m_use_long_and_short_dist_mprims = enable;
}

void ManipLatticeActionSpace::ampThresh(
    MotionPrimitive::Type type,
    double thresh)
{
    if (type != MotionPrimitive::LONG_DISTANCE) {
        m_mprim_thresh[type] = thresh;
    }
}

void ManipLatticeActionSpace::updateStart(const RobotState& start)
{
    RobotPlanningSpaceObserver::updateStart(start);
}

void ManipLatticeActionSpace::updateGoal(const GoalConstraint& goal)
{
    ROS_WARN_STREAM("MANIP LATTICE ACTION SPACE UPDATE GOAL: Goal is accessed from m_space. Call RobotPlanningSpaceObserver::updateGoal");
    GoalConstraint goal1 =  planningSpace()->goal();
    GoalConstraint goal2 = planningSpace()->goal2();


    RobotPlanningSpaceObserver::updateGoal(goal);
    //RobotPlanningSpaceObserver::updateGoal();
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    ROS_ERROR_STREAM("APPLY for no group called");
    if (!m_fk_iface) {
        return false;
    }

    std::vector<double> pose;
    if (!m_fk_iface->computePlanningLinkFK(parent, pose)) {
        SMPL_ERROR("Failed to compute forward kinematics for planning link");
        return false;
    }

    // get distance to the goal pose
    double goal_dist = 0.0;
    double start_dist = 0.0;
    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristic* h = planningSpace()->heuristic(0);
        ROS_ERROR_STREAM("SIMPLE APPLY!!!; Get metric goa ldistance gonna be called, for switching between short and long motion primitives");

        goal_dist = h->getMetricGoalDistance(pose[0], pose[1], pose[2]);
        //start_dist = h->getMetricStartDistance(pose[0], pose[1], pose[2]);
        start_dist = 0; //Is not used at all, lets avoid using a function that is not implemented for dual arm manipulation
    }

    std::vector<Action> act;
    for (const MotionPrimitive& prim : m_mprims) {
    act.clear();
        if (getAction(parent, goal_dist, start_dist, prim, act, false)) {
            
            actions.insert(actions.end(), act.begin(), act.end());
        }
    }
    if (actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified...");
    }

    return true;
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions, ActionsWeight& weights, int group)
{
    //ROS_INFO_STREAM("-------------APPLY---(Group: "<<group<<" ) ---------------");
    if (!m_fk_iface) {
        return false;
    }

    std::vector<double> pose;

    //std::string ee_name = (group == sbpl::motion::GroupType ::R5M) ? "R5M_Jaw" : "ECA_Jaw";
    std::string ee_name;
    switch(group)  {
        case sbpl::motion::GroupType::R5M:
            ee_name = "R5M_Jaw";
            break;
        case sbpl::motion::GroupType::ARM:
            ee_name = "ECA_Jaw";
            break;
        case sbpl::motion::GroupType::BASE:
            ee_name = "AUV_roll_link";
            break;
        default:
            ROS_ERROR_STREAM("MLAS Group not recognized... press enter");
            std::getchar();
    }

    //ROS_WARN_STREAM(" GONNA CALL COMPUTE PLANNING LINK FK for switching between mp");
    //if (!m_fk_iface->computePlanningLinkFK(parent, pose)) { //old
    if (!m_fk_iface->computeFK(parent, ee_name, pose)){
        SMPL_ERROR("Failed to compute forward kinematics for planning link!");
        std::getchar();
        return false;
    }

    // get distance to the goal pose
    double goal_dist = 0.0;
    double start_dist = 0.0;
    if (planningSpace()->numHeuristics() > 0) {

        RobotHeuristic* h = planningSpace()->heuristic(0);
        goal_dist = h->getMetricGoalDistance(pose[0], pose[1], pose[2],sbpl::motion::GroupType(group));

        //start_dist = h->getMetricStartDistance(pose[0], pose[1], pose[2]);
    }

    std::vector<Action> act;
    if(group!=-1)
    {
        std::for_each( m_mprims.begin(), m_mprims.end(),
        [&](MotionPrimitive& mp ) mutable
           {
               //ROS_WARN_STREAM("MP GROUP: "<<mp.group);
            
             if( mp.group == group || mp.group == sbpl::motion::GroupType::ANY)
             { 
                act.clear();
               if (getAction(parent, goal_dist, start_dist, mp, act, false)) {
                actions.insert(actions.end(), act.begin(), act.end());
                weights.push_back(mp.weight);
                SMPL_INFO_STREAM("New Action fetched for group "<<group<<" type: "<<mp.type);
                /*for(int i=0;i<act[0].size();i++)
                    for(size_t j = 0; j < act[0][i].size(); ++j)
                        {
                            SMPL_WARN_STREAM("First dim size "<<act[0].size()<<" Second "<<act[0][i].size());
                            SMPL_WARN_STREAM("val["<<i<<","<<j<<"] "<<(act[0][i][j]));
                        }*/
                    
                }
             }
             
            } );
    }
    else
    {
        ROS_ERROR_STREAM("GROUP -1 Called!!!");
            for (const MotionPrimitive& prim : m_mprims) {
            act.clear();
            if (getAction(parent, goal_dist, start_dist, prim, act, false)) {
                actions.insert(actions.end(), act.begin(), act.end());
                weights.push_back(prim.weight);
            }

        }
    }
    if (actions.empty()) {
        SMPL_WARN_ONCE("MPAS APPLY: No motion primitives specified ");
    }

    /*for(int i=0;i<actions.size();i++){
        for(int j=0;j<actions[i].size();j++)
        {
            //SMPL_WARN("actions of[%i][%i] found of size %i",i,j,actions.size());
            for(int k=0;k<(actions[i][j]).size();k++)
            SMPL_INFO_STREAM("Action ["<<i<<"]["<<j<<"]["<<k<<" is "<<actions[i][j][k]);
        }
    }*/

    //ROS_INFO_STREAM("------------------END APPLY---------------------");

    return true;
}

bool ManipLatticeActionSpace::applyPredActions(const RobotState& parent, std::vector<Action>& actions, ActionsWeight& weights, int group) 
{
    if (!m_fk_iface) {
        return false;
    }

    std::vector<double> pose;
    if (!m_fk_iface->computePlanningLinkFK(parent, pose)) {
        SMPL_ERROR("Failed to compute forward kinematics for planning link");
        return false;
    }

    // get distance to the goal pose
    double goal_dist = 0.0;
    double start_dist = 0.0;
    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristic* h = planningSpace()->heuristic(0);
        goal_dist = h->getMetricGoalDistance(pose[0], pose[1], pose[2]);
        start_dist = h->getMetricStartDistance(pose[0], pose[1], pose[2]);
    }

    std::vector<Action> act;
    
    if(group!=-1)
    {
        std::for_each( m_mprims.begin(), m_mprims.end(),
        [&](MotionPrimitive& mp ) mutable
        {
            if( mp.group == group || mp.group == sbpl::motion::GroupType::ANY)
            { 
                act.clear();
               if (getAction(parent, goal_dist, start_dist, mp, act, false)) {
                actions.insert(actions.end(), act.begin(), act.end());
                weights.push_back(mp.weight);
                }
            }
             
        } );
    }
    else
    {
        for (const MotionPrimitive& prim : m_mprims) {
            act.clear();
            if (getAction(parent, goal_dist, start_dist, prim, act, false)) {
                actions.insert(actions.end(), act.begin(), act.end());
                weights.push_back(prim.weight);
            }
        }
    }
    if (actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified applyPredAct");
    }

    return true;
}

bool ManipLatticeActionSpace::getAction(
    const RobotState& parent,
    double goal_dist,
    double start_dist,
    const MotionPrimitive& mp,
    std::vector<Action>& actions, bool isPredecessor)
{
    //ROS_WARN_STREAM("Group: "<<mp.group <<" R5M Group "<<sbpl::motion::GroupType::R5M);
    if (!mprimActive(start_dist, goal_dist, mp.type)) {
        ROS_DEBUG_STREAM("prim not active");
        return false;
    }

    //GoalType goal_type = planningSpace()->goal().type;
    const std::vector<double>& goal_pose = planningSpace()->goal().pose;
    const std::vector<double>& goal_pose2 = planningSpace()->goal2().pose;

    switch (mp.type) {
    case MotionPrimitive::LONG_DISTANCE:
    {
        actions.resize(1);
        return applyMotionPrimitive(parent, mp, actions[0], isPredecessor);
    }
    case MotionPrimitive::SHORT_DISTANCE:
    {
        actions.resize(1);
        return applyMotionPrimitive(parent, mp, actions[0], isPredecessor);
    }
    case MotionPrimitive::SNAP_TO_RPY:
    {
        if (mp.group == sbpl::motion::GroupType::R5M)
            return computeIkAction(
                parent,
                goal_pose2,
                goal_dist,
                ik_option::RESTRICT_XYZ, "R5M",
                actions);
        else
            return computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::RESTRICT_XYZ, "ECA",
                    actions);

    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        if (mp.group == sbpl::motion::GroupType::R5M)
            return computeIkAction(
                parent,
                goal_pose2,
                goal_dist,
                ik_option::RESTRICT_RPY, "R5M",
                actions);
        else
            return computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::RESTRICT_RPY, "ECA",
                    actions);

    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL && planningSpace()->goal2().type != GoalType::JOINT_STATE_GOAL ) { //assume goal and goal2 are the same type.. but 2nd goal comparisson is added to show an error in the else
            if (mp.group == sbpl::motion::GroupType::R5M)
                return computeIkAction(
                    parent,
                    goal_pose2,
                    goal_dist,
                    ik_option::UNRESTRICTED, "R5M",
                    actions);
            else
                return computeIkAction(
                        parent,
                        goal_pose,
                        goal_dist,
                        ik_option::UNRESTRICTED, "ECA",
                        actions);
        } else {
            ROS_ERROR_STREAM("Check why I'm here");
            std::getchar();
            // goal is 7dof; instead of computing  IK, use the goal itself as
            // the IK solution
            actions.resize(1);
            actions[0].resize(1);
            actions[0][0] = planningSpace()->goal().angles;
        }
        return true;
    }
    default:
        SMPL_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ManipLatticeActionSpace::applyMotionPrimitive(
    const RobotState& state,
    const MotionPrimitive& mp,
    Action& action, bool isPredecessor)
{
    //SIZE
    action = mp.action;
    Action temp = action;

    for (size_t i = 0; i < action.size(); ++i) {
        if (action[i].size() != state.size()) {
            return false;
        }

        
        Eigen::Matrix2d worldToBody;
        worldToBody(0,0) = cos(state[3]);//+action[i][3]);
        worldToBody(0,1) = -sin(state[3]);//+action[i][3]);
        worldToBody(1,0) = sin(state[3]);//+action[i][3]);
        worldToBody(1,1) = cos(state[3]);//+action[i][3]);
        
        
        Eigen::Vector2d actionInWorldFrame = worldToBody*Eigen::Vector2d(action[i][0],action[i][1]);
        temp[i][0] = actionInWorldFrame[0];
        temp[i][1] = actionInWorldFrame[1];

          if(isPredecessor)
            for (size_t j = 0; j < action[i].size(); ++j) {
                temp[i][j] = state[j] - temp[i][j] ;
   
            }
        else
            for (size_t j = 0; j < action[i].size(); ++j) {
                temp[i][j] = temp[i][j] + state[j];
               /* if(j==0 || j==1)
                {
                    ROS_ERROR_STREAM("Action without transformation "<<action[i][j]+state[j]);
                    ROS_ERROR_STREAM("Action with transformation "<<temp[i][j]);
                }*/

            }

        action = temp;

    } 
    return true;
}

bool ManipLatticeActionSpace::computeIkAction(
    const RobotState& state,
    const std::vector<double>& goal,
    double dist_to_goal,
    ik_option::IkOption option,
    std::vector<Action>& actions)
{

    ROS_ERROR_STREAM("ManipLatticeActionSpace computeIKAction: Should not be used"); std::getchar();
    if (!m_ik_iface) {
        return false;
    }

    if (m_use_multiple_ik_solutions) {
        //get actions for multiple ik solutions
        std::vector<std::vector<double>> solutions;
        if (!m_ik_iface->computeIK(goal, state, solutions, option)) {
            SMPL_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)",
                    to_cstring(option), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
            return false;
        }
        actions.resize(solutions.size());
        for (size_t a = 0; a < actions.size(); a++){
            actions[a].resize(1);
            actions[a][0] = solutions[a];
        }
    } else {
        //get single action for single ik solution
        std::vector<double> ik_sol;
        if (!m_ik_iface->computeIK(goal, state, ik_sol)) {
            SMPL_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)", to_cstring(option), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
            return false;
        }
        actions.resize(1);
        actions[0].resize(1);
        actions[0][0] = ik_sol;
    }

    return true;
}

    bool ManipLatticeActionSpace::computeIkAction(
            const RobotState& state,
            const std::vector<double>& goal,
            double dist_to_goal,
            ik_option::IkOption option, const std::string & arm,
            std::vector<Action>& actions)
    {


    ROS_INFO_STREAM("ManipLatticeActionSpace: compute ik action "<<arm);
        if (!m_ik_iface) {
            return false;
        }

        if (m_use_multiple_ik_solutions) {
            //get actions for multiple ik solutions
            std::vector<std::vector<double>> solutions;
            if (!m_ik_iface->computeIK(goal, state, solutions, arm, option)) {
                        SMPL_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)",
                                   to_cstring(option), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
                return false;
            }
            actions.resize(solutions.size());
            for (size_t a = 0; a < actions.size(); a++){
                actions[a].resize(1);
                actions[a][0] = solutions[a];
            }
        } else {
            //get single action for single ik solution
            std::vector<double> ik_sol;
            if (!m_ik_iface->computeIK(goal, state, ik_sol, arm)) {
                        SMPL_DEBUG("IK '%s' failed. (dist_to_goal: %0.3f)  (goal: xyz: %0.3f %0.3f %0.3f rpy: %0.3f %0.3f %0.3f)", to_cstring(option), dist_to_goal, goal[0], goal[1], goal[2], goal[3], goal[4], goal[5]);
                return false;
            }
            actions.resize(1);
            actions[0].resize(1);
            actions[0][0] = ik_sol;
        }

        return true;
    }


    //Not using start_dist for anything
bool ManipLatticeActionSpace::mprimActive(
    double start_dist,
    double goal_dist,
    MotionPrimitive::Type type) const
{

    ROS_WARN_STREAM("Type is "<<type<<", start_dist "<<start_dist<<", goal_dist "<<goal_dist);
    /**/
   if (type == MotionPrimitive::LONG_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return true;
        }
        const bool near_goal =
                goal_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_start =
                start_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_endpoint = near_goal;// || near_start;
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] && near_endpoint);
    } else if (type == MotionPrimitive::SHORT_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return m_mprim_enabled[type];
        }
        const bool near_goal = goal_dist <= m_mprim_thresh[type];
        const bool near_start = start_dist <= m_mprim_thresh[type];
        const bool near_endpoint = near_goal;//|| near_start;
        return m_mprim_enabled[type] && near_endpoint;
    } else {
        return m_mprim_enabled[type] && goal_dist <= m_mprim_thresh[type];
    }
}

} // namespace motion
} // namespace sbpl
