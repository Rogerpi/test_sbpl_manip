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

#include <smpl/graph/manip_lattice.h>

// standard includes
#include <iomanip>
#include <sstream>

// system includes
#include <Eigen/Dense>
#include <sbpl/planners/planner.h>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include "../profiling.h"

auto std::hash<sbpl::motion::ManipLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace motion {

ManipLattice::~ManipLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();
    m_state_to_id.clear();
}

bool ManipLattice::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const PlanningParams* _params,
    const std::vector<double>& resolutions,
    ActionSpace* actions)
{
    istree = false;
    apply_time = 0;
    SMPL_DEBUG_NAMED(_params->graph_log, "Initialize Manip Lattice");

    if (!actions) {
        SMPL_ERROR_NAMED(_params->graph_log, "Action Space is null");
        return false;
    }

    if (resolutions.size() != _robot->jointVariableCount()) {
        SMPL_ERROR_NAMED(_params->graph_log, "Insufficient variable resolutions for robot model");
        return false;
    }

    if (!RobotPlanningSpace::init(_robot, checker, _params)) {
        SMPL_ERROR_NAMED(_params->graph_log, "Failed to initialize Robot Planning Space");
        return false;
    }

    m_fk_iface = _robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = _robot->getExtension<InverseKinematicsInterface>();


    //dual
    //m_fk_iface2 = _robot2->getExtension<ForwardKinematicsInterface>();
    //m_ik_iface2 = _robot2->getExtension<InverseKinematicsInterface>();

    if (!m_ik_iface) {
        SMPL_DEBUG("Manip Lattice requires Inverse Kinematics Interface extension");
        return false;
    }

    m_min_limits.resize(_robot->jointVariableCount());
    m_max_limits.resize(_robot->jointVariableCount());
    m_continuous.resize(_robot->jointVariableCount());
    m_bounded.resize(_robot->jointVariableCount());
    for (int jidx = 0; jidx < _robot->jointVariableCount(); ++jidx) {
        m_min_limits[jidx] = _robot->minPosLimit(jidx);
        m_max_limits[jidx] = _robot->maxPosLimit(jidx);
        m_continuous[jidx] = _robot->isContinuous(jidx);
        m_bounded[jidx] = _robot->hasPosLimit(jidx);

        SMPL_DEBUG_NAMED(_params->graph_log, "variable %d: { min: %f, max: %f, continuous: %s, bounded: %s }",
            jidx,
            m_min_limits[jidx],
            m_max_limits[jidx],
            m_continuous[jidx] ? "true" : "false",
            m_bounded[jidx] ? "true" : "false");
    }

    m_goal_state_id = reserveHashEntry();
    SMPL_DEBUG_NAMED(_params->graph_log, "  goal state has state ID %d", m_goal_state_id);

    std::vector<int> discretization(_robot->jointVariableCount());
    std::vector<double> deltas(_robot->jointVariableCount());
    for (size_t vidx = 0; vidx < _robot->jointVariableCount(); ++vidx) {
        if (m_continuous[vidx]) {
            discretization[vidx] = (int)std::round((2.0 * M_PI) / resolutions[vidx]);
            deltas[vidx] = (2.0 * M_PI) / (double)discretization[vidx];
        } else if (m_bounded[vidx]) {
            const double span = std::fabs(m_max_limits[vidx] - m_min_limits[vidx]);
            discretization[vidx] = std::max(1, (int)std::round(span / resolutions[vidx]));
            deltas[vidx] = span / (double)discretization[vidx];
        } else {
            discretization[vidx] = std::numeric_limits<int>::max();
            deltas[vidx] = resolutions[vidx];
        }
    }

    SMPL_DEBUG_STREAM_NAMED(_params->graph_log, "  coord vals: " << discretization);
    SMPL_DEBUG_STREAM_NAMED(_params->graph_log, "  coord deltas: " << deltas);

    m_coord_vals = std::move(discretization);
    m_coord_deltas = std::move(deltas);

    m_actions = actions;

    return true;
}




void ManipLattice::PrintState(int stateID, bool verbose, FILE* fout)
{
    assert(stateID >= 0 && stateID < (int)m_states.size());

    if (!fout) {
        fout = stdout;
    }

    ManipLatticeState* entry = m_states[stateID];

    std::stringstream ss;

    if (stateID == m_goal_state_id) {
        ss << "<goal state: { ";
        switch (goal().type) {
        case GoalType::XYZ_GOAL:
        case GoalType::XYZ_RPY_GOAL:
            ss << "pose: " << goal().tgt_off_pose;
            break;
        case GoalType::JOINT_STATE_GOAL:
            ss << "state: " << goal().angles;
            break;
        default:
            assert(0);
            break;
        }
        ss << " }>";
    } else {
        ss << "{ ";
        for (size_t i = 0; i < entry->state.size(); ++i) {
            ss << std::setprecision(3) << entry->state[i];
            if (i != entry->state.size() - 1) {
                ss << ", ";
            }
        }
        ss << " }";
    }

    if (fout == stdout) {
        SMPL_DEBUG_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        SMPL_DEBUG("%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

 void ManipLattice::setMotionPlanRequestType (int request_type)
 {
    //if the request is navigation (--> 0) or manipulation (--> 1)
    //set different clearance threshold for each type
    //TODO define the clearance threshold in the yaml file
    if(request_type==0)
    {
        clearance_threshold_ = 0.0;
        dist_threshold_ = 1.2;
        collisionChecker()->setClearanceThreshold(clearance_threshold_);
    }
    else if(request_type==1)
    {
        clearance_threshold_ = 0.0;
        dist_threshold_ = 0.0;
        collisionChecker()->setClearanceThreshold(clearance_threshold_);
    }
    m_actions->setMotionPlanRequestType(request_type);
 }



void ManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{

    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << parent_entry->state);
    SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));


    //SSV_SHOW_DEBUG(getStateVisualization(parent_entry->state, "expansion"));
    auto* vis_name = "expansion";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    int goal_succ_count = 0;

    std::vector<Action> actions;
    ActionsWeight weights;
    if (!m_actions->apply(parent_entry->state, actions,weights,-1)) {
        SMPL_DEBUG("Failed to get actions");
        return;
    }
    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            SMPL_DEBUG("increasing goal succ!");
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, weights[i], is_goal_succ));

        // log successor details
        SMPL_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
        SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
    //ROS_ERROR_STREAM("Expanded node iD is "<<state_id);
    //m_expanded_states.push_back(state_id);

}

void ManipLattice::show_state(int state_id, const std::string& vis_name, int group){
    ManipLatticeState* parent_entry = m_states[state_id];
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualizationByGroup(parent_entry->state, vis_name,group));
}

void ManipLattice::GetSuccsWithExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, int expanion_step)
{
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << parent_entry->state);
    //SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));

    auto* vis_name = "expansion";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    
    int goal_succ_count = 0;

    collisionChecker()->setLastExpansionStep(expanion_step);

    std::vector<Action> actions;
    ActionsWeight weights;
    if (!m_actions->apply(parent_entry->state, actions,weights,-1)) {
        SMPL_DEBUG("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back());

        if (is_goal_succ) {
            // update goal state
            SMPL_DEBUG("increasing goal succ!");
            SMPL_DEBUG_STREAM("state_id "<<succ_state_id<<" and goal "<<m_goal_state_id);
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, weights[i], is_goal_succ));

        // log successor details
        if(is_goal_succ)
        {
            SMPL_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
            SMPL_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
            SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        coord: " << succ_coord);
            SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        state: " << succ_entry->state);
            //SMPL_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
            SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
        }
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
}

void ManipLattice::GetSuccsByGroup(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, std::vector<int>* clearance_cells, int group)
{
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << parent_entry->state);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  Source: " << parent_entry->source);
    SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id, group,0));

    auto* vis_name = "expansion";
    //ROS_INFO_STREAM("Get state visualization by group. press enter.");
    //SV_SHOW_INFO_NAMED(vis_name, getStateVisualizationByGroup(parent_entry->state, vis_name, group));


    int goal_succ_count = 0;

    std::vector<Action> actions;
    ActionsWeight weights;
    if (!m_actions->apply(parent_entry->state, actions, weights, group)) {
        SMPL_DEBUG("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());
        double distToObst = 0;
        int distToObstCells;
        double start = ros::Time::now().toSec();
        bool test;
        //ROS_WARN_STREAM("--------------Check action----------");
        if(clearance_threshold_== 0) 
            test = checkAction(parent_entry->state, action);
        else
            test = checkAction(parent_entry->state, action, distToObst, distToObstCells);
        //ROS_WARN_STREAM("----------END Check action----------");
        double duration = ros::Time::now().toSec() - start;
        //ROS_ERROR_STREAM("duration is "<<duration);
        if (!test) {
            ROS_WARN_STREAM("ML: action didnt pass checkAction");
            continue;
            //std::getchar();
        }

        
        // compute destination coords
        //ROS_WARN_STREAM("-----------state To Coord----------");
        stateToCoord(action.back(), succ_coord);

        //ROS_WARN_STREAM("--------END state To Coord----------");
        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        succ_entry->source = group;

        SV_SHOW_INFO_NAMED(vis_name, getStateVisualizationByGroup(succ_entry->state, "succs",group));




        // check if this state meets the goal criteria
        //ROS_WARN_STREAM("-----------is GOAL----------");
        const bool is_goal_succ = isGoal(action.back());
        ROS_INFO_STREAM("after isGoal");
        //ROS_WARN_STREAM("--------end is goal----------");
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }

        sbpl::motion::ActionStateInfo action_state;
        action_state.parent_id = state_id;
        action_state.state_id = succ_state_id;
        action_state.source = group;
        ROS_INFO_STREAM("Adding action state with source "<<group);
        action_state.state_config = succ_entry->state;
        action_state.parent_state_config = parent_entry->state;
        action_state.dist_collision_time = duration;

        int total_cost = cost(parent_entry, succ_entry, weights[i], is_goal_succ);
        action_state.g.push_back(total_cost);
        ROS_INFO_STREAM("before get goal heuristics");
        int h_auv = GetGoalHeuristic(succ_state_id,0,0);
        int h_eca = GetGoalHeuristic(succ_state_id,1,0);
        int h_r5m = GetGoalHeuristic(succ_state_id,2,0);
        int heur;
        //std::getchar();
        if (group == 0) {
            heur = h_auv ;//+ (h_eca+h_r5m)/4;
        }
        else if(group == 1){
            heur = (2*h_eca/4 + 2*h_r5m/4);
        }
        else if(group == 2){
            heur = (2*h_eca/4 + 2*h_r5m/4);
        }
        else{
            ROS_ERROR_STREAM("GROUP NOT RECOGNIZED "<<group<<" press enter");
            heur =  2147483647; //MAX
            std::getchar();
        }
        action_state.h.push_back(heur);
        ROS_INFO_STREAM("after heuristics");

        action_state.dist_obstacles = distToObst;
        if(clearance_threshold_ > 0 && distToObst<=clearance_threshold_ && distToObst>0)
        {
            total_cost*=4;
            clearance_cells->push_back(distToObstCells);

            action_state.h.push_back(distToObstCells);
            ROS_INFO_STREAM("Penalize the state: Dist to obst is "<<distToObst<<" and num of cells "<<distToObstCells);
        }
        else
        {
            clearance_cells->push_back(0);
            action_state.h.push_back(0);
        }
        action_state.g.push_back(total_cost);
        costs->push_back(total_cost);
        planning_data->actionStates_.push_back(action_state);
        
        // log successor details
        SMPL_INFO_NAMED(params()->expands_log, "      succ: %zu", i);
        SMPL_INFO_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        coord: " << succ_coord);
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        state: " << succ_entry->state);
        SMPL_INFO_NAMED(params()->expands_log, "        heur: %2d", heur);
        SMPL_INFO_NAMED(params()->expands_log, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
        //std::getchar();
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
}


void ManipLattice::GetSuccsByGroupAndExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,  int group, int expansion_step) 
{
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << parent_entry->state);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  Source: " << parent_entry->source);
    SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));

    auto* vis_name = "expansion";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualizationByGroup(parent_entry->state, vis_name, group));

    int goal_succ_count = 0;

    collisionChecker()->setLastExpansionStep(expansion_step);

    std::vector<Action> actions;
    ActionsWeight weights;
    if (!m_actions->apply(parent_entry->state, actions, weights, group)) {
        SMPL_DEBUG("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
           continue;
        }
        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        succ_entry->source = group;

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, weights[i], is_goal_succ));

        // log successor details
        SMPL_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
        SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }
    
    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
}

void ManipLattice::setSelectedStartId (int start_id)
{
    m_start_state_id = start_id;
    displaySelectedGoal(start_id);
}


double ManipLattice::getDistanceToGoal(int state_id)
{
    ROS_WARN_STREAM("Using get Distance To Goal. Needs to redefine what is the distance, but its already calculated for both arms");

    std::vector<double> state = m_states[state_id]->state;
    std::vector<double> goal_state = m_states[m_start_state_id]->state;
    std::vector<bool> isGoalRegion;
    isGoalRegion.resize(state.size(),0);
    double overallDistance  = 0.0;
    double overallDistance2 = 0.0;
    Eigen::Affine3d goal_pose,goal_pose2, current_pose, current_pose2;
    projectToPose(state_id,"ECA_Jaw",current_pose);
    projectToPose(m_start_state_id,"ECA_Jaw",goal_pose);
    projectToPose(m_start_state_id,"R5M_Jaw",goal_pose2);

    ROS_INFO_STREAM("start state id "<<m_start_state_id);
    /*for (int i = 0; i < state.size(); i++) 
    {
        double dist ;
        if (i<3 )//&& fabs(state[i] - goal_state[i]) <= 0.05)
            dist = pow(fabs(state[i] - goal_state[i]),2);
        else
            dist = pow(fabs(angles::shortest_angle_diff(goal_state[i],state[i])),2);

        ROS_INFO_STREAM("Dist to goal for joint ["<<i<<"] is "<<dist);

        if(i<3 && dist< 0.5)
        {
            isGoalRegion[i] = 1;
            overallDistance += dist;
        }    
        else if  (i>=3 && dist< 0.1)
        {
            isGoalRegion[i] = 1;
            overallDistance += dist;
        }
    }   

    if(std::accumulate(isGoalRegion.begin(), isGoalRegion.end(), 0)==state.size())
        overallDistance = sqrt(overallDistance);
    else
        overallDistance = -1;
    ROS_INFO_STREAM("Final dist "<<overallDistance);*/
    overallDistance+= pow(goal_pose.translation()[0]-current_pose.translation()[0],2);
    overallDistance+= pow(goal_pose.translation()[1]-current_pose.translation()[1],2);
    overallDistance+= pow(goal_pose.translation()[2]-current_pose.translation()[2],2);
    overallDistance = sqrt(overallDistance);

    overallDistance2+= pow(goal_pose2.translation()[0]-current_pose2.translation()[0],2);
    overallDistance2+= pow(goal_pose2.translation()[1]-current_pose2.translation()[1],2);
    overallDistance2+= pow(goal_pose2.translation()[2]-current_pose2.translation()[2],2);
    overallDistance2 = sqrt(overallDistance2);

    
    /*double start_to_goal_dist = pow((goal().tgt_off_pose[0])-goal_pose.translation()[0],2);
    start_to_goal_dist += pow((goal().tgt_off_pose[1])-goal_pose.translation()[1],2);
    start_to_goal_dist += pow((goal().tgt_off_pose[2])-goal_pose.translation()[2],2);
    start_to_goal_dist = sqrt(start_to_goal_dist);*/
    
    /*double threshold = 0.9;//start_to_goal_dist/8;
    ROS_INFO_STREAM("Overall dist to goal vs. threshold "
        <<overallDistance<<","<<threshold);
    
    if(overallDistance<=threshold)
        return overallDistance;
    else 
        return -1;*/ 
    return overallDistance;
}

bool ManipLattice::updateMultipleStartStates (std::vector<int>* new_starts, std::vector<double>* new_costs, int restore_step)
{
    ROS_ERROR_STREAM("ManipLattice: updateMultipleStartStates Not set for Dual arm. enter..."); std::getchar();
    std::vector<double> max_speed, q_inc;
    std::vector<double> longest_dur ;
    longest_dur.resize(m_start_states_ids.size(),0);

    max_speed.resize(robot()->jointVariableCount(),0);
    max_speed[0] = 0.08;
    max_speed[1] = 0.05;
    //max_speed[2] = 0.08;
    max_speed[2] = 0.05;
    /*max_speed[4] = 0.1;
    max_speed[5] = 0.1;
    max_speed[6] = 0.1;
    max_speed[7] = 0.1;*/
    
    //collisionChecker()->setLastExpansionStep(-1);
    if(restore_step==1 || restore_step==-1)
    {

        for(int i=0;i<m_start_states_ids.size();i++)
        {
            ManipLatticeState* start_entry = m_states[m_start_states_ids[i]];
            
            if (!collisionChecker()->isStateValid(start_entry->state, true)) {
                auto* vis_name = "invalid_start";
                SV_SHOW_INFO_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(start_entry->state));
                new_starts->push_back(m_start_states_ids[i]);
                new_costs->push_back(32767);
                longest_dur[i] = 32767;
                SMPL_INFO_STREAM("State "<<m_start_states_ids[i]<<" -> in collision after recomputing cost");
                continue;
            }
            new_starts->push_back(0);//m_start_states_ids[i]);
            q_inc.resize(robot()->jointVariableCount(),0);
            for(int j=0;j<robot()->jointVariableCount();j++)
            {
                
                q_inc[j] = start_entry->state[j] - goal().angles[j];
                if(longest_dur[i]<(q_inc[j]/max_speed[j]))
                    longest_dur[i] = (fabs(q_inc[j])/max_speed[j]);

            }
            new_costs->push_back(longest_dur[i]*1000);
            ROS_INFO_STREAM("State ["<<i<<"] with id "<<m_start_states_ids[i]<<" has longest duration "<<longest_dur[i]);
            q_inc.clear();
        }

        int min_idx = std::distance(longest_dur.begin(), std::min_element(longest_dur.begin(), longest_dur.end()));
        auto* vis_name = "lowest_g";
        ManipLatticeState* entry = m_states[m_start_states_ids[min_idx]];
        

        auto markers = collisionChecker()->getCollisionModelVisualization(entry->state);
        /*for (auto& marker : markers) {
           
            visual::Color color;
            color.a=1;
            color.r=0.2;
            color.g=0.2;
            color.b=0.4;
            marker.ns = vis_name + std::to_string(min_idx);
            marker.color = color;
        }
        SV_SHOW_INFO_NAMED(vis_name + std::to_string(min_idx), markers); */
        ROS_INFO_STREAM("Min g-val state is "<<min_idx+1);
        
         if(m_start_states_ids.empty())
        {
            ROS_ERROR_STREAM("Failed to find a valid start state!");
            return false;
        }

        m_start_state_id = m_start_states_ids[0];
    }
    return true;

}

void ManipLattice::displaySelectedGoal (int goalStateID)
{
    m_start_state_id = goalStateID;
    auto* vis_name = "start_config";
    ManipLatticeState* entry = m_states[m_start_states_ids[goalStateID-1]];

    auto markers = collisionChecker()->getCollisionModelVisualization(entry->state);
    for (auto& marker : markers) {
       
        visual::Color color;
        color.a=1;
        color.g=0.2;
        color.r=0.8;
        marker.ns = vis_name;
        marker.color = color;
    }
    SV_SHOW_INFO_NAMED(vis_name, markers); 

}

void ManipLattice::GetPredsByGroupAndExpansion(int state_id, std::vector<int>* preds, 
    std::vector<int>* costs, std::vector<int>* clearance_cells, int group, int expansion_step, int parent_id)
{
    assert(state_id >= 0 && state_id < m_states.size() && "state is out of bounds");
    assert(preds && costs && "predecessor buffer is null");
    assert(m_actions && "action space is uninitialized");
    
    SMPL_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    SMPL_INFO_STREAM("Parent id "<<parent_id<<" and current "<<state_id);
    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* expanded_entry = m_states[state_id];
    double marking_duration  = 0;
    if(parent_id!=-1)
    {
        ManipLatticeState* parent_entry = m_states[parent_id];
        double start = ros::Time::now().toSec();
        collisionChecker()->markGridForExpandedState(expanded_entry->state,parent_entry->state,expansion_step-1);
        marking_duration = ros::Time::now().toSec() - start;
    }
    assert(expanded_entry);
    assert(expanded_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << expanded_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << expanded_entry->state);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  Source: " << expanded_entry->source);
    SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));

    auto* vis_name = "expansion";
    if(group==-1)
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(expanded_entry->state, vis_name));
    else
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualizationByGroup(expanded_entry->state, vis_name, group));

    int goal_succ_count = 0;
    
    //collisionChecker()->setLastExpansionStep(expansion_step);

    std::vector<Action> actions;
    ActionsWeight weights;
    if (!m_actions->applyPredActions(expanded_entry->state, actions, weights, group)) {
        SMPL_ERROR("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    
    // check actions for validity
    RobotCoord pred_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        double distToObst;
        int distToObstCells;
        double start = ros::Time::now().toSec();
        bool test;
        if(clearance_threshold_== 0) 
            test = checkAction(expanded_entry->state, action);
        else
            test = checkAction(expanded_entry->state, action, distToObst, distToObstCells);
        double duration = ros::Time::now().toSec() - start;
        //ROS_ERROR_STREAM("duration is "<<duration);
        if (!test) {
           continue;
        }

        // compute destination coords
        stateToCoord(action.back(), pred_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int pred_state_id = getOrCreateState(pred_coord, action.back());
        ManipLatticeState* pred_entry = getHashEntry(pred_state_id);
        if(group!=-1)
            pred_entry->source = group;

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back());
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            preds->push_back(m_goal_state_id);
        } else {
            preds->push_back(pred_state_id);
        }
        
        double dist = getDistanceToGoal(pred_state_id);
       /* if(group==0 && (i>=6 && i<=7)  && dist_threshold_!=0)
        {
            if(dist<1.4)
            {
                weights[i]*=8;
                ROS_INFO_STREAM("Penalize yaw inside the goal region!");
            }
            else
            {
                weights[i]/=8;
                ROS_INFO_STREAM("reward yaw inside the goal region!");   
            }
        }
        else if (group==1 && dist_threshold_!=0)
        {
            if(dist<0.6)
            {
                weights[i]/=10;
                ROS_INFO_STREAM("Reward arm inside the goal region!");
            }
            else
            {
                weights[i]*=10;
                ROS_INFO_STREAM("Penlize arm inside the goal region!");   
            }
            
        }*/

        sbpl::motion::ActionStateInfo action_state;
        action_state.parent_id = state_id;
        action_state.state_id = pred_state_id;
        action_state.source = group;
        action_state.state_config = pred_entry->state;
        action_state.parent_state_config = expanded_entry->state;
        action_state.expansion_step = expansion_step;
        action_state.dist_collision_time = duration;
        action_state.marking_cell_time = marking_duration;
        action_state.dist_to_goal = dist;

        int total_cost = cost(expanded_entry, pred_entry, weights[i], is_goal_succ);
        
        action_state.g.push_back(total_cost);
        action_state.h.push_back(GetGoalHeuristic(pred_state_id, group,0));
        action_state.dist_obstacles = distToObst;

        if(clearance_threshold_ > 0 && distToObst<=clearance_threshold_ && distToObst>0)
        {
        
            total_cost *=4;
            
            clearance_cells->push_back(distToObstCells);
            action_state.h.push_back(distToObstCells);
            
            ROS_INFO_STREAM("Penalize the state: Dist to obst is "<<distToObst<<" and num of cells "<<distToObstCells<<" new cost "<<total_cost);
        }
        else if(clearance_threshold_>0 && distToObst>0)
        {
            total_cost/=4;
            ROS_INFO_STREAM("Reward clearance, the state  new cost "<<total_cost);

            clearance_cells->push_back(0);
            action_state.h.push_back(0);
        }
        else if(clearance_threshold_==0)
        {
            clearance_cells->push_back(0);
        }

        
        action_state.g.push_back(total_cost);


       /* ROS_INFO_STREAM("total g-cost before is "<<total_cost);
        //group==1 &&
        if( pred_state_id>m_start_states_ids.size())
        {
            if(dist!=-1)
                total_cost /=8;
            else
                total_cost*=8;
            action_state.g.push_back(total_cost);
        }*/

        SMPL_INFO_STREAM("total g-cost is "<<total_cost);
        /*ROS_ERROR_STREAM("total heuristic is "<<GetGoalHeuristic(pred_state_id));*/
        costs->push_back(total_cost);
        planning_data->actionStates_.push_back(action_state);

        // log predecessor details
        SMPL_INFO_NAMED(params()->expands_log, "      pred: %zu", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "        id: %5i", pred_state_id);
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        coord: " << pred_coord);
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        state: " << pred_entry->state);
        SMPL_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(pred_state_id));
        SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(expanded_entry, pred_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
    /*ROS_ERROR_STREAM("current expansion "<<expansion_step<<" group "<<group<<" id "<<state_id);
    */
    //collisionChecker()->setLastExpansionStep(-1);
}

Stopwatch GetLazySuccsStopwatch("GetLazySuccs", 10);

void ManipLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    ROS_WARN_STREAM("On Get Lazy succs, not modified for dual arm. control to continue");

    GetLazySuccsStopwatch.start();
    PROFAUTOSTOP(GetLazySuccsStopwatch);

    assert(state_id >= 0 && state_id < m_states.size());

    SMPL_DEBUG_NAMED(params()->expands_log, "expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* state_entry = m_states[state_id];

    assert(state_entry);
    assert(state_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << state_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  angles: " << state_entry->state);
    SMPL_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));

    const RobotState& source_angles = state_entry->state;
    auto* vis_name = "expansion";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(source_angles, vis_name));

    std::vector<Action> actions;
    if (!m_actions->apply(source_angles, actions)) {
        SMPL_DEBUG("Failed to get successors");
        return;
    }

    SMPL_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    int goal_succ_count = 0;
    RobotCoord succ_coord(robot()->jointVariableCount());
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        stateToCoord(action.back(), succ_coord);

        const bool succ_is_goal_state = isGoal(action.back());
        if (succ_is_goal_state) {
            ++goal_succ_count;
        }

        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        if (succ_is_goal_state) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(state_entry, succ_entry, succ_is_goal_state));
        true_costs->push_back(false);

        // log successor details
        SMPL_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
        SMPL_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(state_entry, succ_entry, succ_is_goal_state));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
}

Stopwatch GetTrueCostStopwatch("GetTrueCost", 10);

int ManipLattice::GetTrueCost(int parentID, int childID)
{

    ROS_WARN_STREAM("Get true cost called, not made for dual arm. control to continue");


    GetTrueCostStopwatch.start();
    PROFAUTOSTOP(GetTrueCostStopwatch);

    SMPL_ERROR_NAMED(params()->expands_log, "evaluating cost of transition %d -> %d", parentID, childID);

    assert(parentID >= 0 && parentID < (int)m_states.size());
    assert(childID >= 0 && childID < (int)m_states.size());

    ManipLatticeState* parent_entry = m_states[parentID];
    ManipLatticeState* child_entry = m_states[childID];
    assert(parent_entry && parent_entry->coord.size() >= robot()->jointVariableCount());
    assert(child_entry && child_entry->coord.size() >= robot()->jointVariableCount());

    const RobotState& parent_angles = parent_entry->state;
    auto* vis_name = "expansion";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(parent_angles, vis_name));

    std::vector<Action> actions;
    if (!m_actions->apply(parent_angles, actions)) {
        SMPL_DEBUG("Failed to get actions");
        return -1;
    }

    const bool goal_edge = (childID == m_goal_state_id);

    size_t num_actions = 0;

    // check actions for validity and find the valid action with the least cost
    RobotCoord succ_coord(robot()->jointVariableCount());
    int best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        const Action& action = actions[aidx];

        stateToCoord(action.back(), succ_coord);

        // check whether this action leads to the child state
        if (goal_edge) {
            // skip actions which don't end up at a goal state
            if (!isGoal(action.back())) {
                continue;
            }
        } else {
            // skip actions which don't end up at the child state
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        SMPL_DEBUG_NAMED(params()->expands_log, "    action %zu:", num_actions++);
        SMPL_DEBUG_NAMED(params()->expands_log, "      waypoints %zu:", action.size());

        if (!checkAction(parent_angles, action)) {
            continue;
        }

        // get the unique state
        int succ_state_id = goal_edge ? getHashEntry(succ_coord) : childID;
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        assert(succ_entry);

        const int edge_cost = cost(parent_entry, succ_entry, 1, goal_edge);
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost != std::numeric_limits<int>::max()) {
        return best_cost;
    } else {
        return -1;
    }
}

const RobotState& ManipLattice::extractState(int state_id)
{
    return m_states[state_id]->state;
}

bool ManipLattice::projectToPose(int state_id, Eigen::Affine3d& pose)
{
    ROS_ERROR_STREAM("Manip Lattice: ProjectToPose without ee name");
    if (state_id == getGoalStateID()) {
        assert(goal().tgt_off_pose.size() >= 6);
        Eigen::Matrix3d R;
        angles::from_euler_zyx(
                goal().tgt_off_pose[5],
                goal().tgt_off_pose[4],
                goal().tgt_off_pose[3],
                R);
        pose =
                Eigen::Translation3d(
                        goal().tgt_off_pose[0],
                        goal().tgt_off_pose[1],
                        goal().tgt_off_pose[2]) *
                Eigen::Affine3d(R);
        return true;
    }
    
    std::vector<double> vpose;
    if (!computePlanningFrameFK(m_states[state_id]->state, vpose)) {
        SMPL_ERROR("Failed to compute fk for state %d", state_id);
        return false;
    }

    Eigen::Matrix3d R;
    angles::from_euler_zyx(vpose[5], vpose[4], vpose[3], R);
    pose =
            Eigen::Translation3d(vpose[0], vpose[1], vpose[2]) *
            Eigen::Affine3d(R);

    return true;
}

    bool ManipLattice::projectToPose(int state_id, const std::string& ee_link, Eigen::Affine3d& pose)
    {
        //ROS_INFO_STREAM("Manip Lattice: ProjectToPose for ee "<<ee_link<<" state_id: "<<state_id<< " enter..");

        if (state_id == getGoalStateID()) {
            //ROS_WARN_STREAM("ManipLattice (pTP): state_id == getGoalStateID. enter...");std::getchar();
            assert(goal().tgt_off_pose.size() >= 6);
            Eigen::Matrix3d R;

            GoalConstraint current_goal;
            if(ee_link == "ECA_Jaw"){
                current_goal = goal();
            }
            else if (ee_link == "R5M_Jaw"){
                current_goal = goal2();
            }
            else{
                ROS_ERROR_STREAM("EE Link Not Recognized... enter to die");
                std::getchar();
                return false;
            }
            angles::from_euler_zyx(
                    current_goal.tgt_off_pose[5],
                    current_goal.tgt_off_pose[4],
                    current_goal.tgt_off_pose[3],
                    R);
            pose =
                    Eigen::Translation3d(
                            current_goal.tgt_off_pose[0],
                            current_goal.tgt_off_pose[1],
                            current_goal.tgt_off_pose[2]) *
                    Eigen::Affine3d(R);
            return true;
        }


        std::vector<double> vpose;
        if (!computePlanningFrameFK(m_states[state_id]->state, vpose,ee_link)) {
                    SMPL_ERROR("Failed to compute fk for state %d", state_id);
            return false;
        }

        Eigen::Matrix3d R;
        angles::from_euler_zyx(vpose[5], vpose[4], vpose[3], R);
        pose =
                Eigen::Translation3d(vpose[0], vpose[1], vpose[2]) *
                Eigen::Affine3d(R);

        return true;
    }

bool ManipLattice::projectToBasePoint(int state_id, Eigen::Vector3d& pos)
{
    if(state_id == m_goal_state_id)
    {
        ROS_WARN_STREAM("ManipLattice (pTBP): state_id == m_goal_state_id. Gonna compute BaseFrameIK for goal1");
        std::vector<double> goalConfig(robot()->jointVariableCount());
        computeBaseFrameIK(goal().tgt_off_pose,goalConfig);
        pos[0] = goalConfig[0];
        pos[1] = goalConfig[1];
        pos[2] = goalConfig[2];
    }
    else
    {
        /*pos[0] = m_states[state_id]->state[0];
        pos[1] = m_states[state_id]->state[1];
        pos[2] = m_states[state_id]->state[2];*/
        std::vector<double> base;
        //TODO CHANGE FOR DUAL
        //ROS_WARN_STREAM("ManipLattice: projectToBasePoint uses now AUV_header as ee for FK");
        m_fk_iface->computeFK(m_states[state_id]->state,"AUV_roll_link",base);
        SMPL_DEBUG_STREAM("FK base "<<base[0]<<","<<base[1]<<","<<base[2]);
        pos[0]=base[0];
        pos[1]=base[1];
        pos[2]=base[2];
        SMPL_DEBUG_STREAM("FK pose "<<pos[0]<<","<<pos[1]<<","<<pos[2]<<","<<m_states[state_id]->state[3]);
        
    }  
    return true;
}

void ManipLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
    SMPL_DEBUG("GetPreds unimplemented");
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
void ManipLattice::coordToState(
    const RobotCoord& coord,
    RobotState& state) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < coord.size(); ++i) {
        if (m_continuous[i]) {
            state[i] = coord[i] * m_coord_deltas[i];
        } else if (!m_bounded[i]) {
            state[i] = (double)coord[i] * m_coord_deltas[i];
        } else {
            state[i] = m_min_limits[i] + coord[i] * m_coord_deltas[i];
        }
    }
}

void ManipLattice::stateToCoord(
    const RobotState& state,
    RobotCoord& coord) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < state.size(); ++i) {
        if (m_continuous[i]) {
            double pos_angle = angles::normalize_angle_positive(state[i]);

            coord[i] = (int)((pos_angle + m_coord_deltas[i] * 0.5) / m_coord_deltas[i]);

            if (coord[i] == m_coord_vals[i]) {
                coord[i] = 0;
            }
        } else if (!m_bounded[i]) {
            if (state[i] >= 0.0) {
                coord[i] = (int)(state[i] / m_coord_deltas[i] + 0.5);
            } else {
                coord[i] = (int)(state[i] / m_coord_deltas[i] - 0.5);
            }
        } else {
            coord[i] = (int)(((state[i] - m_min_limits[i]) / m_coord_deltas[i]) + 0.5);
        }
    }
}

ManipLatticeState* ManipLattice::getHashEntry(int state_id) const
{
    if (state_id < 0 || state_id >= (int)m_states.size()) {
        return nullptr;
    }

    return m_states[state_id];
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int ManipLattice::getHashEntry(const RobotCoord& coord)
{
    ManipLatticeState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int ManipLattice::createHashEntry(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = reserveHashEntry();
    ManipLatticeState* entry = getHashEntry(state_id);

    entry->coord = coord;
    entry->state = state;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int ManipLattice::getOrCreateState(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0) {
        state_id = createHashEntry(coord, state);
    }
    return state_id;
}

int ManipLattice::reserveHashEntry()
{
    ManipLatticeState* entry = new ManipLatticeState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    // map planner state -> graph state
    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

/// NOTE: const although RobotModel::computePlanningLinkFK used underneath may
/// not be
bool ManipLattice::computePlanningFrameFK(

    const RobotState& state,
    std::vector<double>& pose) const
{

    ROS_ERROR_STREAM("COMPUTE PLANNING FRAME FK");
    assert(state.size() == robot()->jointVariableCount());
    assert(m_fk_iface);

    if (!m_fk_iface->computePlanningLinkFK(state, pose)) {
        return false;
    }

    pose = getTargetOffsetPose(pose);

    assert(pose.size() == 6);
    return true;
}

//FOR DUAL
bool ManipLattice::computePlanningFrameFK(

        const RobotState& state,
        std::vector<double>& pose, const std::string& ee_link) const
{

    ROS_INFO_STREAM("DUAL: COMPUTE PLANNING FRAME FK: "<<ee_link);
    assert(state.size() == robot()->jointVariableCount());
    assert(m_fk_iface);


    if (!m_fk_iface->computeFK(state, ee_link, pose)) {
        return false;
    }


    pose = getTargetOffsetPose(pose);

    assert(pose.size() == 6);
    return true;
}



bool ManipLattice::computeBaseFrameIK(
        const std::vector<double>& pose,
        RobotState& state) const
{
    ROS_ERROR_STREAM("COMPUTE BASE FRAME IK");
    std::getchar();
    assert(m_ik_iface);
    std::vector<RobotState> states;
    RobotState seed(robot()->jointVariableCount(), 0);
    if (!m_ik_iface->computeIK(pose, seed, state)) {
        SMPL_ERROR_STREAM("No valid IK solution for the goal pose "<<pose[0]<<","<<pose[1]<<","<<pose[2]<<","<<pose[3]<<","<<pose[4]<<","<<pose[5]);
    }
    //state  = states[0];
}

bool ManipLattice::computeBaseFrameIK(
        const std::vector<double>& pose,
        RobotState& state, const std::string & arm) const {
    ROS_INFO_STREAM("DUAL: COMPUTE BASE FRAME IK");
    assert(m_ik_iface);
    std::vector<RobotState> states;
    RobotState seed(robot()->jointVariableCount(), 0);
    if (!m_ik_iface->computeIK(pose, seed, state, arm)) {
                SMPL_DEBUG_STREAM(
                "No valid IK solution for the goal pose " << pose[0] << "," << pose[1] << "," << pose[2] << ","
                                                          << pose[3] << "," << pose[4] << "," << pose[5]);
    }
}


        int ManipLattice::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    bool bState2IsGoal) const
{

    const int DefaultCostMultiplier = 1000;
    ROS_INFO("before crash");
    int dx = HashEntry1->coord[0] - HashEntry2->coord[0];
    ROS_INFO("wait for it");
    int dy = HashEntry1->coord[1] - HashEntry2->coord[1];
    ROS_INFO("wait for it...");
    double yaw = HashEntry1->coord[3];
    ROS_INFO("waaaaait");
    double heading = std::atan2(dy, dx);
    bool sideways = false;
    sideways = 0.5 * M_PI - angles::shortest_angle_dist(heading, yaw);
    /*if (sideways) {
        int penalty = 2;
        return DefaultCostMultiplier * penalty;
    }

    if(angles::shortest_angle_dist(heading, yaw)>0.1)
    {
        return DefaultCostMultiplier * 2;
    }*/
    
    return DefaultCostMultiplier;
}

int ManipLattice::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    double actionWeight,
    bool bState2IsGoal) const
{
    const int DefaultCostMultiplier = 1000;

    /*
    int dx = HashEntry1->coord[0] - HashEntry2->coord[0];
    int dy = HashEntry1->coord[1] - HashEntry2->coord[1];
    double yaw = HashEntry1->coord[3];
    double heading = std::atan2(dy, dx);
    bool sideways = false;
    sideways = 0.5 * M_PI - angles::shortest_angle_dist(heading, yaw);*/ //TRA??
   /*if (sideways) {
        int penalty = 2;
        return DefaultCostMultiplier * penalty;
    }
    if(angles::shortest_angle_dist(heading, yaw)>0.1)
    {
        return DefaultCostMultiplier * 2;
    }*/
    return DefaultCostMultiplier*actionWeight;
}


bool ManipLattice::checkAction(const RobotState& state, const Action& action, double& distToObst, int& distToObstCells)
{
    std::uint32_t violation_mask = 0x00000000;


    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        //SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        " << iidx << ": " << istate);

        if (!robot()->checkJointLimits(istate)) {
            SMPL_INFO_STREAM_NAMED(params()->expands_log, "        " << iidx << ": " << istate);

            SMPL_INFO_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!collisionChecker()->isStateValid(istate, params()->verbose_collisions_))
//        {
//            SMPL_DEBUG_NAMED(params()->expands_log_, "        -> in collision);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
   if (!collisionChecker()->isStateToStateValid(state, action[0],distToObst,distToObstCells)) {
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        "  << action[0]);

        SMPL_INFO_NAMED(params()->expands_log, "        -> path to first waypoint in collision");
        //SMPL_ERROR("path to waypint in collision!");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        const RobotState& prev_istate = action[j - 1];
        const RobotState& curr_istate = action[j];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate))
        {
            SMPL_INFO_STREAM_NAMED(params()->expands_log, "        " << prev_istate);

            SMPL_INFO_NAMED(params()->expands_log, "        -> path between waypoints %zu and %zu in collision", j - 1, j);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool ManipLattice::checkAction(const RobotState& state, const Action& action)
{
    std::uint32_t violation_mask = 0x00000000;

    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        //SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "        " << iidx << ": " << istate);

        if (!robot()->checkJointLimits(istate)) {
            SMPL_INFO_STREAM_NAMED(params()->expands_log, "        " << iidx << ": " << istate);

            SMPL_ERROR_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!collisionChecker()->isStateValid(istate, params()->verbose_collisions_))
//        {
//            SMPL_DEBUG_NAMED(params()->expands_log_, "        -> in collision);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
    if (!collisionChecker()->isStateToStateValid(state, action[0])) {
        SMPL_INFO_STREAM_NAMED(params()->expands_log, "        "  << action[0]);

        SMPL_INFO_NAMED(params()->expands_log, "        -> path to first waypoint in collision");
        //SMPL_ERROR("path to waypint in collision!");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        const RobotState& prev_istate = action[j - 1];
        const RobotState& curr_istate = action[j];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate))
        {
            SMPL_INFO_STREAM_NAMED(params()->expands_log, "        " << prev_istate);

            SMPL_INFO_NAMED(params()->expands_log, "        -> path between waypoints %zu and %zu in collision", j - 1, j);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool ManipLattice::isGoal(const RobotState& state)
{
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        /*SMPL_ERROR("DINA DOESNT THINK THATS A PROBLEM");
        ManipLatticeState tmp_state;
        tmp_state.coord.resize(robot()->jointVariableCount());
        tmp_state.state = state;
        stateToCoord(state, tmp_state.coord);
        ROS_ERROR_STREAM("CHECKING GOAL STATE = " << state << ", coord = " << tmp_state.coord << ", id = " << m_goal_unique_id);
        auto state_id = getOrCreateState(tmp_state.coord, tmp_state.state);
        return m_goal_unique_id == state_id;*/

        RobotCoord goal_coord(robot()->jointVariableCount());
        stateToCoord(goal().angles, goal_coord);

        RobotCoord current_coord(robot()->jointVariableCount());
        stateToCoord(state, current_coord);

        for (int i = 0; i < goal().angles.size(); i++) {
            if (fabs(current_coord[i] - goal_coord[i]) > goal().angle_tolerances[i]) {
                return false;
            }
        }
        /*for (int i = 0; i < goal().angles.size(); i++) {
            if (fabs(state[i] - goal().angles[i]) > goal().angle_tolerances[i]) {
                return false;
            }
        }*/
        return true;
    }   break;
    case GoalType::XYZ_RPY_GOAL:
    {
        ROS_INFO_STREAM("Goal XYZ RPY");
        // get pose of planning link
        std::vector<double> pose, pose2;
        RobotState newState = state;
        if (!computePlanningFrameFK(state, pose, "ECA_Jaw")) {
                    SMPL_DEBUG("Failed to compute FK for planning frame");
            return false;
        }
        if (!computePlanningFrameFK(state, pose2, "R5M_Jaw")) {
                    SMPL_DEBUG("Failed to compute FK for planning frame");
            return false;
        }
        ROS_INFO_STREAM("POSE ECA:" <<pose[0] <<", "<<pose[1]<<", "<<pose[2]);
        ROS_INFO_STREAM("POSE R5M:" <<pose2[0] <<", "<<pose2[1]<<", "<<pose2[2]);
        /*std::vector<double> goalPose;
        goalPose.resize(6,0);
        goalPose[0]=goal().tgt_off_pose[0];
        goalPose[1]=goal().tgt_off_pose[1];
        goalPose[2]=goal().tgt_off_pose[2];
        goalPose[3]=goal().tgt_off_pose[3];
        goalPose[4]=goal().tgt_off_pose[4];
        goalPose[5]=goal().tgt_off_pose[5];*/
        //computeBaseFrameIK(goalPose,newState);
        const double dx = fabs(pose[0] - goal().tgt_off_pose[0]);
        const double dy = fabs(pose[1] - goal().tgt_off_pose[1]);
        const double dz = fabs(pose[2] - goal().tgt_off_pose[2]);

        const double dx2 = fabs(pose2[0] - goal2().tgt_off_pose[0]);
        const double dy2 = fabs(pose2[1] - goal2().tgt_off_pose[1]);
        const double dz2 = fabs(pose2[2] - goal2().tgt_off_pose[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2] &&
                dx2 <= goal2().xyz_tolerance[0] &&
                dy2 <= goal2().xyz_tolerance[1] &&
                dz2 <= goal2().xyz_tolerance[2])
        {
            ROS_WARN_STREAM("ManipLattice: isCloseToGoal!: goal1 is: "<<goal().tgt_off_pose[0]<<", "<<goal().tgt_off_pose[1]<<", "<<goal().tgt_off_pose[2]);
            ROS_WARN_STREAM("----------------------   ECA_Jaw is at: "<<pose[0] <<", "<<pose[1]<<", "<<pose[2]);
            ROS_WARN_STREAM("----------------------------- goal2 is: "<<goal2().tgt_off_pose[0]<<", "<<goal2().tgt_off_pose[1]<<", "<<goal2().tgt_off_pose[2]);
            ROS_WARN_STREAM("----------------------   R5M_Jaw is at: "<<pose2[0] <<", "<<pose2[1]<<", "<<pose2[2]);
            /*ROS_WARN_STREAM("Press enter");
            std::getchar();*/

            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                using namespace std::chrono;
                auto time_to_goal_region = clock::now() - m_t_start;
                auto time_to_goal_s =
                        duration_cast<duration<double>>(time_to_goal_region);
                m_near_goal = true;
                SMPL_DEBUG_NAMED(params()->expands_log, "Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %0.4f sec.",
                        pose[0], pose[1], pose[2],
                        goal().xyz_tolerance[0],
                        goal().tgt_off_pose[0], goal().tgt_off_pose[1], goal().tgt_off_pose[2],
                        time_to_goal_s.count());
            }
            Eigen::Quaterniond qg(
                    Eigen::AngleAxisd(goal().tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[3], Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));

            Eigen::Quaterniond qg2(
                    Eigen::AngleAxisd(goal2().tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(goal2().tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(goal2().tgt_off_pose[3], Eigen::Vector3d::UnitX()));

            Eigen::Quaterniond q2(
                Eigen::AngleAxisd(pose2[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd((pose2[4]), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(pose2[3], Eigen::Vector3d::UnitX()));

            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

            if (q2.dot(qg2) < 0.0) {
                qg2 = Eigen::Quaterniond(-qg2.w(), -qg2.x(), -qg2.y(), -qg2.z());
            }

//            const double theta = angles::normalize_angle(Eigen::AngleAxisd(qg.conjugate() * q).angle());
            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            const double theta2 = angles::normalize_angle(2.0 * acos(q2.dot(qg2)));
            SMPL_INFO_STREAM("Theta vs. goal tolerance "<<theta<<","<<goal().rpy_tolerance[0]);
                    SMPL_INFO_STREAM("Theta vs. goal tolerance "<<theta2<<","<<goal2().rpy_tolerance[0]);
            if (theta < goal().rpy_tolerance[0] && theta2 < goal2().rpy_tolerance[0]) {
                SMPL_INFO("Goal state found");
                return true;
            }
            SMPL_WARN("Goal state not found yet");
        }
    }   break;
    case GoalType::XYZ_GOAL:
    {
        ROS_INFO_STREAM("Goal XYZ");
        // get pose of planning link
        std::vector<double> pose,pose2;
        if (!computePlanningFrameFK(state, pose,"ECA_Jaw")) {
            SMPL_DEBUG("Failed to compute FK for planning frame");
            return false;
        }
        if (!computePlanningFrameFK(state, pose2,"R5M_Jaw")) {
                    SMPL_DEBUG("Failed to compute FK for planning frame");
            return false;
        }

        if (fabs(pose[0] - goal().tgt_off_pose[0]) <= goal().xyz_tolerance[0] &&
            fabs(pose[1] - goal().tgt_off_pose[1]) <= goal().xyz_tolerance[1] &&
            fabs(pose[2] - goal().tgt_off_pose[2]) <= goal().xyz_tolerance[2] &&
                fabs(pose2[0] - goal2().tgt_off_pose[0]) <= goal2().xyz_tolerance[0] &&
                fabs(pose2[1] - goal2().tgt_off_pose[1]) <= goal2().xyz_tolerance[1] &&
                fabs(pose2[2] - goal2().tgt_off_pose[2]) <= goal2().xyz_tolerance[2])
        {
            return true;
        }
    }   break;
    default:
    {
        SMPL_ERROR_NAMED(params()->graph_log, "Unknown goal type.");
    }   break;
    }

    return false;
}

sbpl::motion::GroupType ManipLattice::switchPlanningGroup(int state_id, double switchThreshold)
{
    ROS_ERROR_STREAM("SwitchPlanningGroup not modified for dual arm. enter to continue"); std::getchar();
    RobotState state = m_states[state_id]->state;
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        for (int i = 0; i < goal().angles.size(); i++) {
            if (fabs(state[i] - goal().angles[i]) > switchThreshold) {
                return sbpl::motion::GroupType::BASE;
            }
        }
        return sbpl::motion::GroupType::ARM;
    }   break;
    case GoalType::XYZ_RPY_GOAL:
    {
        // get pose of planning link
        std::vector<double> pose;
        if (!computePlanningFrameFK(state, pose)) {
            SMPL_DEBUG("Failed to compute FK for planning frame");
            return sbpl::motion::GroupType::FAILURE;
        }

        const double dx = fabs(pose[0] - goal().tgt_off_pose[0]);
        const double dy = fabs(pose[1] - goal().tgt_off_pose[1]);
        const double dz = fabs(pose[2] - goal().tgt_off_pose[2]);
        if (dx <= switchThreshold &&
            dy <= switchThreshold &&
            dz <= switchThreshold)
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                using namespace std::chrono;
                auto time_to_goal_region = clock::now() - m_t_start;
                auto time_to_goal_s =
                        duration_cast<duration<double>>(time_to_goal_region);
                m_near_goal = true;
                SMPL_DEBUG_NAMED(params()->expands_log, "Switching Group Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %0.4f sec.",
                        pose[0], pose[1], pose[2],
                        goal().xyz_tolerance[0],
                        goal().tgt_off_pose[0], goal().tgt_off_pose[1], goal().tgt_off_pose[2],
                        time_to_goal_s.count());
            }
            Eigen::Quaterniond qg(
                    Eigen::AngleAxisd(goal().tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[3], Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));
            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < switchThreshold) {
                return sbpl::motion::GroupType::ARM;
            }
            else
                return sbpl::motion::GroupType::BASE;
        }
        else
            return sbpl::motion::GroupType::BASE;
    }   break;
    case GoalType::XYZ_GOAL:
    {
        // get pose of planning link
        std::vector<double> pose;
        if (!computePlanningFrameFK(state, pose)) {
            SMPL_DEBUG("Failed to compute FK for planning frame");
            return sbpl::motion::GroupType::FAILURE;
        }

        if (fabs(pose[0] - goal().tgt_off_pose[0]) <= switchThreshold &&
            fabs(pose[1] - goal().tgt_off_pose[1]) <= switchThreshold &&
            fabs(pose[2] - goal().tgt_off_pose[2]) <= switchThreshold)
        {
            return sbpl::motion::GroupType::ARM;
        }
        else
            return sbpl::motion::GroupType::BASE;
    }   break;
    default:
    {
        SMPL_ERROR_NAMED(params()->graph_log, "Unknown goal type.");
    }   break;
    }

    return sbpl::motion::GroupType::FAILURE;
}

auto ManipLattice::getStateVisualization(
    const RobotState& state,
    const std::string& ns)
    -> std::vector<visual::Marker>
{
    auto markers = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

auto ManipLattice::getStateVisualizationByGroup(const RobotState& state, const std::string& ns, int group)
        -> std::vector<visual::Marker>
{
    auto markers = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
       
        visual::Color color;
        color.a=0.8;
        color.g=0;
        if(group == 1)
        {
            color.b=1;
            color.r=0;
             marker.ns = ns+std::string("_eca");//ns;
        }
        else if(group == 0)
        {
           color.b=0;
           color.r=1; 
            marker.ns =  ns+std::string("_base");
        } else{
            color.g = 1;
            color.b = 0;
            color.r = 0;
            marker.ns =  ns+std::string("_r5m");
        }

        if (ns=="expansion") {
            color.g *= 0.4;
            color.b *= 0.4;
            color.r *= 0.4;
        }

        marker.color = color;
        
    }
    return markers;
}


auto ManipLattice::makePathVisualization(
    const std::vector<RobotState>& path, std::vector<int> sourceGroup) 
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    double cinc = 1.0 / double(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = collisionChecker()->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            const float r = 0.1f;
            const float g = cinc * (float)(path.size() - (i + 1));
            const float b = cinc * (float)i;
            if(sourceGroup[i]==0)
                marker.color = visual::Color{ 1, 0, 0, 1.0f };
            else
                marker.color = visual::Color{ 0, 0, 1, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory_source";
        marker.id = i;
    }

    return ma;
}

bool ManipLattice::setMultipleStart(const std::vector<RobotState>& states)
{
    /*RobotCoord virtual_start_coord(robot()->jointVariableCount());
    RobotState virtual_start_state(robot()->jointVariableCount());

    for (size_t i = 0; i < virtual_start_state.size(); ++i) 
    {
        virtual_start_state[i] = -1;
        virtual_start_coord[i] = -1;
    }

    m_start_state_id = getOrCreateState(virtual_start_coord, virtual_start_state);
    */
    ROS_ERROR_STREAM("Start states size "<<states.size());
    for(int i=0;i<states.size();i++)
    {
        if ((int)states[i].size() < robot()->jointVariableCount()) {
            SMPL_ERROR_NAMED(params()->graph_log, "start state does not contain enough joint positions");
            return false;
        }

        SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "  state: " << states[i]);
        // check joint limits of starting configuration
        if (!robot()->checkJointLimits(states[i], true)) {
            SMPL_ERROR(" -> violates the joint limits");
            continue;
        }
        // check if the start configuration is in collision
        if (!collisionChecker()->isStateValid(states[i], true)) {
            auto* vis_name = "invalid_start";
            SV_SHOW_INFO_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(states[i]));
            SMPL_ERROR(" -> in collision");
            continue;
        }

        
        // get arm position in environment
        RobotCoord start_coord(robot()->jointVariableCount());
        stateToCoord(states[i], start_coord);
        SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "  coord: " << start_coord);
        int start_state_id = getOrCreateState(start_coord, states[i]);

        if(std::find(m_start_states_ids.begin(),m_start_states_ids.end(),start_state_id)==m_start_states_ids.end())
        {
            m_start_states_ids.push_back(start_state_id);
            std::string vis_name = "start_config";
        
            auto markers = collisionChecker()->getCollisionModelVisualization(states[i]);
            for (auto& marker : markers) 
            {
                visual::Color color;
                
                color.g = 0.2;
                color.b = 0.2;
                color.r = 0.2;
                color.a=1;
                marker.ns = vis_name + std::to_string(i);
                marker.color = color;
            }
            SV_SHOW_INFO_NAMED(vis_name+ std::to_string(i), markers); 
        }

    }

    if(m_start_states_ids.empty())
    {
        ROS_ERROR_STREAM("Failed to find a valid start state!");
        return false;
    }

    m_start_state_id = m_start_states_ids[0];
    
     // notify observers of updated start state
    return RobotPlanningSpace::setStart(states[0]);
}

bool ManipLattice::setStart(const RobotState& state)
{
    SMPL_DEBUG_NAMED(params()->graph_log, "set the start state");

    if ((int)state.size() < robot()->jointVariableCount()) {
        SMPL_ERROR_NAMED(params()->graph_log, "start state does not contain enough joint positions");
        return false;
    }

    SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "  state: " << state);

    // check joint limits of starting configuration
    if (!robot()->checkJointLimits(state, true)) {
        SMPL_ERROR(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    if (!collisionChecker()->isStateValid(state, true)) {
        auto* vis_name = "invalid_start";
        SV_SHOW_INFO_NAMED(vis_name, collisionChecker()->getCollisionModelVisualization(state));
        SMPL_DEBUG(" -> in collision");
        return false;
    }
    
    auto* vis_name = "start_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(state, vis_name));

    // get arm position in environment
    RobotCoord start_coord(robot()->jointVariableCount());
    stateToCoord(state, start_coord);
    SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "  coord: " << start_coord);

    m_start_state_id = getOrCreateState(start_coord, state);
    // notify observers of updated start state
    return RobotPlanningSpace::setStart(state);
}

bool ManipLattice::setGoal(const GoalConstraint& goal)
{

    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL: {
        return setGoalPose(goal);
    }   break;
    case GoalType::JOINT_STATE_GOAL:
    {
        return setGoalConfiguration(goal);
    }
    default:
        return false;
    }
}


void ManipLattice::setVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

const std::string& ManipLattice::visualizationFrameId() const
{
    return m_viz_frame_id;
}

RobotState ManipLattice::getDiscreteCenter(const RobotState& state) const {
    RobotCoord coord(robot()->jointVariableCount());
    RobotState center(robot()->jointVariableCount());
    stateToCoord(state, coord);
    coordToState(coord, center);
    return center;
}

bool ManipLattice::extractPath(
    const std::vector<int>& idpath, std::vector<RobotState>& path ) //, std::vector<geometry_msgs::PoseStamped>& eePath)
{
    if (idpath.empty()) {
        return true;
    }

    //ROS_WARN_STREAM("Extract path calls apply not for dual arm. check when you reach here ");



    SMPL_INFO_STREAM("Extracting Path & saving data");
    std::vector<RobotState> opath;
    std::vector<int> statesSource;
    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        const int state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            const ManipLatticeState* entry = getHashEntry(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            opath.push_back(entry->state);
        } else {
            const ManipLatticeState* entry = getHashEntry(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            opath.push_back(entry->state);

            SMPL_INFO_STREAM(state_id<<","<<entry->source);
            statesSource.push_back(entry->source);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(opath.back(), vis_name));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        SMPL_ERROR_NAMED(params()->graph_log, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    // grab the first point
    {
        const ManipLatticeState* entry = getHashEntry(idpath[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
        SMPL_INFO_STREAM(idpath[0]<<","<<entry->source);
        statesSource.push_back(entry->source);
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        const int prev_id = idpath[i - 1];
        const int curr_id = idpath[i];
        SMPL_DEBUG_NAMED(params()->graph_log, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(params()->graph_log, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            SMPL_INFO_NAMED(params()->graph_log, "Search for transition to goal state");

            ManipLatticeState* prev_entry = m_states[prev_id];
            const RobotState& prev_state = prev_entry->state;

            std::vector<Action> actions;
            ROS_WARN_STREAM("In manip lattice extract path, gonna call apply with no group!!1");
            if (!m_actions->apply(prev_state, actions)) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get actions while extracting the path");
                return false;
            }

            // find the goal state corresponding to the cheapest valid action
            ManipLatticeState* best_goal_state = nullptr;
            RobotCoord succ_coord(robot()->jointVariableCount());
            int best_cost = std::numeric_limits<int>::max();
            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                // skip non-goal states
                ROS_INFO("Before isGoal in extractPath");
               if (!isGoal(action.back())) {
                   ROS_INFO("after failed isGoal in extractPath");
                    continue;
                }
                ROS_INFO("after isGoal in extractPath");
                // check the validity of this transition
                if (!checkAction(prev_state, action)) {
                    ROS_INFO("after check action failed");
                    continue;
                }
                ROS_INFO("after check action");

                stateToCoord(action.back(), succ_coord);
                getOrCreateState(succ_coord,action.back()); //IDK

                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                    ROS_INFO("1");
                assert(succ_entry);
                ROS_INFO("1");
                
                const int edge_cost = cost(prev_entry, succ_entry, 1, true); //hardcoded cost because it was crashing sometimes
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_goal_state = succ_entry;
                }
                ROS_INFO("1");
            }

            if (!best_goal_state) {
                SMPL_ERROR_STREAM_NAMED(params()->graph_log, "Failed to find valid goal successor from state " << prev_entry->state << " during path extraction");
                return false;
            }

            opath.push_back(best_goal_state->state);

            SMPL_INFO_STREAM(curr_id<<","<<best_goal_state->source);
            statesSource.push_back(best_goal_state->source);
        } else {
            const ManipLatticeState* entry = getHashEntry(curr_id);
            if (!entry) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to get state entry state %d", curr_id);
                return false;
            }

            SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "Extract successor state " << entry->state);
            opath.push_back(entry->state);
            SMPL_INFO_STREAM(curr_id<<","<<entry->source);
            statesSource.push_back(entry->source);
        }
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_DEBUG_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    SV_SHOW_INFO_NAMED("trajectory_source",makePathVisualization(path,statesSource));
    return true;
}

Extension* ManipLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>())
    {
        return this;
    }

    if (class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<PoseProjectionExtension>())
    {
        if (m_fk_iface) {
            return this;
        }
    }

    return nullptr;
}

/// \brief Return the ID of the goal state or -1 if no goal has been set.
int ManipLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

/// \brief Return the ID of the start state or -1 if no start has been set.
///
/// This returns the reserved id corresponding to all states which are goal
/// states and not the state id of any particular unique state.
int ManipLattice::getStartStateID() const
{
    return m_start_state_id;
}

std::vector<int> ManipLattice::getStartStatesID() const
{
    return m_start_states_ids;
}

/// \brief Get the (heuristic) distance from the planning frame position to the
///     start
RobotState ManipLattice::getStartConfiguration() const
{
    if (m_start_state_id >= 0) {
        return getHashEntry(m_start_state_id)->state;
    } else {
        return RobotState();
    }
}

/// Set a 6-dof goal pose for the planning link
bool ManipLattice::setGoalPose(const GoalConstraint& gc)
{

    const std::vector<double> offset = {0.0,-0.5,0.0,0,0,0};
    // check arguments
    if (gc.pose.size() != 6) {
        SMPL_ERROR_NAMED(params()->graph_log, "Goal pose has incorrect format");
        return false;
    }

    if (gc.tgt_off_pose.size() != 6) {
        SMPL_ERROR_NAMED(params()->graph_log, "Goal target offset pose has incorrect format");
        return false;
    }

    auto gc2(gc);

    gc2.tgt_off_pose[0]+=offset[0];
    gc2.tgt_off_pose[1]+=offset[1];
    gc2.tgt_off_pose[2]+=offset[2];
    gc2.tgt_off_pose[3]+=offset[3];
    gc2.tgt_off_pose[4]+=offset[4];
    gc2.tgt_off_pose[5]+=offset[5];

    gc2.pose[0]+=offset[0];
    gc2.pose[1]+=offset[1];
    gc2.pose[2]+=offset[2];
    gc2.pose[3]+=offset[3];
    gc2.pose[4]+=offset[4];
    gc2.pose[5]+=offset[5];

    Eigen::Affine3d goal_pose(
            Eigen::Translation3d(
                    gc.tgt_off_pose[0],
                    gc.tgt_off_pose[1],
                    gc.tgt_off_pose[2]) *
            Eigen::AngleAxisd(gc.tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(gc.tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(gc.tgt_off_pose[3], Eigen::Vector3d::UnitX()));

    Eigen::Affine3d goal_pose2(
            Eigen::Translation3d(
                    gc2.tgt_off_pose[0],
                    gc2.tgt_off_pose[1],
                    gc2.tgt_off_pose[2]) *
            Eigen::AngleAxisd(gc2.tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(gc2.tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(gc2.tgt_off_pose[3], Eigen::Vector3d::UnitX()));
    auto* vis_name = "goal_pose";
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(goal_pose, m_viz_frame_id, vis_name));
    //ROS_INFO_STREAM("plot 1");
    ;
    SV_SHOW_INFO_NAMED(vis_name, visual::MakePoseMarkers(goal_pose2, m_viz_frame_id, "goal_pose2"));
    //ROS_INFO_STREAM("plot2");
    ;

    using namespace std::chrono;
    auto now = clock::now();
    auto now_s = duration_cast<duration<double>>(now.time_since_epoch());
    SMPL_DEBUG_NAMED(params()->graph_log, "time: %f", now_s.count());
    SMPL_DEBUG_NAMED(params()->graph_log, "A new goal has been set.");
    SMPL_DEBUG_NAMED(params()->graph_log, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", gc.pose[0], gc.pose[1], gc.pose[2]);
    SMPL_DEBUG_NAMED(params()->graph_log, "    tol (meters): %0.3f", gc.xyz_tolerance[0]);
    SMPL_DEBUG_NAMED(params()->graph_log, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", gc.pose[3], gc.pose[4], gc.pose[5]);
    SMPL_DEBUG_NAMED(params()->graph_log, "    tol (radians): %0.3f", gc.rpy_tolerance[0]);

    startNewSearch();
    // set the (modified) goal
    ROS_INFO_STREAM("Manip Lattice: Set Goals in RobotPlanningSpace");
    return RobotPlanningSpace::setGoal2(gc2) && RobotPlanningSpace::setGoal(gc);
}

/// \brief Set a full joint configuration goal.
bool ManipLattice::setGoalConfiguration(const GoalConstraint& goal)
{
    ROS_ERROR_STREAM("NOT IMPLEMENTED FOR DUAL ARM. ENTER to cont..."); std::getchar();
    if (goal.type == GoalType::XYZ_GOAL ||
        goal.type == GoalType::XYZ_RPY_GOAL)
    {
        if (!m_fk_iface) {
            SMPL_DEBUG("ForwardKinematicsInterface required for pose goals");
            return false;
        }
    }

    startNewSearch();

    /*RobotCoord goal_coord(robot()->jointVariableCount());
    stateToCoord(goal.angles, goal_coord);
    SMPL_ERROR_STREAM_NAMED(params()->graph_log, "  coord: " << goal_coord);
    m_goal_unique_id = getOrCreateState(goal_coord, goal.angles);
    SMPL_ERROR_NAMED(params()->graph_log, "A UNIQUE GOAL!!!!! = %d", m_goal_unique_id);*/

    auto* vis_name = "goal_config";

    auto markers = collisionChecker()->getCollisionModelVisualization(goal.angles);
    for (auto& marker : markers) {
       
        visual::Color color;
        color.a=1;
        color.g=1;
        marker.ns = vis_name;

        marker.color = color;
        
    }
    

    SV_SHOW_INFO_NAMED(vis_name, markers);


    // notify observers of updated goal
    return RobotPlanningSpace::setGoal(goal);
}

// Reset any variables that should be set just before a new search is started.
void ManipLattice::startNewSearch()
{
    m_near_goal = false;
    m_t_start = clock::now();
}

/// \brief Return the 6-dof goal pose for the offset from the tip link.
std::vector<double> ManipLattice::getTargetOffsetPose(
    const std::vector<double>& tip_pose) const
{
    ROS_WARN_STREAM("ManipLattice:: getTargetOffsetPose not updated for dual arm.");
    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff = // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(tip_pose[0], tip_pose[1], tip_pose[2]) *
            Eigen::AngleAxisd(tip_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(tip_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tip_pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    goal().xyz_offset[0],
                    goal().xyz_offset[1],
                    goal().xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    return { voff.x(), voff.y(), voff.z(), tip_pose[3], tip_pose[4], tip_pose[5] };
}

} // namespace motion
} // namespace sbpl
