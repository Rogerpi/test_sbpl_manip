/* Author: Dina Youakim*/

#include <smpl/search/trastar.h>

#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>

#define ARAMDP_STATEID2IND_AD STATEID2IND_SLOT1
#define ARA_AD_INCONS_LIST_ID 1

namespace sbpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

TRAStar::TRAStar( DiscreteSpaceInformation* space, Heuristic* heuristic, bool bSearchForward)
:
   SBPLPlanner(),
    m_space(space),
    m_heur(heuristic),
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_graph_to_search_map(),
    m_open_base(),
    m_open_base_iso(),
    m_open_arm(),
    m_incons(),
    m_curr_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity()),
    bforwardsearch(bSearchForward),
    bsearchuntilfirstsolution(false),
    MaxMemoryCounter(0),
    costChanged(false),
    expansion_step(1),
    restore_step(-1)
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
    ROS_WARN_STREAM("TRA* constructor");
}

TRAStar::~TRAStar()
{
    for (TRAState* s : m_states) {
        delete s;
    }
}

/// Return statistics for each completed search iteration.
void TRAStar::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
    stats.cost;
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

int TRAStar::replan(const TimeParameters& params,std::vector<int>* solution,int* cost)
{
    ROS_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        ROS_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        ROS_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;
    
    start_state = getSearchState(m_start_state_id);
    goal_state = getSearchState(m_goal_state_id);

    SMPL_INFO_STREAM("Start State "<<start_state->state_id<< " has :");
    SMPL_INFO_STREAM("h-val "<<start_state->h<<" and g-val "<<start_state->g<<" and f-val "<<start_state->f);            
    SMPL_INFO_STREAM("E "<<start_state->E<<" and C "<<start_state->C);

    SMPL_INFO_STREAM("Goal State "<<goal_state->state_id<< " has :");
    SMPL_INFO_STREAM("h-val "<<goal_state->h<<" and g-val "<<goal_state->g<<" and f-val "<<goal_state->f);            
    SMPL_INFO_STREAM("E "<<goal_state->E<<" and C "<<goal_state->C<<",call "<<goal_state->call_number);


    if (m_start_state_id != m_last_start_state_id) {
        SMPL_INFO_STREAM("Start changed: Reinitialize search");
        m_open_arm.clear(sbpl::motion::BASE);
        m_open_base.clear(sbpl::motion::ARM);
        m_open_base_iso.clear(sbpl::motion::BASE_ISO);
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations


        SMPL_ERROR_NAMED(SLOG, "Reinitialize search start");
        reinitSearchState(start_state);
        SMPL_ERROR_NAMED(SLOG, "Reinitialize search goal");
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->f[0] = computeKey(start_state,sbpl::motion::GroupType::BASE);
        start_state->f[1] = computeKey(start_state,sbpl::motion::GroupType::ARM);
        m_open_base.push(start_state,sbpl::motion::GroupType::BASE);
        m_open_base_iso.push(start_state,sbpl::motion::GroupType::BASE_ISO);
        m_open_arm.push(start_state,sbpl::motion::GroupType::ARM);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;

        expansion_step = 1;
        start_state->C = 0;
        seen_states.push_back(start_state);
        start_state->E = INFINITECOST;

    }

    //TODO this condition has to change to incorporate the change of edges cost or heuristics, do I need to manually set the new start state ?
    if (m_goal_state_id != m_last_goal_state_id ) 
    {
        ROS_WARN_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();
        m_last_goal_state_id = m_goal_state_id;
    }

    if(costChanged)
    {
        ROS_WARN_NAMED(SLOG, "Cost Changed!!!");
        heuristicChanged();
        m_last_goal_state_id = m_goal_state_id;

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();
        costChanged = false;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
    /*while (m_satisfied_eps > m_final_eps)
    {
        if (m_curr_eps == m_satisfied_eps) {
            if (!m_time_params.improve) {
                break;
            }
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            for (TRAState* s : m_incons) {
                s->incons = false;
                m_open_base.push(s,sbpl::motion::GroupType::BASE);
                m_open_base_iso.push(s,sbpl::motion::GroupType::BASE_ISO);
                m_open_arm.push(s,sbpl::motion::GroupType::ARM);
            }
            reorderOpen();
            m_incons.clear();
            ROS_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        ROS_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

     //TODO check!!!!!!
    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open_base.empty()) {
            SearchState* next_state = m_open_base.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        if (m_allow_partial_solutions && !m_open_base_iso.empty()) {
            SearchState* next_state = m_open_base_iso.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        if (m_allow_partial_solutions && !m_open_arm.empty()) {
            SearchState* next_state = m_open_arm.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }
*/
    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int TRAStar::replan( double allowed_time, std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int TRAStar::replan( double allowed_time, std::vector<int>* solution, int* cost)
{
    TimeParameters tparams = m_time_params;
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    return replan(tparams, solution, cost);
}

int TRAStar::replan( std::vector<int>* solution, ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int TRAStar::replan( std::vector<int>* solution, ReplanParams params, int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
     convertReplanParamsToTimeParams(params, tparams);
    return replan(tparams, solution, cost);
}


int TRAStar::set_goal(int goal_state_id)
{
    /*if (bforwardsearch) {
        if (SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    else {
        if (SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }

    return 1;*/
    m_goal_state_id = goal_state_id;
    return 1;
}

int TRAStar::set_start(int start_state_id)
{
    /*if (bforwardsearch) {
        if (SetSearchStartState(start_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }
    else {
        if (SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }

    return 1;*/
    m_start_state_id = start_state_id;
    return 1;
}

//TODO: change implementation to recompute expansion and heuristics if needed
void TRAStar::costs_changed(const StateChangeQuery& stateChange)
{
    ROS_ERROR_NAMED(SLOG,"costs_changed(..) reInit!");
    force_planning_from_scratch();
}

void TRAStar::costs_changed()
{
    ROS_ERROR_NAMED(SLOG,"costs_changed() reInit!");
    costChanged = true;
}


int TRAStar::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    restore_step = -1;
    expansion_step = 1;
    return 0;
}

int TRAStar::set_search_mode(bool bSearchUntilFirstSolution)
{
    ROS_DEBUG_NAMED(SLOG,"planner: search mode set to %d\n", bSearchUntilFirstSolution);

    m_time_params.bounded = !bSearchUntilFirstSolution;
    return 0;
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int TRAStar::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open_base.clear(sbpl::motion::GroupType::BASE);
    m_open_base_iso.clear(sbpl::motion::GroupType::BASE_ISO);
    m_open_arm.clear(sbpl::motion::GroupType::ARM);
    m_graph_to_search_map.clear();
    m_graph_to_search_map.shrink_to_fit();
    for (TRAState* s : m_states) {
        delete s;
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

void TRAStar::update_succs_of_changededges(std::vector<int>* succsIDV)
{

}

//TODO: implement!!!!!!!!!!!
/// \brief direct form of informing the search about the new edge costs
/// \param predsIDV array of predecessors of changed edges
/// \note this is used when the search is run backwards
void TRAStar::update_preds_of_changededges(std::vector<int>* predsIDV)
{

}


// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void TRAStar::convertTimeParamsToReplanParams(const TimeParameters& t, ReplanParams& r) const
{
    r.max_time = to_seconds(t.max_allowed_time_init);
    r.return_first_solution = !t.bounded && !t.improve;
    if (t.max_allowed_time_init == t.max_allowed_time) {
        r.repair_time = -1.0;
    } else {
        r.repair_time = to_seconds(t.max_allowed_time);
    }

    r.initial_eps = m_initial_eps;
    r.final_eps = m_final_eps;
    r.dec_eps = m_delta_eps;
}

// Convert ReplanParams to TimeParameters. Sets the current initial, final, and
// delta eps from ReplanParams.
void TRAStar::convertReplanParamsToTimeParams(const ReplanParams& r, TimeParameters& t)
{
    t.type = TimeParameters::TIME;

    t.bounded = !r.return_first_solution;
    t.improve = !r.return_first_solution;

    t.max_allowed_time_init = to_duration(r.max_time);
    if (r.repair_time > 0.0) {
        t.max_allowed_time = to_duration(r.repair_time);
    } else {
        t.max_allowed_time = t.max_allowed_time_init;
    }

    m_initial_eps = r.initial_eps;
    m_final_eps = r.final_eps;
    m_delta_eps = r.dec_eps;
    if(r.restore_step!=-1)
    {
        restore_step = r.restore_step;
        //m_curr_eps = m_initial_eps;
        costChanged = true;
    }
}

// Test whether the search has run out of time.
bool TRAStar::timedOut(int elapsed_expansions, const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    default:
        ROS_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int TRAStar::improvePath( const clock::time_point& start_time,TRAState* goal_state, int& elapsed_expansions, clock::duration& elapsed_time)
{
    std::vector<int> succs;
    std::vector<int> costs;
    
    bool empty = m_open_base.empty() || m_open_arm.empty();// || m_open_base_iso.empty();
    while (!empty) {

        auto now = clock::now();
        elapsed_time = now - start_time;
        TRAState* min_state;
    
        if(!m_open_base_iso.empty())
            {
                min_state = m_open_base_iso.min();

                SMPL_DEBUG_STREAM("Base_iso min state "<<min_state->state_id<<" and f-val "<<min_state->f[0]);
                auto now = clock::now();
                elapsed_time = now - start_time;

                // path to goal found
                
                if (min_state->f[0] >= goal_state->f[0] || min_state->f[1] >= goal_state->f[1] || min_state == goal_state) {
                    SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
                    return SUCCESS;
                }

            
                if (timedOut(elapsed_expansions, elapsed_time)) {
                    SMPL_DEBUG_NAMED(SLOG, "BASE_ISO Ran out of time");
                    return TIMED_OUT;
                }

                SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);
                
                m_open_base_iso.pop(sbpl::motion::GroupType::BASE_ISO);
                

                assert(min_state->iteration_closed[sbpl::motion::GroupType::BASE] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::BASE] = m_iteration;

                assert(min_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;

                min_state->eg = min_state->g;

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::BASE_ISO));

                ++ elapsed_expansions;
            }

            SMPL_DEBUG_STREAM("base_iso path condition first "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));
        

         if(!m_open_base.empty())
            {
                min_state = m_open_base.min();

                SMPL_DEBUG_STREAM("Base min state "<<min_state->state_id<<" and f-val "<<min_state->f[0]);
                auto now = clock::now();
                elapsed_time = now - start_time;

                // path to goal found
                
                if (min_state->f[0] >= goal_state->f[0] || min_state->f[1] >= goal_state->f[1] || min_state == goal_state) {
                    SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
                    return SUCCESS;
                }

            
                if (timedOut(elapsed_expansions, elapsed_time)) {
                    SMPL_DEBUG_NAMED(SLOG, "BASE Ran out of time");
                    return TIMED_OUT;
                }

                SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);
                
                m_open_base.pop(sbpl::motion::GroupType::BASE);
                
                assert(min_state->iteration_closed[sbpl::motion::GroupType::BASE] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::BASE] = m_iteration;

                assert(min_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;

                min_state->eg = min_state->g;

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::BASE));

                ++ elapsed_expansions;
            }   

            SMPL_DEBUG_STREAM("base path condition first "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));

            if(!m_open_arm.empty())
            {
                min_state = m_open_arm.min();

                SMPL_DEBUG_STREAM("Arm min state "<<min_state->state_id<<" and f-val "<<min_state->f[1]);

                // path to goal found
                if (min_state->f[1] >= goal_state->f[1] || min_state->f[0] >= goal_state->f[0] || min_state == goal_state) {
                    SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
                    return SUCCESS;
                }

                if (timedOut(elapsed_expansions, elapsed_time)) {
                    SMPL_DEBUG_NAMED(SLOG, "ARM Ran out of time");
                    return TIMED_OUT;
                }

                SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);
                //SMPL_DEBUG_STREAM("Expanding state for "<<current_planning_group<<" with id "<<min_state->state_id<< " with f-val "<<min_state->f);

                m_open_arm.pop(sbpl::motion::GroupType::ARM);
               
                assert(min_state->iteration_closed[sbpl::motion::GroupType::BASE] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::BASE] = m_iteration;

                assert(min_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;

                min_state->eg = min_state->g;

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::ARM));

                ++ elapsed_expansions;
            }
            SMPL_DEBUG_STREAM("arm path condition "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));
            empty = m_open_base.empty() || m_open_arm.empty();// || m_open_base_iso.empty();
            SMPL_DEBUG_STREAM("one full cycle done. base_iso:"<<m_open_base_iso.empty()<<",base: "<<m_open_base.empty()<<",arm:"<<m_open_arm.empty()<<" and both:"<<empty);
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and TRAStar list appropriately.
void TRAStar::expand(TRAState* s, sbpl::motion::GroupType group)
{
    s->v = s->g;
    std::vector<int> succs;
    std::vector<int> costs;


    if(group==sbpl::motion::GroupType::BASE_ISO)
    {
        m_space->GetSuccsByGroupAndExpansion(s->state_id, &succs, &costs, sbpl::motion::GroupType::BASE, expansion_step); 
        
     
        SMPL_DEBUG_STREAM("group "<<group<<" has "<<succs.size()<<" successors");
        
        for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
            int succ_state_id = succs[sidx];
            int cost = costs[sidx];

            TRAState* succ_state = getSearchState(succ_state_id);
            reinitSearchState(succ_state);

            int new_cost = s->eg + cost;
            SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->bestpredstate = s;

                storeParent(succ_state,s,new_cost,expansion_step);

                if (succ_state->iteration_closed[sbpl::motion::BASE] != m_iteration) {
                    succ_state->f[0] = computeKey(succ_state,sbpl::motion::BASE);
                    
                    SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has base_iso f-val "<<succ_state->f[0]);
                    if (m_open_base_iso.contains(succ_state,sbpl::motion::BASE_ISO)) {
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base_iso.decrease(succ_state,sbpl::motion::BASE_ISO);
                    }
                    else {
                        m_open_base_iso.push(succ_state,sbpl::motion::BASE_ISO);
                        seen_states.push_back(succ_state);
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base_iso open list!");
                    }


                    SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has base f-val "<<succ_state->f[0]);
                    if (m_open_base.contains(succ_state,sbpl::motion::BASE)) {
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base.decrease(succ_state,sbpl::motion::BASE);
                    }
                    else {
                        m_open_base.push(succ_state,sbpl::motion::BASE);
                        seen_states.push_back(succ_state);
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }


                if (succ_state->iteration_closed[sbpl::motion::ARM] != m_iteration) {
                    succ_state->f[1] = computeKey(succ_state,sbpl::motion::ARM);
                    SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has arm f-val "<<succ_state->f[1]);
                    
                    if (m_open_arm.contains(succ_state,sbpl::motion::ARM)) {
                       SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has already been expanded in arm decrease priority!");
                        m_open_arm.decrease(succ_state,sbpl::motion::ARM);
                    }
                    else {
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to arm open list!");
                        seen_states.push_back(succ_state);
                        m_open_arm.push(succ_state,sbpl::motion::ARM);
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

            }
        }
    }

    else
    {
        m_space->GetSuccsByGroupAndExpansion(s->state_id, &succs, &costs, group, expansion_step); 
        
     
        SMPL_DEBUG_STREAM("group "<<group<<" has "<<succs.size()<<" successors");
        
        for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
            int succ_state_id = succs[sidx];
            int cost = costs[sidx];

            TRAState* succ_state = getSearchState(succ_state_id);
            reinitSearchState(succ_state);

            int new_cost = s->eg + cost;
            SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->bestpredstate = s;

                storeParent(succ_state,s,new_cost,expansion_step);

                if (succ_state->iteration_closed[sbpl::motion::BASE] != m_iteration) {
                    succ_state->f[0] = computeKey(succ_state,sbpl::motion::BASE);
                    
                    SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has base f-val "<<succ_state->f[0]);
                    if (m_open_base.contains(succ_state,sbpl::motion::BASE)) {
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base.decrease(succ_state,sbpl::motion::BASE);
                    }
                    else {
                        m_open_base.push(succ_state,sbpl::motion::BASE);
                        seen_states.push_back(succ_state);
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

                if (succ_state->iteration_closed[sbpl::motion::ARM] != m_iteration) {
                    succ_state->f[1] = computeKey(succ_state,sbpl::motion::ARM);
                    SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has arm f-val "<<succ_state->f[1]);
                    
                    if (m_open_arm.contains(succ_state,sbpl::motion::ARM)) {
                       SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has already been expanded in arm decrease priority!");
                        m_open_arm.decrease(succ_state,sbpl::motion::ARM);
                    }
                    else {
                        SMPL_DEBUG_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to arm open list!");
                        seen_states.push_back(succ_state);
                        m_open_arm.push(succ_state,sbpl::motion::ARM);
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

            }
        }
    }

    //update E of current state
    s->E = expansion_step;
    if(s->firstExpansionStep == -1)
        s->firstExpansionStep = expansion_step;
    //ROS_WARN_STREAM("Expansion at step "<<expansion_step);
    expansion_step++;

}

// Recompute heuristics for all states.
void TRAStar::recomputeHeuristics()
{
    for (TRAState* s : m_states) {
        s->h[0] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        s->h[1] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        SMPL_DEBUG_STREAM("Recompute: Base heuristic is "<<s->h[0]<<" arm heuristic is "<<s->h[1]);
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void TRAStar::reorderOpen()
{
   ROS_DEBUG_STREAM("reorderOpen called!");
    
     m_open_base_iso.make(sbpl::motion::GroupType::BASE_ISO);

    for (auto it = m_open_base.begin(); it != m_open_base.end(); ++it) {
        (*it)->f[0] = computeKey(*it, sbpl::motion::BASE);
    }
    m_open_base.make(sbpl::motion::GroupType::BASE);

    for (auto it = m_open_arm.begin(); it != m_open_arm.end(); ++it) {
        (*it)->f[1] = computeKey(*it, sbpl::motion::ARM);
    }
    m_open_arm.make(sbpl::motion::GroupType::ARM);
}


int TRAStar::computeKey(TRAState* s, sbpl::motion::GroupType group) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h[group]);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
TRAState* TRAStar::getSearchState(int state_id)
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }
    
    auto& state = m_states[state_id];
    if (state == NULL) {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
TRAState* TRAStar::createState(int state_id)
{
    assert(state_id < m_states.size());

    //m_graph_to_search_map[state_id] = (int)m_states.size();

    TRAState* ss = new TRAState;
    ss->state_id = state_id;
    ss->call_number = m_call_number;
    ss->E = INFINITECOST;
    ss->C = INFINITECOST;
    ss->v = 0;
    ss->g = INFINITECOST;
    ss->h[0] = m_heur->GetGoalHeuristic(ss->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
    ss->h[1] = m_heur->GetGoalHeuristic(ss->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
    ss->f[0] = ss->f[1] = INFINITECOST;
    ss->eg = INFINITECOST;
    ss->iteration_closed[0] = ss->iteration_closed[1] = 0;
    ss->call_number = m_call_number;
    ss->bestpredstate = nullptr;
    ss->bestnextstate = nullptr;
    ss->incons = false;
    ss->firstExpansionStep = -1;
    //m_states.push_back(ss);

    return ss;
}


// Lazily (re)initialize a search state.
void TRAStar::reinitSearchState(TRAState* state)
{
     if (state->call_number != m_call_number){// && state->C==INFINITECOST) {
        
        state->g = INFINITECOST;
        state->h[0] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        state->h[1] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        
        state->f[0] = state->f[1] = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed[0] = state->iteration_closed[1] = 0;
        state->call_number = m_call_number;
        state->bestpredstate = nullptr;
        state->bestnextstate = nullptr;
        state->incons = false;
        state->E = INFINITECOST;
        state->C = INFINITECOST;
        state->v = INFINITECOST;
        state->firstExpansionStep = -1;
    }
}

// Extract the path from the start state up to a new state.
void TRAStar::extractPath(TRAState* to_state, std::vector<int>& solution, int& cost) const
{
    //forward search
    for (TRAState* s = to_state; s; s = s->bestpredstate) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

bool TRAStar::RestoreSearchTree(int restoreStep)
{
    if(restoreStep<=0)
    {  
       //InitializeSearch();
    }
    else
    {
        m_open_base.clear(sbpl::motion::GroupType::BASE);
        m_open_base_iso.clear(sbpl::motion::GroupType::BASE_ISO);
        m_open_arm.clear(sbpl::motion::GroupType::ARM);
        m_states.clear();
        goal_state = getSearchState(m_goal_state_id);
        reinitSearchState(goal_state);
        //need to clear closed
        std::vector<TRAState*> current_seen;
        SMPL_INFO_STREAM("in restore seen states are "<<seen_states.size());
        TRAState* parent;
        unsigned int parentGVal;
        for(int i=0;i<seen_states.size();i++)
        {
            TRAState* current = seen_states[i];
            SMPL_INFO_STREAM("Seen State "<<current->state_id<< " has  C "<<current->C<< " & has  E "<<current->E);
            //state created and expanded
            if(current->E <= restoreStep)
            {
                SMPL_INFO_STREAM("Fully Restored!");
                updateParents(current,restoreStep,current->bestpredstate,&parentGVal);
                current->g = parentGVal;
                current->h[0] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
                current->h[1] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
                current->f[0] = computeKey(current,sbpl::motion::GroupType::BASE);
                current->f[1] = computeKey(current,sbpl::motion::GroupType::ARM);
                //insert in closed
                m_states.push_back(current);
                current_seen.push_back(current);
            }
            //state created only
            else if(current->C <= restoreStep)
            {
                SMPL_INFO_STREAM("State generated only!");
                updateParents(current,restoreStep,current->bestpredstate,&parentGVal);
                current->g = parentGVal;
                current->v = INFINITECOST;
                current->h[0] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
                current->h[1] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
                current->f[0] = computeKey(current,sbpl::motion::GroupType::BASE);
                current->f[1] = computeKey(current,sbpl::motion::GroupType::ARM);
                current->E = INFINITECOST;
                current_seen.push_back(current);
                //m_open.push(current);
                SMPL_INFO_STREAM("State generated only with C, E, g, h "<<current->C);
            }
            //state not created yet
            else
            {
                SMPL_INFO_STREAM("Clear State!");
                current->v = INFINITECOST;
                current->f[0] = current->f[1] = INFINITECOST;
                current->g = INFINITECOST;
                current->C = INFINITECOST;
                current->E = INFINITECOST;
                current->eg = INFINITECOST;
                current->iteration_closed[0] = current->iteration_closed[1] = 0;
                current->bestpredstate = nullptr;
                current->bestnextstate = nullptr;
                current->parent_hist.clear();
                current->gval_hist.clear(); 
                current->h[0] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
                current->h[1] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE); 
                //current->f = computeKey(current);
                //current->call_number = m_call_number;
                //current->incons = false;
                current->firstExpansionStep = -1;
            }
        }
        seen_states = current_seen;
        expansion_step = restoreStep+1;
        ROS_WARN_STREAM("Restore to step "<<restoreStep<<" done.");
    }

}

bool TRAStar::updateParents(TRAState* state, unsigned int expansionStep, TRAState* latestParent, unsigned int *latestGVal)
{
    unsigned int latestParentStep = 0;
    latestParent = nullptr;
    *latestGVal = 0;

    for(int i=0;i<state->parent_hist.size();i++)
    {
        TRAState* parent = state->parent_hist[i];
        //this parent is a valid one for the given expansion step
        if(parent->E <= expansionStep)
        {
            SMPL_INFO_STREAM("updating parent with id "<<parent->state_id<<", E "<<parent->E);
            if(parent->E > latestParentStep)
            {
                latestParent = parent;
                latestGVal = &state->gval_hist[i];
                latestParentStep = parent->E;
                state->bestpredstate = latestParent;
            }
        }
        //this parent is not valid for the given expansion step
        else
        {
            SMPL_INFO_STREAM("Erasing Parent with id "<<parent->state_id<<", E "<<parent->E);
            state->parent_hist.erase(state->parent_hist.begin() + i);
            state->gval_hist.erase(state->gval_hist.begin() + i);          
        }
    }   
}

bool TRAStar::storeParent(TRAState* succ_state, TRAState* state, unsigned int gVal, unsigned int expansionStep)
{
    state->parent_hist.push_back(succ_state);
    state->gval_hist.push_back(gVal);
    return 1;
}

void TRAStar::heuristicChanged()
{   
    

    bool done = false;
    std::vector<unsigned int> inconsE;

    RestoreSearchTree(restore_step);

    recomputeHeuristics();
    reorderOpen();
    //How the edges are identified (cell to edge mapping), I don't see it handled here
    /*while(!done)
    {
        TRAState* minState = m_open.min();
        //loop on closed
        for (TRAState* s : m_states) 
        {
            unsigned int cost = s->v + (unsigned int)(m_curr_eps * s->h);
            if(costChanged)
               // ROS_WARN_STREAM("Cost "<<cost<<" ,F "<<minState->f<<" ,C "<<minState->C<<" ,E "<<s->E);
            if(cost > minState->f && minState->C < s->E)
            {
                ROS_ERROR_STREAM("The new E pushed "<<s->E);
                inconsE.push_back(s->E);
            }
        }
        if(inconsE.empty())
            done = true;
        else 
        {
            unsigned int newStep  = (*std::min_element(inconsE.begin(),inconsE.end())) - 1;
           
            RestoreSearchTree(newStep); 
        }
    }*/

}

void TRAStar::InitializeSearch()
{
    //clear closed
    TRAState* start = getSearchState(m_start_state_id);
    //m_open.push(start);
    start->g = 0;
    //computeKey(start);
    expansion_step = 1;
    start->C = 0;
    seen_states.push_back(start);
    start->E = INFINITECOST;
}


void TRAStar::Recomputegval(TRAState* state)
{

}


// used for backward search
void TRAStar::UpdatePreds(TRAState* state)
{
    /*std::vector<int> PredIDV;
    std::vector<int> CostV;
    CKey key;
    TRAState* predstate;

    m_space->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

    // iterate through predecessors of s
    for (int pind = 0; pind < (int)PredIDV.size(); pind++) {
        CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
        predstate = (TRAState*)(PredMDPState->PlannerSpecificData);
        if (predstate->callnumberaccessed != pSearchStateSpace->callnumber) {
            ReInitializeSearchStateInfo(predstate, pSearchStateSpace);
        }

        // see if we can improve the value of predstate
        if (predstate->g > state->v + CostV[pind]) {
            predstate->g = state->v + CostV[pind];
            predstate->bestnextstate = state->MDPstate;
            predstate->costtobestnextstate = CostV[pind];

            // re-insert into heap if not closed yet
            if (predstate->iterationclosed != pSearchStateSpace->searchiteration) {
                key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
//                key.key[1] = predstate->h;
                if (predstate->heapindex != 0) {
                    pSearchStateSpace->heap->updateheap(predstate,key);
                }
                else {
                    pSearchStateSpace->heap->insertheap(predstate,key);
                }
            }
            else if (predstate->listelem[ARA_AD_INCONS_LIST_ID] == NULL) {
                // take care of incons list
                pSearchStateSpace->inconslist->insert(predstate, ARA_AD_INCONS_LIST_ID);
            }
        }
    } // for predecessors*/
}

// used for forward search
void TRAStar::UpdateSuccs(TRAState* state)
{
    /*std::vector<int> SuccIDV;
    std::vector<int> CostV;
    CKey key;
    TRAState* succstate;

    m_space->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

    // iterate through predecessors of s
    for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
        CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
        int cost = CostV[sind];

        succstate = (TRAState*)(SuccMDPState->PlannerSpecificData);
        if (succstate->callnumberaccessed != pSearchStateSpace->callnumber) {
            ReInitializeSearchStateInfo(succstate, pSearchStateSpace);
        }

        // update generated index

        // see if we can improve the value of succstate
        // taking into account the cost of action
        if (succstate->g > state->v + cost) {
            succstate->g = state->v + cost;
            succstate->bestpredstate = state->MDPstate;

            // re-insert into heap if not closed yet
            if (succstate->iterationclosed != pSearchStateSpace->searchiteration) {
                key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);
//                key.key[1] = succstate->h;

                if (succstate->heapindex != 0) {
                    pSearchStateSpace->heap->updateheap(succstate,key);
                }
                else {
                    pSearchStateSpace->heap->insertheap(succstate,key);
                }
            }
            else if (succstate->listelem[ARA_AD_INCONS_LIST_ID] == NULL) {
                // take care of incons list
                pSearchStateSpace->inconslist->insert(succstate, ARA_AD_INCONS_LIST_ID);
            }
        } // check for cost improvement

    } // for actions
}

void TRAStar::BuildNewOPENList()
{
    /*TRAState* state;
    CKey key;
    CHeap* pheap = pSearchStateSpace->heap;
    CList* pinconslist = pSearchStateSpace->inconslist;

    // move incons into open
    while (pinconslist->firstelement != NULL) {
        state = (TRAState*)pinconslist->firstelement->liststate;

        // compute f-value
        key.key[0] = state->g + (int)(pSearchStateSpace->eps * state->h);
//        key.key[1] = state->h;

        // insert into OPEN
        pheap->insertheap(state, key);
        // remove from INCONS
        pinconslist->remove(state, ARA_AD_INCONS_LIST_ID);
    }*/
}


void TRAStar::PrintSearchState(TRAState* state, FILE* fOut)
{
    SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d heapind=%d inconslist=%d\n", state->MDPstate->StateID, state->h, state->g, state->v, state->iterationclosed, state->callnumberaccessed, state->heapindex, state->listelem[ARA_AD_INCONS_LIST_ID] ? 1 : 0);
    m_space->PrintState(state->state_id, true, fOut);
}

} // namespace sbpl
