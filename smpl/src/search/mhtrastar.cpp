/* Author: Dina Youakim*/

#include <smpl/search/mhtrastar.h>

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

MHTRAStar::MHTRAStar( DiscreteSpaceInformation* space, Heuristic* heuristic, bool bSearchForward)
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
    restore_step(-1),
    last_expanded_state_id (-1)
{
    SMPL_ERROR_STREAM("MHTRA constructor!!");
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
}

MHTRAStar::~MHTRAStar()
{
    SMPL_INFO_STREAM("in MHTRA destructor!!!!!! "<<m_states.size());
    /*for (MHTRAState* s : m_states) {
        if(s!=NULL)
         delete s;
    }*/
}

/// Return statistics for each completed search iteration.
void MHTRAStar::get_search_stats(std::vector<PlannerStats>* s)
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

int MHTRAStar::replan(const TimeParameters& params,std::vector<int>* solution,int* cost)
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

    ROS_INFO_STREAM("Goal State "<<goal_state->state_id<< " has :");
    ROS_INFO_STREAM("h-val "<<goal_state->h[0]<<" & "<<goal_state->h[1]<<" and g-val "<<goal_state->g<<" and f-val "<<goal_state->f[0]<<" & "<<goal_state->f[1]);            
    ROS_INFO_STREAM("E "<<goal_state->E<<" and C "<<goal_state->C<<",call "<<goal_state->call_number);


    if (m_start_state_id != m_last_start_state_id) {
        SMPL_INFO_STREAM("Start changed: Reinitialize search");
        m_open_arm.clear(sbpl::motion::BASE);
        m_open_base.clear(sbpl::motion::ARM);
        m_open_base_iso.clear(sbpl::motion::BASE_ISO);
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations


        SMPL_INFO_NAMED(SLOG, "Reinitialize search goal");
        reinitSearchState(goal_state);
        
        initializeStartStates();
        SMPL_INFO_NAMED(SLOG, "Initialize multiple search starts");
        
        /*ROS_INFO_STREAM("Start State "<<start_state->state_id<< " has :");
        ROS_INFO_STREAM("h-val "<<start_state->h[0]<<" & "<<start_state->h[1]<<" and g-val "<<start_state->g<<" and f-val "<<start_state->f[0]<<" & "<<start_state->f[1]);            
        ROS_INFO_STREAM("E "<<start_state->E<<" and C "<<start_state->C);*/

        //start_state = m_states[m_start_state_id];
        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;

        //TO CHECK???
        expansion_step = 1;

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
        ROS_INFO_STREAM("Cost Changed!!!");
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
    int num_expansions = expansion_step;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    //err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
    //while (m_satisfied_eps > m_final_eps)
    {
        if (m_curr_eps == m_satisfied_eps) {
            /*if (!m_time_params.improve) {
                break;
            }*/
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            /*for (MHTRAState* s : m_incons) {
                s->incons = false;
                m_open_base.push(s,sbpl::motion::GroupType::BASE);
                m_open_base_iso.push(s,sbpl::motion::GroupType::BASE_ISO);
                m_open_arm.push(s,sbpl::motion::GroupType::ARM);
            }*/
            reorderOpen();
            m_incons.clear();
            ROS_INFO_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        ROS_INFO_STREAM("Improve done with result "<<err);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }

        /*if (err) {
            break;
        }*/
        ROS_INFO_NAMED(SLOG, "Improved solution with err %i", err);
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

     //TODO check!!!!!!
    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open_base.empty()) {
            MHTRAState* next_state = m_open_base.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        if (m_allow_partial_solutions && !m_open_base_iso.empty()) {
            MHTRAState* next_state = m_open_base_iso.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        if (m_allow_partial_solutions && !m_open_arm.empty()) {
            MHTRAState* next_state = m_open_arm.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }
    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int MHTRAStar::replan( double allowed_time, std::vector<int>* solution)
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
int MHTRAStar::replan( double allowed_time, std::vector<int>* solution, int* cost)
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

int MHTRAStar::replan( std::vector<int>* solution, ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int MHTRAStar::replan( std::vector<int>* solution, ReplanParams params, int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
    convertReplanParamsToTimeParams(params, tparams);
    bool result = replan(tparams, solution, cost);
    ROS_WARN_STREAM("Replan done with result "<<result);
    return result;
}


int MHTRAStar::set_goal(int goal_state_id)
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

int MHTRAStar::set_start(int start_state_id)
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

int MHTRAStar::set_multiple_start(std::vector<int> start_statesID)
{
    m_start_state_id = start_statesID[0];
    m_start_states_ids.resize(start_statesID.size(),0);
    std::copy(start_statesID.begin(),start_statesID.end(),m_start_states_ids.begin());
}

//TODO: change implementation to recompute expansion and heuristics if needed
void MHTRAStar::costs_changed(const StateChangeQuery& stateChange)
{
    ROS_ERROR_NAMED(SLOG,"costs_changed(..) reInit!");
    force_planning_from_scratch();
}

void MHTRAStar::costs_changed()
{
    ROS_ERROR_NAMED(SLOG,"costs_changed() reInit!");
    costChanged = true;
}


int MHTRAStar::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    restore_step = -1;
    expansion_step = 1;
    return 0;
}

int MHTRAStar::set_search_mode(bool bSearchUntilFirstSolution)
{
    ROS_DEBUG_NAMED(SLOG,"planner: search mode set to %d\n", bSearchUntilFirstSolution);

    m_time_params.bounded = !bSearchUntilFirstSolution;
    return 0;
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int MHTRAStar::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open_base.clear(sbpl::motion::GroupType::BASE);
    m_open_base_iso.clear(sbpl::motion::GroupType::BASE_ISO);
    m_open_arm.clear(sbpl::motion::GroupType::ARM);
    m_graph_to_search_map.clear();
    m_graph_to_search_map.shrink_to_fit();
    for (MHTRAState* s : m_states) {
        delete s;
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

void MHTRAStar::update_succs_of_changededges(std::vector<int>* succsIDV)
{

}

//TODO: implement!!!!!!!!!!!
/// \brief direct form of informing the search about the new edge costs
/// \param predsIDV array of predecessors of changed edges
/// \note this is used when the search is run backwards
void MHTRAStar::update_preds_of_changededges(std::vector<int>* predsIDV)
{

}


// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void MHTRAStar::convertTimeParamsToReplanParams(const TimeParameters& t, ReplanParams& r) const
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
void MHTRAStar::convertReplanParamsToTimeParams(const ReplanParams& r, TimeParameters& t)
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
        t.max_allowed_time = t.max_allowed_time_init;
    }
}

// Test whether the search has run out of time.
bool MHTRAStar::timedOut(int elapsed_expansions, const clock::duration& elapsed_time) const
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
int MHTRAStar::improvePath( const clock::time_point& start_time,MHTRAState* goal_state, int& elapsed_expansions, clock::duration& elapsed_time)
{
    std::vector<int> succs;
    std::vector<int> costs;
    
    bool empty = m_open_base.empty() || m_open_arm.empty();// || m_open_base_iso.empty();
    while (!empty) {

        auto now = clock::now();
        elapsed_time = now - start_time;
        MHTRAState* min_state;
        
        if(!m_open_base_iso.empty())
            {
                min_state = m_open_base_iso.min();


                SMPL_INFO_STREAM("Base_iso min state "<<min_state->state_id<<" has a heuristics "<<min_state->h[0]<<" & "<<min_state->h[1]<<" and g-val "<<min_state->g
                <<" f-val "<<min_state->f[0]<<" & "<<min_state->f[1]);

                   
                // path to goal found
                
                if (min_state == goal_state) {
                    SMPL_INFO_STREAM("Found path to goal with f-val "<<goal_state->f[0]<<","<<goal_state->f[1]<<" and min f-val "<<min_state->f[0]<<","<<min_state->f[1]);
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

                if(elapsed_expansions==1)
                {
                    start_state = min_state;
                    seen_states.push_back(min_state);
                    m_space->setSelectedStartId(min_state->state_id);
                    m_start_state_id = min_state->state_id;
                }
                
                if(min_state->state_id>0 && min_state->state_id<m_start_states_ids.size() && min_state->state_id!= m_start_state_id)
                {
                    seen_states.push_back(min_state);
                    ROS_INFO_STREAM("One of the non-chosen start states is the min, add it to seen");
                }

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::BASE_ISO));
                
                //last_expanded_state_id = min_state->state_id;

                ++ elapsed_expansions;
            }

            SMPL_INFO_STREAM("base_iso path condition first "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));
        
        
         if(!m_open_base.empty())
            {
                min_state = m_open_base.min();

                SMPL_INFO_STREAM("Base min state "<<min_state->state_id<<" has a heuristics "<<min_state->h[0]<<" & "<<min_state->h[1]<<" and g-val "<<min_state->g
                <<" f-val "<<min_state->f[0]<<" & "<<min_state->f[1]);
               

                // path to goal found
                
                if (min_state == goal_state) {
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
                
                 if(elapsed_expansions==1)
                {
                    start_state = min_state;
                    seen_states.push_back(min_state);
                    m_space->setSelectedStartId(min_state->state_id);
                    m_start_state_id = min_state->state_id;
                }
                
                if(min_state->state_id>0 && min_state->state_id<m_start_states_ids.size() && min_state->state_id!= m_start_state_id)
                {
                    seen_states.push_back(min_state);
                    ROS_INFO_STREAM("One of the non-chosen start states is the min, add it to seen");
                }

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::BASE));

                //last_expanded_state_id = min_state->state_id;

                ++ elapsed_expansions;
            }   

            SMPL_INFO_STREAM("base path condition first "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));

        

       // ROS_INFO_STREAM("Min f-val "<<min_state->f[0]<<" & "<<min_state->f[1]<<" vs. goal f-val "<<goal_state->f);
        /*if(!m_open_arm.empty())
            {
                min_state = m_open_arm.min();

                SMPL_INFO_STREAM("Arm min state "<<min_state->state_id<<" has a heuristics "<<min_state->h[0]<<" & "<<min_state->h[1]<<" and g-val "<<min_state->g
                <<" f-val "<<min_state->f[0]<<" & "<<min_state->f[1]);

                // path to goal found
                if (min_state == goal_state) {
                    SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
                    return SUCCESS;
                }

                if (timedOut(elapsed_expansions, elapsed_time)) {
                    SMPL_DEBUG_NAMED(SLOG, "ARM Ran out of time");
                    return TIMED_OUT;
                }

                SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);
                //SMPL_INFO_STREAM("Expanding state for "<<current_planning_group<<" with id "<<min_state->state_id<< " with f-val "<<min_state->f);

                m_open_arm.pop(sbpl::motion::GroupType::ARM);
               
                assert(min_state->iteration_closed[sbpl::motion::GroupType::BASE] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::BASE] = m_iteration;

                assert(min_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration);
                assert(min_state->g != INFINITECOST);
                min_state->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;

                min_state->eg = min_state->g;

                  //Add the selected start state to the seen states
               /* if(elapsed_expansions==1 && min_state->state_id>0 && min_state->state_id<=m_start_states_ids.size())
                {
                    SMPL_INFO_STREAM("Adding the start state ["<<min_state->state_id<<"] to the seen_states");
                    seen_states.push_back(min_state);
                    //m_space->displaySelectedGoal(min_state->state_id);
                }*/

                /* update the start state ID to be the one with the min heuristic in the space */
                /*if(elapsed_expansions==1)
                {
                    start_state = min_state;
                    seen_states.push_back(min_state);
                    m_space->setSelectedStartId(min_state->state_id);
                    m_start_state_id = min_state->state_id;
                }
                
                if(min_state->state_id>0 && min_state->state_id<m_start_states_ids.size() && min_state->state_id!= m_start_state_id)
                {
                    seen_states.push_back(min_state);
                    ROS_INFO_STREAM("One of the non-chosen start states is the min, add it to seen");
                }

                expand(min_state, sbpl::motion::GroupType(sbpl::motion::GroupType::ARM));

                //last_expanded_state_id = min_state->state_id;

                ++ elapsed_expansions;
            }
            SMPL_INFO_STREAM("arm path condition "<<(min_state->f[0] >= goal_state->f[0])<<
                "first arm "<<(min_state->f[1] >= goal_state->f[1])<<
                ",second "<<(min_state == goal_state));
            */
            
            empty = m_open_base.empty() || m_open_arm.empty();// || m_open_base_iso.empty();
            SMPL_INFO_STREAM("one full cycle done. base_iso:"<<m_open_base_iso.empty()<<",base: "<<m_open_base.empty()<<",arm:"<<m_open_arm.empty()<<" and both:"<<empty);
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and MHTRAStar list appropriately.
void MHTRAStar::expand(MHTRAState* s, sbpl::motion::GroupType group)
{
    s->eg = s->g;
    std::vector<int> succs;
    std::vector<int> costs;
    m_clearance_cells.clear();
    //update E of current state
    s->E = expansion_step;
    
   // ROS_INFO_STREAM("Expanded parent ID "<<last_expanded_state_id);
    if(!s->parent_hist.empty())
    {
        last_expanded_state_id = s->parent_hist.back();
        ROS_INFO_STREAM("From parent list "<<s->parent_hist.back());
    }
    /*s->source = group;
    int id = s->state_id;
    int index;
    auto it = find_if(seen_states.begin(), seen_states.end(), [&id](MHTRAState* obj) {return obj->state_id == id;});
    if (it != seen_states.end())
    {
        index = std::distance(seen_states.begin(), it);
        seen_states[index]->source = group;
        seen_states[index]->E = expansion_step;
        SMPL_INFO("seen & found ");
    }*/
    if(s->firstExpansionStep == -1)
        s->firstExpansionStep = expansion_step;

    if(group==sbpl::motion::GroupType::BASE_ISO)
    {
        m_space->GetPredsByGroupAndExpansion(s->state_id, &succs, &costs, &m_clearance_cells,sbpl::motion::GroupType::BASE, expansion_step, last_expanded_state_id); 
        
        ROS_INFO_NAMED(SELOG, " State %zu has %zu successors with expansion step %zu", s->state_id,succs.size(),expansion_step);
   
        for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
            int succ_state_id = succs[sidx];
            int cost = costs[sidx];

            MHTRAState* succ_state = getSearchState(succ_state_id);
            reinitSearchState(succ_state);
            succ_state->h[0] += m_clearance_cells[sidx];
            succ_state->h[1] += m_clearance_cells[sidx]; 
            succ_state->source = group;
            int new_cost = s->eg + cost;
            bool addedToSeen = false;
            ROS_INFO_NAMED(SELOG, "Compare new cost %d vs old cost %d for state %i ", new_cost, succ_state->g, succ_state->state_id, s->g);
            
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->bestpredstate = s;
                ROS_WARN_STREAM("Pred of state "<<succ_state->state_id<<" is "<<s->state_id);
                storeParent(succ_state,s,new_cost,expansion_step);

                if (succ_state->iteration_closed[sbpl::motion::BASE] != m_iteration) {
                    succ_state->f[0] = computeKey(succ_state,sbpl::motion::BASE);
                    
                    SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has base_iso f-val "<<succ_state->f[0]);
                    if (m_open_base_iso.contains(succ_state,sbpl::motion::BASE_ISO)) {
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base_iso.decrease(succ_state,sbpl::motion::BASE_ISO);
                    }
                    else {
                        m_open_base_iso.push(succ_state,sbpl::motion::BASE_ISO);
                        succ_state->C = expansion_step;
                        seen_states.push_back(succ_state);
                        addedToSeen = true;
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base_iso open list!");
                    }


                    SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has base f-val "<<succ_state->f[0]);
                    if (m_open_base.contains(succ_state,sbpl::motion::BASE)) {
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base.decrease(succ_state,sbpl::motion::BASE);
                    }
                    else {
                        m_open_base.push(succ_state,sbpl::motion::BASE);
                        succ_state->C = expansion_step;
                        if(!addedToSeen)
                        {
                            seen_states.push_back(succ_state);
                            addedToSeen = true;
                        }
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }


                if (succ_state->iteration_closed[sbpl::motion::ARM] != m_iteration) {
                    succ_state->f[1] = computeKey(succ_state,sbpl::motion::ARM);
                    SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has arm f-val "<<succ_state->f[1]);
                    
                    if (m_open_arm.contains(succ_state,sbpl::motion::ARM)) {
                       SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has already been expanded in arm decrease priority!");
                        m_open_arm.decrease(succ_state,sbpl::motion::ARM);
                    }
                    else {
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to arm open list!");
                        succ_state->C= expansion_step;
                        m_open_arm.push(succ_state,sbpl::motion::ARM);
                        if(!addedToSeen)
                        {
                            seen_states.push_back(succ_state);
                            addedToSeen = true;
                        }
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

            }

            ROS_INFO_STREAM("State "<<succ_state->state_id<< " has :");
            ROS_INFO_STREAM("h-val "<<succ_state->h[0]<<" & "<<succ_state->h[1]<<" and g-val "<<succ_state->g<<" and f-val "<<succ_state->f[0]<<" & "<<succ_state->f[1]);            
            ROS_INFO_STREAM("E "<<succ_state->E<<" and C "<<succ_state->C);

        }
    }

    else
    {
       m_space->GetPredsByGroupAndExpansion(s->state_id, &succs, &costs, &m_clearance_cells,group, expansion_step,last_expanded_state_id); 
        
        ROS_INFO_NAMED(SELOG, " State %zu has %zu successors with expansion step %zu", s->state_id,succs.size(),expansion_step);
        
        for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
            int succ_state_id = succs[sidx];
            int cost = costs[sidx];

            MHTRAState* succ_state = getSearchState(succ_state_id);
            reinitSearchState(succ_state);
            succ_state->h[0] += m_clearance_cells[sidx];
            succ_state->h[1] += m_clearance_cells[sidx]; 

            succ_state->source = group;
            int new_cost = s->eg + cost;
            bool addedToSeen = false;
            ROS_INFO_NAMED(SELOG, "Compare new cost %d vs old cost %d for state %i ", new_cost, succ_state->g, succ_state->state_id, s->g);
            
            if (new_cost < succ_state->g) {
                succ_state->g = new_cost;
                succ_state->bestpredstate = s;

                ROS_WARN_STREAM("Pred of state "<<succ_state->state_id<<" is "<<s->state_id);
                storeParent(succ_state,s,new_cost,expansion_step);

                if (succ_state->iteration_closed[sbpl::motion::BASE] != m_iteration) {
                    succ_state->f[0] = computeKey(succ_state,sbpl::motion::BASE);
                    
                    SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has base f-val "<<succ_state->f[0]);
                    if (m_open_base.contains(succ_state,sbpl::motion::BASE)) {
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                        m_open_base.decrease(succ_state,sbpl::motion::BASE);
                    }
                    else {
                        m_open_base.push(succ_state,sbpl::motion::BASE);
                        succ_state->C = expansion_step;
                        if(!addedToSeen)
                        {
                            seen_states.push_back(succ_state);
                            addedToSeen = true;
                        }
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

                if (succ_state->iteration_closed[sbpl::motion::ARM] != m_iteration) {
                    succ_state->f[1] = computeKey(succ_state,sbpl::motion::ARM);
                    SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has arm f-val "<<succ_state->f[1]);
                    
                    if (m_open_arm.contains(succ_state,sbpl::motion::ARM)) {
                       SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has already been expanded in arm decrease priority!");
                        m_open_arm.decrease(succ_state,sbpl::motion::ARM);
                    }
                    else {
                        SMPL_INFO_STREAM("State "<<succ_state->state_id<< " has never been expanded, added to arm open list!");
                        succ_state->C = expansion_step;
                        m_open_arm.push(succ_state,sbpl::motion::ARM);
                        if(!addedToSeen)
                        {
                            seen_states.push_back(succ_state);
                            addedToSeen = true;
                        }

                    }
                } else if (!succ_state->incons) {
                    m_incons.push_back(succ_state);
                }

            }

            ROS_INFO_STREAM("State "<<succ_state->state_id<< " has :");
            ROS_INFO_STREAM("h-val "<<succ_state->h[0]<<" & "<<succ_state->h[1]<<" and g-val "<<succ_state->g<<" and f-val "<<succ_state->f[0]<<" & "<<succ_state->f[1]);            
            ROS_INFO_STREAM("E "<<succ_state->E<<" and C "<<succ_state->C);

        }
    }
    if(succs.size()>0)
        expansion_step++;
}

int MHTRAStar::getParentStateIdByExpansionStep()
{
    if(expansion_step==1)
        return -1;
    auto t1 = ros::Time::now();
    for (int i=0; i< seen_states.size();i++) 
    {   
        MHTRAState* s = seen_states[i];
        if(s->E==expansion_step-1)
        {
           // SMPL_INFO_STREAM("Looking for parent took "<<ros::Time::now().toSec()-t1.toSec());
            return s->state_id;
        }
    }
    return -1;
}
// Recompute heuristics for all states.
void MHTRAStar::recomputeHeuristics()
{   
    for (MHTRAState* s : m_states) {
        if (s != NULL) 
        {
            s->h[0] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
            s->h[1] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
            //s->h[1] = std::max(s->h[0],s->h[1]);
            SMPL_INFO_STREAM("after Recompute for state: "<<s->state_id);
            SMPL_INFO_STREAM(" Base heuristic is "<<s->h[0]<<" arm heuristic is "<<s->h[1]);
        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void MHTRAStar::reorderOpen()
{
   ROS_DEBUG_STREAM("reorderOpen called! with epsilon "<<m_curr_eps);
    
    for (auto it = m_open_base_iso.begin(); it != m_open_base_iso.end(); ++it) {
        (*it)->f[0] = computeKey(*it, sbpl::motion::BASE);
       /* SMPL_INFO_STREAM("base_iso Reorder for state: "<<(*it)->state_id<<"h-val "<<(*it)->h[0]<<" & "<<(*it)->h[1]<<"  g-val "<<
            (*it)->g<<" and f-val "<<(*it)->f[0]);*/
    }
    m_open_base_iso.make(sbpl::motion::GroupType::BASE_ISO);

    for (auto it = m_open_base.begin(); it != m_open_base.end(); ++it) {
        (*it)->f[0] = computeKey(*it, sbpl::motion::BASE);
        /*SMPL_INFO_STREAM("base Reorder for state: "<<(*it)->state_id<<"h-val "<<(*it)->h[0]<<" & "<<(*it)->h[1]<<"  g-val "<<
            (*it)->g<<" and f-val "<<(*it)->f[0]);*/
    }
    m_open_base.make(sbpl::motion::GroupType::BASE);

    for (auto it = m_open_arm.begin(); it != m_open_arm.end(); ++it) {
        (*it)->f[1] = computeKey(*it, sbpl::motion::ARM);
       /* SMPL_INFO_STREAM("arm Reorder for state: "<<(*it)->state_id<<"h-val "<<(*it)->h[0]<<" & "<<(*it)->h[1]<<"  g-val "<<
            (*it)->g<<" and f-val "<<(*it)->f[1]);*/
    }
    m_open_arm.make(sbpl::motion::GroupType::ARM);

}


int MHTRAStar::computeKey(MHTRAState* s, sbpl::motion::GroupType group) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h[group]);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
MHTRAState* MHTRAStar::getSearchState(int state_id)
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

void MHTRAStar::initializeStartStates()
{
    std::vector<int> succs;
    std::vector<double> costs;
    bool result = m_space->updateMultipleStartStates(&succs, &costs, restore_step);
    int minCostIdx = std::distance(costs.begin(),std::max_element(costs.begin(),costs.end()));
    for(int i=0;i<succs.size();i++)
    {
        if (m_states.size() <= m_start_states_ids[i]) {
            m_states.resize(m_start_states_ids[i] + 1, nullptr);
        }

        auto& state = m_states[m_start_states_ids[i]];
        if (state == NULL) {
            state = createState(m_start_states_ids[i]);
        }

        state->g = costs[i];
        state->h[0] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        state->h[1] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        //state->h[1] = std::max(state->h[0],state->h[1]);
        state->f[0] = computeKey(state,sbpl::motion::GroupType::BASE);
        state->f[1] = computeKey(state,sbpl::motion::GroupType::ARM);
        SMPL_ERROR_STREAM("Start state with ID "<<state->state_id<<", has a heuristics "<<state->h[0]<<" & "<<state->h[1]<<" and g-val "<<state->g
            <<" f-val "<<state->f[0]<<" & "<<state->f[1]);
        
        state->C = 0;
        state->E = INFINITECOST;
        std::vector<int> parents(0);
        state->parent_hist = parents;
        state->gval_hist.push_back(0);
        state->source = sbpl::motion::GroupType::ANY;
        state->bestpredstate = nullptr;
        state->call_number = m_call_number;
        m_open_base_iso.push(state,sbpl::motion::GroupType::BASE_ISO);
        m_open_base.push(state,sbpl::motion::GroupType::BASE);
        m_open_arm.push(state,sbpl::motion::GroupType::ARM);
        

        if(restore_step == 1 && i==0)
            start_state = state;
    }
}

// Create a new search state for a graph state.
MHTRAState* MHTRAStar::createState(int state_id)
{
    assert(state_id < m_states.size());

    //m_graph_to_search_map[state_id] = (int)m_states.size();

    MHTRAState* ss = new MHTRAState;
    ss->state_id = state_id;
    ss->call_number = m_call_number;
    ss->E = INFINITECOST;
    ss->C = INFINITECOST;
    ss->v = 0;
    ss->g = INFINITECOST;
    ss->h[0] = m_heur->GetGoalHeuristic(ss->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
    ss->h[1] = m_heur->GetGoalHeuristic(ss->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
    //ss->h[1] = std::max(ss->h[0],ss->h[1]);
    ss->f[0] = ss->f[1] = INFINITECOST;
    ss->eg = INFINITECOST;
    ss->iteration_closed[0] = ss->iteration_closed[1] = 0;
    ss->call_number = m_call_number;
    ss->bestpredstate = nullptr;
    ss->bestnextstate = nullptr;
    ss->incons = false;
    ss->firstExpansionStep = -1;
    ss->to_erase_parents.clear();

    ss->source = sbpl::motion::GroupType::ANY;
    return ss;
}


// Lazily (re)initialize a search state.
void MHTRAStar::reinitSearchState(MHTRAState* state)
{
     if (state->call_number != m_call_number){
        state->g = INFINITECOST;
        state->h[0] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        state->h[1] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        //state->h[1] = std::max(state->h[0],state->h[1]);
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
        state->to_erase_parents.clear();
     }
}

// Extract the path from the start state up to a new state.
void MHTRAStar::extractPath(MHTRAState* to_state, std::vector<int>& solution, int& cost) const
{
    ROS_INFO_STREAM("Start & goal IDs "<<m_start_state_id<<","<<m_goal_state_id);
    for (MHTRAState* s = to_state; s; s = s->bestpredstate) {
        
        solution.push_back(s->state_id);
        if(s->bestpredstate!=nullptr && s->bestpredstate == s->bestpredstate->bestpredstate)
        {
            solution.clear();
            return;
        }
        if(s && s->bestpredstate!=nullptr)
            ROS_INFO_STREAM("state "<<s->state_id<<", pred "<<s->bestpredstate->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    ROS_WARN_STREAM("Extracted path with size "<<solution.size());
    cost = to_state->g;
}

bool MHTRAStar::RestoreSearchTree(int restoreStep)
{   
    if(restoreStep>-1)
    {
        m_iteration = 1;
        SMPL_INFO_STREAM("in restore seen states are "<<seen_states.size()<<" and m_states "<<m_states.size());
        
        m_open_base.clear(sbpl::motion::GroupType::BASE);
        m_open_base_iso.clear(sbpl::motion::GroupType::BASE_ISO);
        m_open_arm.clear(sbpl::motion::GroupType::ARM);
        m_states.clear();
        goal_state = getSearchState(m_goal_state_id);
        reinitSearchState(goal_state);
        initializeStartStates();
        //need to clear closed
        std::vector<MHTRAState*> current_seen;
        //SMPL_INFO_STREAM("start states size "<<m_start_states_ids.size());
        for(int i=0;i<seen_states.size();i++)
        {
            MHTRAState* current = seen_states[i];
            bool addedToSeen = false;
            
            SMPL_INFO_STREAM("State to be checked has ID "<<current->state_id);
            //Start states
            if(current->state_id>0 && current->state_id<=m_start_states_ids.size())
            {
                if(std::find(m_start_states_ids.begin(),m_start_states_ids.end(),current->state_id)==m_start_states_ids.end())
                {
                    SMPL_WARN_STREAM("The start state ["<<current->state_id<<"] chosen before is in collision now, skip restoring it!");
                    continue;
                }
                current->C = 0;
                std::vector<int> parents(0);
                current->parent_hist = parents;
                current->gval_hist.push_back(0);

                /*current->f[0] = computeKey(current,sbpl::motion::GroupType::BASE);
                current->f[1] = computeKey(current,sbpl::motion::GroupType::ARM);
                m_open_base.push(current,sbpl::motion::GroupType::BASE);
                m_open_base_iso.push(current,sbpl::motion::GroupType::BASE_ISO);
                m_open_arm.push(current,sbpl::motion::GroupType::ARM);
                current->E = 1;
                current_seen.push_back(current);
                 m_states.push_back(current);*/
            }
            
            
                SMPL_INFO_STREAM("state to be restored g-val "<<current->g<<" Es are "<<current->E<<" parent hist size "<<current->parent_hist.size()
                    <<" its source "<<current->source);
                
                SMPL_INFO_STREAM("Restoring State "<<current->state_id);
                int latestParenIdx;
                unsigned int parentGVal;
                if(current->E <= restoreStep)
                {
                    updateParents(current,restoreStep,latestParenIdx,parentGVal, current->source);
                    SMPL_INFO_STREAM("Fully Restored ! "<<parentGVal);
                    if(current->state_id > m_start_states_ids.size())
                    {
                        current->bestpredstate = seen_states[latestParenIdx];
                        current->g  = parentGVal;
                        current->h[0] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
                        current->h[1] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
                        //current->h[1] = std::max(current->h[0],current->h[1]);
                        current->f[0] = computeKey(current,sbpl::motion::GroupType::BASE);
                        current->f[1] = computeKey(current,sbpl::motion::GroupType::ARM);
                    }
                    else
                        current->bestpredstate = nullptr;
                        
                    current->eg  = current->g;
                    if(current->bestpredstate)
                        ROS_WARN_STREAM("after restoring pred is "<<current->bestpredstate->state_id);
                    else
                        ROS_WARN_STREAM("after restoring null pred");
                    //insert in closed
                    current->iteration_closed[sbpl::motion::GroupType::BASE] = m_iteration;
                    current->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;
                    current->call_number = m_call_number;
                    if(!addedToSeen)
                    {
                        SMPL_INFO_STREAM("Expanded State pushed with ID "<<current->state_id);
                        m_states.push_back(current);
                        current_seen.push_back(current);
                        addedToSeen=true;
                    }
                }
                //state created only
                else if(current->C <= restoreStep)
                {
                    SMPL_INFO_STREAM("State generated only! "<<current->source);
                    updateParents(current,restoreStep,latestParenIdx,parentGVal,current->source);
                    if(current->state_id > m_start_states_ids.size())
                    {
                        current->bestpredstate = seen_states[latestParenIdx];
                        current->g = parentGVal;
                        current->h[0] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
                        current->h[1] = m_heur->GetGoalHeuristic(current->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
                        //current->h[1] = std::max(current->h[0],current->h[1]);
                        current->f[0] = computeKey(current,sbpl::motion::GroupType::BASE);
                        current->f[1] = computeKey(current,sbpl::motion::GroupType::ARM);
                    }
                    else
                        current->bestpredstate = nullptr;

                    current->v = INFINITECOST;
                    current->E = INFINITECOST;
                    current->eg  = INFINITECOST;
                    current->call_number = m_call_number;
                    if(!addedToSeen)
                    {
                        SMPL_INFO_STREAM("Created State pushed with ID "<<current->state_id);
                        current_seen.push_back(current);
                        m_states.push_back(current);
                        addedToSeen = true;
                    }
                    if( current->source == sbpl::motion::GroupType::BASE_ISO)
                    {
                        SMPL_INFO("base iso source!");
                        if(!m_open_base_iso.contains(current,sbpl::motion::BASE_ISO))
                            m_open_base_iso.push(current,sbpl::motion::GroupType::BASE_ISO);
                        if(!m_open_base.contains(current,sbpl::motion::BASE))
                            m_open_base.push(current,sbpl::motion::GroupType::BASE);
                        if(!m_open_arm.contains(current,sbpl::motion::ARM))
                            m_open_arm.push(current,sbpl::motion::GroupType::ARM);
                    }
                    else
                    {
                        if(!m_open_arm.contains(current,sbpl::motion::ARM))
                            m_open_arm.push(current,sbpl::motion::GroupType::ARM);
                        if(!m_open_base.contains(current,sbpl::motion::BASE))
                            m_open_base.push(current,sbpl::motion::GroupType::BASE);
                    }
                    if(current->bestpredstate)
                        ROS_WARN_STREAM("after generating pred is "<<current->bestpredstate->state_id);
                    else
                        ROS_WARN_STREAM("after generating null pred");
                    //SMPL_INFO_STREAM("State generated only with C, E, g, h "<<current->C);
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
                    //current->h[1] = std::max(current->h[0],current->h[1]);
                    //current->f = computeKey(current);
                    current->call_number = m_call_number;
                    //current->incons = false;
                    current->firstExpansionStep = -1;
                }
                    
                ///RESTIRATION DONE ////////
                SMPL_INFO_STREAM("state after 3rd and last restoration g-val "<<current->g<<
                    " Es are "<<current->E);
                /*for(int i=0;i<current->parent_hist.size();i++)
                {
                    SMPL_INFO_STREAM("parent to be erased "<< current->to_erase_parents.size()<<","<<current->parent_hist.size());
                    if (current->to_erase_parents[i])
                    {
                        SMPL_INFO_STREAM("parent physcially erased");
                        current->parent_hist.erase(current->parent_hist.begin() + i);
                        current->gval_hist.erase(current->gval_hist.begin() + i); 
                    }
                }*/
            }

        seen_states = current_seen;
        expansion_step = restoreStep+1;
        ROS_INFO_STREAM("Restore to step "<<restoreStep<<" done with number of seen states "<<seen_states.size()
            <<" and m_states "<<m_states.size());
    }

}



bool MHTRAStar::RestorePerGroup(MHTRAState* current, int restoreStep, int group, std::vector<MHTRAState*>& current_seen, bool& addedToSeen)
{
    
    
}

bool MHTRAStar::updateParents(MHTRAState* state, unsigned int expansionStep, int& latestParenIdx, unsigned int& latestGVal, int group)
{
    unsigned int latestParentStep = 0;
    //latestParent = nullptr;
    latestGVal = 0;
    SMPL_INFO_STREAM("Updating parent for state "<<state->state_id<<" for group "<<group
        <<" parent hist size "<<state->parent_hist.size());
    for(int i=0;i<state->parent_hist.size();i++)
    {
        MHTRAState* parent;// = state->parent_hist[i];
        int index;
        int id = state->parent_hist[i];
        SMPL_INFO_STREAM("looking for parent id "<<id);
        auto it = find_if(seen_states.begin(), seen_states.end(), [&id](MHTRAState* obj) {return obj->state_id == id;});
        
        if (it != seen_states.end())
        {
          // found element. it is an iterator to the first matching element.
          // if you really need the index, you can also get it:
          index = std::distance(seen_states.begin(), it);
          parent = seen_states[index];
          latestGVal = state->gval_hist[i];
          SMPL_INFO_STREAM("found! "<<latestGVal);
        }
        else
            return false;
        
            
        //this parent is a valid one for the given expansion step
        if(parent->E <= expansionStep)
        {   
            //parent->bestpredstate = state;
            SMPL_INFO_STREAM("Group "<<group<<" updating parent with id "<<parent->state_id<<", E "<<parent->E);
            if(parent->E > latestParentStep)
            {
                latestParenIdx = index;
                latestParentStep = parent->E;
                
                SMPL_INFO_STREAM("New parent info "<<seen_states[latestParenIdx]->state_id<<",E "<<seen_states[latestParenIdx]->E<<", and g-val "<<latestGVal);
                
            }
        }
        //this parent is not valid for the given expansion step
        else
        {
            state->parent_hist.erase(state->parent_hist.begin() + i);
            state->gval_hist.erase(state->gval_hist.begin() + i);

            SMPL_INFO_STREAM("Group "<<group<<" Erasing Parent with id "<<parent->state_id<<", E "<<parent->E<<" and g-val "<<state->gval_hist[i]);
        }
    }   
}

bool MHTRAStar::storeParent(MHTRAState* succ_state, MHTRAState* state, unsigned int gVal, unsigned int expansionStep)
{
    succ_state->parent_hist.push_back(state->state_id);
    SMPL_INFO_STREAM("Storing parent with id "<<state->state_id<<" and E "<<state->E);/*[0]<<","
        <<state->E[1]<<","<<state->E[2]);*/
    succ_state->gval_hist.push_back(gVal);
    return 1;
}

void MHTRAStar::heuristicChanged()
{   
    ROS_ERROR_STREAM("in heuristic changed and cost "<<costChanged<< " state size "<<seen_states.size());
    bool done = false;
    bool restore = false;
    std::vector<unsigned int> baseIsoInconsE, baseInconsE, armInconsE;
    unsigned int  base_iso_step = INFINITECOST, base_step = INFINITECOST, arm_step = INFINITECOST;
    unsigned int current_h;
    //goal_state = getSearchState(m_goal_state_id);
    /*auto now = clock::now();
    RestoreSearchTree(restore_step);
    recomputeHeuristics();
    reorderOpen();
    clock::duration elapsed_time = now - clock::now();
    ROS_WARN_STREAM("Restore done with time "<<to_seconds(elapsed_time));*/
    goal_state = getSearchState(m_goal_state_id);
    reinitSearchState(goal_state);
    initializeStartStates();

    recomputeHeuristics();
    ROS_INFO_STREAM("recompute done");
    reorderOpen();
    ROS_WARN_STREAM("reorder done");
   
    //How the edges are identified (cell to edge mapping), I don't see it handled here
    auto now = clock::now();
    RestoreSearchTree(restore_step);
    /*while(!done)
    {
        MHTRAState* baseMinState = m_open_base.min();
        MHTRAState* armMinState = m_open_arm.min();
        MHTRAState* baseIsoMinState = m_open_base_iso.min();
        ROS_INFO_STREAM("Base ISO min f "<<baseIsoMinState->f[0]<<" and C"<<baseIsoMinState->C);
        ROS_INFO_STREAM("Base min f "<<baseMinState->f[0]<<" and C"<<baseMinState->C);
        ROS_INFO_STREAM("Arm min f "<<armMinState->f[0]<<" and C"<<armMinState->C);
               
        restore = false;
        //loop on closed
        for (MHTRAState* s : m_states) 
        {   
            if(s!=NULL && s->E!=INFINITECOST)
            {
                if(s->source == sbpl::motion::GroupType::BASE_ISO || s->source == sbpl::motion::GroupType::BASE)
                    current_h = s->h[0];
                else
                    current_h = s->h[1];

                unsigned int cost = s->eg + (unsigned int)(m_curr_eps * current_h);
                ROS_INFO_STREAM("Current seen state id "<<s->state_id<<", h "<<current_h<<" eg "<<s->eg
                    <<" cost "<<cost<<" E "<<s->E<<" C "<<s->C<<" source "<<s->source);
                if(s->source == sbpl::motion::GroupType::BASE_ISO && cost < baseIsoMinState->f[0] && baseIsoMinState->C < s->E)
                {
                    ROS_INFO_STREAM("The new base_iso E pushed "<<s->E);
                    baseIsoInconsE.push_back(s->E);
                }
                else if(s->source == sbpl::motion::GroupType::BASE && cost < baseMinState->f[0] && baseMinState->C < s->E)
                {
                    ROS_INFO_STREAM("The new base E pushed "<<s->E);
                    baseInconsE.push_back(s->E);
                }
                else if(s->source == sbpl::motion::GroupType::ARM && cost < armMinState->f[1] && armMinState->C < s->E)
                {
                    ROS_INFO_STREAM("The new arm E pushed "<<s->E);
                    armInconsE.push_back(s->E);
                }
            }
            else if(s==NULL)
                ROS_INFO_STREAM("NULL STATE FOUND !");
        }
        if(baseIsoInconsE.empty() && baseInconsE.empty() && armInconsE.empty())
            done = true;
        if(!baseIsoInconsE.empty())
            base_iso_step  = (*std::min_element(baseIsoInconsE.begin(),baseIsoInconsE.end())) - 1;
        if(!baseInconsE.empty())
            base_step  = (*std::min_element(baseInconsE.begin(),baseInconsE.end())) - 1;
        if(!armInconsE.empty())
            arm_step =  (*std::min_element(armInconsE.begin(),armInconsE.end())) - 1;
        unsigned int min_step = std::min(base_iso_step,std::min(base_step,arm_step));
        SMPL_ERROR_STREAM("Base_ISO step "<<base_iso_step<<", base step "<<base_step<<", arm step "<<arm_step<<", min "<<min_step);
        
        if(min_step<INFINITECOST)
        {
            restore = true;
            SMPL_INFO_STREAM("heuristic changed restore to "<<min_step<<" cost computed step is "<<restore_step);
            restore_step = min_step;
            RestoreSearchTree(min_step); 
        }
        else
        {
            restore = true;
            SMPL_INFO_STREAM("Nothing restored before, restore the cost changed step "<<restore_step);
            RestoreSearchTree(restore_step);
        }
        baseIsoInconsE.clear();
        baseInconsE.clear();
        armInconsE.clear();
        base_iso_step = INFINITECOST, base_step = INFINITECOST, arm_step = INFINITECOST;
    }

    if(!restore)
    {
            SMPL_ERROR_STREAM("Nothing restored before, restore the cost changed step "<<restore_step);
            RestoreSearchTree(restore_step);
    }*/
    clock::duration elapsed_time =  clock::now() - now;
    SMPL_INFO_STREAM("Restoration time "<<to_seconds(elapsed_time));
    sbpl::motion::RobotPlanningSpace* robot_space = dynamic_cast<sbpl::motion::RobotPlanningSpace*>(m_space);
    robot_space->getPlanningData()->restorationTime_ = to_seconds(elapsed_time);
    robot_space->getPlanningData()->restorationStep_ = restore_step;
}

void MHTRAStar::InitializeSearch()
{
    //clear closed
    /*MHTRAState* start = getSearchState(m_start_state_id);
    //m_open.push(start);
    start->g = 0;
    //computeKey(start);
    expansion_step = 1;
    start->C = 0;
    seen_states.push_back(start);
    start->E = INFINITECOST;*/
}


void MHTRAStar::Recomputegval(MHTRAState* state)
{

}


// used for backward search
void MHTRAStar::UpdatePreds(MHTRAState* state)
{
    /*std::vector<int> PredIDV;
    std::vector<int> CostV;
    CKey key;
    MHTRAState* predstate;

    m_space->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

    // iterate through predecessors of s
    for (int pind = 0; pind < (int)PredIDV.size(); pind++) {
        CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
        predstate = (MHTRAState*)(PredMDPState->PlannerSpecificData);
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
void MHTRAStar::UpdateSuccs(MHTRAState* state)
{
    /*std::vector<int> SuccIDV;
    std::vector<int> CostV;
    CKey key;
    MHTRAState* succstate;

    m_space->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

    // iterate through predecessors of s
    for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
        CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
        int cost = CostV[sind];

        succstate = (MHTRAState*)(SuccMDPState->PlannerSpecificData);
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

void MHTRAStar::BuildNewOPENList()
{
    /*MHTRAState* state;
    CKey key;
    CHeap* pheap = pSearchStateSpace->heap;
    CList* pinconslist = pSearchStateSpace->inconslist;

    // move incons into open
    while (pinconslist->firstelement != NULL) {
        state = (MHTRAState*)pinconslist->firstelement->liststate;

        // compute f-value
        key.key[0] = state->g + (int)(pSearchStateSpace->eps * state->h);
//        key.key[1] = state->h;

        // insert into OPEN
        pheap->insertheap(state, key);
        // remove from INCONS
        pinconslist->remove(state, ARA_AD_INCONS_LIST_ID);
    }*/
}


void MHTRAStar::PrintSearchState(MHTRAState* state, FILE* fOut)
{
    SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d heapind=%d inconslist=%d\n", state->MDPstate->StateID, state->h, state->g, state->v, state->iterationclosed, state->callnumberaccessed, state->heapindex, state->listelem[ARA_AD_INCONS_LIST_ID] ? 1 : 0);
    m_space->PrintState(state->state_id, true, fOut);
}

} // namespace sbpl

