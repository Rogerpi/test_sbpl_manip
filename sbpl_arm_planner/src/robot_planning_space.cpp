#include <sbpl_arm_planner/robot_planning_space.h>

namespace sbpl {
namespace manip {

RobotPlanningSpace::RobotPlanningSpace(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams* params)
:
    m_robot(robot),
    m_checker(checker),
    m_params(params),
    m_actions(nullptr),
    m_heuristics(),
    m_obs()
{
}

RobotPlanningSpace::~RobotPlanningSpace()
{
}

bool RobotPlanningSpace::setActionSpace(ActionSpace* as)
{
    m_actions = as;
    return true;
}

bool RobotPlanningSpace::insertHeuristic(RobotHeuristic* h)
{
    auto hit = std::find(m_heuristics.begin(), m_heuristics.end(), h);
    if (hit != m_heuristics.end()) {
        return false;
    }
    m_heuristics.push_back(h);
    return true;
}

bool RobotPlanningSpace::eraseHeuristic(RobotHeuristic* h)
{
    auto hit = std::remove(m_heuristics.begin(), m_heuristics.end(), h);
    if (hit == m_heuristics.end()) {
        return false;
    }
    m_heuristics.erase(hit, m_heuristics.end());
    return true;
}

bool RobotPlanningSpace::hasHeuristic(RobotHeuristic* h)
{
    auto hit = std::find(m_heuristics.begin(), m_heuristics.end(), h);
    return hit != m_heuristics.end();
}

bool RobotPlanningSpace::setStart(const RobotState& start)
{
    m_start = start;
    notifyStartChanged(start);
    return true;
}

bool RobotPlanningSpace::setGoal(const GoalConstraint& goal)
{
    m_goal = goal;
    notifyGoalChanged(goal);
    return true;
}

/// Add an observer to the list of observers if it is not already observing
void RobotPlanningSpace::insertObserver(RobotPlanningSpaceObserver* obs)
{
    if (std::find(m_obs.begin(), m_obs.begin(), obs) == m_obs.end()) {
        m_obs.push_back(obs);
    }
}

/// Remove an observer from the list of observers
void RobotPlanningSpace::eraseObserver(RobotPlanningSpaceObserver* obs)
{
    auto it = std::remove(m_obs.begin(), m_obs.end(), obs);
    m_obs.erase(it, m_obs.end());
}

/// Return whether an observer is in the list of observers
bool RobotPlanningSpace::hasObserver(RobotPlanningSpaceObserver* obs) const
{
    return std::find(m_obs.begin(), m_obs.end(), obs) != m_obs.end();
}

void RobotPlanningSpace::notifyStartChanged(const RobotState& state)
{
    for (RobotPlanningSpaceObserver* obs : m_obs) {
        obs->updateStart(state);
    }
}

void RobotPlanningSpace::notifyGoalChanged(const GoalConstraint& goal)
{
    for (RobotPlanningSpaceObserver* obs : m_obs) {
        obs->updateGoal(goal);
    }
}

void RobotPlanningSpace::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    GetSuccs(state_id, succs, costs);
    true_costs->assign(succs->size(), true);
}

int RobotPlanningSpace::GetTrueCost(int parent_id, int child_id)
{
    std::vector<int> succs;
    std::vector<int> costs;
    GetSuccs(parent_id, &succs, &costs);
    auto sit = std::find(succs.begin(), succs.end(), child_id);
    if (sit == succs.end()) {
        return -1;
    }

    return costs[std::distance(succs.begin(), sit)];
}

} // namespace manip
} // namespace sbpl