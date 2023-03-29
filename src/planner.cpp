#include "planner.h"
#include "utils.h"

#include <time.h>

#include <stdio.h>

ManipulatorPlanner::ManipulatorPlanner(size_t dof, mjModel* model, mjData* data)
{
    _dof = dof;
    _model = model;
    _data = data;
    initPrimitiveSteps();
}


// assign/override contact margin
mjtNum mj_assignMargin(const mjModel* m, mjtNum source) {
  if (mjENABLED(mjENBL_OVERRIDE)) {
    return m->opt.o_margin;
  } else {
    return source;
  }
}

// plane to geom_center squared distance, g1 is a plane
static mjtNum plane_geom(const mjModel* m, mjData* d, int g1, int g2) {
  mjtNum* mat1 = d->geom_xmat + 9*g1;
  mjtNum norm[3] = {mat1[2], mat1[5], mat1[8]};
  mjtNum dif[3];

  mju_sub3(dif, d->geom_xpos + 3*g2, d->geom_xpos + 3*g1);
  return mju_dot3(dif, norm);
}

// squared Euclidean distance between 3D vectors
static inline mjtNum squaredDist3(const mjtNum pos1[3], const mjtNum pos2[3]) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2];
}

// bounding-sphere collision
static int mj_collideSphere(const mjModel* m, mjData* d, int g1, int g2, mjtNum margin) {
  // neither geom is a plane
  if (m->geom_rbound[g1]>0 && m->geom_rbound[g2]>0) {
    mjtNum bound = m->geom_rbound[g1] + m->geom_rbound[g2] + margin;
    if (squaredDist3(d->geom_xpos+3*g1, d->geom_xpos+3*g2) > bound*bound) {
      return 0;
    }
  }

  // one geom is a plane
  if (m->geom_type[g1]==mjGEOM_PLANE && m->geom_rbound[g2]>0
      && plane_geom(m, d, g1, g2) > margin+m->geom_rbound[g2]) {
      return 0;
  }
  if (m->geom_type[g2]==mjGEOM_PLANE && m->geom_rbound[g1]>0
      && plane_geom(m, d, g2, g1) > margin+m->geom_rbound[g1]) {
      return 0;
  }
  return 1;
}

bool mj_light_collideGeoms(const mjModel* m, mjData* d, int g1, int g2) {
    int num, type1, type2, condim;
    mjtNum margin, gap, mix, friction[5], solref[mjNREF], solimp[mjNIMP];
    mjContact con[mjMAXCONPAIR];
    int ipair = (g2<0 ? g1 : -1);

    // get explicit geom ids from pair
    if (ipair>=0) {
        g1 = m->pair_geom1[ipair];
        g2 = m->pair_geom2[ipair];
    }

    // order geoms by type
    if (m->geom_type[g1] > m->geom_type[g2]) {
        std::swap(g1, g2);
    }

    // copy types and bodies
    type1 = m->geom_type[g1];
    type2 = m->geom_type[g2];

    // return if no collision function
    if (!mjCOLLISIONFUNC[type1][type2]) {
        return false;
    }
    margin = m->pair_margin[ipair];
    gap = m->pair_gap[ipair];
    condim = m->pair_dim[ipair];

    // bounding sphere filter
    if (!mj_collideSphere(m, d, g1, g2, margin)) {
        return false;
    }

    // call collision detector to generate contacts
    num = mjCOLLISIONFUNC[type1][type2](m, d, con, g1, g2, margin);
    return num;
}

// use only for predefined format
// code is copied from mujoco src
bool mj_light_collision(mjModel* m, mjData* d)
{
    mj_kinematics(m, d);

    for (int pairadr = 0; pairadr < m->npair; pairadr++)
    {
        if (mj_light_collideGeoms(m, d, pairadr, -1))
        {
            return true;
        }
    }
    return false;
}

bool ManipulatorPlanner::checkCollision(const JointState& position) const
{
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = position.rad(i);
    }
    return mj_light_collision(_model, _data);
}

bool ManipulatorPlanner::checkCollisionAction(const JointState& start, const JointState& delta) const
{
    startProfiling();
    if (_model == nullptr || _data == nullptr) // if we have not data for check
    {
        return false;
    }

    for (size_t i = 0; i < _dof; ++i)
    {
        _data->qpos[i] = start.rad(i);
    }

    int jump = 8;
    for (size_t t = 0; t < g_unitSize; t += jump)
    {
        for (size_t i = 0; i < _dof; ++i)
        {
            _data->qpos[i] += g_worldEps * delta[i] * jump; // temporary we use global constant here for speed
        }
        if (mj_light_collision(_model, _data))
        {
            stopProfiling();
            return true;
        }
    }
    stopProfiling();
    return false;
}

vector<string> ManipulatorPlanner::configurationSpace() const
{
    vector<string> cSpace(g_units * 2, string(g_units * 2, '.'));
    for (int i = -g_units; i < g_units; ++i)
    {
        for (int j = -g_units; j < g_units; ++j)
        {
            if (checkCollision({i, j}))
                cSpace[g_units - 1 - j][i + g_units] = '@';
        }
    }
    return cSpace;
}

Solution ManipulatorPlanner::planSteps(const JointState& startPos, const JointState& goalPos, int alg)
{
    clearAllProfiling(); // reset profiling

    if (checkCollision(goalPos))
    {
        return Solution(_primitiveSteps, _zeroStep); // incorrect aim
    }
    switch (alg)
    {
    case ALG_LINEAR:
        return linearPlanning(startPos, goalPos);
    case ALG_ASTAR:
        return astarPlanning(startPos, goalPos, manhattanDistance);
    default:
        return Solution(_primitiveSteps, _zeroStep);
    }
}

void ManipulatorPlanner::initPrimitiveSteps()
{
    _zeroStep = JointState(_dof, 0);

    _primitiveSteps.assign(2 * _dof, JointState(_dof, 0));

    for (int i = 0; i < _dof; ++i)
    {
        _primitiveSteps[i][i] = 1;
        _primitiveSteps[i + _dof][i] = -1;
    }
}

Solution ManipulatorPlanner::linearPlanning(const JointState& startPos, const JointState& goalPos)
{
    Solution solution(_primitiveSteps, _zeroStep);

    JointState currentPos = startPos;
    for (size_t i = 0; i < _dof; ++i)
    {
        while (currentPos[i] != goalPos[i])
        {
            size_t t = -1;
            if (currentPos[i] < goalPos[i]) // + eps
            {
                t = i;
            }
            else if (currentPos[i] > goalPos[i]) // - eps
            {
                t = i + _dof;
            }

            if (checkCollisionAction(currentPos, _primitiveSteps[t]))
            {
                return solution; // we temporary need to give up : TODO
            }
            currentPos += _primitiveSteps[t];
            solution.addStep(t);
        }
    }

    solution.stats.pathFound = true;
    return solution;
}

int ManipulatorPlanner::costMove(const JointState& state1, const JointState& state2)
{
    return manhattanDistance(state1, state2);
}
vector<astar::SearchNode*> ManipulatorPlanner::generateSuccessors(
    astar::SearchNode* node,
    const JointState& goal,
    int (*heuristicFunc)(const JointState& state1, const JointState& state2)
)
{
    startProfiling();
    vector<astar::SearchNode*> result;
    for (size_t i = 0; i < _primitiveSteps.size(); ++i)
    {
        JointState newState = node->state() + _primitiveSteps[i];
        if (checkCollisionAction(node->state(), _primitiveSteps[i]))
        {
            continue;
        }
        if (!newState.isCorrect())
        {
            continue;
        }
        result.push_back(
            new astar::SearchNode(
                node->g() + costMove(node->state(), newState),
                heuristicFunc(newState, goal),
                newState,
                i,
                node
            )
        );
    }

    stopProfiling();
    return result;
}

Solution ManipulatorPlanner::astarPlanning(
    const JointState& startPos, const JointState& goalPos,
    int (*heuristicFunc)(const JointState& state1, const JointState& state2)
)
{
    Solution solution(_primitiveSteps, _zeroStep);

    // start timer
    clock_t start = clock();

    // init search tree
    astar::SearchTree tree;
    astar::SearchNode* startNode = new astar::SearchNode(0, heuristicFunc(startPos, goalPos), startPos);
    tree.addToOpen(startNode);
    astar::SearchNode* currentNode = tree.extractBestNode();

    while (currentNode != nullptr)
    {
        if (currentNode->state() == goalPos)
        {
            break;
        }
        // count statistic
        solution.stats.maxTreeSize = std::max(solution.stats.maxTreeSize, tree.size());
        ++solution.stats.expansions;
        // expand current node
        vector<astar::SearchNode*> successors = generateSuccessors(currentNode, goalPos, heuristicFunc);
        for (auto successor : successors)
        {
            tree.addToOpen(successor);
        }
        // retake node from tree
        tree.addToClosed(currentNode);
        currentNode = tree.extractBestNode();
    }

    // end timer
    clock_t end = clock();
    solution.stats.runtime = (double)(end - start) / CLOCKS_PER_SEC;

    if (currentNode != nullptr)
    {
        solution.stats.pathFound = true;
        solution.plannerProfile = getNamedProfileInfo();
        solution.searchTreeProfile = tree.getNamedProfileInfo();
        
        vector<size_t> steps;
        while (currentNode->parent() != nullptr)
        {
            // count stats
            solution.stats.pathCost += costMove(currentNode->state(), currentNode->parent()->state());
            //
            steps.push_back(currentNode->stepNum());
            currentNode = currentNode->parent();
        }

        // push steps
        for (int i = steps.size() - 1; i >= 0; --i)
        {
            solution.addStep(steps[i]);
        }
    }

    return solution;
}
