#include "IConstraintSolver.h"
#include "../IIsland.h"
#include "IJoint.h"

namespace IPhysics
{


// Constructor
IConstraintSolver::IConstraintSolver(const std::map<IRigidBody*, u32>& mapBodyToVelocityIndex)
  : mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
    mIsWarmStartingActive(true),
    mConstraintSolverData(mapBodyToVelocityIndex)
{

}

// Destructor
IConstraintSolver::~IConstraintSolver()
{

}

// Initialize the constraint solver for a given island
void IConstraintSolver::initializeForIsland(scalar dt, IIsland* island)
{

    assert(island != NULL);
    assert(island->getNbBodies() > 0);
    assert(island->getNbJoints() > 0);

    // Set the current time step
    mTimeStep = dt;

    // Initialize the constraint solver data used to initialize and solve the constraints
    mConstraintSolverData.timeStep = mTimeStep;
    mConstraintSolverData.isWarmStartingActive = mIsWarmStartingActive;



    // For each joint of the island
    IJoint** joints = island->getJoints();
    for (u32 i=0; i<island->getNbJoints(); i++)
    {

        // Initialize the constraint before solving it
        joints[i]->initBeforeSolve(mConstraintSolverData);

        // Warm-start the constraint if warm-starting is enabled
        if (mIsWarmStartingActive)
        {
            joints[i]->warmstart(mConstraintSolverData);
        }
    }
}

// Solve the velocity constraints
void IConstraintSolver::solveVelocityConstraints(IIsland* island)
{
    assert(island != NULL);
    assert(island->getNbJoints() > 0);

    // For each joint of the island
    IJoint** joints = island->getJoints();
    for (u32 i=0; i<island->getNbJoints(); i++)
    {
        // Solve the constraint
        joints[i]->solveVelocityConstraint(mConstraintSolverData);
    }
}





// Solve the position constraints
void IConstraintSolver::solvePositionConstraints(IIsland* island)
{
    assert(island != NULL);
    assert(island->getNbJoints() > 0);

    // For each joint of the island
    IJoint** joints = island->getJoints();
    for (u32 i=0; i < island->getNbJoints(); i++)
    {
        // Solve the constraint
        joints[i]->solvePositionConstraint(mConstraintSolverData);
    }
}

}
