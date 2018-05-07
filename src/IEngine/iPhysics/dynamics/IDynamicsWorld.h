#ifndef IDYNAMICSWORLD_H
#define IDYNAMICSWORLD_H



#include "contacts/IContactSolver.h"
#include "../common/math/IMatematical.h"
#include "../common/timer/ITimer.h"
#include "../body/IRigidBody.h"

#include "ICollisionWorld.h"
#include "ITimeStep.h"
#include "IIsland.h"

#include "joint/IJoint.h"
#include "joint/IBallAndSocketJoint.h"
#include "joint/IHingeJoint.h"
#include "joint/ISliderJoint.h"
#include "joint/IFixedJoint.h"
#include "joint/IConstraintSolver.h"


namespace IPhysics
{



class IDynamicsWorld : public ICollisionWorld
{

 private:

     //---------------------- Attributes -----------------------//

    /// Map body to their index in the constrained velocities array
    std::map<IRigidBody*, u32> mMapBodyToConstrainedVelocityIndex;

    /// Split velocities for the position contact solver (split impulse)
    IVelocity* mSplitVelocities;

    /// Array of constrained velocities (state of the velocities
    /// after solving the constraints)
    IVelocity* mConstrainedVelocities;

    /// Array of constrained motion (for position error correction)
    ILocationsAndRotation* mConstrainedPositions;


    /// Contact solver
    IContactSolver    mContactSolver;

    /// Constraint solver
    IConstraintSolver mConstraintSolver;

    //-----------------------------------------------------------//

    /// Gravitation status
    bool mIsGravityEnabled;

    /// Update time step correctly interval
    ITimer mTimer;

    /// True if the spleeping technique for inactive bodies is enabled
    bool mIsSleepingEnabled;

    /// Number of iterations for the velocity solver of the Sequential Impulses technique
    u32 mNbVelocitySolverIterations;

    /// Number of iterations for the position solver of the Sequential Impulses technique
    u32 mNbPositionSolverIterations;



    /// Gravity vector3
    IVector3 mGravity;



    /// All the joints of the world
    std::set<IJoint*>     mPhysicsJoints;

    /// All the rigid bodies of the physics world
    std::set<IRigidBody*> mPhysicsBodies;



    /// Current allocated capacity for the bodies
    u32 mNbBodiesCapacity;

    /// Current allocated capacity for the islands
    u32 mNbIslandsCapacity;




    /// Number of islands in the world
    u32 mNbIslands;

    /// Array with all the islands of awaken bodies
    IIsland** mIslands;



    // -------------------- Methods -------------------- //


    /// Private copy-constructor
    IDynamicsWorld(const IDynamicsWorld& world);

    /// Private assignment operator
    IDynamicsWorld& operator=(const IDynamicsWorld& world);




    /// Put bodies to sleep if needed.
    void updateSleepingBodies(scalar timeStep);


    //// Compute the islands of awake bodies.
    void computeIslands();


    /// Integrate the Velocity .
    void integrateVelocities(scalar TimeStep);


    /// Integrate the motion .
    void integratePositions(scalar TimeStep);


    //// Compute Solver
    void solveContactsAndConstraints(scalar TimeStep);


    /// Solve the position error correction of the constraints
    void solvePositionCorrection();



    void resetBodiesForceAndTorque();


public:

    IDynamicsWorld( const IVector3& gravity );

    virtual ~IDynamicsWorld();

    //***************************************************//

    void initVelocityArrays();


    void updateBodiesState(scalar TimeStep);

    //***************************************************//

    ///  Destroy
    void destroy();

    //***************************************************//

     /// Create a rigid body into the physics world.
    IRigidBody* createRigidBody(const ITransform& transform);


    /// Destroy a rigid body and all the joints which it belongs
    void destroyBody(IRigidBody* rigidBody);


    /// Create a joint between two bodies in the world and return a pointer to the new joint
    IJoint* createJoint(const IJointInfo& jointInfo);


    /// Destroy a joint
    void destroyJoint(IJoint* joint);


    /// Add the joint to the list of joints of the two bodies involved in the joint
    void addJointToBody(IJoint* joint);


    //***************************************************//


    ///  Update Physics simulation - Real-Time ( Semi-AntiFixed timestep )
    void update(scalar timeStep);

    ///  Update Physics simulation - Real-Time ( Fixed timestep )
    void updateFixedTimeStep(scalar timeStep);

    /// Get the number of iterations for the velocity constraint solver
    u32 getNbIterationsVelocitySolver() const;

    /// Set the number of iterations for the velocity constraint solver
    void setNbIterationsVelocitySolver(u32 nbIterations);

    /// Get the number of iterations for the position constraint solver
    u32 getNbIterationsPositionSolver() const;

    /// Set the number of iterations for the position constraint solver
    void setNbIterationsPositionSolver(u32 nbIterations);

};


}

#endif // IDYNAMICSWORLD_H
