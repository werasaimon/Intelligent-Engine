#include "IDynamicsWorld.h"

#include "contacts/IContactSolver.h"
#include "../common/math/IMatematical.h"
#include "../common/timer/ITimer.h"
#include "../body/IRigidBody.h"





namespace IPhysics
{




/// Helper function for daping vector*s
static void Damping( IVector3& Velocity , const scalar& min_damping , const scalar& damping )
{
    if( Velocity.lengthSquare()  < min_damping )
    {
        if( Velocity.length() > damping )
        {
            Velocity -= ( Velocity.getUnit() * damping);
        }
        else
        {
            Velocity  = IVector3(0,0,0);
        }
    }
}



IDynamicsWorld::IDynamicsWorld(const IVector3& gravity)
: mGravity(gravity),
  mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
  mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
  mTimer( scalar(1.0) ) ,
  mIsSleepingEnabled(SLEEPING_ENABLED) ,
  mNbIslands(0),
  mNbIslandsCapacity(0),
  mIslands(NULL),
  mNbBodiesCapacity(0),
  mContactSolver(mMapBodyToConstrainedVelocityIndex) ,
  mConstraintSolver(mMapBodyToConstrainedVelocityIndex) ,
  mSplitVelocities(NULL) ,
  mConstrainedVelocities(NULL) ,
  mConstrainedPositions(NULL)
{
    resetContactManifoldListsOfBodies();
    mTimer.start();
}


// Destroy all the  world
IDynamicsWorld::~IDynamicsWorld()
{
  destroy();
}




// Destroy all the  world
void IDynamicsWorld::destroy()
{

    // Stop timer
    mTimer.stop();


    // Destroy all the joints that have not been removed
    for (auto itJoints = mPhysicsJoints.begin(); itJoints != mPhysicsJoints.end();)
    {
        std::set<IJoint*>::iterator itToRemove = itJoints;
        ++itJoints;
        destroyJoint(*itToRemove);
    }



    // Destroy all the rigid bodies that have not been removed
    for (auto itRigidBodies = mPhysicsBodies.begin(); itRigidBodies != mPhysicsBodies.end();)
    {
        std::set<IRigidBody*>::iterator itToRemove = itRigidBodies;
        ++itRigidBodies;
        destroyBody(*itToRemove);
    }



    // Destroy all pair collisions that have not been removed
    if(!mCollisionDetection.mContactOverlappingPairs.empty())
    {
        for( auto pair : mCollisionDetection.mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mCollisionDetection.mContactOverlappingPairs.clear();
    }

    assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);
    assert(mCollisionDetection.mContactOverlappingPairs.empty());


    // Release the memory allocated for the bodies velocity arrays
    if (mNbBodiesCapacity > 0)
    {
        delete[] mSplitVelocities;
        delete[] mConstrainedVelocities;
        delete[] mConstrainedPositions;
    }

    //assert(mPhysicsJoints.size() == 0);
    assert(mPhysicsBodies.size() == 0);


}


void IDynamicsWorld::updateFixedTimeStep(scalar timeStep)
{
   mTimer.setTimeStep(timeStep);

  if(mTimer.getIsRunning())
  {
     mTimer.update();

     while( mTimer.isPossibleToTakeStep() )
     {
         update(timeStep);

         // next step simulation
         mTimer.nextStep();
     }
  }

}

void IDynamicsWorld::update(scalar timeStep)
{

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Overlapping pairs in contact (during the current Narrow-phase collision detection)
    mCollisionDetection.FindNewContacts();

    // Compute the islands (separate groups of bodies with constraints between each others)
    computeIslands();



    //------------------------------------------------------------------------//

    // Initialize the bodies velocity arrays
    initVelocityArrays();

    // Integrate the velocities
    integrateVelocities(timeStep);

    // Solve the contacts and constraints
    solveContactsAndConstraints(timeStep);

    // Integrate the motion of each body
    integratePositions(timeStep);

    // Solve the position correction for constraints
    solvePositionCorrection();

    // Update the state (positions and velocities) of the bodies
    updateBodiesState(timeStep);

    // Sleeping for all bodies
    if(mIsSleepingEnabled)
    {
       updateSleepingBodies(timeStep);
    }

    // Reset the external force and torque applied to the bodies
    resetBodiesForceAndTorque();



}



/**/
//// Put bodies to sleep if needed.
///// For each island, if all the bodies have been almost still for a long enough period of
///// time, we put all the bodies of the island to sleep.
void IDynamicsWorld::updateSleepingBodies(scalar timeStep)
{

    const scalar sleepLinearVelocitySquare  = (DEFAULT_SLEEP_LINEAR_VELOCITY * DEFAULT_SLEEP_LINEAR_VELOCITY);
    const scalar sleepAngularVelocitySquare = (DEFAULT_SLEEP_ANGULAR_VELOCITY * DEFAULT_SLEEP_ANGULAR_VELOCITY);
    //const scalar sleepAngularSplitSquare    = (DEFAULT_SLEEP_SPLIT  * DEFAULT_SLEEP_SPLIT);


    // For each island of the world
    for (u32 i=0; i<mNbIslands; i++)
    {

        scalar minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        IRigidBody** bodies = mIslands[i]->getBodies();
        for (u32 b=0; b < mIslands[i]->getNbBodies(); b++)
        {

            // Skip static bodies
            if (bodies[b]->getType() == STATIC) continue;

            // If the body is velocity is large enough to stay awake

            // If the body is velocity is large enough to stay awake
            if (bodies[b]->mLinearVelocity.lengthSquare()       >  sleepLinearVelocitySquare   ||
                bodies[b]->mAngularVelocity.lengthSquare()      >  sleepAngularVelocitySquare  ||
               !bodies[b]->isAllowedToSleep())
            {
                // Reset the sleep time of the body
                bodies[b]->mSleepTime = scalar(0.0);
                minSleepTime = scalar(0.0);
            }
            else
            {  // If the body velocity is bellow the sleeping velocity threshold

                // Increase the sleep time
                bodies[b]->mSleepTime += timeStep;
                if (bodies[b]->mSleepTime < minSleepTime)
                {
                    minSleepTime = bodies[b]->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if ( minSleepTime >= DEFAULT_TIME_BEFORE_SLEEP )
        {

            // Put all the bodies of the island to sleep
            for (u32 b=0; b < mIslands[i]->getNbBodies(); b++)
            {
                bodies[b]->setIsSleeping(true);
            }
        }
    }


}



// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void IDynamicsWorld::computeIslands()
{


    u32 nbBodies = mPhysicsBodies.size();
    u32 nbJoints = mPhysicsJoints.size();


    // Clear all the islands
    for (u32 i=0; i<mNbIslands; i++)
    {
        // Call the island destructor
        // Release the allocated memory for the island
        delete mIslands[i];
    }

    // Allocate and create the array of islands
    if (mNbIslandsCapacity != nbBodies && nbBodies > 0)
    {
        if (mNbIslandsCapacity > 0)
        {
            delete[] mIslands;
        }

        mNbIslandsCapacity = nbBodies;
        mIslands = new IIsland*[mNbIslandsCapacity];
    }

    mNbIslands = 0;
    int nbContactManifolds = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {
        int nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
        nbContactManifolds += nbBodyManifolds;
    }

    for (auto it = mPhysicsJoints.begin(); it !=  mPhysicsJoints.end(); ++it)
    {
        (*it)->mIsAlreadyInIsland = false;
    }


    IRigidBody** stackBodiesToVisit = new IRigidBody*[nbBodies];
    int i=0;
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it )
    {
        IRigidBody* body = *it;
        stackBodiesToVisit[i++] =    static_cast<IRigidBody*>(body);
    }


    // For each rigid body of the world
    for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
    {

        IRigidBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is static, we go to the next body
        if (body->getType() == STATIC) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->isSleeping() || !body->isActive()) continue;

        // Reset the stack of bodies to visit
        u32 stackIndex = 0;
        stackBodiesToVisit[stackIndex] = static_cast<IRigidBody*>(body);
        stackIndex++;
        body->mIsAlreadyInIsland = true;



        // Create the new island
        mIslands[mNbIslands] = new IIsland( nbBodies , nbContactManifolds , nbJoints );



        // While there are still some bodies to visit in the stack
        while (stackIndex > 0)
        {

            // Get the next body to visit from the stack
            stackIndex--;
            IRigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->isActive());

            // Awake the body if it is slepping
            bodyToVisit->setIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->addBody(bodyToVisit);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            if (bodyToVisit->getType() == STATIC) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != NULL;  contactElement = contactElement->getNext())
            {
                IContactManifold* contactManifold = contactElement->getPointer();

                assert(contactManifold->getNbContactPoints() > 0);

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;


                // Add the contact manifold into the island
                mIslands[mNbIslands]->addContactManifold(contactManifold);
                contactManifold->mIsAlreadyInIsland = true;


                // Get the other body of the contact manifold
                IRigidBody* body1 = static_cast<IRigidBody*>(contactManifold->getBody1());
                IRigidBody* body2 = static_cast<IRigidBody*>(contactManifold->getBody2());
                IRigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }



            /**/
            IListElement<IJoint>* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != NULL; jointElement = jointElement->getNext())
            {
                IJoint* joint = jointElement->getPointer();

                // Check if the current joint has already been added into an island
                if (joint->isAlreadyInIsland()) continue;

                // Add the joint into the island
                mIslands[mNbIslands]->addJoint(joint);
                joint->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                IRigidBody* body1 = static_cast<IRigidBody*>(joint->getBody1());
                IRigidBody* body2 = static_cast<IRigidBody*>(joint->getBody2());
                IRigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }
            /**/
        }



        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (u32 i=0; i < mIslands[mNbIslands]->mNbBodies; i++)
        {

            if (mIslands[mNbIslands]->mBodies[i]->getType() == STATIC)
            {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;

    }

    delete[] stackBodiesToVisit;

}



IRigidBody *IDynamicsWorld::createRigidBody(const ITransform &transform)
{

    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<bodyindex>::max());

    // Create the rigid body
    IRigidBody* rigidBody = new IRigidBody(transform, &mCollisionDetection, bodyID);
    assert(rigidBody != NULL);


    // Add the rigid body to the physics world
    mBodies.insert(rigidBody);
    mPhysicsBodies.insert(rigidBody);


    // Return the pointer to the rigid body
    return rigidBody;

}

void IDynamicsWorld::destroyBody(IRigidBody *rigidBody)
{

    // Remove all the collision shapes of the body
    rigidBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->getID());


    /**/
    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    if(rigidBody->mJointsList != NULL )
    {
      for (element = rigidBody->mJointsList; element != NULL; element = element->getNext() )
      {
           destroyJoint(element->getPointer());
           element = NULL;
      }
    }
    rigidBody->mJointsList = NULL;
    /**/


    // Reset the contact manifold list of the body
    rigidBody->resetContactManifoldsList();


    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mPhysicsBodies.erase(rigidBody);


    // Call the destructor of the rigid body
    // Free the object from the memory allocator
    delete rigidBody;

}


/**/
IJoint* IDynamicsWorld::createJoint(const IJointInfo& jointInfo)
{

       IJoint* newJoint = NULL;


       switch (jointInfo.type)
       {

          case BALLSOCKETJOINT:
          {
              const IBallAndSocketJointInfo& info = static_cast<const IBallAndSocketJointInfo&>(jointInfo);
              newJoint = new IBallAndSocketJoint(info);

              break;
          }


          case HINGEJOINT:
          {
              const IHingeJointInfo& info = static_cast<const IHingeJointInfo&>(jointInfo);
              newJoint = new IHingeJoint(info);

              break;
          }

          case SLIDERJOINT:
          {
              const ISliderJointInfo& info = static_cast<const ISliderJointInfo&>(jointInfo);
              newJoint = new ISliderJoint(info);

              break;
          }

          case FIXEDJOINT:
          {
              const IFixedJointInfo& info = static_cast<const IFixedJointInfo&>(jointInfo);
              newJoint = new IFixedJoint(info);

              break;
          }

                default:
            //cout << "is not init joint info " <<endl;
                        break;
        }



        // If the collision between the two bodies of the constraint is disabled
        if (!jointInfo.isCollisionEnabled)
        {
            // Add the pair of bodies in the set of body pairs that cannot collide with each other
            mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
        }

        // Add the joint into the world
        mPhysicsJoints.insert(newJoint);


        // Add the joint into the joint list of the bodies involved in the joint
        addJointToBody(newJoint);

        // Return the pointer to the created joint
        return newJoint;
}
/**/


/**/
void IDynamicsWorld::destroyJoint(IJoint* joint)
{

    assert(joint != NULL);
    // If the collision between the two bodies of the constraint was disabled
    if (!joint->isCollisionEnabled())
    {
        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
    }

    // Wake up the two bodies of the joint
    joint->getBody1()->setIsSleeping(false);
    joint->getBody2()->setIsSleeping(false);

    // Remove the joint from the world
    mPhysicsJoints.erase(joint);

    // Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->removeJointFromJointsList( joint );
    joint->mBody2->removeJointFromJointsList( joint );

    size_t nbBytes = joint->getSizeInBytes();

    // Call the destructor of the joint
    delete joint;

}
/**/


/**/
void IDynamicsWorld::addJointToBody(IJoint *joint)
{
    assert(joint != NULL);

    // Add the joint at the beginning of the linked list of joints of the first body
    JointListElement* jointListElement1 = new JointListElement(joint , joint->mBody1->mJointsList , NULL );
    joint->mBody1->mJointsList = jointListElement1;

    // Add the joint at the beginning of the linked list of joints of the second body
    JointListElement* jointListElement2 = new JointListElement(joint , joint->mBody2->mJointsList , NULL );
    joint->mBody2->mJointsList = jointListElement2;

}
/**/



u32 IDynamicsWorld::getNbIterationsVelocitySolver() const
{
    return mNbVelocitySolverIterations;
}

void IDynamicsWorld::setNbIterationsVelocitySolver(u32 nbIterations)
{
    mNbVelocitySolverIterations = nbIterations;
}

u32 IDynamicsWorld::getNbIterationsPositionSolver() const
{
    return mNbPositionSolverIterations;
}


void IDynamicsWorld::setNbIterationsPositionSolver(u32 nbIterations)
{
     mNbPositionSolverIterations = nbIterations;
}




//*************************************************************************************//


void IDynamicsWorld::initVelocityArrays()
{
    // Allocate memory for the bodies velocity arrays
    u32 nbBodies = mPhysicsBodies.size();
    if (mNbBodiesCapacity != nbBodies && nbBodies > 0)
    {
        if (mNbBodiesCapacity > 0)
        {
            delete[] mSplitVelocities;
            delete[] mConstrainedVelocities;
            delete[] mConstrainedPositions;
        }

        mNbBodiesCapacity = nbBodies;


        // TODO : Use better memory allocation here
        mSplitVelocities       = new IVelocity[mNbBodiesCapacity];
        mConstrainedVelocities = new IVelocity[mNbBodiesCapacity];
        mConstrainedPositions  = new ILocationsAndRotation[mNbBodiesCapacity];
        assert(mSplitVelocities != NULL);
        assert(mConstrainedVelocities != NULL);
        assert(mConstrainedPositions  != NULL);

    }



  // Reset the velocities arrays
  for (u32 i=0; i<mNbBodiesCapacity; i++)
  {
      mSplitVelocities[i].v.setToZero();
      mSplitVelocities[i].w.setToZero();
  }

  // Initialize the map of body indexes in the velocity arrays
  mMapBodyToConstrainedVelocityIndex.clear();
  u32 indexBody = 0;
  for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
  {
      // Add the body into the map
      mMapBodyToConstrainedVelocityIndex.insert(std::make_pair(*it, indexBody));
      indexBody++;
  }
}


// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void IDynamicsWorld::integrateVelocities(scalar TimeStep)
{

  // For each island of the world
  for (u32 i=0; i < mNbIslands; i++)
  {

     IRigidBody** bodies = mIslands[i]->getBodies();

      // For each body of the island
      for (u32 b=0; b < mIslands[i]->getNbBodies(); b++)
      {

          // Insert the body into the map of constrained velocities
          u32 indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

          assert(mSplitVelocities[indexBody].v == IVector3(0, 0, 0));
          assert(mSplitVelocities[indexBody].w == IVector3(0, 0, 0));


          // If the gravity has to be applied to this rigid body
          //if (bodies[b]->isGravityEnabled() && mIsGravityEnabled)
          if(bodies[b]->mType == BodyType::DYNAMIC)
          {
              // Integrate the external force to get the new velocity of the body
              mConstrainedVelocities[indexBody].v = bodies[b]->getLinearVelocity()  + TimeStep * bodies[b]->mExternalForce;
              mConstrainedVelocities[indexBody].w = bodies[b]->getAngularVelocity() + TimeStep * bodies[b]->mExternalTorque;

              // Integrate the gravity force
              mConstrainedVelocities[indexBody].v += TimeStep * mGravity;
          }

          // If it is a static body
          if (bodies[b]->mType == STATIC || bodies[b]->mType == KINEMATIC)
          {
              // Reset the velocity to zero
              mConstrainedVelocities[indexBody].v=IVector3(0,0,0);
              mConstrainedVelocities[indexBody].w=IVector3(0,0,0);

          }

          indexBody++;
      }
  }

}


// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void IDynamicsWorld::integratePositions(scalar TimeStep)
{

  // For each island of the world
     for (u32 i=0; i < mNbIslands; i++)
     {

        IRigidBody** bodies = mIslands[i]->getBodies();

         // For each body of the island
         for (u32 b=0; b < mIslands[i]->getNbBodies(); b++)
         {

             // Get the constrained velocity
             u32 indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;


             /*********************************************
              *          Damping  velocity
              ********************************************/
             scalar linDampingFactor = bodies[b]->getLinearDamping();
             scalar angDampingFactor = bodies[b]->getAngularDamping();
             Damping( mConstrainedVelocities[indexArray].v , MINIMUM_FOR_DAPING , linDampingFactor  );
             Damping( mConstrainedVelocities[indexArray].w , MINIMUM_FOR_DAPING , angDampingFactor  );


             // Get current position and orientation of the body
             const IVector3&    currentPosition    = bodies[b]->mCenterOfMassWorld;
             const IQuaternion& currentOrientation = bodies[b]->getTransform().getRotation();


             IVector3 newLinVelocity = mConstrainedVelocities[indexArray].v;
             IVector3 newAngVelocity = mConstrainedVelocities[indexArray].w;

             // Add the split impulse velocity from Contact Solver (only used
             // to update the position)
             if (mContactSolver.isSplitImpulseActive())
             {

                 newLinVelocity += mSplitVelocities[indexArray].v;
                 newAngVelocity += mSplitVelocities[indexArray].w;

             }


             /**
             // Update the new constrained position and orientation of the body
             mConstrainedPositions[indexArray].x = currentPosition + newLinVelocity * TimeStep;
             mConstrainedPositions[indexArray].q = currentOrientation + IQuaternion(0, newAngVelocity) * currentOrientation * scalar(0.5) * TimeStep;
             /**/

             mConstrainedPositions[indexArray].x = ITransform::integrateLinear(  currentPosition    , newLinVelocity , TimeStep );
             mConstrainedPositions[indexArray].q = ITransform::integrateAngular( currentOrientation , newAngVelocity , TimeStep );

             /**/

        }
     }

}



// Update the postion/orientation of the bodies
void IDynamicsWorld::updateBodiesState( scalar TimeStep )
{
  // For each island of the world
  for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
  {
      // For each body of the island
      IRigidBody** bodies = mIslands[islandIndex]->getBodies();

      for (u32 b=0; b < mIslands[islandIndex]->getNbBodies(); b++)
      {

          u32 index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

          // Update the linear and angular velocity of the body
          bodies[b]->mLinearVelocity  = mConstrainedVelocities[index].v;
          bodies[b]->mAngularVelocity = mConstrainedVelocities[index].w;


          // Update the position of the center of mass of the body
          bodies[b]->mCenterOfMassWorld = mConstrainedPositions[index].x;

          // Update the orientation of the body
          bodies[b]->mTransform.setBasis(mConstrainedPositions[index].q.getMatrix());

          // Update the transform of the body (using the new center of mass and new orientation)
          bodies[b]->updateTransformWithCenterOfMass();


//          // Update the matrices of the body
//          bodies[b]->UpdateMatrices();

          // Update the broad-phase state of the body
          //bodies[b]->updateBroadPhaseState();
          bodies[b]->updateBroadPhaseStatee(TimeStep);

      }
   }
}



/**/
void IDynamicsWorld::solveContactsAndConstraints( scalar TimeStep )
{


     if(mPhysicsBodies.empty() && mPhysicsJoints.empty()) return;


     // ---------- Solve velocity constraints for joints and contacts ---------- //

     // Set the velocities arrays
     mContactSolver.setSplitVelocitiesArrays(mSplitVelocities);
     mContactSolver.setConstrainedVelocitiesArrays(mConstrainedVelocities);


     mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedVelocities);
     mConstraintSolver.setConstrainedPositionsArrays(mConstrainedPositions);



     // For each island of the world
     for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
     {

         // Check if there are contacts and constraints to solve
         bool isConstraintsToSolve = mIslands[islandIndex]->getNbJoints() > 0;
         bool isContactsToSolve = mIslands[islandIndex]->getNbContactManifolds() > 0;
         if (!isContactsToSolve && !isConstraintsToSolve) continue;

         // If there are contacts in the current island
         if (isContactsToSolve)
         {
             // Initialize the solver
             mContactSolver.initializeForIsland( TimeStep , mIslands[islandIndex] );
             // Warm start the contact solver
             mContactSolver.warmStart();
         }

         // If there are constraints
         if (isConstraintsToSolve)
         {
             // Initialize the constraint solver
             mConstraintSolver.initializeForIsland( TimeStep , mIslands[islandIndex]);
         }



         // For each iteration of the velocity solver
         for (u32 i=0; i<mNbVelocitySolverIterations; i++)
         {
             // Solve the contacts
             if (isContactsToSolve) mContactSolver.solveVelocityConstraint();

             // Solve the constraints
             if (isConstraintsToSolve)  mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);

         }


         // For each iteration of the split velocity solver
         for (u32 i=0; i<mNbPositionSolverIterations; i++)
         {
             // Solve the contacts
             if (isContactsToSolve) mContactSolver.solveSplitVelocityConstraint();
         }


//         // For each iteration of the position (error correction) solver
//         for (u32 i=0; i<mNbPositionSolverIterations; i++)
//         {
//             // Solve the position constraints
//             mContactSolver.slovePositionConstraint();
//         }

         // Cache the lambda values in order to use them in the next
         // step and cleanup the contact solver
         if (isContactsToSolve)
         {
             mContactSolver.storeImpulsesWarmstart();
             mContactSolver.cleanup();
         }
     }




}


// Solve the position error correction of the constraints
void IDynamicsWorld::solvePositionCorrection()
{

    // Do not continue if there is no constraints
    if (mPhysicsJoints.empty()) return;

    // For each island of the world
    for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
    {

        // ---------- Solve the position error correction for the constraints ---------- //


        if(!mIslands[islandIndex]->getNbJoints()) continue;

        // For each iteration of the position (error correction) solver
        for (u32 i=0; i<mNbPositionSolverIterations; i++)
        {
            // Solve the position constraints
            mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
        }
    }
}

void IDynamicsWorld::resetBodiesForceAndTorque()
{

  // For each body of the world
  for (auto it = mPhysicsBodies.begin(); it != mPhysicsBodies.end(); ++it)
  {
      (*it)->mExternalForce.setToZero();
      (*it)->mExternalTorque.setToZero();
  }
}
/**/



}
