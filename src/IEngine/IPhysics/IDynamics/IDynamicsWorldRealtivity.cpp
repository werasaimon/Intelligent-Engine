#include "IDynamicsWorldRealtivity.h"


namespace IPhysics
{


    namespace
    {
        /// Helper function for daping vector*s
        static void Damping( IVector3& Velocity , const scalar& min_damping , const scalar& damping )
        {
            if( Velocity.LengthSquare()  < min_damping )
            {
                if( Velocity.Length() > damping )
                {
                    Velocity -= ( Velocity.GetUnit() * damping);
                }
                else
                {
                    Velocity  = IVector3(0,0,0);
                }
            }
        }
    }



	IDynamicsWorldRealtivity::IDynamicsWorldRealtivity(const IVector3 &gravity, IIndependentObserver *_ObserverSysytem, const scalar _LightSpeed)
    : IDynamicsWorld (gravity) ,
	mLightSpeed(_LightSpeed) ,
	mObserverSysytem(_ObserverSysytem)
    {

    }

    IDynamicsWorldRealtivity::~IDynamicsWorldRealtivity()
    {
        Destroy();
    }

    IRigidRealtivity *IDynamicsWorldRealtivity::CreateRigidBody(const ITransform &transform)
    {
        // Compute the body ID
        bodyindex bodyID = computeNextAvailableBodyID();

        // Largest index cannot be used (it is used for invalid index)
        assert(bodyID < std::numeric_limits<bodyindex>::max());

        // Create the rigid body
        IRigidRealtivity* rigidBody = new IRigidRealtivity(transform, &mCollisionDetection, bodyID , mObserverSysytem , mLightSpeed );
        assert(rigidBody != NULL);


        // Add the rigid body to the physics world
        mBodies.insert(rigidBody);
        mPhysicsBodies.insert(rigidBody);

        std::cout << "new body relatevity" << std::endl;

        // Return the pointer to the rigid body
        return rigidBody;
    }

    void IDynamicsWorldRealtivity::IntegrateVelocities(scalar TimeStep)
    {
		mObserverSysytem->Derivative();

        // For each island of the world
        for (u32 i=0; i < mNbIslands; i++)
        {

            IRigidBody** bodies = mIslands[i]->GetBodies();

            // For each body of the island
            for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
            {

                // Insert the body into the map of constrained velocities
                u32 indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

                assert(mSplitVelocities[indexBody].v == IVector3(0, 0, 0));
                assert(mSplitVelocities[indexBody].w == IVector3(0, 0, 0));


                // If the gravity has to be applied to this rigid body
                //if (bodies[b]->isGravityEnabled() && mIsGravityEnabled)
                if(bodies[b]->mType == BodyType::DYNAMIC)
                {

					// Integrate the gravity force and acceleration
					IVector3 LinearAcceleration  =  (bodies[b]->mExternalForce + mGravity) * TimeStep;
					IVector3 AngularAcceleration =   bodies[b]->mExternalTorque * TimeStep;


					scalar LightSpeed = mLightSpeed + 0.5;//mObserverSysytem->vel.Length();//
					scalar gamma =  (1.0 + IAbs((LinearAcceleration).Dot(bodies[b]->GetLinearVelocity()))/(LightSpeed*LightSpeed));


					// Integrate the external force to get the new velocity of the body
					mConstrainedVelocities[indexBody].v = (bodies[b]->GetLinearVelocity()  + LinearAcceleration) / gamma;
					mConstrainedVelocities[indexBody].w = (bodies[b]->GetAngularVelocity() + AngularAcceleration) / gamma;



					IMatrix3x3 Lambda = IMatrix3x3::CreateLorentzRotationBoost(IVector3::Y * 0.9);

					// Update the orientation of the body
					bodies[b]->mTransform.SetBasis( bodies[b]->mTransform.GetBasis() * Lambda.GetInverse());

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


    void IDynamicsWorldRealtivity::IntegratePositions(scalar TimeStep)
    {

        // For each island of the world
           for (u32 i=0; i < mNbIslands; i++)
           {

              IRigidBody** bodies = mIslands[i]->GetBodies();

               // For each body of the island
               for (u32 b=0; b < mIslands[i]->GetNbBodies(); b++)
               {

                   // Get the constrained velocity
                   u32 indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;




                   /*********************************************
                    *          Damping  velocity
                    ********************************************/
                   scalar linDampingFactor = bodies[b]->GetLinearDamping();
                   scalar angDampingFactor = bodies[b]->GetAngularDamping();
                   Damping( mConstrainedVelocities[indexArray].v , MINIMUM_FOR_DAPING , linDampingFactor  );
                   Damping( mConstrainedVelocities[indexArray].w , MINIMUM_FOR_DAPING , angDampingFactor  );


                   // Get current position and orientation of the body
                   const IVector3&    currentPosition    = bodies[b]->mCenterOfMassWorld;
                   const IQuaternion& currentOrientation = bodies[b]->GetTransform().GetRotation();


				   IVector3 newLinVelocity = mConstrainedVelocities[indexArray].v;
				   IVector3 newAngVelocity = mConstrainedVelocities[indexArray].w;

				   IVector3 newSplitLinVelocity = mSplitVelocities[indexArray].v;
				   IVector3 newSplitAngVelocity = mSplitVelocities[indexArray].w;


				   /**/
				  //if( bodies[b]->mInitMass  >= 1.0 )
				  // {
				   scalar LightSpeed = mLightSpeed + 0.5;//mObserverSysytem->vel.Length();//

				   IVector3 relativityVelocity = bodies[b]->GetLinearVelocity() * TimeStep;

				   scalar gamma = ISqrt( 1.0 - ( Dot( relativityVelocity , relativityVelocity )) / (LightSpeed*LightSpeed) );

					   newLinVelocity = newLinVelocity * gamma;
					   newAngVelocity = newAngVelocity * gamma;
				  // }
				   /**/



				   std::cout << "Integrate Lorentz  " << newLinVelocity.Length() << std::endl;



                   /// Translation Object
                   ITransform resulTransform( currentPosition , bodies[b]->GetTransform().GetBasis());
                   resulTransform = IIntegrateUtil::IntegrateTransform(resulTransform,newLinVelocity,newAngVelocity,TimeStep );
                   resulTransform = IIntegrateUtil::IntegrateTransform(resulTransform,newSplitLinVelocity,newSplitAngVelocity,TimeStep );


                   mConstrainedPositions[indexArray].x = resulTransform.GetPosition();
                   mConstrainedPositions[indexArray].q = resulTransform.GetRotation();



              }
           }

    }

    void IDynamicsWorldRealtivity::UpdateBodiesState(scalar TimeStep)
    {

        // For each island of the world
        for (u32 islandIndex = 0; islandIndex < mNbIslands; islandIndex++)
        {
            // For each body of the island
            IRigidBody** bodies = mIslands[islandIndex]->GetBodies();

            for (u32 b=0; b < mIslands[islandIndex]->GetNbBodies(); b++)
            {

                u32 index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

                // Update the linear and angular velocity of the body
                bodies[b]->mLinearVelocity  = mConstrainedVelocities[index].v;
                bodies[b]->mAngularVelocity = mConstrainedVelocities[index].w;


                // Update the position of the center of mass of the body
                bodies[b]->mCenterOfMassWorld = mConstrainedPositions[index].x;


                IMatrix3x3 Lambda = IMatrix3x3::CreateLorentzRotationBoost(IVector3::Y * 0.9);

                // Update the orientation of the body
                bodies[b]->mTransform.SetBasis(mConstrainedPositions[index].q.GetRotMatrix() * Lambda);

                // Update the transform of the body (using the new center of mass and new orientation)
                bodies[b]->UpdateTransformWithCenterOfMass();


                // Update the broad-phase state of the body
                //bodies[b]->updateBroadPhaseState();
                bodies[b]->UpdateBroadPhaseStatee(TimeStep);

            }
         }

    }


}

