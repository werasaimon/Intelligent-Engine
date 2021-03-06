#ifndef ICONTACTSOLVER_H
#define ICONTACTSOLVER_H

// Libraries
#include "../../collision/shapes/ICollisionShape.h"
#include "../../body/IRigidBody.h"
#include "../ITimeStep.h"

#include "IContactManifold.h"
#include "IContactManifoldSet.h"
#include "IContactPoint.h"

#include <map>

namespace IPhysics
{

class IIsland;
class IContactSolver
{
    // Structure ContactPointSolver
    /**
    * Contact solver internal data structure that to store all the
    * information relative to a contact point
    */
    struct IContactPointSolver
    {

        /// Accumulated normal impulse
        scalar AccumulatedPenetrationImpulse;

        /// Accumulated impulse in the 1st friction direction
        scalar AccumulatedFriction1Impulse;

        /// Accumulated impulse in the 2nd friction direction
        scalar AccumulatedFriction2Impulse;

        /// Accumulated split impulse for penetration correction
        scalar AccumulatedPenetrationSplitImpulse;

        /// Accumulated rolling resistance impulse
        IVector3 AccumulatedRollingResistanceImpulse;

        /// Accumulated rolling resistance impulse
        IVector3 AccumulatedRollingResistanceSplitImpulse;

        /// Normal vector of the contact
        IVector3 normal;

        /// First friction vector in the tangent plane
        IVector3 frictionVector1;

        /// Second friction vector in the tangent plane
        IVector3 frictionVector2;

        /// Old first friction vector in the tangent plane
        IVector3 oldFrictionVector1;

        /// Old second friction vector in the tangent plane
        IVector3 oldFrictionVector2;

        /// Vector from the body 1 center to the contact point
        IVector3 r1;

        /// Vector from the body 2 center to the contact point
        IVector3 r2;

        /// Cross product of r1 with 1st friction vector
        IVector3 r1CrossT1;

        /// Cross product of r1 with 2nd friction vector
        IVector3 r1CrossT2;

        /// Cross product of r2 with 1st friction vector
        IVector3 r2CrossT1;

        /// Cross product of r2 with 2nd friction vector
        IVector3 r2CrossT2;

        /// Cross product of r1 with the contact normal
        IVector3 r1CrossN;

        /// Cross product of r2 with the contact normal
        IVector3 r2CrossN;

        /// Penetration depth
        scalar penetrationDepth;

        /// Velocity restitution bias
        scalar restitutionBias;

        /// Inverse of the matrix K for the penenetration
        scalar inversePenetrationMass;

        /// Inverse of the matrix K for the 1st friction
        scalar inverseFriction1Mass;

        /// Inverse of the matrix K for the 2nd friction
        scalar inverseFriction2Mass;

        /// True if the contact was existing last time step
        bool isRestingContact;

        /// Pointer to the external contact
        IContactPoint* externalContact;

    };



    struct IContactManifoldSolver
    {

        /// Index of body 1 in the constraint solver
        u32 indexBody1;

        /// Index of body 2 in the constraint solver
        u32 indexBody2;

        /// Inverse of the mass of body 1
        scalar massInverseBody1;

        /// Inverse of the mass of body 2
        scalar massInverseBody2;

        /// Inverse inertia tensor of body 1
        IMatrix3x3 inverseInertiaTensorBody1;

        /// Inverse inertia tensor of body 2
        IMatrix3x3 inverseInertiaTensorBody2;

        /// Contact point constraints
        IContactPointSolver contacts[MAX_CONTACT_POINTS_IN_MANIFOLD];

        /************************************************/

        /// Number of contact points
        u32 nbContacts;

        /// True if the body 1 is of type dynamic
        bool isBody1DynamicType;

        /// True if the body 2 is of type dynamic
        bool isBody2DynamicType;

        /// Mix of the restitution factor for two bodies
        scalar restitutionFactor;

        /// Mix friction coefficient for the two bodies
        scalar frictionCoefficient;

        /// Rolling resistance factor between the two bodies
        scalar rollingResistanceFactor;

        /// Pointer to the external contact manifold
        IContactManifold* externalContactManifold;

        // - Variables used when friction constraints are apply at the center of the manifold-//

        /// Average normal vector of the contact manifold
        IVector3 normal;

        /// Point on body 1 where to apply the friction constraints
        IVector3 frictionPointBody1;

        /// Point on body 2 where to apply the friction constraints
        IVector3 frictionPointBody2;

        /// R1 vector for the friction constraints
        IVector3 r1Friction;

        /// R2 vector for the friction constraints
        IVector3 r2Friction;

        /// Cross product of r1 with 1st friction vector
        IVector3 r1CrossT1;

        /// Cross product of r1 with 2nd friction vector
        IVector3 r1CrossT2;

        /// Cross product of r2 with 1st friction vector
        IVector3 r2CrossT1;

        /// Cross product of r2 with 2nd friction vector
        IVector3 r2CrossT2;

        /// Matrix K for the first friction constraint
        scalar inverseFriction1Mass;

        /// Matrix K for the second friction constraint
        scalar inverseFriction2Mass;

        /// Matrix K for the twist friction constraint
        scalar inverseTwistFrictionMass;

        /// Matrix K for the rolling resistance constraint
        IMatrix3x3 inverseRollingResistance;

        /// First friction direction at contact manifold center
        IVector3 frictionVector1;

        /// Second friction direction at contact manifold center
        IVector3 frictionVector2;

        /// Old 1st friction direction at contact manifold center
        IVector3 oldFrictionVector1;

        /// Old 2nd friction direction at contact manifold center
        IVector3 oldFrictionVector2;

        /// First friction direction impulse at manifold center
        scalar  AccumulatedFriction1Impulse;

        /// Second friction direction impulse at manifold center
        scalar  AccumulatedFriction2Impulse;

        /// Twist friction impulse at contact manifold center
        scalar  AccumulatedFrictionTwistImpulse;

        /// Rolling resistance impulse
        IVector3 AccumulatedRollingResistanceImpulse;

        /// Rolling resistance split impulse
        IVector3 AccumulatedRollingResistanceSplitImpulse;
    };



private:


    // -------------------- Constants --------------------- //
    /// Beta value for the penetration depth position correction without split impulses
    static const scalar BETA;

    /// Beta value for the penetration depth position correction with split impulses
    static const scalar BETA_SPLIT_IMPULSE;

    /// Slop distance (allowed penetration distance between bodies)
    static const scalar SLOP;


    //********************************************************************//


    bool mIsError = false;
    bool atLeastOneRestingContactPoint = false;

    bool mIsWarmStartingActive = true;
    bool mIsSplitImpulseActive = true;
    bool mIsStaticFriction = true;
    bool mIsSolveFrictionAtContactManifoldCenterActive = true;


    //********************************************************************//

    /// Number of contact constraints
    u32 mNbContactManifolds;

    /// Contact constraints
    IContactManifoldSolver* mContactConstraints;

    //********************************************************************//

    scalar mTimeStep;

    const std::map<IRigidBody*, u32>& mMapBodyToConstrainedVelocityIndex;

    IVelocity* mVelocities;
    IVelocity* mSplitVelocities;


    /*********************************************************/


    /// Compute the collision restitution factor from the restitution factor of each body
    scalar computeMixedRestitutionFactor(IRigidBody*  body1,
                                         IRigidBody*  body2) const;

    /// Compute the mixed friction coefficient from the friction coefficient of each body
    scalar computeMixedFrictionCoefficient(IRigidBody*  body1,
                                           IRigidBody*  body2) const;

    /// Compute th mixed rolling resistance factor between two bodies
    scalar computeMixedRollingResistance(IRigidBody*  body1,
                                         IRigidBody*  body2) const;


    /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
    /// plane for a contact manifold. The two vectors have to be
    /// such that : t1 x t2 = contactNormal.
    void computeFrictionVectors( const IVector3& deltaVelocity , IContactPointSolver& contactPoint) const;


    /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
    /// plane for a contact manifold. The two vectors have to be
    /// such that : t1 x t2 = contactNormal.
    void computeFrictionVectors( const IVector3& deltaVelocity , IContactManifoldSolver* contact ) const;


    /*********************************************************/

    void ApplyImpulseBody1( IContactManifoldSolver* contactManifold , const IVector3& _impulse , const IVector3& R_pivot )
    {
        mVelocities[contactManifold->indexBody1].v += _impulse * contactManifold->massInverseBody1;
        mVelocities[contactManifold->indexBody1].w += (R_pivot).cross(_impulse) * contactManifold->inverseInertiaTensorBody1;
    }

    void ApplyImpulseBody2( IContactManifoldSolver* contactManifold , const IVector3& _impulse , const IVector3& R_pivot )
    {
        mVelocities[contactManifold->indexBody2].v += _impulse * contactManifold->massInverseBody2;
        mVelocities[contactManifold->indexBody2].w += (R_pivot).cross(_impulse) * contactManifold->inverseInertiaTensorBody2;
    }


    void ApplyAngularImpulseBody1( IContactManifoldSolver* contactManifold , const IVector3& _AngularImpulse )
    {
        mVelocities[contactManifold->indexBody1].w += _AngularImpulse * contactManifold->inverseInertiaTensorBody1;
    }

    void ApplyAngularImpulseBody2( IContactManifoldSolver* contactManifold , const IVector3& _AngularImpulse )
    {
        mVelocities[contactManifold->indexBody2].w += _AngularImpulse * contactManifold->inverseInertiaTensorBody2;
    }

public:


    // Constructor
    IContactSolver(const std::map<IRigidBody*, u32>& mapBodyToVelocityIndex)
        :mSplitVelocities(NULL),
          mVelocities(NULL),
          mContactConstraints(NULL),
          mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
          mIsWarmStartingActive(true),
          mIsSplitImpulseActive(true),
          mIsSolveFrictionAtContactManifoldCenterActive(true)
    {

    }

    /// Initilization iSland
    void  initializeForIsland(scalar dt, IIsland* island);
    void  initializeContactConstraints();


    /// Warm start the solver.
    void warmStart();

    /// Solver velocity
    void solveVelocityConstraint();

    /// Solver split velocity
    void solveSplitVelocityConstraint();

    /// Store the computed impulses to use them to
    /// warm start the solver at the next iteration
    void storeImpulsesWarmstart();

    ///Clenup
    void cleanup();



    //-----------------------------------------//


    /// Set the split velocities arrays
    void setSplitVelocitiesArrays(IVelocity* splitVelocities)
    {
        assert(splitVelocities != NULL);;
        mSplitVelocities = splitVelocities;

    }

    /// Set the constrained velocities arrays
    void setConstrainedVelocitiesArrays(IVelocity* constrainedVelocities )
    {
        assert(constrainedVelocities != NULL);
        mVelocities = constrainedVelocities;

    }


    /******************************************************/

    /// Return true if the split impulses position correction technique is used for contacts
    bool isSplitImpulseActive() const;

    /// Activate or Deactivate the split impulses for contacts
    void setIsSplitImpulseActive(bool isActive);

    /// Activate or deactivate the solving of friction constraints at the center of
    /// the contact manifold instead of solving them at each contact point
    void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

    /// Return true if the warmstart impulses
    bool isIsWarmStartingActive() const;

    /// Activate or Deactivate the warmstart impulses
    void setIsWarmStartingActive(bool isWarmStartingActive);

    //-------------------- Friendship --------------------//

    friend class IDynamicsWorld;

};


/********************************************************************************************************/


// Return true if the split impulses position correction technique is used for contacts
SIMD_INLINE  bool IContactSolver::isSplitImpulseActive() const
{
    return mIsSplitImpulseActive;
}

// Activate or Deactivate the split impulses for contacts
SIMD_INLINE  void IContactSolver::setIsSplitImpulseActive(bool isActive)
{
    mIsSplitImpulseActive = isActive;
}


// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
SIMD_INLINE  void IContactSolver::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive)
{
    mIsSolveFrictionAtContactManifoldCenterActive = isActive;
}

// /// Return true if the warmstart impulses
SIMD_INLINE  bool IContactSolver::isIsWarmStartingActive() const
{
    return mIsWarmStartingActive;
}

// Activate or Deactivate the warmstart impulses
SIMD_INLINE  void IContactSolver::setIsWarmStartingActive(bool isWarmStartingActive)
{
    mIsWarmStartingActive = isWarmStartingActive;
}



}


#endif // ICONTACTSOLVER_H
