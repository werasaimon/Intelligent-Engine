#ifndef IFIXEDJOINT_H
#define IFIXEDJOINT_H

#include "IJoint.h"

namespace IPhysics
{


// Structure FixedJointInfo
/**
 * This structure is used to gather the information needed to create a fixed
 * joint. This structure will be used to create the actual fixed joint.
 */
struct IFixedJointInfo : public IJointInfo
{

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        IVector3 anchorPointWorldSpace;

        /// Constructor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point of the joint in
         *                                  world-space coordinates
         */
        IFixedJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2, const IVector3& initAnchorPointWorldSpace)
                       :IJointInfo(rigidBody1, rigidBody2, FIXEDJOINT),
                        anchorPointWorldSpace(initAnchorPointWorldSpace)
        {

        }
};


// Class FixedJoint
/**
 * This class represents a fixed joint that is used to forbid any translation or rotation
 * between two bodies.
 */
class IFixedJoint : public IJoint
{

    private :

      bool isSplitActive = true;
      bool isWarmStartingActive = true;


        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const scalar BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        IVector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        IVector3 mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        IVector3 mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        IVector3 mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
       IMatrix3x3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
       IMatrix3x3 mI2;

        /// Accumulated impulse for the 3 translation constraints
        IVector3 mImpulseTranslation;

        /// Accumulate impulse for the 3 rotation constraints
        IVector3 mImpulseRotation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 translation constraints (3x3 matrix)
       IMatrix3x3 mInverseMassMatrixTranslation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 rotation constraints (3x3 matrix)
       IMatrix3x3 mInverseMassMatrixRotation;

        /// Bias vector for the 3 translation constraints
        IVector3 mBiasTranslation;

        /// Bias vector for the 3 rotation constraints
        IVector3 mBiasRotation;

        /// Inverse of the initial orientation difference between the two bodies
        IQuaternion mInitOrientationDifferenceInv;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IFixedJoint(const  IFixedJoint& constraint);

        /// Private assignment operator
        IFixedJoint& operator=(const  IFixedJoint& constraint);

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve( const IConstraintSolverData& constraintSolverData  );

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart( const IConstraintSolverData& constraintSolverData );

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint( const IConstraintSolverData& constraintSolverData );

        /// Solve the position constraint (for position error correction)
        virtual void solvePositionConstraint( const IConstraintSolverData& constraintSolverData );

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        IFixedJoint(const IFixedJointInfo& jointInfo);

        /// Destructor
        virtual ~IFixedJoint();
};

// Return the number of bytes used by the joint
SIMD_INLINE size_t  IFixedJoint::getSizeInBytes() const
{
    return sizeof(IFixedJoint);
}

}

#endif // IFIXEDJOINT_H
