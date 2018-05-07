#ifndef IHINGEJOINT_H
#define IHINGEJOINT_H

#include "IJoint.h"

namespace IPhysics
{

// Structure HingeJointInfo
/**
 * This structure is used to gather the information needed to create a hinge joint.
 * This structure will be used to create the actual hinge joint.
 */
struct IHingeJointInfo : public IJointInfo
{

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        IVector3 anchorPointWorldSpace;

        /// Hinge rotation axis (in world-space coordinates)
        IVector3 rotationAxisWorld;

        /// True if the hinge joint limits are enabled
        bool isLimitEnabled;

        /// True if the hinge joint motor is enabled
        bool isMotorEnabled;

        /// Minimum allowed rotation angle (in radian) if limits are enabled.
        /// The angle must be in the range [-2*pi, 0]
        scalar minAngleLimit;

        /// Maximum allowed rotation angle (in radian) if limits are enabled.
        /// The angle must be in the range [0, 2*pi]
        scalar maxAngleLimit;

        /// Motor speed (in radian/second)
        scalar motorSpeed;

        /// Maximum motor torque (in Newtons * meters) that can be applied to reach
        /// to desired motor speed
        scalar maxMotorTorque;

        /// Constructor without limits and without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         *                                  coordinates
         * @param initRotationAxisWorld The initial rotation axis in world-space
         *                              coordinates
         */
        IHingeJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                        const IVector3& initAnchorPointWorldSpace,
                        const IVector3& initRotationAxisWorld)
      :IJointInfo(rigidBody1, rigidBody2, HINGEJOINT),
        anchorPointWorldSpace(initAnchorPointWorldSpace),
        rotationAxisWorld(initRotationAxisWorld),
        isLimitEnabled(false),
        isMotorEnabled(false),
        minAngleLimit(-1),
        maxAngleLimit(1),
        motorSpeed(0),
        maxMotorTorque(0)
        {

        }

        /// Constructor with limits but without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space coordinates
         * @param initRotationAxisWorld The intial rotation axis in world-space coordinates
         * @param initMinAngleLimit The initial minimum limit angle (in radian)
         * @param initMaxAngleLimit The initial maximum limit angle (in radian)
         */
        IHingeJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                        const IVector3& initAnchorPointWorldSpace,
                        const IVector3& initRotationAxisWorld,
                        scalar initMinAngleLimit, scalar initMaxAngleLimit)
        :IJointInfo(rigidBody1, rigidBody2, HINGEJOINT),
          anchorPointWorldSpace(initAnchorPointWorldSpace),
          rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
          isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
          maxAngleLimit(initMaxAngleLimit), motorSpeed(0),
          maxMotorTorque(0)
        {

        }

        /// Constructor with limits and motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initRotationAxisWorld The initial rotation axis in world-space
         * @param initMinAngleLimit The initial minimum limit angle (in radian)
         * @param initMaxAngleLimit The initial maximum limit angle (in radian)
         * @param initMotorSpeed The initial motor speed of the joint (in radian per second)
         * @param initMaxMotorTorque The initial maximum motor torque (in Newtons)
         */
        IHingeJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2,
                        const IVector3& initAnchorPointWorldSpace,
                        const IVector3& initRotationAxisWorld,
                        scalar initMinAngleLimit, scalar initMaxAngleLimit,
                        scalar initMotorSpeed, scalar initMaxMotorTorque)
        :IJointInfo(rigidBody1, rigidBody2, HINGEJOINT),
          anchorPointWorldSpace(initAnchorPointWorldSpace),
          rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
          isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
          maxAngleLimit(initMaxAngleLimit), motorSpeed(initMotorSpeed),
          maxMotorTorque(initMaxMotorTorque)
        {
        }
};




// Class HingeJoint
/**
 * This class represents a hinge joint that allows arbitrary rotation
 * between two bodies around a single axis. This joint has one degree of freedom. It
 * can be useful to simulate doors or pendulumns.
 */
class IHingeJoint : public IJoint
{


    private :


    bool isSplitActive = true;
    bool isWarmStartingActive = true;


        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        scalar BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        IVector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        IVector3 mLocalAnchorPointBody2;

        /// Hinge rotation axis (in local-space coordinates of body 1)
        IVector3 mHingeLocalAxisBody1;

        /// Hinge rotation axis (in local-space coordiantes of body 2)
        IVector3 mHingeLocalAxisBody2;

        /// Inertia tensor of body 1 (in world-space coordinates)
        IMatrix3x3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        IMatrix3x3 mI2;

        /// Hinge rotation axis (in world-space coordinates) computed from body 1
        IVector3 mA1;

        /// Vector from center of body 2 to anchor point in world-space
        IVector3 mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        IVector3 mR2World;

        /// Cross product of vector b2 and a1
        IVector3 mB2CrossA1;

        /// Cross product of vector c2 and a1;
        IVector3 mC2CrossA1;

        /// Impulse for the 3 translation constraints
        IVector3 mImpulseTranslation;

        /// Impulse for the 2 rotation constraints
        IVector2 mImpulseRotation;

        /// Accumulated impulse for the lower limit constraint
        scalar mImpulseLowerLimit;

        /// Accumulated impulse for the upper limit constraint
        scalar mImpulseUpperLimit;

        /// Accumulated impulse for the motor constraint;
        scalar mImpulseMotor;

        /// Inverse mass matrix K=JM^-1J^t for the 3 translation constraints
        IMatrix3x3 mInverseMassMatrixTranslation;

        /// Inverse mass matrix K=JM^-1J^t for the 2 rotation constraints
        IMatrix2x2 mInverseMassMatrixRotation;

        /// Inverse of mass matrix K=JM^-1J^t for the limits and motor constraints (1x1 matrix)
        scalar mInverseMassMatrixLimitMotor;

        /// Inverse of mass matrix K=JM^-1J^t for the motor
        scalar mInverseMassMatrixMotor;

        /// Bias vector for the error correction for the translation constraints
        IVector3 mBTranslation;

        /// Bias vector for the error correction for the rotation constraints
        IVector2 mBRotation;

        /// Bias of the lower limit constraint
        scalar mBLowerLimit;

        /// Bias of the upper limit constraint
        scalar mBUpperLimit;

        /// Inverse of the initial orientation difference between the bodies
        IQuaternion mInitOrientationDifferenceInv;

        /// True if the joint limits are enabled
        bool mIsLimitEnabled;

        /// True if the motor of the joint in enabled
        bool mIsMotorEnabled;

        /// Lower limit (minimum allowed rotation angle in radian)
        scalar mLowerLimit;

        /// Upper limit (maximum translation distance)
        scalar mUpperLimit;

        /// True if the lower limit is violated
        bool mIsLowerLimitViolated;

        /// True if the upper limit is violated
        bool mIsUpperLimitViolated;

        /// Motor speed (in rad/s)
        scalar mMotorSpeed;

        /// Maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
        scalar mMaxMotorTorque;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IHingeJoint(const IHingeJoint& constraint);

        /// Private assignment operator
        IHingeJoint& operator=(const IHingeJoint& constraint);

        /// Reset the limits
        void resetLimits();

        /// Given an angle in radian, this method returns the corresponding
        /// angle in the range [-pi; pi]
        scalar computeNormalizedAngle(scalar angle) const;

        /// Given an "inputAngle" in the range [-pi, pi], this method returns an
        /// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
        /// two angle limits in arguments.
        scalar computeCorrespondingAngleNearLimits(scalar inputAngle, scalar lowerLimitAngle,
                                                    scalar upperLimitAngle) const;

        /// Compute the current angle around the hinge axis
        scalar computeCurrentHingeAngle(const IQuaternion& orientationBody1,
                                        const IQuaternion& orientationBody2);

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve( const IConstraintSolverData& constraintSolverData );

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart( const IConstraintSolverData& constraintSolverData );

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint( const IConstraintSolverData& constraintSolverData );


        /// Solve the position constraint (for position error correction)
        virtual void solvePositionConstraint( const IConstraintSolverData& constraintSolverData );

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        IHingeJoint(const IHingeJointInfo& jointInfo);

        /// Destructor
        virtual ~IHingeJoint();

        /// Return true if the limits or the joint are enabled
        bool isLimitEnabled() const;

        /// Return true if the motor of the joint is enabled
        bool isMotorEnabled() const;

        /// Enable/Disable the limits of the joint
        void enableLimit(bool isLimitEnabled);

        /// Enable/Disable the motor of the joint
        void enableMotor(bool isMotorEnabled);

        /// Return the minimum angle limit
        scalar getMinAngleLimit() const;

        /// Set the minimum angle limit
        void setMinAngleLimit(scalar lowerLimit);

        /// Return the maximum angle limit
        scalar getMaxAngleLimit() const;

        /// Set the maximum angle limit
        void setMaxAngleLimit(scalar upperLimit);

        /// Return the motor speed
        scalar getMotorSpeed() const;

        /// Set the motor speed
        void setMotorSpeed(scalar motorSpeed);

        /// Return the maximum motor torque
        scalar getMaxMotorTorque() const;

        /// Set the maximum motor torque
        void setMaxMotorTorque(scalar maxMotorTorque);

        /// Return the intensity of the current torque applied for the joint motor
        scalar getMotorTorque(scalar timeStep) const;



        /**

        /// Velocity angular of body1
        void setAbsolutliVelocityMotor1( IVector3 angular_speed )
        {
            static_cast<IRigidBody*>(mBody1)->setAngularVelocity( angular_speed );
        }

        /// Velocity angular of body2
        void setAbsolutliVelocityMotor2( IVector3 angular_speed )
        {
            static_cast<IRigidBody*>(mBody2)->setAngularVelocity( angular_speed );
        }

        /**/
};

// Return true if the limits of the joint are enabled
/**
 * @return True if the limits of the joint are enabled and false otherwise
 */
SIMD_INLINE bool IHingeJoint::isLimitEnabled() const
{
    return mIsLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the motor of joint is enabled and false otherwise
 */
SIMD_INLINE bool IHingeJoint::isMotorEnabled() const
{
    return mIsMotorEnabled;
}

// Return the minimum angle limit
/**
 * @return The minimum limit angle of the joint (in radian)
 */
SIMD_INLINE scalar IHingeJoint::getMinAngleLimit() const
{
    return mLowerLimit;
}

// Return the maximum angle limit
/**
 * @return The maximum limit angle of the joint (in radian)
 */
SIMD_INLINE scalar IHingeJoint::getMaxAngleLimit() const
{
    return mUpperLimit;
}

// Return the motor speed
/**
 * @return The current speed of the joint motor (in radian per second)
 */
SIMD_INLINE scalar IHingeJoint::getMotorSpeed() const
{
    return mMotorSpeed;
}

// Return the maximum motor torque
/**
 * @return The maximum torque of the joint motor (in Newtons)
 */
SIMD_INLINE scalar IHingeJoint::getMaxMotorTorque() const
{
    return mMaxMotorTorque;
}

// Return the intensity of the current torque applied for the joint motor
/**
 * @param timeStep The current time step (in seconds)
 * @return The intensity of the current torque (in Newtons) of the joint motor
 */
SIMD_INLINE scalar IHingeJoint::getMotorTorque(scalar timeStep) const
{
    return mImpulseMotor / timeStep;
}

// Return the number of bytes used by the joint
SIMD_INLINE size_t IHingeJoint::getSizeInBytes() const
{
    return sizeof(IHingeJoint);
}

}


#endif // IHINGEJOINT_H
