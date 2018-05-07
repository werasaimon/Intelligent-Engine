#include "ISliderJoint.h"
#include "IConstraintSolver.h"

namespace IPhysics
{


// Static variables definition
const scalar ISliderJoint::BETA = scalar(0.2);

// Constructor
ISliderJoint::ISliderJoint(const ISliderJointInfo& jointInfo)
            : IJoint(jointInfo), mImpulseTranslation(0, 0), mImpulseRotation(0, 0, 0),
              mImpulseLowerLimit(0), mImpulseUpperLimit(0), mImpulseMotor(0),
              mIsLimitEnabled(jointInfo.isLimitEnabled), mIsMotorEnabled(jointInfo.isMotorEnabled),
              mLowerLimit(jointInfo.minTranslationLimit),
              mUpperLimit(jointInfo.maxTranslationLimit), mIsLowerLimitViolated(false),
              mIsUpperLimitViolated(false), mMotorSpeed(jointInfo.motorSpeed),
              mMaxMotorForce(jointInfo.maxMotorForce){

    assert(mUpperLimit >= 0.0);
    assert(mLowerLimit <= 0.0);
    assert(mMaxMotorForce >= 0.0);

    isWarmStartingActive = true;

    // Compute the local-space anchor point for each body
    const ITransform& transform1 = mBody1->getTransform();
    const ITransform& transform2 = mBody2->getTransform();
    mLocalAnchorPointBody1 = transform1.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = transform2.getInverse() * jointInfo.anchorPointWorldSpace;

    // Compute the inverse of the initial orientation difference between the two bodies
    mInitOrientationDifferenceInv = transform2.getRotation() *
                                    transform1.getRotation().getInverse();
    mInitOrientationDifferenceInv.normalize();
    mInitOrientationDifferenceInv.getInverse();

    // Compute the slider axis in local-space of body 1
    mSliderAxisBody1 = mBody1->getTransform().getRotation().getInverse().getMatrix() *  jointInfo.sliderAxisWorldSpace;
    mSliderAxisBody1.normalize();
}

// Destructor
ISliderJoint::~ISliderJoint()
{

}

// Initialize before solving the constraint
void ISliderJoint::initBeforeSolve(const IConstraintSolverData &constraintSolverData )
{

  // Initialize the bodies index in the veloc ity array
   mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
   mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

   // Get the bodies positions and orientations
   const IVector3& x1 = mBody1->mCenterOfMassWorld;
   const IVector3& x2 = mBody2->mCenterOfMassWorld;
   const IQuaternion& orientationBody1 = mBody1->getTransform().getRotation();
   const IQuaternion& orientationBody2 = mBody2->getTransform().getRotation();



    // Get the inertia tensor of bodies
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = orientationBody1.getMatrix() * mLocalAnchorPointBody1;
    mR2 = orientationBody2.getMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const IVector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = orientationBody1.getMatrix() * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    bool oldIsLowerLimitViolated = mIsLowerLimitViolated;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    if (mIsLowerLimitViolated != oldIsLowerLimitViolated)
    {
        mImpulseLowerLimit = 0.0;
    }
    bool oldIsUpperLimitViolated = mIsUpperLimitViolated;
    mIsUpperLimitViolated = upperLimitError <= 0;
    if (mIsUpperLimitViolated != oldIsUpperLimitViolated)
    {
        mImpulseUpperLimit = 0.0;
    }

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const IVector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    IVector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    IVector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    IVector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    IVector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
    IMatrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the bias "b" of the translation constraint
    mBTranslation.setToZero();
    scalar biasFactor = (BETA/constraintSolverData.timeStep);
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        mBTranslation.x = u.dot(mN1);
        mBTranslation.y = u.dot(mN2);
        mBTranslation *= biasFactor;
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
    }

    // Compute the bias "b" of the rotation constraint
    mBRotation.setToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        IQuaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
        currentOrientationDifference.normalize();
        const IQuaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        mBRotation = biasFactor * scalar(2.0) * qError.getV();
    }

    // If the limits are enabled
    if (mIsLimitEnabled && (mIsLowerLimitViolated || mIsUpperLimitViolated))
    {

        // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                  mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                  mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
        mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                  scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);

        // Compute the bias "b" of the lower limit constraint
        mBLowerLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBLowerLimit = biasFactor * lowerLimitError;
        }

        // Compute the bias "b" of the upper limit constraint
        mBUpperLimit = 0.0;
        if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
        {
            mBUpperLimit = biasFactor * upperLimitError;
        }
    }

    // If the motor is enabled
    if (mIsMotorEnabled)
    {
        // Compute the inverse of mass matrix K=JM^-1J^t for the motor (1x1 matrix)
        mInverseMassMatrixMotor = mBody1->mMassInverse + mBody2->mMassInverse;
        mInverseMassMatrixMotor = (mInverseMassMatrixMotor > 0.0) ?
                    scalar(1.0) / mInverseMassMatrixMotor : scalar(0.0);
    }

    // If warm-starting is not enabled
    if (!isWarmStartingActive)
    {
        // Reset all the accumulated impulses
        mImpulseTranslation.setToZero();
        mImpulseRotation.setToZero();
        mImpulseLowerLimit = 0.0;
        mImpulseUpperLimit = 0.0;
        mImpulseMotor = 0.0;
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void ISliderJoint::warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    IVector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    IVector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    IVector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    IVector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Get the inverse mass and inverse inertia tensors of the bodies
    const scalar inverseMassBody1 = mBody1->mMassInverse;
    const scalar inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    scalar impulseLimits = mImpulseUpperLimit - mImpulseLowerLimit;
    IVector3 linearImpulseLimits = impulseLimits * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    IVector3 impulseMotor = mImpulseMotor * mSliderAxisWorld;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    IVector3 linearImpulseBody1 = -mN1 * mImpulseTranslation.x - mN2 * mImpulseTranslation.y;
    IVector3 angularImpulseBody1 = -mR1PlusUCrossN1 * mImpulseTranslation.x -
                                   mR1PlusUCrossN2 * mImpulseTranslation.y;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 1
    linearImpulseBody1 += linearImpulseLimits;
    angularImpulseBody1 += impulseLimits * mR1PlusUCrossSliderAxis;

    // Compute the impulse P=J^T * lambda for the motor constraint of body 1
    linearImpulseBody1 += impulseMotor;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    IVector3 linearImpulseBody2 = mN1 * mImpulseTranslation.x + mN2 * mImpulseTranslation.y;
    IVector3 angularImpulseBody2 = mR2CrossN1 * mImpulseTranslation.x +
                                  mR2CrossN2 * mImpulseTranslation.y;



    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 += mImpulseRotation;

    // Compute the impulse P=J^T * lambda for the lower and upper limits constraints of body 2
    linearImpulseBody2 += -linearImpulseLimits;
    angularImpulseBody2 += -impulseLimits * mR2CrossSliderAxis;



    // Compute the impulse P=J^T * lambda for the motor constraint of body 2
    linearImpulseBody2 += -impulseMotor;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void ISliderJoint::solveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    IVector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    IVector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    IVector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    IVector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = mBody1->mMassInverse;
    scalar inverseMassBody2 = mBody2->mMassInverse;

    // --------------- Translation Constraints --------------- //

    // Compute J*v for the 2 translation constraints
    const scalar el1 = -mN1.dot(v1) - w1.dot(mR1PlusUCrossN1) + mN1.dot(v2) + w2.dot(mR2CrossN1);
    const scalar el2 = -mN2.dot(v1) - w1.dot(mR1PlusUCrossN2) + mN2.dot(v2) + w2.dot(mR2CrossN2);
    const IVector2 JvTranslation(el1, el2);

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    IVector2 deltaLambda = mInverseMassMatrixTranslationConstraint * (-JvTranslation -mBTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const IVector3 linearImpulseBody1 = -mN1 * deltaLambda.x - mN2 * deltaLambda.y;
    IVector3 angularImpulseBody1 = -mR1PlusUCrossN1 * deltaLambda.x -
                                   mR1PlusUCrossN2 * deltaLambda.y;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const IVector3 linearImpulseBody2 = mN1 * deltaLambda.x + mN2 * deltaLambda.y;
    IVector3 angularImpulseBody2 = mR2CrossN1 * deltaLambda.x + mR2CrossN2 * deltaLambda.y;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const IVector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    IVector3 deltaLambda2 = mInverseMassMatrixRotationConstraint * (-JvRotation - mBRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body to body 1
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = deltaLambda2;

    // Apply the impulse to the body 2
    w2 += mI2 * angularImpulseBody2;

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {
            // Compute J*v for the lower limit constraint
            const scalar JvLowerLimit = mSliderAxisWorld.dot(v2) + mR2CrossSliderAxis.dot(w2) -
                                        mSliderAxisWorld.dot(v1) - mR1PlusUCrossSliderAxis.dot(w1);

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar deltaLambdaLower = mInverseMassMatrixLimit * (-JvLowerLimit -mBLowerLimit);
            scalar lambdaTemp = mImpulseLowerLimit;
            mImpulseLowerLimit = IMax(mImpulseLowerLimit + deltaLambdaLower, scalar(0.0));
            deltaLambdaLower = mImpulseLowerLimit - lambdaTemp;



            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const IVector3 linearImpulseBody1 = -deltaLambdaLower * mSliderAxisWorld;
            const IVector3 angularImpulseBody1 = -deltaLambdaLower * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;



            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const IVector3 linearImpulseBody2 = deltaLambdaLower * mSliderAxisWorld;
            const IVector3 angularImpulseBody2 = deltaLambdaLower * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;


        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute J*v for the upper limit constraint
            const scalar JvUpperLimit = mSliderAxisWorld.dot(v1) + mR1PlusUCrossSliderAxis.dot(w1)
                                      - mSliderAxisWorld.dot(v2) - mR2CrossSliderAxis.dot(w2);

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar deltaLambdaUpper = mInverseMassMatrixLimit * (-JvUpperLimit -mBUpperLimit);
            scalar lambdaTemp = mImpulseUpperLimit;
            mImpulseUpperLimit = IMax(mImpulseUpperLimit + deltaLambdaUpper, scalar(0.0));
            deltaLambdaUpper = mImpulseUpperLimit - lambdaTemp;



            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const IVector3 linearImpulseBody1 = deltaLambdaUpper * mSliderAxisWorld;
            const IVector3 angularImpulseBody1 = deltaLambdaUpper * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            v1 += inverseMassBody1 * linearImpulseBody1;
            w1 += mI1 * angularImpulseBody1;



            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const IVector3 linearImpulseBody2 = -deltaLambdaUpper * mSliderAxisWorld;
            const IVector3 angularImpulseBody2 = -deltaLambdaUpper * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            v2 += inverseMassBody2 * linearImpulseBody2;
            w2 += mI2 * angularImpulseBody2;
        }
    }

    // --------------- Motor --------------- //

    if (mIsMotorEnabled)
    {

        // Compute J*v for the motor
        const scalar JvMotor = mSliderAxisWorld.dot(v1) - mSliderAxisWorld.dot(v2);

        // Compute the Lagrange multiplier lambda for the motor
        const scalar maxMotorImpulse = mMaxMotorForce;
        scalar deltaLambdaMotor = mInverseMassMatrixMotor * (-JvMotor - mMotorSpeed);
        scalar lambdaTemp = mImpulseMotor;
        mImpulseMotor = IClamp(mImpulseMotor + deltaLambdaMotor, -maxMotorImpulse, maxMotorImpulse);
        deltaLambdaMotor = mImpulseMotor - lambdaTemp;

        // Compute the impulse P=J^T * lambda for the motor of body 1
        const IVector3 linearImpulseBody1 = deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 1
        v1 += inverseMassBody1 * linearImpulseBody1;

        // Compute the impulse P=J^T * lambda for the motor of body 2
        const IVector3 linearImpulseBody2 = -deltaLambdaMotor * mSliderAxisWorld;

        // Apply the impulse to the body 2
        v2 += inverseMassBody2 * linearImpulseBody2;
    }
}

// Solve the position constraint (for position error correction)
void ISliderJoint::solvePositionConstraint(const IConstraintSolverData &constraintSolverData)
{

    // If the error position correction technique is not the non-linear-gauss-seidel, we do
    // do not execute this method
    if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;


    // Get the bodies positions and orientations
    IVector3&    x1 = constraintSolverData.Positions[mIndexBody1].x;
    IVector3&    x2 = constraintSolverData.Positions[mIndexBody2].x;
    IQuaternion& q1 = constraintSolverData.Positions[mIndexBody1].q;
    IQuaternion& q2 = constraintSolverData.Positions[mIndexBody2].q;



    // Get the inverse mass and inverse inertia tensors of the bodies
    scalar inverseMassBody1 = mBody1->mMassInverse;
    scalar inverseMassBody2 = mBody2->mMassInverse;

    // Recompute the inertia tensor of bodies
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Vector from body center to the anchor point
    mR1 = q1.getMatrix() * mLocalAnchorPointBody1;
    mR2 = q2.getMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const IVector3 u = x2 + mR2 - x1 - mR1;

    // Compute the two orthogonal vectors to the slider axis in world-space
    mSliderAxisWorld = q1.getMatrix() * mSliderAxisBody1;
    mSliderAxisWorld.normalize();
    mN1 = mSliderAxisWorld.getOneUnitOrthogonalVector();
    mN2 = mSliderAxisWorld.cross(mN1);

    //IVector3::BiUnitOrthogonalVector(mSliderAxisWorld , mN1 , mN2 );

    // Check if the limit constraints are violated or not
    scalar uDotSliderAxis = u.dot(mSliderAxisWorld);
    scalar lowerLimitError = uDotSliderAxis - mLowerLimit;
    scalar upperLimitError = mUpperLimit - uDotSliderAxis;
    mIsLowerLimitViolated = lowerLimitError <= 0;
    mIsUpperLimitViolated = upperLimitError <= 0;

    // Compute the cross products used in the Jacobians
    mR2CrossN1 = mR2.cross(mN1);
    mR2CrossN2 = mR2.cross(mN2);
    mR2CrossSliderAxis = mR2.cross(mSliderAxisWorld);
    const IVector3 r1PlusU = mR1 + u;
    mR1PlusUCrossN1 = (r1PlusU).cross(mN1);
    mR1PlusUCrossN2 = (r1PlusU).cross(mN2);
    mR1PlusUCrossSliderAxis = (r1PlusU).cross(mSliderAxisWorld);

    // --------------- Translation Constraints --------------- //

    // Recompute the inverse of the mass matrix K=JM^-1J^t for the 2 translation
    // constraints (2x2 matrix)
    scalar sumInverseMass = mBody1->mMassInverse + mBody2->mMassInverse;
    IVector3 I1R1PlusUCrossN1 = mI1 * mR1PlusUCrossN1;
    IVector3 I1R1PlusUCrossN2 = mI1 * mR1PlusUCrossN2;
    IVector3 I2R2CrossN1 = mI2 * mR2CrossN1;
    IVector3 I2R2CrossN2 = mI2 * mR2CrossN2;
    const scalar el11 = sumInverseMass + mR1PlusUCrossN1.dot(I1R1PlusUCrossN1) + mR2CrossN1.dot(I2R2CrossN1);
    const scalar el12 = mR1PlusUCrossN1.dot(I1R1PlusUCrossN2) + mR2CrossN1.dot(I2R2CrossN2);
    const scalar el21 = mR1PlusUCrossN2.dot(I1R1PlusUCrossN1) + mR2CrossN2.dot(I2R2CrossN1);
    const scalar el22 = sumInverseMass + mR1PlusUCrossN2.dot(I1R1PlusUCrossN2) + mR2CrossN2.dot(I2R2CrossN2);
    IMatrix2x2 matrixKTranslation(el11, el12, el21, el22);
    mInverseMassMatrixTranslationConstraint.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslationConstraint = matrixKTranslation.getInverse();
    }

    // Compute the position error for the 2 translation constraints
    const IVector2 translationError(u.dot(mN1), u.dot(mN2));

    // Compute the Lagrange multiplier lambda for the 2 translation constraints
    IVector2 lambdaTranslation = mInverseMassMatrixTranslationConstraint * (-translationError);

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 1
    const IVector3 linearImpulseBody1 = -mN1 * lambdaTranslation.x -
                                         mN2 * lambdaTranslation.y;

    IVector3 angularImpulseBody1 = -mR1PlusUCrossN1 * lambdaTranslation.x -
                                    mR1PlusUCrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 1
    const IVector3 v1 = inverseMassBody1 * linearImpulseBody1;
    IVector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 2 translation constraints of body 2
    const IVector3 linearImpulseBody2 = mN1 * lambdaTranslation.x + mN2 * lambdaTranslation.y;
    IVector3 angularImpulseBody2 = mR2CrossN1 * lambdaTranslation.x +
                                  mR2CrossN2 * lambdaTranslation.y;

    // Apply the impulse to the body 2
    const IVector3 v2 = inverseMassBody2 * linearImpulseBody2;
    IVector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotationConstraint = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotationConstraint = mInverseMassMatrixRotationConstraint.getInverse();
    }

    // Compute the position error for the 3 rotation constraints
    IQuaternion currentOrientationDifference = q2 * q1.getInverse();
    currentOrientationDifference.normalize();
    const IQuaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
    const IVector3 errorRotation = scalar(2.0) * qError.getV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    IVector3 lambdaRotation = mInverseMassMatrixRotationConstraint * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Apply the impulse to the body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 2
    angularImpulseBody2 = lambdaRotation;

    // Apply the impulse to the body 2
    w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

    // --------------- Limits Constraints --------------- //

    if (mIsLimitEnabled)
    {

        if (mIsLowerLimitViolated || mIsUpperLimitViolated)
        {

            // Compute the inverse of the mass matrix K=JM^-1J^t for the limits (1x1 matrix)
            mInverseMassMatrixLimit = mBody1->mMassInverse + mBody2->mMassInverse +
                                    mR1PlusUCrossSliderAxis.dot(mI1 * mR1PlusUCrossSliderAxis) +
                                    mR2CrossSliderAxis.dot(mI2 * mR2CrossSliderAxis);
            mInverseMassMatrixLimit = (mInverseMassMatrixLimit > 0.0) ?
                                      scalar(1.0) / mInverseMassMatrixLimit : scalar(0.0);
        }

        // If the lower limit is violated
        if (mIsLowerLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the lower limit constraint
            scalar lambdaLowerLimit = mInverseMassMatrixLimit * (-lowerLimitError);

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 1
            const IVector3 linearImpulseBody1 = -lambdaLowerLimit * mSliderAxisWorld;
            const IVector3 angularImpulseBody1 = -lambdaLowerLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const IVector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const IVector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the lower limit constraint of body 2
            const IVector3 linearImpulseBody2 = lambdaLowerLimit * mSliderAxisWorld;
            const IVector3 angularImpulseBody2 = lambdaLowerLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const IVector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const IVector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
            q2.normalize();
        }

        // If the upper limit is violated
        if (mIsUpperLimitViolated)
        {

            // Compute the Lagrange multiplier lambda for the upper limit constraint
            scalar lambdaUpperLimit = mInverseMassMatrixLimit * (-upperLimitError);

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 1
            const IVector3 linearImpulseBody1 = lambdaUpperLimit * mSliderAxisWorld;
            const IVector3 angularImpulseBody1 = lambdaUpperLimit * mR1PlusUCrossSliderAxis;

            // Apply the impulse to the body 1
            const IVector3 v1 = inverseMassBody1 * linearImpulseBody1;
            const IVector3 w1 = mI1 * angularImpulseBody1;

            // Update the body position/orientation of body 1
            x1 += v1;
            q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
            q1.normalize();

            // Compute the impulse P=J^T * lambda for the upper limit constraint of body 2
            const IVector3 linearImpulseBody2 = -lambdaUpperLimit * mSliderAxisWorld;
            const IVector3 angularImpulseBody2 = -lambdaUpperLimit * mR2CrossSliderAxis;

            // Apply the impulse to the body 2
            const IVector3 v2 = inverseMassBody2 * linearImpulseBody2;
            const IVector3 w2 = mI2 * angularImpulseBody2;

            // Update the body position/orientation of body 2
            x2 += v2;
            q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
            q2.normalize();
        }
    }

}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void ISliderJoint::enableLimit(bool isLimitEnabled)
{

    if (isLimitEnabled != mIsLimitEnabled)
    {

        mIsLimitEnabled = isLimitEnabled;

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *                       otherwise
 */
void ISliderJoint::enableMotor(bool isMotorEnabled)
{

    mIsMotorEnabled = isMotorEnabled;
    mImpulseMotor = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
scalar ISliderJoint::getTranslation() const
{

    // TODO : Check if we need to compare rigid body position or center of mass here

    const IVector3& x1 = mBody1->mCenterOfMassWorld;
    const IVector3& x2 = mBody2->mCenterOfMassWorld;
    const IQuaternion& q1 = mBody1->getTransform().getRotation();
    const IQuaternion& q2 = mBody2->getTransform().getRotation();

    // Compute the two anchor points in world-space coordinates
    const IVector3 anchorBody1 = x1 + q1.getMatrix() * mLocalAnchorPointBody1;
    const IVector3 anchorBody2 = x2 + q2.getMatrix() * mLocalAnchorPointBody2;

    // Compute the vector u (difference between anchor points)
    const IVector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    IVector3 sliderAxisWorld = q1.getMatrix() * mSliderAxisBody1;
    sliderAxisWorld.normalize();

    // Compute and return the translation value
    return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void ISliderJoint::setMinTranslationLimit(scalar lowerLimit)
{

    assert(lowerLimit <= mUpperLimit);

    if (lowerLimit != mLowerLimit)
    {
        mLowerLimit = lowerLimit;

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param lowerLimit The maximum translation limit of the joint (in meters)
 */
void ISliderJoint::setMaxTranslationLimit(scalar upperLimit)
{

    assert(mLowerLimit <= upperLimit);

    if (upperLimit != mUpperLimit)
    {
        mUpperLimit = upperLimit;

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void ISliderJoint::resetLimits()
{

    // Reset the accumulated impulses for the limits
    mImpulseLowerLimit = 0.0;
    mImpulseUpperLimit = 0.0;

    // Wake up the two bodies of the joint
    mBody1->setIsSleeping(false);
    mBody2->setIsSleeping(false);
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void ISliderJoint::setMotorSpeed(scalar motorSpeed)
{

    if (motorSpeed != mMotorSpeed)
    {

        mMotorSpeed = motorSpeed;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void ISliderJoint::setMaxMotorForce(scalar maxMotorForce)
{

    if (maxMotorForce != mMaxMotorForce)
    {

        assert(mMaxMotorForce >= 0.0);
        mMaxMotorForce = maxMotorForce;

        // Wake up the two bodies of the joint
        mBody1->setIsSleeping(false);
        mBody2->setIsSleeping(false);
    }
}




}
