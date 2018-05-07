#include "IFixedJoint.h"
#include "IConstraintSolver.h"


namespace IPhysics
{


// Static variables definition
const scalar  IFixedJoint::BETA = scalar(0.02);

// Constructor
IFixedJoint:: IFixedJoint(const IFixedJointInfo& jointInfo)
   :IJoint(jointInfo), mImpulseTranslation(0, 0, 0), mImpulseRotation(0, 0, 0)
{

    isSplitActive = true;
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

}

// Destructor
IFixedJoint::~IFixedJoint()
{

}

// Initialize before solving the constraint
void  IFixedJoint::initBeforeSolve(const IConstraintSolverData &constraintSolverData  )
{

    // Initialize the bodies index in the velocity array
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

    // Compute the vector from body center to the anchor point in world-space
    mR1World = orientationBody1.getMatrix() * mLocalAnchorPointBody1;
    mR2World = orientationBody2.getMatrix() * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    IMatrix3x3 skewSymmetricMatrixU1=IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    IMatrix3x3 skewSymmetricMatrixU2=IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    IMatrix3x3 massMatrix =IMatrix3x3(inverseMassBodies, 0, 0,
                                      0, inverseMassBodies, 0,
                                      0, 0, inverseMassBodies) +
                                      skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                                      skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

    // Compute the inverse mass matrix K^-1 for the 3 translation constraints
    mInverseMassMatrixTranslation.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslation = massMatrix.getInverse();
    }

    // Compute the bias "b" of the constraint for the 3 translation constraints
    scalar biasFactor = (BETA / constraintSolverData.timeStep);
    mBiasTranslation.setToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS)
    {
        mBiasTranslation = biasFactor * (x2 + mR2World -
                                         x1 - mR1World);
    }

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.getInverse();
    }

    // Compute the bias "b" for the 3 rotation constraints
    mBiasRotation.setToZero();
    if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS) {
        IQuaternion currentOrientationDifference = orientationBody2 * orientationBody1.getInverse();
        currentOrientationDifference.normalize();
        const IQuaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
        mBiasRotation = biasFactor * scalar(2.0) * qError.getV();
    }

    // If warm-starting is not enabled
    if (!isWarmStartingActive)
    {
        // Reset the accumulated impulses
        mImpulseTranslation.setToZero();
        mImpulseRotation.setToZero();
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void  IFixedJoint::warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    IVector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    IVector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    IVector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    IVector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;


    // Get the inverse mass of the bodies
    const scalar inverseMassBody1 = mBody1->mMassInverse;
    const scalar inverseMassBody2 = mBody2->mMassInverse;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 1
    IVector3 linearImpulseBody1 = -mImpulseTranslation;
    IVector3 angularImpulseBody1 = mImpulseTranslation.cross(mR1World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 += -mImpulseRotation;

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda for the 3 translation constraints for body 2
    IVector3 angularImpulseBody2 = -mImpulseTranslation.cross(mR2World);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 2
    angularImpulseBody2 += mImpulseRotation;

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * mImpulseTranslation;
    w2 += mI2 * angularImpulseBody2;
}

// Solve the velocity constraint
void  IFixedJoint::solveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
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

    // Compute J*v for the 3 translation constraints
    const IVector3 JvTranslation = v2 + w2.cross(mR2World) -
            v1 - w1.cross(mR1World);

    // Compute the Lagrange multiplier lambda
    const IVector3 deltaLambda = mInverseMassMatrixTranslation * (-JvTranslation - mBiasTranslation);
    mImpulseTranslation += deltaLambda;

    // Compute the impulse P=J^T * lambda for body 1
    const IVector3 linearImpulseBody1 = -deltaLambda;
    IVector3 angularImpulseBody1 = deltaLambda.cross(mR1World);

    // Apply the impulse to the body 1
    v1 += inverseMassBody1 * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Compute the impulse P=J^T * lambda  for body 2
    const IVector3 angularImpulseBody2 = -deltaLambda.cross(mR2World);

    // Apply the impulse to the body 2
    v2 += inverseMassBody2 * deltaLambda;
    w2 += mI2 * angularImpulseBody2;

    // --------------- Rotation Constraints --------------- //

    // Compute J*v for the 3 rotation constraints
    const IVector3 JvRotation = w2 - w1;

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    IVector3 deltaLambda2 = mInverseMassMatrixRotation * (-JvRotation - mBiasRotation);
    mImpulseRotation += deltaLambda2;

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints for body 1
    angularImpulseBody1 = -deltaLambda2;

    // Apply the impulse to the body 1
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    w2 += mI2 * deltaLambda2;
}

// Solve the position constraint (for position error correction)
void IFixedJoint::solvePositionConstraint(const IConstraintSolverData &constraintSolverData)
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

    // Recompute the inverse inertia tensors
    mI1 = mBody1->getInertiaTensorInverseWorld();
    mI2 = mBody2->getInertiaTensorInverseWorld();

    // Compute the vector from body center to the anchor point in world-space
    mR1World = q1.getMatrix() * mLocalAnchorPointBody1;
    mR2World = q2.getMatrix() * mLocalAnchorPointBody2;

    // Compute the corresponding skew-symmetric matrices
    IMatrix3x3 skewSymmetricMatrixU1=IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
    IMatrix3x3 skewSymmetricMatrixU2=IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

    // --------------- Translation Constraints --------------- //

    // Compute the matrix K=JM^-1J^t (3x3 matrix) for the 3 translation constraints
    scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
    IMatrix3x3 massMatrix =IMatrix3x3(inverseMassBodies, 0, 0,
                                      0, inverseMassBodies, 0,
                                      0, 0, inverseMassBodies) +
            skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
            skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();
    mInverseMassMatrixTranslation.setToZero();
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixTranslation = massMatrix.getInverse();
    }

    // Compute position error for the 3 translation constraints
    const IVector3 errorTranslation = x2 + mR2World -
            x1 - mR1World;

    // Compute the Lagrange multiplier lambda
    const IVector3 lambdaTranslation = mInverseMassMatrixTranslation * (-errorTranslation);

    // Compute the impulse of body 1
    IVector3 linearImpulseBody1 = -lambdaTranslation;
    IVector3 angularImpulseBody1 = lambdaTranslation.cross(mR1World);

    // Compute the pseudo velocity of body 1
    const IVector3 v1 = inverseMassBody1 * linearImpulseBody1;
    IVector3 w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    x1 += v1;
    q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the impulse of body 2
    IVector3 angularImpulseBody2 = -lambdaTranslation.cross(mR2World);

    // Compute the pseudo velocity of body 2
    const IVector3 v2 = inverseMassBody2 * lambdaTranslation;
    IVector3 w2 = mI2 * angularImpulseBody2;

    // Update the body position/orientation of body 2
    x2 += v2;
    q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

    // --------------- Rotation Constraints --------------- //

    // Compute the inverse of the mass matrix K=JM^-1J^t for the 3 rotation
    // contraints (3x3 matrix)
    mInverseMassMatrixRotation = mI1 + mI2;
    if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
    {
        mInverseMassMatrixRotation = mInverseMassMatrixRotation.getInverse();
    }

    // Compute the position error for the 3 rotation constraints
    IQuaternion currentOrientationDifference = q2 * q1.getInverse();
    currentOrientationDifference.normalize();
    const IQuaternion qError = currentOrientationDifference * mInitOrientationDifferenceInv;
    const IVector3 errorRotation = scalar(2.0) * qError.getV();

    // Compute the Lagrange multiplier lambda for the 3 rotation constraints
    IVector3 lambdaRotation = mInverseMassMatrixRotation * (-errorRotation);

    // Compute the impulse P=J^T * lambda for the 3 rotation constraints of body 1
    angularImpulseBody1 = -lambdaRotation;

    // Compute the pseudo velocity of body 1
    w1 = mI1 * angularImpulseBody1;

    // Update the body position/orientation of body 1
    q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
    q1.normalize();

    // Compute the pseudo velocity of body 2
    w2 = mI2 * lambdaRotation;

    // Update the body position/orientation of body 2
    q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
    q2.normalize();

}

}
