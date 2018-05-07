#include "IBallAndSocketJoint.h"
#include "IConstraintSolver.h"

namespace IPhysics
{



// Static variables definition
const scalar IBallAndSocketJoint::BETA = scalar(0.2);

// Constructor
IBallAndSocketJoint::IBallAndSocketJoint(const IBallAndSocketJointInfo& jointInfo)
:IJoint(jointInfo), mImpulse(IVector3(0, 0, 0))
{

    isWarmStartingActive = true;

    // Compute the local-space anchor point the constraint error
    mLocalAnchorPointBody1 = mBody1->mTransform.getInverse() * jointInfo.anchorPointWorldSpace;
    mLocalAnchorPointBody2 = mBody2->mTransform.getInverse() * jointInfo.anchorPointWorldSpace;

    softness = 0.0001f;

}


// Destructor
IBallAndSocketJoint::~IBallAndSocketJoint()
{

}



// Initialize before solving the constraint
void IBallAndSocketJoint::initBeforeSolve(const IConstraintSolverData &constraintSolverData  )
{

         // Initialize the bodies index in the velocity array
          mIndexBody1 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody1)->second;
          mIndexBody2 = constraintSolverData.mapBodyToConstrainedVelocityIndex.find(mBody2)->second;

         // Get the bodies center of mass and orientations
          const IVector3& x1 = mBody1->mCenterOfMassWorld;
          const IVector3& x2 = mBody2->mCenterOfMassWorld;
          const IQuaternion& orientationBody1 = mBody1->mTransform.getRotation();
          const IQuaternion& orientationBody2 = mBody2->mTransform.getRotation();

          // Get the inertia tensor of bodies
          mI1 = mBody1->getInertiaTensorInverseWorld();
          mI2 = mBody2->getInertiaTensorInverseWorld();

          // Compute the vector from body center to the anchor point in world-space
          mR1World = orientationBody1.getMatrix() * mLocalAnchorPointBody1;
          mR2World = orientationBody2.getMatrix() * mLocalAnchorPointBody2;



          // Compute the corresponding skew-symmetric matrices
          IMatrix3x3 skewSymmetricMatrixU1= IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
          IMatrix3x3 skewSymmetricMatrixU2= IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

          // Compute the matrix K=JM^-1J^t (3x3 matrix)
          scalar inverseMassBodies = mBody1->mMassInverse + mBody2->mMassInverse;
          IMatrix3x3 massMatrix = IMatrix3x3(inverseMassBodies, 0, 0,
                                             0, inverseMassBodies, 0,
                                             0, 0, inverseMassBodies) +
                                             skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                                             skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

          // Compute the inverse mass matrix K^-1
          mInverseMassMatrix.setToZero();
          if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC )
          {
                  mInverseMassMatrix = massMatrix.getInverse();
          }

          // Compute the bias "b" of the constraint
          mBiasVector.setToZero();
          if (mPositionCorrectionTechnique == BAUMGARTE_JOINTS )
          {
              scalar biasFactor = BETA/constraintSolverData.timeStep;
              mBiasVector = biasFactor * (x2 + mR2World -
                                          x1 - mR1World);
          }


          // If warm-starting is not enabled
          if (!isWarmStartingActive)
          {
              // Reset the accumulated impulse
              mImpulse.setToZero();
          }


}




// Warm start the constraint (apply the previous impulse at the beginning of the step)
void IBallAndSocketJoint::warmstart(const IConstraintSolverData &constraintSolverData)
{

    // Get the velocities
    IVector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    IVector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    IVector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    IVector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;



    // Compute the impulse P=J^T * lambda for the body 1
    const IVector3 linearImpulseBody1 = -mImpulse;
    const IVector3 linearImpulseBody2 =  mImpulse;

    // Compute the impulse P=J^T * lambda for the body 2
    const IVector3 angularImpulseBody1 =  mImpulse.cross(mR1World);
    const IVector3 angularImpulseBody2 = -mImpulse.cross(mR2World);



    // Apply the impulse to the body 1
    v1 += mBody1->mMassInverse * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body to the body 2
    v2 += mBody2->mMassInverse * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;


}




// Solve the velocity constraint
void IBallAndSocketJoint::solveVelocityConstraint(const IConstraintSolverData &constraintSolverData)
{


  // Get the velocities
    IVector3& v1 = constraintSolverData.Velocities[mIndexBody1].v;
    IVector3& v2 = constraintSolverData.Velocities[mIndexBody2].v;
    IVector3& w1 = constraintSolverData.Velocities[mIndexBody1].w;
    IVector3& w2 = constraintSolverData.Velocities[mIndexBody2].w;

    // Compute J*v
    const IVector3 Jv = v2 + w2.cross(mR2World) -
                        v1 - w1.cross(mR1World);



    // Compute the Lagrange multiplier lambda
    const IVector3 deltaLambda = mInverseMassMatrix * (-Jv - mBiasVector);

    mImpulse += (deltaLambda - (deltaLambda.getUnit() * mImpulse.length() * softness));

    // Compute the impulse P=J^T * lambda for the body 1
    const IVector3 linearImpulseBody1 = -deltaLambda;
    const IVector3 linearImpulseBody2 =  deltaLambda;

    // Compute the impulse P=J^T * lambda for the body 2
    const IVector3 angularImpulseBody1 =  deltaLambda.cross(mR1World);
    const IVector3 angularImpulseBody2 = -deltaLambda.cross(mR2World);


    // Apply the impulse to the body 1
    v1 += mBody1->mMassInverse * linearImpulseBody1;
    w1 += mI1 * angularImpulseBody1;

    // Apply the impulse to the body 2
    v2 += mBody2->mMassInverse * linearImpulseBody2;
    w2 += mI2 * angularImpulseBody2;


}





// Solve the position constraint (for position error correction)
void IBallAndSocketJoint::solvePositionConstraint(const IConstraintSolverData &constraintSolverData)
{


  // If the error position correction technique is not the non-linear-gauss-seidel, we do
  // do not execute this method
  if (mPositionCorrectionTechnique != NON_LINEAR_GAUSS_SEIDEL) return;


  // Get the bodies center of mass and orientations
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
  IMatrix3x3 skewSymmetricMatrixU1= IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR1World);
  IMatrix3x3 skewSymmetricMatrixU2= IMatrix3x3::computeSkewSymmetricMatrixForCrossProduct(mR2World);

  // Recompute the inverse mass matrix K=J^TM^-1J of of the 3 translation constraints
  scalar inverseMassBodies = inverseMassBody1 + inverseMassBody2;
  IMatrix3x3 massMatrix = IMatrix3x3(inverseMassBodies, 0, 0,
                                   0, inverseMassBodies, 0,
                                   0, 0, inverseMassBodies) +
                                   skewSymmetricMatrixU1 * mI1 * skewSymmetricMatrixU1.getTranspose() +
                                   skewSymmetricMatrixU2 * mI2 * skewSymmetricMatrixU2.getTranspose();

  mInverseMassMatrix.setToZero();
  if (mBody1->getType() == DYNAMIC || mBody2->getType() == DYNAMIC)
  {
      mInverseMassMatrix = massMatrix.getInverse();
  }


  // Compute the constraint error (value of the C(x) function)
  IVector3 constraintError = (x2 + mR2World -
                              x1 - mR1World);

//  // Relaxation offset damping to the constraint error
//  scalar dampingRelaxation = 0;//0.0001;
//  if( constraintError.length() > dampingRelaxation)
//  {
//      constraintError *= scalar(1.0 - dampingRelaxation);
//  }
//  else
//  {
//      constraintError = Vector3::ZERO;
//  }



  // Compute the Lagrange multiplier lambda
  // TODO : Do not solve the system by computing the inverse each time and multiplying with the
  //        right-hand side vector but instead use a method to directly solve the linear system.
  const IVector3 lambda = mInverseMassMatrix * ( -constraintError  );




  // Compute the impulse of body
  const IVector3 linearImpulseBody1 = -lambda;
  const IVector3 linearImpulseBody2 =  lambda;

  // Compute the impulse of body
  const IVector3 angularImpulseBody1 =  lambda.cross(mR1World);
  const IVector3 angularImpulseBody2 = -lambda.cross(mR2World);


  // Compute the pseudo velocity of body 1
  const IVector3 v1 = inverseMassBody1 * linearImpulseBody1;
  const IVector3 w1 = mI1 * angularImpulseBody1;

  // Compute the pseudo velocity of body 2
  const IVector3 v2 = inverseMassBody2 * linearImpulseBody2;
  const IVector3 w2 = mI2 * angularImpulseBody2;


  /**/
  // Update the body center of mass and orientation of body 1
  x1 += v1;
  q1 += IQuaternion(0, w1) * q1 * scalar(0.5);
  q1.normalize();
  // Update the body position/orientation of body 2
  x2 += v2;
  q2 += IQuaternion(0, w2) * q2 * scalar(0.5);
  q2.normalize();

  // Update the body center of mass and orientation of body 1
//  x1 = Transform::integrateLinear(  x1 , v1 , 1.0);
//  q1 = Transform::integrateAngular( q1 , w1 , 1.0);
//  // Update the body position/orientation of body 2
//  x2 = Transform::integrateLinear(  x2 , v2 , 1.0);
//  q2 = Transform::integrateAngular( q2 , w2 , 1.0);

}



}
