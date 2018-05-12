#include "IJoint.h"

namespace IPhysics
{

// Constructor
IJoint::IJoint(const IJointInfo& jointInfo)
    :mBody1(jointInfo.body1),
     mBody2(jointInfo.body2),
     mType(jointInfo.type),
     mPositionCorrectionTechnique(jointInfo.positionCorrectionTechnique),
     mIsCollisionEnabled(jointInfo.isCollisionEnabled),
     mIsAlreadyInIsland(false)
{

    assert(mBody1 != NULL);
    assert(mBody2 != NULL);
}

// Destructor
IJoint::~IJoint()
{

}

/// set stoffnes constraint-joint
void IJoint::setSoftness(const scalar &value)
{
    softness = value;
}



}
