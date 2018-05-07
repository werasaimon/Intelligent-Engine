#include "ICollisionAlgorithm.h"

namespace IPhysics
{

// Constructor
ICollisionAlgorithm::ICollisionAlgorithm()
{

}

// Destructor
ICollisionAlgorithm::~ICollisionAlgorithm()
{

}


OutputCollisionInfo ICollisionAlgorithm::outputCollisionInfo() const
{
    return mOutputCollisionInfo;
}




}
