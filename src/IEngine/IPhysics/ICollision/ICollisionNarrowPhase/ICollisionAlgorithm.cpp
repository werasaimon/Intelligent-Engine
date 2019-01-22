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


IOutputCollisionInfo ICollisionAlgorithm::GetOutputCollisionInfo() const
{
    return mOutputCollisionInfo;
}




}
