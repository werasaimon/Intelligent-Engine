#include "IObjectTransform.h"

namespace IEngine
{

// Constructor
IObjectTransform::IObjectTransform()
{
    // Set the transformation matrix to the identity
    setToIdentity();
}

// Destructor
IObjectTransform::~IObjectTransform()
{

}

}
