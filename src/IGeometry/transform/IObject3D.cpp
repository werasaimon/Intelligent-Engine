#include "IObject3D.h"

namespace IGeometry
{

// Constructor
IObject3D::IObject3D()
{
    // Set the transformation matrix to the identity
    setToIdentity();
}

// Destructor
IObject3D::~IObject3D()
{

}


}
