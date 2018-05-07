#include "IMaterial.h"

namespace IPhysics
{

// Constructor
 IMaterial:: IMaterial()
         : mFrictionCoefficient(DEFAULT_FRICTION_COEFFICIENT),
           mRollingResistance(DEFAULT_ROLLING_RESISTANCE),
           mBounciness(DEFAULT_BOUNCINESS)
{

}

// Copy-constructor
 IMaterial:: IMaterial(const  IMaterial& material)
         : mFrictionCoefficient(material.mFrictionCoefficient),
           mRollingResistance(material.mRollingResistance),
           mBounciness(material.mBounciness)
{

}

// Destructor
 IMaterial::~ IMaterial()
{

}


}
