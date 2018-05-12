#include "IIsland.h"

namespace IPhysics
{



IIsland::IIsland(u32 nbMaxBodies, u32 nbMaxContactManifolds , u32 nbMaxJoints )
    : mBodies(NULL),
      mContactManifolds(NULL),
      mJoints(NULL),
      mNbBodies(0),
      mNbContactManifolds(0),
      mNbJoints(0)
{
    // Allocate memory for the arrays
     mBodies                = new IRigidBody*[nbMaxBodies];
     mContactManifolds      = new IContactManifold*[nbMaxContactManifolds];
     mJoints                = new IJoint*[nbMaxJoints];

}



IIsland::~IIsland()
{
    delete[] mBodies;
    delete[] mContactManifolds;
    delete[] mJoints;
}

}
