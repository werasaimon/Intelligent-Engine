#ifndef IISLAND_H
#define IISLAND_H


#include "../body/IRigidBody.h"
#include "contacts/IContactManifold.h"
#include "contacts/IContactManifoldSet.h"
#include "joint/IJoint.h"
#include "IOverlappingPair.h"


namespace IPhysics
{

class IIsland
{

  private:

     //-------------------- Attributes -----------------//

     /// Array with all the bodies of the island
     IRigidBody** mBodies;

     /// Current number of bodies in the island
     u32 mNbBodies;



     /// Array with all the contact manifolds between bodies of the island
     IContactManifold** mContactManifolds;

     /// Current number of contact manifold in the island
     u32 mNbContactManifolds;



     /// Array with all the joints between bodies of the island
     IJoint** mJoints;

     /// Current number of joints in the island
     u32 mNbJoints;

     //-------------------- Methods -------------------//

     /// Private assignment operator
     IIsland& operator=(const IIsland& island);

     /// Private copy-constructor
     IIsland(const IIsland& island);



  public:


    //-------------------- Methods --------------------//

     /// Constructor
     IIsland(u32 nbMaxBodies , u32 nbMaxContactManifolds , u32 nbMaxJoints );

     /// Destructor
     ~IIsland();



     /// Add a body into the island
     void addBody(IRigidBody* body);

     /// Add a contact manifold into the island
     void addContactManifold(IContactManifold* contactManifold);

     /// Add a joint into the island
     void addJoint(IJoint* joint);





     /// Return the number of bodies in the island
     u32 getNbBodies() const;

     /// Return the number of contact manifolds in the island
     u32 getNbContactManifolds() const;

     /// Return the number of joints in the island
     u32 getNbJoints() const;


     /// Return a pointer to the array of bodies
     IRigidBody** getBodies();

     /// Return a pointer to the array of contact manifolds
     IContactManifold** getContactManifold();

     /// Return a pointer to the array of joints
     IJoint** getJoints();



     //-------------------- Friendship --------------------//

     friend class IDynamicsWorld;

};


// Add a body into the island
SIMD_INLINE void IIsland::addBody(IRigidBody* body)
{
    assert(!body->isSleeping());
    mBodies[mNbBodies] = body;
    mNbBodies++;
}

// Add a contact manifold into the island
SIMD_INLINE void IIsland::addContactManifold(IContactManifold* contactManifold)
{
    mContactManifolds[mNbContactManifolds] = contactManifold;
    mNbContactManifolds++;
}

SIMD_INLINE void IIsland::addJoint(IJoint *joint)
{
    mJoints[mNbJoints] = joint;
    mNbJoints++;
}


// Return the number of bodies in the island
SIMD_INLINE u32 IIsland::getNbBodies() const
{
    return mNbBodies;
}

// Return the number of contact manifolds in the island
SIMD_INLINE u32 IIsland::getNbContactManifolds() const
{
    return mNbContactManifolds;
}


// Return a pointer to the array of bodies
SIMD_INLINE IRigidBody** IIsland::getBodies()
{
    return mBodies;
}

// Return a pointer to the array of contact manifolds
SIMD_INLINE IContactManifold** IIsland::getContactManifold()
{
    return mContactManifolds;
}


// Return the number of joints in the island
SIMD_INLINE  u32 IIsland::getNbJoints() const
{
   return mNbJoints;
}


// Return a pointer to the array of joints
SIMD_INLINE  IJoint** IIsland::getJoints()
{
   return mJoints;
}

}

#endif // IISLAND_H
