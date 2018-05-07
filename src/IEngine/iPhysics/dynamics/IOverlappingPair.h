#ifndef IOVERLAPPINGPAIR_H
#define IOVERLAPPINGPAIR_H

#include "contacts/IContactManifoldSet.h"
#include "../collision/IProxyShape.h"
#include <vector>



namespace IPhysics
{

// Type for the overlapping pair ID
typedef std::pair<u32, u32> overlappingpairid;

/**
 * This class represents a pair of two proxy collision shapes that are overlapping
 * during the broad-phase collision detection. It is created when
 * the two proxy collision shapes start to overlap and is destroyed when they do not
 * overlap anymore. This class contains a contact manifold that
 * store all the contact points between the two bodies.
 */
class IOverlappingPair
{

    private:

        bool isFakeCollision;

        // -------------------- Attributes -------------------- //

        /// Set of persistent contact manifolds
        IContactManifoldSet mContactManifoldSet;

        IProxyShape* mShape1;
        IProxyShape* mShape2;


        /// Cached previous separating axis
        IVector3 mCachedSeparatingAxis;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IOverlappingPair(const IOverlappingPair& pair);

        /// Private assignment operator
        IOverlappingPair& operator=(const IOverlappingPair& pair);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IOverlappingPair( IProxyShape* shape1, IProxyShape* shape2 , i32 nbMaxContactManifolds = 1);

        /// Destructor
        ~IOverlappingPair();

        /// Return the pointer to first proxy collision shape
        IProxyShape* getShape1() const;

        /// Return the pointer to second body
        IProxyShape* getShape2() const;


        /// Return the cached separating axis
        IVector3 getCachedSeparatingAxis() const;

        /// Set the cached separating axis
        void setCachedSeparatingAxis(const IVector3& axis);



        /// Add a contact to the contact cache
        void addContact(IContactPoint* contact);

        /// Update the contact cache
        void update();

        /// Delete the contact cache.
        void update_delete_not_uset_contact();


        /// Clear the contact points of the contact manifold
        void clearContactPoints();


        /// Return the number of contacts in the cache
        i32 getNbContactManifolds() const;

        /// Return the a reference to the contact manifold set
        const IContactManifoldSet& getContactManifoldSet() const;


        /// Return the a contact points
        std::vector<IContactPoint> getContactPoints()
        {
            std::vector<IContactPoint> contacts(0);
            for ( u32  i = 0; i < mContactManifoldSet.getNbContactManifolds(); ++i)
            {
                 IContactManifold* manifold = mContactManifoldSet.getContactManifold(i);
                 for ( u32 j = 0; j < manifold->getNbContactPoints(); ++j)
                 {
                     contacts.push_back( *manifold->getContactPoint(j) );
                 }
            }
            return contacts;
        }

//        /// Return the a contact manifolds
//        std::vector<rpContactManifold*> getContactManifolds()
//        {
//           std::vector<rpContactManifold*> manifolds(0);
//           for ( u32  i = 0; i < mContactManifoldSet.getNbContactManifolds(); ++i)
//           {
//              manifolds.push_back(mContactManifoldSet.getContactManifold(i));
//           }
//           return manifolds;
//        }


        /// Return the pair of bodies index
        static overlappingpairid computeID(IProxyShape* shape1,
                                           IProxyShape* shape2);

        /// Return the pair of bodies index of the pair
        static bodyindexpair computeBodiesIndexPair( ICollisionBody* body1,
                                                     ICollisionBody* body2);

        // -------------------- Friendship -------------------- //

        friend class IContactManager;
        friend class ICollisionWorld;
        friend class IDynamicsWorld;


};

// Return the pointer to first body
SIMD_INLINE  IProxyShape* IOverlappingPair::getShape1() const
{
    return mShape1;
}
// Return the pointer to first body
SIMD_INLINE  IProxyShape* IOverlappingPair::getShape2() const
{
    return mShape2;
}

// Return the cached separating axis
SIMD_INLINE  IVector3 IOverlappingPair::getCachedSeparatingAxis() const
{
    return mCachedSeparatingAxis;
}

// Set the cached separating axis
SIMD_INLINE  void IOverlappingPair::setCachedSeparatingAxis(const IVector3& axis)
{
    mCachedSeparatingAxis = axis;
}



SIMD_INLINE i32 IOverlappingPair::getNbContactManifolds() const
{
    return mContactManifoldSet.getNbContactManifolds();
}

SIMD_INLINE const IContactManifoldSet& IOverlappingPair::getContactManifoldSet() const
{
     return mContactManifoldSet;
}



// Return the pair of bodies index
SIMD_INLINE  bodyindexpair IOverlappingPair::computeBodiesIndexPair(ICollisionBody* body1,
                                                                    ICollisionBody* body2)
{
    // Construct the pair of body index
    bodyindexpair indexPair = body1->getID() < body2->getID() ?
                                 std::make_pair(body1->getID(), body2->getID()) :
                                 std::make_pair(body2->getID(), body1->getID());
    assert(indexPair.first != indexPair.second);
    return indexPair;
}


// Return the pair of bodies index
SIMD_INLINE   overlappingpairid IOverlappingPair::computeID(IProxyShape* shape1,
                                                            IProxyShape* shape2)
{

    assert(shape1->mBroadPhaseID >= 0 && shape2->mBroadPhaseID >= 0);

    // Construct the pair of body index
    overlappingpairid pairID =  shape1->mBroadPhaseID < shape2->mBroadPhaseID ?
                                std::make_pair(shape1->mBroadPhaseID, shape2->mBroadPhaseID) :
                                std::make_pair(shape2->mBroadPhaseID, shape1->mBroadPhaseID);
    assert(pairID.first != pairID.second);
    return pairID;
}






}

#endif // IOVERLAPPINGPAIR_H
