#ifndef ICONTACTMANAGER_H
#define ICONTACTMANAGER_H

// Libraries
#include <map>
#include <set>

#include "IContactManifold.h"
#include "IContactManifoldSet.h"
#include "../IOverlappingPair.h"
#include "../../collision/broad_phase/IBroadPhase.h"
#include "../../collision/contacts/IGenerationContactPoints.h"

//// Declarations
//class IBroadPhase;
//class ICollisionWorld;

namespace IPhysics
{


// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class IContactManager : public AbstractBroadPhaseNotifyOverlappingPair
{

    private :


        // -------------------- Attributes -------------------- //

        /// Set of pair of bodies that cannot collide between each other
        std::set<bodyindexpair>                        mNoCollisionPairs;

        /// Broad-phase overlapping pairs
        std::map<overlappingpairid, IOverlappingPair*> mOverlappingPairs;

        /// Real overlapping pairs
        std::map<overlappingpairid, IOverlappingPair*> mContactOverlappingPairs;

        /// Broad-phase algorithm
        IBroadPhase                                    mBroadPhaseAlgorithm;


        /// True if some collision shapes have been added previously
        bool mIsCollisionShapesAdded;



        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IContactManager(const IContactManager& collisionDetection);

        /// Private assignment operator
        IContactManager& operator=(const IContactManager& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();



         /// Compute the collision contact
        void computeContacts(const IProxyShape* ,const IProxyShape* , IOverlappingPair* ,IContactVertex* , u32 , bool);


        /// Add all the contact manifold of colliding pairs to their bodies
        void addAllContactManifoldsToBodies();


        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void addContactManifoldToBody(IOverlappingPair* pair);


        void broadPhaseNotifyOverlappingPair(void *_shape1 ,
                                             void *_shape2 );


    public :



        // -------------------- Methods -------------------- //

        /// Constructor
         IContactManager();

        /// Destructor
        ~IContactManager();



        /// Compute find contacts
        void FindNewContacts();



        /// Create point
        void createContact(IOverlappingPair* overlappingPair , IContactPoint *contact);

        /// Delete all the contact points in the currently overlapping pairs
        void clearContactPoints();


        /// Add a proxy collision shape to the collision detection
        void addProxyCollisionShape(IProxyShape* proxyShape, const IAABB& aabb);

        /// Remove a proxy collision shape from the collision detection
        void removeProxyCollisionShape(IProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void updateProxyCollisionShape(IProxyShape* shape, const IAABB& aabb,  const IVector3& displacement = IVector3(0, 0, 0) , bool forceReinsert = false );

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(ICollisionBody* body1, ICollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(ICollisionBody* body1, ICollisionBody* body2);


        /// Ask for a collision shape to be tested again during broad-phase.
        void askForBroadPhaseCollisionCheck(IProxyShape* shape);


        /// Ray casting method
        void raycast(IRaycastCallback* raycastCallback, const IRay& ray, unsigned short raycastWithCategoryMaskBits) const;


        // -------------------- Friendships -------------------- //

        friend class IDynamicsWorld;
        friend class ICollisionWorld;
        friend class IConvexShape;
        friend class IBroadPhase;

};


}

#endif // ICONTACTMANAGER_H
