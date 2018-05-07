#ifndef ICONTACTMANIFOLD_H
#define ICONTACTMANIFOLD_H


#include "IContactPoint.h"
#include "../../collision/IProxyShape.h"

namespace IPhysics
{


class IContactManifold
{

    private:

        //-------------------- Attributes --------------------//

        /// Pointer to the first proxy shape of the contact
        IProxyShape* mShape1;

        /// Pointer to the second proxy shape of the contact
        IProxyShape* mShape2;


        /// Number of contacts in the cache
        u32 mNbContactPoints;


        /// Contact points in the manifold
        IContactPoint* mContactPoints[MAX_CONTACT_POINTS_IN_MANIFOLD];


        /// Normal direction Id (Unique Id representing the normal direction)
        short int mNormalDirectionId;


        /// First new the contact info manifold
        bool mIsNewContactInfoManiflod;


        /// First friction vector of the contact manifold
        IVector3 mFrictionVector1;

        /// Second friction vector of the contact manifold
        IVector3 mFrictionVector2;



        /// First friction constraint accumulated impulse
        scalar  mAccumulatedFrictionImpulse1;

        /// Second friction constraint accumulated impulse
        scalar  mAccumulatedFrictionImpulse2;

        /// Twist friction constraint accumulated impulse
        scalar  mAccumulatedFrictionTwistImpulse;

        /// Accumulated rolling resistance impulse
        IVector3 mAccumulatedRollingResistanceImpulse;



        /// True if the contact manifold has already been added into an island
        bool mIsAlreadyInIsland;

        /// Extremal Penetration
        scalar mExtremalPenetration;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IContactManifold(const IContactManifold& contactManifold);

        /// Private assignment operator
        IContactManifold& operator=(const IContactManifold& contactManifold);

        /// Return the index of maximum area
        u32 getMaxArea(scalar area0, scalar area1, scalar area2, scalar area3) const;

        /// Return the index of the contact with the larger penetration depth.
        u32 getIndexOfDeepestPenetration(IContactPoint* newContact) const;

        /// Return the index that will be removed.
        u32 getIndexToRemove(u32 indexMaxPenetration, const IVector3& newPoint) const;

        /// Remove a contact point from the manifold
        void removeContactPoint(u32 index);

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
         IContactManifold( IProxyShape* shape1, IProxyShape* shape2 , short int normalDirectionId = 0);

        /// Destructor
        ~IContactManifold();


        /// Return a pointer to the first body of the contact manifold
        ICollisionBody* getBody1() const;

        /// Return a pointer to the second body of the contact manifold
        ICollisionBody* getBody2() const;


        /// Return a pointer to the first proxy shape of the contact manifold
        IProxyShape* getShape1() const;

        /// Return a pointer to the second proxy shape of the contact manifold
        IProxyShape* getShape2() const;




        /// Return the normal direction Id
        short int getNormalDirectionId() const;

        /// Add a contact point to the manifold
        void addContactPoint(IContactPoint* contact);


        /// Update the contact manifold.
        void update(const ITransform& transform1, const ITransform& transform2);

        /// Delete the contact manifold.
        void update_delete_not_uset_contact();


        /// Clear the contact manifold
        void clear();

        /// Return the number of contact points in the manifold
        u32 getNbContactPoints() const;

        /// Return the valid contact info manifold
        bool getIsNewContactInfoManiflod() const;

        /// Set the valid contact info manifold
        void setIsNewContactInfoManiflod(bool isNewContactInfoManiflod);

        /// Return the first friction vector at the center of the contact manifold
        const IVector3& getFrictionVector1() const;

        /// set the first friction vector at the center of the contact manifold
        void setFrictionVector1(const IVector3& mFrictionVector1);

        /// Return the second friction vector at the center of the contact manifold
        const IVector3& getFrictionVector2() const;

        /// set the second friction vector at the center of the contact manifold
        void setFrictionVector2(const IVector3& mFrictionVector2);





        /// Return the first friction accumulated impulse
        scalar getAccumulatedFrictionImpulse1() const;

        /// Set the first friction accumulated impulse
        void setAccumulatedFrictionImpulse1(scalar frictionImpulse1);



        /// Return the second friction accumulated impulse
        scalar getAccumulatedFrictionImpulse2() const;

        /// Set the second friction accumulated impulse
        void setAccumulatedFrictionImpulse2(scalar frictionImpulse2);



        /// Return the friction twist accumulated impulse
        scalar getAccumulatedFrictionTwistImpulse() const;

        /// Set the friction twist accumulated impulse
        void setAccumulatedFrictionTwistImpulse(scalar frictionTwistImpulse);



        /// Set the accumulated rolling resistance impulse
        IVector3 getAccumulatedRollingResistanceImpulse() const;


        /// Set the accumulated rolling resistance impulse
        void setAccumulatedRollingResistanceImpulse(const IVector3& rollingResistanceImpulse);




        /// Set the accumulated rolling resistance split impulse
        IVector3 getAccumulatedRollingResistanceSplitImpulse() const;


        /// Set the accumulated rolling resistance split impulse
        void setAccumulatedRollingResistanceSplitImpulse(const IVector3& rollingResistanceImpulse);




        /// Return a contact point of the manifold
        IContactPoint* getContactPoint(u32 index) const;

        /// Return the normalized averaged normal vector
        IVector3 getAverageContactNormal() const;

        /// Return the largest depth of all the contact points
        scalar getLargestContactDepth() const;





        // -------------------- Friendship -------------------- //
        friend class IContactManifoldSet;
        friend class IDynamicsWorld;
        friend class IIsland;
        friend class ICollisionBody;

};


// Return the valid contact info manifold
SIMD_INLINE bool IContactManifold::getIsNewContactInfoManiflod() const
{
    return mIsNewContactInfoManiflod;
}


// Set the valid contact info manifold
SIMD_INLINE void IContactManifold::setIsNewContactInfoManiflod(bool isNewContactInfoManiflod)
{
    mIsNewContactInfoManiflod = isNewContactInfoManiflod;
}


// Return a pointer to the first proxy shape of the contact
SIMD_INLINE IProxyShape *IContactManifold::getShape1() const
{
    return mShape1;
}

// Return a pointer to the second proxy shape of the contact
SIMD_INLINE IProxyShape *IContactManifold::getShape2() const
{
    return mShape2;
}

// Return a pointer to the first body of the contact manifold
SIMD_INLINE ICollisionBody* IContactManifold::getBody1() const
{
    return mShape1->getBody();
}

// Return a pointer to the second body of the contact manifold
SIMD_INLINE ICollisionBody* IContactManifold::getBody2() const
{
    return mShape2->getBody();
}




// Return the normal direction Id
SIMD_INLINE short int IContactManifold::getNormalDirectionId() const
{
    return mNormalDirectionId;
}

// Return the number of contact points in the manifold
SIMD_INLINE u32 IContactManifold::getNbContactPoints() const
{
    return mNbContactPoints;
}




// Return the first friction vector at the center of the contact manifold
SIMD_INLINE const IVector3& IContactManifold::getFrictionVector1() const
{
    return mFrictionVector1;
}

// set the first friction vector at the center of the contact manifold
SIMD_INLINE void IContactManifold::setFrictionVector1(const IVector3& frictionVector1)
{
    mFrictionVector1 = frictionVector1;
}


// Return the second friction vector at the center of the contact manifold
SIMD_INLINE const IVector3& IContactManifold::getFrictionVector2() const
{
    return mFrictionVector2;
}

// set the second friction vector at the center of the contact manifold
SIMD_INLINE void IContactManifold::setFrictionVector2(const IVector3& frictionVector2)
{
    mFrictionVector2 = frictionVector2;
}




// Return the first friction accumulated impulse
SIMD_INLINE scalar IContactManifold::getAccumulatedFrictionImpulse1() const
{
    return mAccumulatedFrictionImpulse1;
}

// Set the first friction accumulated impulse
SIMD_INLINE void IContactManifold::setAccumulatedFrictionImpulse1(scalar frictionImpulse1)
{
    mAccumulatedFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
SIMD_INLINE scalar IContactManifold::getAccumulatedFrictionImpulse2() const
{
    return mAccumulatedFrictionImpulse2;
}

// Set the second friction accumulated impulse
SIMD_INLINE void IContactManifold::setAccumulatedFrictionImpulse2(scalar frictionImpulse2)
{
    mAccumulatedFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
SIMD_INLINE scalar IContactManifold::getAccumulatedFrictionTwistImpulse() const
{
    return mAccumulatedFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
SIMD_INLINE void IContactManifold::setAccumulatedFrictionTwistImpulse(scalar frictionTwistImpulse)
{
    mAccumulatedFrictionTwistImpulse = frictionTwistImpulse;
}

// Return accumulated rolling resistance impulse
SIMD_INLINE IVector3 IContactManifold::getAccumulatedRollingResistanceImpulse() const
{
    return mAccumulatedRollingResistanceImpulse;
}

// Set the accumulated rolling resistance impulse
SIMD_INLINE void IContactManifold::setAccumulatedRollingResistanceImpulse(const IVector3& rollingResistanceImpulse)
{
    mAccumulatedRollingResistanceImpulse = rollingResistanceImpulse;
}


// Return a contact point of the manifold
SIMD_INLINE IContactPoint* IContactManifold::getContactPoint(u32 index) const
{
    assert(index < mNbContactPoints);
    return mContactPoints[index];
}

// Return true if the contact manifold has already been added into an island
SIMD_INLINE bool IContactManifold::isAlreadyInIsland() const
{
    return mIsAlreadyInIsland;
}

// Return the normalized averaged normal vector
SIMD_INLINE IVector3 IContactManifold::getAverageContactNormal() const
{
    IVector3 averageNormal;

    for (u32 i=0; i<mNbContactPoints; i++)
    {
        averageNormal += mContactPoints[i]->getNormal();
    }

    return averageNormal.getUnit();
}

// Return the largest depth of all the contact points
SIMD_INLINE scalar IContactManifold::getLargestContactDepth() const
{
    scalar largestDepth = 0.0f;

    for (u32 i=0; i<mNbContactPoints; i++)
    {
        scalar depth = mContactPoints[i]->getPenetration();
        if (depth > largestDepth)
        {
            largestDepth = depth;
        }
    }

    return largestDepth;
}


}

#endif // ICONTACTMANIFOLD_H
