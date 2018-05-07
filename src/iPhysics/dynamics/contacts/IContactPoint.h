#ifndef ICONTACTPOINT_H
#define ICONTACTPOINT_H

#include "../../common/math/IMatematical.h"

namespace IPhysics
{


// Structure ContactPointInfo
/**
 * This structure contains informations about a collision contact
 * computed during the narrow-phase collision detection. Those
 * informations are used to compute the contact set for a contact
 * between two bodies.
 */
struct IContactPointInfo
{

    private:

        //-------------------- Methods --------------------//

    public:


        //-------------------- Attributes -----------------//

        /// Normalized normal vector of the collision contact in world space
        IVector3  normal;

        /// Penetration depth of the contact
        scalar   penetration;

        /// Contact point of body 1 in local space of body 1
        IVector3  localPoint1;

        /// Contact point of body 2 in local space of body 2
        IVector3  localPoint2;

        //-------------------- Methods --------------------//

        IContactPointInfo(void){}

        /// Constructor
        IContactPointInfo(const IVector3& normal, scalar penetration,
                           const IVector3& localPoint1,
                           const IVector3& localPoint2)
            : normal(normal),
              penetration(penetration),
              localPoint1(localPoint1),
              localPoint2(localPoint2)
        {

        }
};


// Class ContactPoint
/**
 * This class represents a collision contact point between two
 * bodies in the physics engine.
 */
class IContactPoint
{

    private :

        // -------------------- Attributes -------------------- //

        /// Normalized normal vector of the contact (from body1 toward body2) in world space
        IVector3 mNormal;

        /// Contact point on body 1 in local space of body 1
        IVector3 mLocalPointOnBody1;

        /// Contact point on body 2 in local space of body 2
        IVector3 mLocalPointOnBody2;

        /// Contact point on body 1 in world space
        IVector3 mWorldPointOnBody1;

        /// Contact point on body 2 in world space
        IVector3 mWorldPointOnBody2;

        /// True if the contact is a resting contact (exists for more than one time step)
        bool mIsRestingContact;

        /// Penetration depth
        scalar  mPenetration;

        /// Two orthogonal vectors that span the tangential friction plane
        IVector3 mFrictionVectors[2];

        /// Cached penetration impulse
        scalar  mAccumulatedPenetrationImpulse;

        /// Cached first friction impulse
        scalar  mAccumulatedFrictionImpulse1;

        /// Cached second friction impulse
        scalar  mAccumulatedFrictionImpulse2;

        /// Cached rolling resistance impulse
        IVector3 mAccumulatedRollingResistanceImpulse;



        //-------------------- Methods --------------------//

//        // Private copy-constructor
//        IContactPoint(const IContactPoint& contact);

//        // Private assignment operator
//        IContactPoint& operator=(const IContactPoint& contact);

    public :



        // -------------------- Methods -------------------- //

         /// Constructor
         IContactPoint(){}

        /// Constructor
         IContactPoint(const IContactPointInfo &contactInfo);

        /// Destructor
        ~IContactPoint();


         void setContactInfo( const IContactPointInfo &contactInfo )
         {
              mNormal            = contactInfo.normal;
              mPenetration       = contactInfo.penetration;
              mLocalPointOnBody1 = contactInfo.localPoint1;
              mLocalPointOnBody2 = contactInfo.localPoint2;
              mWorldPointOnBody1 = contactInfo.localPoint1;
              mWorldPointOnBody2 = contactInfo.localPoint2;
         }


         /// Set the penetration depth of the contact
         void setPenetration(scalar penetration);

        /// Return the penetration depth of the contact
        scalar getPenetration() const;


        /// Return the normal vector of the contact
        IVector3 getNormal() const;

        /// Return the contact local point on body 1
        IVector3 getLocalPointOnBody1() const;

        /// Return the contact local point on body 2
        IVector3 getLocalPointOnBody2() const;

        /// Return the contact world point on body 1
        IVector3 getWorldPointOnBody1() const;

        /// Return the contact world point on body 2
        IVector3 getWorldPointOnBody2() const;


        /// Return the cached penetration impulse
        scalar getAccumulatedPenetrationImpulse() const;

        /// Return the cached first friction impulse
        scalar getAccumulatedFrictionImpulse1() const;

        /// Return the cached second friction impulse
        scalar getAccumulatedFrictionImpulse2() const;

        /// Return the cached rolling resistance impulse
        IVector3 getAccumulatedRollingResistanceImpulse() const;


        /// Set the cached penetration impulse
        void setAccumulatedPenetrationImpulse(scalar impulse);

        /// Set the first cached friction impulse
        void setAccumulatedFrictionImpulse1(scalar impulse);

        /// Set the second cached friction impulse
        void setAccumulatedFrictionImpulse2(scalar impulse);

        /// Set the cached rolling resistance impulse
        void setAccumulatedRollingResistanceImpulse(const IVector3& impulse);



        /// Set the contact local point on body 1
        void setLocalPointOnBody1(const IVector3& localPoint);

        /// Set the contact local point on body 2
        void setLocalPointOnBody2(const IVector3& localPoint);

        /// Set the contact world point on body 1
        void setWorldPointOnBody1(const IVector3& worldPoint);

        /// Set the contact world point on body 2
        void setWorldPointOnBody2(const IVector3& worldPoint);





        /// Return true if the contact is a resting contact
        bool getIsRestingContact() const;

        /// Set the mIsRestingContact variable
        void setIsRestingContact(bool isRestingContact);

        /// Get the first friction vector
        IVector3 getFrictionVector1() const;

        /// Set the first friction vector
        void setFrictionVector1(const IVector3& frictionVector1);

        /// Get the second friction vector
        IVector3 getFrictionVector2() const;

        /// Set the second friction vector
        void setFrictionVector2(const IVector3& frictionVector2);


        /// Return the number of bytes used by the contact point
        size_t getSizeInBytes() const;


        //-------------------- Friendships --------------------//


};



// Return the normal vector of the contact
SIMD_INLINE IVector3 IContactPoint::getNormal() const
{
    return mNormal;
}

// Set the penetration depth of the contact
SIMD_INLINE void IContactPoint::setPenetration(scalar penetration)
{
    this->mPenetration = penetration;
}


// Return the contact point on body 1
SIMD_INLINE IVector3 IContactPoint::getLocalPointOnBody1() const
{
    return mLocalPointOnBody1;
}

// Return the contact point on body 2
SIMD_INLINE IVector3 IContactPoint::getLocalPointOnBody2() const
{
    return mLocalPointOnBody2;
}

// Return the contact world point on body 1
SIMD_INLINE IVector3 IContactPoint::getWorldPointOnBody1() const
{
    return mWorldPointOnBody1;
}

// Return the contact world point on body 2
SIMD_INLINE IVector3 IContactPoint::getWorldPointOnBody2() const
{
    return mWorldPointOnBody2;
}




// Return the cached penetration impulse
SIMD_INLINE scalar IContactPoint::getAccumulatedPenetrationImpulse() const
{
    return mAccumulatedPenetrationImpulse;
}

// Return the cached first friction impulse
SIMD_INLINE scalar IContactPoint::getAccumulatedFrictionImpulse1() const
{
    return mAccumulatedFrictionImpulse1;
}

// Return the cached second friction impulse
SIMD_INLINE scalar IContactPoint::getAccumulatedFrictionImpulse2() const
{
    return mAccumulatedFrictionImpulse2;
}

// Return the cached rolling resistance impulse
SIMD_INLINE IVector3 IContactPoint::getAccumulatedRollingResistanceImpulse() const
{
    return mAccumulatedRollingResistanceImpulse;
}



// Set the cached penetration impulse
SIMD_INLINE void IContactPoint::setAccumulatedPenetrationImpulse(scalar impulse)
{
    mAccumulatedPenetrationImpulse = impulse;
}

// Set the first cached friction impulse
SIMD_INLINE void IContactPoint::setAccumulatedFrictionImpulse1(scalar impulse)
{
    mAccumulatedFrictionImpulse1 = impulse;
}

// Set the second cached friction impulse
SIMD_INLINE void IContactPoint::setAccumulatedFrictionImpulse2(scalar impulse)
{
    mAccumulatedFrictionImpulse2 = impulse;
}

// Set the cached rolling resistance impulse
SIMD_INLINE void IContactPoint::setAccumulatedRollingResistanceImpulse(const IVector3& impulse)
{
    mAccumulatedRollingResistanceImpulse = impulse;
}


// Set the contact local point on body 1
SIMD_INLINE void IContactPoint::setLocalPointOnBody1(const IVector3 &localPoint)
{
   mLocalPointOnBody1 = localPoint;
}

// Set the contact local point on body 2
SIMD_INLINE void IContactPoint::setLocalPointOnBody2(const IVector3 &localPoint)
{
   mLocalPointOnBody2 = localPoint;
}


// Set the contact world point on body 1
SIMD_INLINE void IContactPoint::setWorldPointOnBody1(const IVector3& worldPoint)
{
    mWorldPointOnBody1 = worldPoint;
}

// Set the contact world point on body 2
SIMD_INLINE void IContactPoint::setWorldPointOnBody2(const IVector3& worldPoint)
{
    mWorldPointOnBody2 = worldPoint;
}

// Return true if the contact is a resting contact
SIMD_INLINE bool IContactPoint::getIsRestingContact() const
{
    return mIsRestingContact;
}

// Set the mIsRestingContact variable
SIMD_INLINE void IContactPoint::setIsRestingContact(bool isRestingContact)
{
    mIsRestingContact = isRestingContact;
}

// Get the first friction vector
SIMD_INLINE IVector3 IContactPoint::getFrictionVector1() const
{
    return mFrictionVectors[0];
}

// Set the first friction vector
SIMD_INLINE void IContactPoint::setFrictionVector1(const IVector3& frictionVector1)
{
    mFrictionVectors[0] = frictionVector1;
}

// Get the second friction vector
SIMD_INLINE IVector3 IContactPoint::getFrictionVector2() const
{
    return mFrictionVectors[1];
}

// Set the second friction vector
SIMD_INLINE void IContactPoint::setFrictionVector2(const IVector3& frictionVector2)
{
    mFrictionVectors[1] = frictionVector2;
}

// Return the penetration depth of the contact
SIMD_INLINE scalar IContactPoint::getPenetration() const
{
    return mPenetration;
}

// Return the number of bytes used by the contact point
SIMD_INLINE size_t IContactPoint::getSizeInBytes() const
{
    return sizeof(IContactPoint);
}

}

#endif // ICONTACTPOINT_H
