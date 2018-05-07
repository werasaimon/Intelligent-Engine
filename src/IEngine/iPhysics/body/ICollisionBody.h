#ifndef ICOLLISIONBODY_H
#define ICOLLISIONBODY_H


#include "IBody.h"
#include "../common/math/IMatematical.h"
#include "../common/memory/IList.h"

namespace IPhysics
{



struct IRaycastInfo;
class  IAABB;
class  ICollisionShape;


// Class declarations
class   IProxyShape;
class   IContactManager;
class   IContactManifold;
class   ICollisionWorld;

typedef IListElement<IContactManifold> ContactManifoldListElement;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
enum BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class ICollisionBody : public IBody
{

    protected:


        //-------------------- Attributes --------------------//

        /// Type of body (static, kinematic or dynamic)
        BodyType                    mType;

        /// Position and orientation of the body
        ITransform                   mTransform;

        /// First element of the linked list of proxy collision shapes of this body
        IProxyShape*               mProxyCollisionShapes;

        /// Number of collision shapes
        u32                        mNbCollisionShapes;

        /// Collision detection object
        IContactManager           *mCollisionDetection;

        /// First element of the linked list of contact manifolds involving this body
        ContactManifoldListElement* mContactManifoldsList = NULL;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionBody(const ICollisionBody& body);

        /// Private assignment operator
        ICollisionBody& operator=(const ICollisionBody& body);

        /// Reset the contact manifold lists
        void resetContactManifoldsList();

        /// Remove all the collision shapes
        void removeAllCollisionShapes();

        /// Update the broad-phase state of a proxy collision shape of the body
        void updateProxyShapeInBroadPhase(IProxyShape* proxyShape, const IVector3& displacement , bool forceReinsert = false) const;

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

        /// Reset the mIsAlreadyInIsland variable of the body and contact manifolds
        i32 resetIsAlreadyInIslandAndCountManifolds();


    public:


        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionBody(const ITransform& transform, IContactManager *collide_meneger, bodyindex id );

        /// Destructor
        virtual ~ICollisionBody();

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        virtual void setType(BodyType type);

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return the current position and orientation
        const ITransform& getTransform() const;

        /// Set the current position and orientation
        virtual void setTransform(const ITransform& transform);

        /// Add a collision shape to the body.
        virtual IProxyShape* addCollisionShape(ICollisionShape* collisionShape, scalar massa , const ITransform& transform = ITransform::identity());

        /// Remove a collision shape from the body
        virtual void removeCollisionShape(const IProxyShape* proxyShape);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsList() const;

        /// Return true if a point is inside the collision body
        bool testPointInside(const IVector3& worldPoint) const;

        /// Raycast method with feedback information
        bool raycast(const IRay& ray, IRaycastInfo& raycastInfo);

        /// Compute and return the AABB of the body by merging all proxy shapes AABBs
        IAABB getAABB() const;

        /// Return the linked list of proxy shapes of that body
        IProxyShape* getProxyShapesList();

        /// Return the linked list of proxy shapes of that body
        const IProxyShape* getProxyShapesList() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        IVector3 getWorldPoint(const IVector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        IVector3 getWorldVector(const IVector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        IVector3 getLocalPoint(const IVector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        IVector3 getLocalVector(const IVector3& worldVector) const;


        //-------------------- Friendship --------------------//
        friend class ICollisionWorld;
        friend class IDynamicsWorld;
        friend class IContactManager;
        friend class IBroadPhase;
        friend class IConvexMeshShape;
        friend class IProxyShape;
        friend class IRigidBody;
        friend class IIsland;
        friend class IContactSolver;
};

// Return the type of the body
/**
 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
SIMD_INLINE BodyType ICollisionBody::getType() const
{
    return mType;
}

// Set the type of the body
/// The type of the body can either STATIC, KINEMATIC or DYNAMIC as described bellow:
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
/**
 * @param type The type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
SIMD_INLINE void ICollisionBody::setType(BodyType type)
{
    mType = type;

    if (mType == STATIC)
    {
        // Update the broad-phase state of the body
        updateBroadPhaseState();
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
SIMD_INLINE const ITransform& ICollisionBody::getTransform() const
{
    return mTransform;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
SIMD_INLINE void ICollisionBody::setTransform(const ITransform& transform)
{
    // Update the transform of the body
    mTransform = transform;

    // Update the broad-phase state of the body
    updateBroadPhaseState();

}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *         manifolds of this body
 */
SIMD_INLINE const ContactManifoldListElement* ICollisionBody::getContactManifoldsList() const
{
    return mContactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE IProxyShape* ICollisionBody::getProxyShapesList()
{
    return mProxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE const IProxyShape* ICollisionBody::getProxyShapesList() const
{
    return mProxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
SIMD_INLINE IVector3 ICollisionBody::getWorldPoint(const IVector3& localPoint) const
{
    return mTransform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
SIMD_INLINE IVector3 ICollisionBody::getWorldVector(const IVector3& localVector) const
{
    return mTransform.getBasis() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
SIMD_INLINE IVector3 ICollisionBody::getLocalPoint(const IVector3& worldPoint) const
{
    return mTransform.getInverse() * worldPoint;
}



// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
SIMD_INLINE IVector3 ICollisionBody::getLocalVector(const IVector3& worldVector) const
{
    return mTransform.getBasis().getInverse() * worldVector;
}



}

#endif // ICOLLISIONBODY_H
