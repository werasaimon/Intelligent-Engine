#ifndef ICOLLISIONBODY_H
#define ICOLLISIONBODY_H


#include "IBody.h"
#include "../ICommon/IMaths.h"
#include "../ICommon/IMemory/IList.h"

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
        ContactManifoldListElement* mContactManifoldsList = nullptr;


        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ICollisionBody(const ICollisionBody& body);

        /// Private assignment operator
        ICollisionBody& operator=(const ICollisionBody& body);

        /// Reset the contact manifold lists
        void ResetContactManifoldsList();

        /// Remove all the collision shapes
        void RemoveAllCollisionShapes();

        /// Update the broad-phase state of a proxy collision shape of the body
        void UpdateProxyShapeInBroadPhase(IProxyShape* proxyShape, const IVector3& displacement , bool forceReinsert = false) const;

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void UpdateBroadPhaseState() const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void AskForBroadPhaseCollisionCheck() const;

        /// Reset the mIsAlreadyInIsland variable of the body and contact manifolds
        i32 ResetIsAlreadyInIslandAndCountManifolds();


    public:


        // -------------------- Methods -------------------- //

        /// Constructor
        ICollisionBody(const ITransform& transform, IContactManager *collide_meneger, bodyindex id );

        /// Destructor
        virtual ~ICollisionBody();

        /// Return the type of the body
        BodyType GetType() const;

        /// Set the type of the body
        virtual void SetType(BodyType type);

        /// Set whether or not the body is active
        virtual void SetIsActive(bool isActive);

        /// Return the current position and orientation
        const ITransform& GetTransform() const;

        /// Set the current position and orientation
        virtual void SetTransform(const ITransform& transform);

        /// Add a collision shape to the body.
        virtual IProxyShape* AddCollisionShape(ICollisionShape* collisionShape, scalar massa , const ITransform& transform = ITransform::Identity());

        /// Remove a collision shape from the body
        virtual void RemoveCollisionShape(const IProxyShape* proxyShape);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* GetContactManifoldsList() const;

        /// Return true if a point is inside the collision body
        bool TestPointInside(const IVector3& worldPoint) const;

        /// Raycast method with feedback information
        bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo);

        /// Compute and return the AABB of the body by merging all proxy shapes AABBs
        IAABB GetAABB() const;

        /// Return the linked list of proxy shapes of that body
        IProxyShape* GetProxyShapesList();

        /// Return the linked list of proxy shapes of that body
        const IProxyShape* GetProxyShapesList() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        IVector3 GetWorldPoint(const IVector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        IVector3 GetWorldVector(const IVector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        IVector3 GetLocalPoint(const IVector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        IVector3 GetLocalVector(const IVector3& worldVector) const;


        //-------------------- Friendship --------------------//
        friend class ICollisionWorld;
        friend class IDynamicsWorld;
        friend class IDynamicsWorldRealtivity;
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
SIMD_INLINE BodyType ICollisionBody::GetType() const
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
SIMD_INLINE void ICollisionBody::SetType(BodyType type)
{
    mType = type;

    if (mType == STATIC)
    {
        // Update the broad-phase state of the body
        UpdateBroadPhaseState();
    }
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
SIMD_INLINE const ITransform& ICollisionBody::GetTransform() const
{
    return mTransform;
}

// Set the current position and orientation
/**
 * @param transform The transformation of the body that transforms the local-space
 *                  of the body into world-space
 */
SIMD_INLINE void ICollisionBody::SetTransform(const ITransform& transform)
{
    // Update the transform of the body
    mTransform = transform;

    // Update the broad-phase state of the body
    UpdateBroadPhaseState();

}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *         manifolds of this body
 */
SIMD_INLINE const ContactManifoldListElement* ICollisionBody::GetContactManifoldsList() const
{
    return mContactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE IProxyShape* ICollisionBody::GetProxyShapesList()
{
    return mProxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
SIMD_INLINE const IProxyShape* ICollisionBody::GetProxyShapesList() const
{
    return mProxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
SIMD_INLINE IVector3 ICollisionBody::GetWorldPoint(const IVector3& localPoint) const
{
    return mTransform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
SIMD_INLINE IVector3 ICollisionBody::GetWorldVector(const IVector3& localVector) const
{
    return mTransform.GetBasis() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
SIMD_INLINE IVector3 ICollisionBody::GetLocalPoint(const IVector3& worldPoint) const
{
    return mTransform.GetInverse() * worldPoint;
}



// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
SIMD_INLINE IVector3 ICollisionBody::GetLocalVector(const IVector3& worldVector) const
{
    return mTransform.GetBasis().GetInverse() * worldVector;
}



}

#endif // ICOLLISIONBODY_H
