#include "ICollisionBody.h"

#include "../collision/IProxyShape.h"
#include "../collision/IRaycastInfo.h"
#include "../collision/IAABB.h"
#include "../dynamics/IContactManager.h"


namespace IPhysics
{


// Constructor
/**
 * @param transform The transform of the body
 * @param world The physics world where the body is created
 * @param id ID of the body
 */
ICollisionBody::ICollisionBody(const ITransform &transform, IContactManager* collide_meneger , bodyindex id)
    : IBody(id),
      mType(DYNAMIC),
      mTransform(transform),
      mProxyCollisionShapes(NULL),
      mNbCollisionShapes(0) ,
      mCollisionDetection(collide_meneger) ,
      mContactManifoldsList(NULL)
{

}

// Destructor
ICollisionBody::~ICollisionBody()
{
    assert(mContactManifoldsList == NULL);
    // Remove all the proxy collision shapes of the body
    removeAllCollisionShapes();
}



// Add a collision shape to the body. Note that you can share a collision
// shape between several bodies using the same collision shape instance to
// when you add the shape to the different bodies. Do not forget to delete
// the collision shape you have created at the end of your program.
/// This method will return a pointer to a new proxy shape. A proxy shape is
/// an object that links a collision shape and a given body. You can use the
/// returned proxy shape to get and set information about the corresponding
/// collision shape for that body.
/**
 * @param collisionShape A pointer to the collision shape you want to add to the body
 * @param transform The transformation of the collision shape that transforms the
 *        local-space of the collision shape into the local-space of the body
 * @return A pointer to the proxy shape that has been created to link the body to
 *         the new collision shape you have added.
 */
IProxyShape* ICollisionBody::addCollisionShape(ICollisionShape* collisionShape, scalar massa , const ITransform &transform)
{
    // Create a new proxy collision shape to attach the collision shape to the body
    IProxyShape* proxyShape = new IProxyShape( this, collisionShape, transform , massa);

    // Add it to the list of proxy collision shapes of the body
    if (mProxyCollisionShapes == NULL)
    {
        mProxyCollisionShapes = proxyShape;
    }
    else
    {
        proxyShape->mNext = mProxyCollisionShapes;
        mProxyCollisionShapes = proxyShape;
    }
    mNbCollisionShapes++;


    /**********************************/
    // Compute the world-space AABB of the new collision shape
    IAABB aabb;
    collisionShape->computeAABB(aabb, mTransform , transform);

    // Notify the collision detection about this new collision shape
    mCollisionDetection->addProxyCollisionShape(proxyShape, aabb);
    /*********************************/

    // Return a pointer to the collision shape
    return proxyShape;
}



// Remove a collision shape from the body
/// To remove a collision shape, you need to specify the pointer to the proxy
/// shape that has been returned when you have added the collision shape to the
/// body
/**
 * @param proxyShape The pointer of the proxy shape you want to remove
 */
void ICollisionBody::removeCollisionShape(const IProxyShape* proxyShape)
{
  IProxyShape* current = mProxyCollisionShapes;

  // If the the first proxy shape is the one to remove
  if (current == proxyShape)
    {
      mProxyCollisionShapes = current->mNext;

      if (mIsActive)
      {
          mCollisionDetection->removeProxyCollisionShape(current);
      }

      current->~IProxyShape();
      delete current;

      mNbCollisionShapes--;
      return;
    }

  // Look for the proxy shape that contains the collision shape in parameter
  while(current->mNext != NULL)
    {
      // If we have found the collision shape to remove
      if (current->mNext == proxyShape)
        {

          // Remove the proxy collision shape
          IProxyShape* elementToRemove = current->mNext;
          current->mNext = elementToRemove->mNext;

          if (mIsActive)
          {
              mCollisionDetection->removeProxyCollisionShape(elementToRemove);
          }

          elementToRemove->~IProxyShape();
          delete elementToRemove;

          mNbCollisionShapes--;
          return;
        }

      // Get the next element in the list
      current = current->mNext;
    }
}


// Remove all the collision shapes
void ICollisionBody::removeAllCollisionShapes()
{
        IProxyShape* current = mProxyCollisionShapes;

        // Look for the proxy shape that contains the collision shape in parameter
        while(current != NULL)
        {

            // Remove the proxy collision shape
            IProxyShape* nextElement = current->mNext;

            if (mIsActive)
            {
               mCollisionDetection->removeProxyCollisionShape(current);
            }

            current->~IProxyShape();

            delete current->mCollisionShape;
            delete current;
            current = NULL;
            // Get the next element in the list
            current = nextElement;

        }

        mProxyCollisionShapes = NULL;
}


// Reset the contact manifold lists
void ICollisionBody::resetContactManifoldsList()
{
    // Delete the linked list of contact manifolds of that body
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != NULL)
    {
        ContactManifoldListElement* nextElement = currentElement->getNext();

        // Delete the current element
        delete currentElement;

        currentElement = nextElement;
    }
    mContactManifoldsList = NULL;
}



// Update the broad-phase state for this body (because it has moved for instance)
void ICollisionBody::updateBroadPhaseState() const
{
    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Update the proxy
        updateProxyShapeInBroadPhase( shape , IVector3(0,0,0) );
    }
}



// Update the broad-phase state of a proxy collision shape of the body
void ICollisionBody::updateProxyShapeInBroadPhase(IProxyShape* proxyShape, const IVector3& displacement, bool forceReinsert) const
{
    // Recompute the world-space AABB of the collision shape
    IAABB aabb;
    proxyShape->getCollisionShape()->computeAABB(aabb, mTransform , proxyShape->getLocalToBodyTransform() );


    // Update the broad-phase state for the proxy collision shape
    mCollisionDetection->updateProxyCollisionShape(proxyShape, aabb , displacement , forceReinsert);

}



// Ask the broad-phase to test again the collision shapes of the body for collision
// (as if the body has moved).
void ICollisionBody::askForBroadPhaseCollisionCheck() const
{
    // For all the proxy collision shapes of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
       mCollisionDetection->askForBroadPhaseCollisionCheck(shape);
    }
}



// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void ICollisionBody::setIsActive(bool isActive)
{

    // If the state does not change
    if (mIsActive == isActive) return;

    IBody::setIsActive(isActive);

    // If we have to activate the body
    if (isActive)
    {

        // For each proxy shape of the body
        for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
        {

            // Compute the world-space AABB of the new collision shape
            IAABB aabb;
            shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->mLocalToBodyTransform );

            // Add the proxy shape to the collision detection
            mCollisionDetection->addProxyCollisionShape(shape, aabb);
        }
    }
    else
    {  // If we have to deactivate the body

        // For each proxy shape of the body
        for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
        {
            // Remove the proxy shape from the collision detection
            mCollisionDetection->removeProxyCollisionShape(shape);
        }

        // Reset the contact manifold list of the body
        resetContactManifoldsList();
    }
}



//// Reset the mIsAlreadyInIsland variable of the body and contact manifolds.
///// This method also returns the number of contact manifolds of the body.
int ICollisionBody::resetIsAlreadyInIslandAndCountManifolds()
{

    mIsAlreadyInIsland = false;

    // Reset the mIsAlreadyInIsland variable of the contact manifolds for
    // this body
    int nbManifolds = 0;
    ContactManifoldListElement* currentElement = mContactManifoldsList;
    while (currentElement != NULL)
    {
        //currentElement->getPointer()->mIsAlreadyInIsland = false;
        currentElement = currentElement->getNext();
        nbManifolds++;
    }
    return nbManifolds;
}


// Return true if a point is inside the collision body
/// This method returns true if a point is inside any collision shape of the body
/**
 * @param worldPoint The point to test (in world-space coordinates)
 * @return True if the point is inside the body
 */
bool ICollisionBody::testPointInside(const IVector3& worldPoint) const
{
    // For each collision shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Test if the point is inside the collision shape
        if (shape->testPointInside(worldPoint)) return true;
    }

    return false;
}



// Raycast method with feedback information
/// The method returns the closest hit among all the collision shapes of the body
/**
* @param ray The ray used to raycast agains the body
* @param[out] raycastInfo Structure that contains the result of the raycasting
*                         (valid only if the method returned true)
* @return True if the ray hit the body and false otherwise
*/
bool ICollisionBody::raycast(const IRay& ray, IRaycastInfo& raycastInfo)
{
    // If the body is not active, it cannot be hit by rays
    if (!mIsActive) return false;

    bool isHit = false;
    IRay rayTemp(ray);


    // For each collision shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes; shape != NULL; shape = shape->mNext)
    {
        // Test if the ray hits the collision shape
        if (shape->raycast(rayTemp, raycastInfo))
        {
            rayTemp.maxFraction = raycastInfo.hitFraction;
            isHit = true;
        }
    }

    return isHit;
}




// Compute and return the AABB of the body by merging all proxy shapes AABBs
/**
* @return The axis-aligned bounding box (AABB) of the body in world-space coordinates
*/
IAABB ICollisionBody::getAABB() const
{
    IAABB bodyAABB;

    if (mProxyCollisionShapes == NULL) return bodyAABB;

     mProxyCollisionShapes->getCollisionShape()->computeAABB(bodyAABB, mTransform , mProxyCollisionShapes->getLocalToBodyTransform());

    // For each proxy shape of the body
    for (IProxyShape* shape = mProxyCollisionShapes->mNext; shape != NULL; shape = shape->mNext)
    {
        // Compute the world-space AABB of the collision shape
        IAABB aabb;
        shape->getCollisionShape()->computeAABB(aabb, mTransform , shape->getLocalToBodyTransform());

        // Merge the proxy shape AABB with the current body AABB
        bodyAABB.mergeWithAABB(aabb);
    }

    return bodyAABB;
}


}
