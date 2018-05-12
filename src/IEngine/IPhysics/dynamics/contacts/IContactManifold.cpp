#include "IContactManifold.h"
#include "../../common/ISettings.h"

namespace IPhysics
{


IContactManifold::IContactManifold(IProxyShape* shape1, IProxyShape* shape2 , short int normalDirectionId)
: mShape1(shape1),
  mShape2(shape2),
  mNormalDirectionId(normalDirectionId),
  mNbContactPoints(0),
  mAccumulatedFrictionImpulse1(0.0),
  mAccumulatedFrictionImpulse2(0.0),
  mAccumulatedFrictionTwistImpulse(0.0),
  mAccumulatedRollingResistanceImpulse(IVector3(0,0,0)),
  mIsAlreadyInIsland(false)
{
  mNbContactPoints = 0;
}


// Destructor
IContactManifold::~IContactManifold()
{
    clear();
}

// Add a contact point in the manifold
void IContactManifold::addContactPoint(IContactPoint* contact)
{

    // For contact already in the manifold
    for (u32 i=0; i<mNbContactPoints; i++)
    {
        // Check if the new point point does not correspond to a same contact point
        // already in the manifold.
        scalar distance = (mContactPoints[i]->getWorldPointOnBody1() - contact->getWorldPointOnBody1()).lengthSquare();
        if (distance <= PERSISTENT_CONTACT_DIST_THRESHOLD*PERSISTENT_CONTACT_DIST_THRESHOLD)
        {
            // Delete the new contact
            delete contact;
            assert(mNbContactPoints > 0);

            return;
        }
    }

    // If the contact manifold is full
    if (mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD)
    {
        int indexMaxPenetration = getIndexOfDeepestPenetration(contact);
        int indexToRemove = getIndexToRemove(indexMaxPenetration, contact->getLocalPointOnBody1());
        removeContactPoint(indexToRemove);
    }

    // Add the new contact point in the manifold
    mContactPoints[mNbContactPoints] = contact;
    mNbContactPoints++;

    assert(mNbContactPoints > 0);

}

// Remove a contact point from the manifold
void IContactManifold::removeContactPoint(u32 index)
{
    assert(index < mNbContactPoints);
    assert(mNbContactPoints > 0);

    // Call the destructor explicitly and tell the memory allocator that
    // the corresponding memory block is now free
    delete mContactPoints[index];
           mContactPoints[index] = NULL;

    // If we don't remove the last index
    if (index < mNbContactPoints - 1)
    {
        mContactPoints[index] = mContactPoints[mNbContactPoints - 1];
    }

    mNbContactPoints--;
}



// Update the contact manifold
/// First the world space coordinates of the current contacts in the manifold are recomputed from
/// the corresponding transforms of the bodies because they have moved. Then we remove the contacts
/// with a negative penetration depth (meaning that the bodies are not penetrating anymore) and also
/// the contacts with a too large distance between the contact points in the plane orthogonal to the
/// contact normal.
void IContactManifold::update(const ITransform &transform1, const ITransform &transform2)
{

    if (mNbContactPoints == 0) return;


    // Update the world coordinates and penetration depth of the contact points in the manifold
    for (u32 i=0; i<mNbContactPoints; i++)
    {
        //mContactPoints[i]->setWorldPointOnBody1(transform1 * mContactPoints[i]->getLocalPointOnBody1());
        //mContactPoints[i]->setWorldPointOnBody2(transform2 * mContactPoints[i]->getLocalPointOnBody2());
//        mContactPoints[i]->setPenetration(( mContactPoints[i]->getWorldPointOnBody2() -
//                                            mContactPoints[i]->getWorldPointOnBody1()).dot(mContactPoints[i]->getNormal()));


    }


}

void IContactManifold::update_delete_not_uset_contact()
{
    /**
    for (u32 i=0; i<mNbContactPoints; i++)
    {
        mContactPoints[i]->setPenetration(( mContactPoints[i]->getWorldPointOnBody1() -
                                            mContactPoints[i]->getWorldPointOnBody2()).dot(mContactPoints[i]->getNormal()));

    }

    /**/
    const scalar squarePersistentContactThreshold = PERSISTENT_CONTACT_DIST_THRESHOLD * PERSISTENT_CONTACT_DIST_THRESHOLD;


    // Remove the contact points that don't represent very well the contact manifold
    for (int i=static_cast<int>(mNbContactPoints)-1; i>=0; i--)
    {
        assert(i < static_cast<int>(mNbContactPoints));

        // Compute the distance between contact points in the normal direction
        scalar distanceNormal = mContactPoints[i]->getPenetration();
        distanceNormal = (mContactPoints[i]->getWorldPointOnBody1() -
                          mContactPoints[i]->getWorldPointOnBody2()).dot(mContactPoints[i]->getNormal());



        // If the contacts points are too far from each other in the normal direction
        if (distanceNormal > squarePersistentContactThreshold)
        {
          // removeContactPoint(i);
        }
        else
        {
            // Compute the distance of the two contact points in the plane
            // orthogonal to the contact normal
            IVector3 projOfPoint1 = mContactPoints[i]->getWorldPointOnBody1() - mContactPoints[i]->getNormal() * distanceNormal;
            IVector3 projDifference = mContactPoints[i]->getWorldPointOnBody2() - projOfPoint1;

            // If the orthogonal distance is larger than the valid distance
            // threshold, we remove the contact
            if (projDifference.lengthSquare() > squarePersistentContactThreshold)
            {
                removeContactPoint(i);
            }
        }
    }
    /**


    for (u32 i=0; i<mNbContactPoints; i++)
    {
        mContactPoints[i]->setWorldPointOnBody1(mContactPoints[i]->getWorldPointOnBody2());
    }

    /**/
}

// Return the index of the contact point with the larger penetration depth.
/// This corresponding contact will be kept in the cache. The method returns -1 is
/// the new contact is the deepest.
u32 IContactManifold::getIndexOfDeepestPenetration(IContactPoint* newContact) const
{
    assert(mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);
    int indexMaxPenetrationDepth = -1;
    scalar maxPenetrationDepth = newContact->getPenetration();

    // For each contact in the cache
    for (u32 i=0; i<mNbContactPoints; i++)
    {
        // If the current contact has a larger penetration depth
        if (mContactPoints[i]->getPenetration() > maxPenetrationDepth)
        {
            maxPenetrationDepth = mContactPoints[i]->getPenetration();
            indexMaxPenetrationDepth = i;
        }
    }

    // Return the index of largest penetration depth
    return indexMaxPenetrationDepth;
}

// Return the index that will be removed.
/// The index of the contact point with the larger penetration
/// depth is given as a parameter. This contact won't be removed. Given this contact, we compute
/// the different area and we want to keep the contacts with the largest area. The new point is also
/// kept. In order to compute the area of a quadrilateral, we use the formula :
/// Area = 0.5 * | AC x BD | where AC and BD form the diagonals of the quadrilateral. Note that
/// when we compute this area, we do not calculate it exactly but we
/// only estimate it because we do not compute the actual diagonals of the quadrialteral. Therefore,
/// this is only a guess that is faster to compute. This idea comes from the Bullet Physics library
/// by Erwin Coumans (http://wwww.bulletphysics.org).
u32 IContactManifold::getIndexToRemove(u32 indexMaxPenetration, const IVector3& newPoint) const
{

    assert(mNbContactPoints == MAX_CONTACT_POINTS_IN_MANIFOLD);

    scalar area0 = 0.0;       // Area with contact 1,2,3 and newPoint
    scalar area1 = 0.0;       // Area with contact 0,2,3 and newPoint
    scalar area2 = 0.0;       // Area with contact 0,1,3 and newPoint
    scalar area3 = 0.0;       // Area with contact 0,1,2 and newPoint

    if (indexMaxPenetration != 0)
    {
        // Compute the area
        IVector3 vector1 = newPoint - mContactPoints[1]->getLocalPointOnBody1();
        IVector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[2]->getLocalPointOnBody1();

        IVector3 crossProduct = vector1.cross(vector2);
        area0 = crossProduct.lengthSquare();

    }

    if (indexMaxPenetration != 1)
    {
        // Compute the area
        IVector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        IVector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[2]->getLocalPointOnBody1();
        IVector3 crossProduct = vector1.cross(vector2);
        area1 = crossProduct.lengthSquare();

    }

    if (indexMaxPenetration != 2)
    {
        // Compute the area
        IVector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        IVector3 vector2 = mContactPoints[3]->getLocalPointOnBody1() - mContactPoints[1]->getLocalPointOnBody1();
        IVector3 crossProduct = vector1.cross(vector2);
        area2 = crossProduct.lengthSquare();

    }


    if (indexMaxPenetration != 3)
    {
        // Compute the area
        IVector3 vector1 = newPoint - mContactPoints[0]->getLocalPointOnBody1();
        IVector3 vector2 = mContactPoints[2]->getLocalPointOnBody1() -  mContactPoints[1]->getLocalPointOnBody1();
        IVector3 crossProduct = vector1.cross(vector2);
        area3 = crossProduct.lengthSquare();
    }

    // Return the index of the contact to remove
    return getMaxArea(area0, area1, area2, area3);
}

// Return the index of maximum area
u32 IContactManifold::getMaxArea(scalar area0, scalar area1, scalar area2, scalar area3) const
{
  if (area0 < area1)
  {
    if (area1 < area2)
    {
      if (area2 < area3) return 3;
            else return 2;
        }
        else
        {
            if (area1 < area3) return 3;
            else return 1;
        }
    }
    else
    {
        if (area0 < area2)
        {
            if (area2 < area3) return 3;
            else return 2;
        }
        else
        {
            if (area0 < area3) return 3;
            else return 0;
        }
    }
}

// Clear the contact manifold
void IContactManifold::clear()
{
    for ( u32 i=0; i<mNbContactPoints; i++)
    {
        // Call the destructor explicitly and tell the memory allocator that
        // the corresponding memory block is now free
        delete mContactPoints[i];
               mContactPoints[i] = NULL;
    }

    mNbContactPoints = 0;
}



}
