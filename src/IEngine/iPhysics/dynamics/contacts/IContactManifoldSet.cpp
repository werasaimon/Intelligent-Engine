#include "IContactManifoldSet.h"

namespace IPhysics
{

// Constructor
IContactManifoldSet::IContactManifoldSet(IProxyShape* shape1,
                                         IProxyShape* shape2,
                                         i32 nbMaxManifolds)
: mNbMaxManifolds(nbMaxManifolds),
  mNbManifolds(0),
  mShape1(shape1),
  mShape2(shape2)
{
    assert(nbMaxManifolds >= 1);
}

// Destructor
IContactManifoldSet::~IContactManifoldSet()
{
    // Clear all the contact manifolds
    clear();
}




// Add a contact point to the manifold set
void IContactManifoldSet::addContactPoint(IContactPoint* contact)
{

    // Compute an Id corresponding to the normal direction (using a cubemap)
    short int normalDirectionId = computeCubemapNormalId(contact->getNormal());

    // If there is no contact manifold yet
    if (mNbManifolds == 0)
    {

        createManifold(normalDirectionId);
        mManifolds[0]->addContactPoint(contact);
        assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
        for (i32 i=0; i<mNbManifolds; i++)
        {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }

        return;
    }

    // Select the manifold with the most similar normal (if exists)
    i32 similarManifoldIndex = 0;
    if (mNbMaxManifolds > 1)
    {
        similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
    }

    // If a similar manifold has been found
    if (similarManifoldIndex != -1)
    {

        // Add the contact point to that similar manifold
        mManifolds[similarManifoldIndex]->addContactPoint(contact);
        assert(mManifolds[similarManifoldIndex]->getNbContactPoints() > 0);

        return;
    }

    // If the maximum number of manifold has not been reached yet
    if (mNbManifolds < mNbMaxManifolds)
    {
        // Create a new manifold for the contact point
        createManifold(normalDirectionId);
        mManifolds[mNbManifolds-1]->addContactPoint(contact);
        for (i32 i=0; i<mNbManifolds; i++)
        {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }
        return;
    }

    // The contact point will be in a new contact manifold, we now have too much
    // manifolds condidates. We need to remove one. We choose to keep the manifolds
    // with the largest contact depth among their points
    i32 smallestDepthIndex = -1;
    scalar minDepth = contact->getPenetration();
    assert(mNbManifolds == mNbMaxManifolds);
    for (i32 i=0; i<mNbManifolds; i++)
    {
        scalar depth = mManifolds[i]->getLargestContactDepth();
        if (depth < minDepth)
        {
            minDepth = depth;
            smallestDepthIndex = i;
        }
    }

    // If we do not want to keep to new manifold (not created yet) with the
    // new contact point
    if (smallestDepthIndex == -1)
    {
        // Delete the new contact
        delete contact;
        return;
    }

    assert(smallestDepthIndex >= 0 && smallestDepthIndex < mNbManifolds);

    // Here we need to replace an existing manifold with a new one (that contains
    // the new contact point)
    removeManifold(smallestDepthIndex);
    createManifold(normalDirectionId);
    mManifolds[mNbManifolds-1]->addContactPoint(contact);
    assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
    for (i32 i=0; i<mNbManifolds; i++)
    {
        assert(mManifolds[i]->getNbContactPoints() > 0);
    }

    return;
}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
i32 IContactManifoldSet::selectManifoldWithSimilarNormal(short int normalDirectionId) const
{
    // Return the Id of the manifold with the same normal direction id (if exists)
    for (i32 i=0; i<mNbManifolds; i++)
    {
        if (normalDirectionId == mManifolds[i]->getNormalDirectionId())
        {
            return i;
        }
    }

    return -1;
}

// Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided into 4x4 buckets. This method maps the
// normal vector into of the of the bucket and returns a unique Id for the bucket
short int IContactManifoldSet::computeCubemapNormalId(const IVector3& normal) const
{

    assert(normal.lengthSquare() > MACHINE_EPSILON);

    i32 faceNo;
    scalar u, v;
    scalar max = IMax3(IAbs(normal.x), IAbs(normal.y), IAbs(normal.z));
    IVector3 normalScaled = normal / max;

    if (normalScaled.x >= normalScaled.y && normalScaled.x >= normalScaled.z)
    {
        faceNo = normalScaled.x > 0 ? 0 : 1;
        u = normalScaled.y;
        v = normalScaled.z;
    }
    else if (normalScaled.y >= normalScaled.x && normalScaled.y >= normalScaled.z)
    {
        faceNo = normalScaled.y > 0 ? 2 : 3;
        u = normalScaled.x;
        v = normalScaled.z;
    }
    else
    {
        faceNo = normalScaled.z > 0 ? 4 : 5;
        u = normalScaled.x;
        v = normalScaled.y;
    }

    i32 indexU = floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    i32 indexV = floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
    if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

    const i32 nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
    return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

// Update the contact manifolds
void IContactManifoldSet::update()
{
    for (i32 i=mNbManifolds-1; i>=0; i--)
    {
        // Update the contact manifold
        mManifolds[i]->update(mShape1->getWorldTransform(),
                              mShape2->getWorldTransform());

        // Remove the contact manifold if has no contact points anymore
        if (mManifolds[i]->getNbContactPoints() == 0)
        {
            removeManifold(i);
        }
    }
}


void IContactManifoldSet::update_delete_not_uset_contact()
{
    for (i32 i=mNbManifolds-1; i>=0; i--)
    {
        // Update the contact manifold
        mManifolds[i]->update_delete_not_uset_contact();

        // Remove the contact manifold if has no contact points anymore
        if (mManifolds[i]->getNbContactPoints() == 0)
        {
            removeManifold(i);
        }
    }
}



// Clear the contact manifold set
void IContactManifoldSet::clear()
{
    // Destroy all the contact manifolds
    for (i32 i=mNbManifolds-1; i>=0; i--)
    {
        removeManifold(i);
    }
    assert(mNbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void IContactManifoldSet::createManifold(short int normalDirectionId)
{
    assert(mNbManifolds < mNbMaxManifolds);

    mManifolds[mNbManifolds] = new IContactManifold(mShape1, mShape2 , normalDirectionId);
    mNbManifolds++;
}

// Remove a contact manifold from the set
void IContactManifoldSet::removeManifold(i32 index)
{
    assert(mNbManifolds > 0);
    assert(index >= 0 && index < mNbManifolds);

    // Delete the new contact
    delete mManifolds[index];

    for (i32 i=index; (i+1) < mNbManifolds; i++)
    {
        mManifolds[i] = mManifolds[i+1];
    }

    mNbManifolds--;
}



// Return the total number of contact points in the set of manifolds
SIMD_INLINE i32 IContactManifoldSet::getTotalNbContactPoints() const
{
    i32 nbPoints = 0;
    for (i32 i=0; i<mNbManifolds; i++)
    {
        nbPoints += mManifolds[i]->getNbContactPoints();
    }
    return nbPoints;
}

}
