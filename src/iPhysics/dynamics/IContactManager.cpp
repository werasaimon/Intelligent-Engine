#include "IContactManager.h"
#include "../collision/broad_phase/IBroadPhase.h"
#include "../collision/narrow_phase/ICollisionAlgorithmGjkEpa.h"


//#include <iostream>
//using namespace std;

namespace IPhysics
{


IContactManager::IContactManager()
 : mBroadPhaseAlgorithm( this )
{

}


IContactManager::~IContactManager()
{
   mBroadPhaseAlgorithm.~IBroadPhase();
}


void IContactManager::FindNewContacts()
{
    // Compute the broad-phase collision detection
    computeBroadPhase();

    // Compute the narrow-phase collision detection
    computeNarrowPhase();
}


void IContactManager::computeBroadPhase()
{

    // If new collision shapes have been added to bodies
    if (mIsCollisionShapesAdded)
    {

        // Ask the broad-phase to recompute the overlapping pairs of collision
        // shapes. This call can only add new overlapping pairs in the collision
        // detection.
         mBroadPhaseAlgorithm.computeOverlappingPairs();
    }

}

void IContactManager::computeNarrowPhase()
{
    ///-----------------------------------///
    for( auto pair : mContactOverlappingPairs )
    {
        pair.second->isFakeCollision = true;
    }

    /*********************************
     * clear memory all pairs
     ********************************
    if(!mContactOverlappingPairs.empty())
    {
        for( auto pair : mContactOverlappingPairs )
        {
            delete pair.second;
        }

        mContactOverlappingPairs.clear();
    }
    /*********************************/


    i32 CollisionPairNbCount = 0;

    // For each possible collision pair of bodies
    // std::map<overlappingpairid, OverlappingPair*>::iterator it;
    for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
    {

        IOverlappingPair* pair = it->second;

        IProxyShape* shape1 = pair->getShape1();
        IProxyShape* shape2 = pair->getShape2();

        assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

        // Check if the collision filtering allows collision between the two shapes and
        // that the two shapes are still overlapping. Otherwise, we destroy the
        // overlapping pair
        if (((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0  ||
             (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) ||
             !mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2))
        {

            std::map<overlappingpairid, IOverlappingPair*>::iterator itToRemove = it;
            ++it;

            // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

            // Destroy the overlapping pair
            delete itToRemove->second;
            mOverlappingPairs.erase(itToRemove);

            continue;
        }
        else
        {
            ++it;
        }



        ICollisionBody* const body1 = shape1->getBody();
        ICollisionBody* const body2 = shape2->getBody();

        // Update the contact cache of the overlapping pair
        pair->update();

        // Check that at least one body is awake and not static
        bool isBody1Active = !body1->isSleeping() && body1->getType() != STATIC;
        bool isBody2Active = !body2->isSleeping() && body2->getType() != STATIC;
        if (!isBody1Active && !isBody2Active) continue;

        // Check if the bodies are in the set of bodies that cannot collide between each other
        bodyindexpair bodiesIndex = IOverlappingPair::computeBodiesIndexPair(body1, body2);
        if (mNoCollisionPairs.count(bodiesIndex) > 0) continue;

        CollisionPairNbCount++;




        /**********************************************************************/

        //rpCollisionAlgorithm* narrowPhaseAlgorithm = new  mCollisionMatrix[shape1Type][shape2Type];
        ICollisionAlgorithm* narrowPhaseAlgorithm = new  ICollisionAlgorithmGjkEpa;

        // If there is no collision algorithm between those two kinds of shapes
        if (narrowPhaseAlgorithm == NULL) continue;

        IGenerationContactPoints generate_contact( shape1 , shape2 , narrowPhaseAlgorithm );

        overlappingpairid pairId = IOverlappingPair::computeID(shape1,  shape2);

        if( generate_contact.ComputeCollisionAndFindContactPoints() && generate_contact.getContactPoints() != NULL )
        {

            bool isValidNewContactInfo = false;
            if( mContactOverlappingPairs.find(pairId) == mContactOverlappingPairs.end())
            {
                isValidNewContactInfo = true;
                ///Compute the maximum number of contact manifolds for this pair
                int nbMaxManifolds = ICollisionShape::computeNbMaxContactManifolds(shape1->getCollisionShape()->getType(),
                                                                                   shape2->getCollisionShape()->getType());

                mContactOverlappingPairs.insert( std::make_pair( pairId , new IOverlappingPair(shape1,shape2,nbMaxManifolds)) );
            }

            /// Update world transform to contscts
            mContactOverlappingPairs[pairId]->update();

            ///Compute contacts solve and warmsart c-store
            computeContacts( shape1 ,
                             shape2 ,
                             mContactOverlappingPairs[pairId] ,
                             generate_contact.getContactPoints() ,
                             generate_contact.getCountContactPoints() ,
                             isValidNewContactInfo);

             /// Delete non-used to contacts
             mContactOverlappingPairs[pairId]->update_delete_not_uset_contact();


            //mContactOverlappingPairs[pairId]->update();
            mContactOverlappingPairs[pairId]->setCachedSeparatingAxis(generate_contact.getCachedSeparatingAxis());
            mContactOverlappingPairs[pairId]->isFakeCollision = false;

        }


        delete narrowPhaseAlgorithm;

        /*********************************************************************/

    }

    // Delete Pairs
    for( auto pair : mContactOverlappingPairs )
    {
        if(pair.second->isFakeCollision)
        {
            delete pair.second;
            mContactOverlappingPairs.erase(pair.first);
        }
    }


    // Add all the contact manifolds (between colliding bodies) to the bodies
    addAllContactManifoldsToBodies();


}

void IContactManager::computeContacts(  const IProxyShape* Shape1 ,
                                        const IProxyShape* Shape2 ,
                                        IOverlappingPair*  OverlappingPair,
                                        IContactVertex* NewContactPoints ,
                                        u32 _NbNewContactPoints ,
                                        bool _isValidNewContactInfo )
{




      ITransform transform_1 = Shape1->getWorldTransform();
      ITransform transform_2 = Shape2->getWorldTransform();

       struct TempContactManiflod
       {
           IVector3 OldFriction1;
           IVector3 OldFriction2;

           scalar   AccumulatedFriction1Impulse;
           scalar   AccumulatedFriction2Impulse;
           scalar   AccumulatedFrictionTwistImpulse;
           IVector3 AccumulatedRollingResistanceImpulse;

       };


       const u32 NbCountStoreManiflod = OverlappingPair->getNbContactManifolds();
       TempContactManiflod storee[NbCountStoreManiflod];
       std::map<short int , int> mapIndexNormalId;
       for( u32 i = 0; i < NbCountStoreManiflod; ++i )
       {
           IContactManifold *m =  OverlappingPair->getContactManifoldSet().getContactManifold(i);

           mapIndexNormalId[m->getNormalDirectionId()] = i;

           storee[i].AccumulatedFriction1Impulse = m->getAccumulatedFrictionImpulse1();
           storee[i].AccumulatedFriction2Impulse = m->getAccumulatedFrictionImpulse2();
           storee[i].AccumulatedFrictionTwistImpulse = m->getAccumulatedFrictionTwistImpulse();
           storee[i].AccumulatedRollingResistanceImpulse = m->getAccumulatedRollingResistanceImpulse();

           storee[i].OldFriction1 = m->getFrictionVector1();
           storee[i].OldFriction2 = m->getFrictionVector2();
       }


       std::vector<IContactPoint> storyContacts = OverlappingPair->getContactPoints();
       OverlappingPair->clearContactPoints();


       const scalar MIN_EPS = 0.02;
       for (u32 i = 0; i < _NbNewContactPoints ; ++i)
       {

           /**
           glPushMatrix();
           glTranslatef(  NewContactPoints[i].mPositionA.x ,
                          NewContactPoints[i].mPositionA.y ,
                          NewContactPoints[i].mPositionA.z );
           glutWireSphere( 0.2 , 10 , 10);
           glPopMatrix();


           glPushMatrix();
           glTranslatef(  NewContactPoints[i].mPositionB.x ,
                          NewContactPoints[i].mPositionB.y ,
                          NewContactPoints[i].mPositionB.z );
           glutWireSphere( 0.2 , 10 , 10);
           glPopMatrix();
           /**/



           IVector3 prev_A =   NewContactPoints[i].point1;
           IVector3 prev_B =   NewContactPoints[i].point2;

           NewContactPoints[i].point1 = ((transform_1.getInverse() * NewContactPoints[i].point1));
           NewContactPoints[i].point2 = ((transform_2.getInverse() * NewContactPoints[i].point2));


           ///*************************************************************************************************///

           IContactPoint *contact = new IContactPoint( IContactPointInfo( NewContactPoints[i].normal      ,
                                                                          NewContactPoints[i].penetration ,
                                                                          NewContactPoints[i].point1   ,
                                                                          NewContactPoints[i].point2 ));

           contact->setWorldPointOnBody1(prev_A);
           contact->setWorldPointOnBody2(prev_B);

           contact->setIsRestingContact( /*!_isValidNewContactInfo*/ false);
           for( u32 it = 0; it < storyContacts.size() && !_isValidNewContactInfo; ++it )
           {

               if((contact->getWorldPointOnBody1() - transform_1 * storyContacts[it].getLocalPointOnBody1()).lengthSquare() <= MIN_EPS ||
                  (contact->getWorldPointOnBody2() - transform_2 * storyContacts[it].getLocalPointOnBody2()).lengthSquare() <= MIN_EPS)
                 {
                   // Get the cached accumulated impulses from the previous step
                   contact->setIsRestingContact( true );
                   contact->setAccumulatedPenetrationImpulse(storyContacts[it].getAccumulatedPenetrationImpulse());
                   contact->setAccumulatedFrictionImpulse1(storyContacts[it].getAccumulatedFrictionImpulse1());
                   contact->setAccumulatedFrictionImpulse2(storyContacts[it].getAccumulatedFrictionImpulse2());
                   contact->setAccumulatedRollingResistanceImpulse(storyContacts[it].getAccumulatedRollingResistanceImpulse());

                   contact->setFrictionVector1(storyContacts[it].getFrictionVector1());
                   contact->setFrictionVector2(storyContacts[it].getFrictionVector2());
                   break;
                 }
           }


           this->createContact( OverlappingPair , contact );
         }



       for( unsigned int i = 0; i < OverlappingPair->getNbContactManifolds() && !_isValidNewContactInfo; ++i )
       {
          IContactManifold *m =  OverlappingPair->getContactManifoldSet().getContactManifold(i);


          if(mapIndexNormalId.find(m->getNormalDirectionId()) != mapIndexNormalId.end())
          {
              int indexMap = mapIndexNormalId.find(m->getNormalDirectionId())->second;


                   m->setAccumulatedFrictionImpulse1( storee[indexMap].AccumulatedFriction1Impulse );
                   m->setAccumulatedFrictionImpulse2( storee[indexMap].AccumulatedFriction2Impulse );
                   m->setAccumulatedFrictionTwistImpulse( storee[indexMap].AccumulatedFrictionTwistImpulse );
                   m->setAccumulatedRollingResistanceImpulse( storee[indexMap].AccumulatedRollingResistanceImpulse );

                   m->setFrictionVector1( storee[indexMap].OldFriction1 );
                   m->setFrictionVector2( storee[indexMap].OldFriction2 );
          }

              m->setIsNewContactInfoManiflod(_isValidNewContactInfo);
       }

      mapIndexNormalId.clear();
      storyContacts.clear();

}

void IContactManager::addAllContactManifoldsToBodies()
{
    for (auto it = mContactOverlappingPairs.begin(); it != mContactOverlappingPairs.end(); ++it)
    {
          // Add all the contact manifolds of the pair into the list of contact manifolds
          // of the two bodies involved in the contact
          addContactManifoldToBody(it->second);
    }

}

void IContactManager::addContactManifoldToBody(IOverlappingPair *pair)
{
    assert(pair != NULL);

    ICollisionBody* body1 = pair->getShape1()->getBody();
    ICollisionBody* body2 = pair->getShape2()->getBody();
    const IContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    for (i32 i=0; i<manifoldSet.getNbContactManifolds(); i++)
    {

        IContactManifold* contactManifold = manifoldSet.getContactManifold(i);

        assert(contactManifold->getNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement *listElement1 = new ContactManifoldListElement( contactManifold , body1->mContactManifoldsList , NULL );
        body1->mContactManifoldsList = listElement1;


        // Add the contact manifold at the beginning of the linked
        // list of the contact manifolds of the second body
        ContactManifoldListElement *listElement2 = new ContactManifoldListElement( contactManifold , body2->mContactManifoldsList , NULL );
        body2->mContactManifoldsList = listElement2;

    }

}

void IContactManager::broadPhaseNotifyOverlappingPair(void *_shape1, void *_shape2)
{
    IProxyShape *shape1 = static_cast<IProxyShape*>(_shape1);
    IProxyShape *shape2 = static_cast<IProxyShape*>(_shape2);

    assert(shape1->mBroadPhaseID != shape2->mBroadPhaseID);

    // If the two proxy collision shapes are from the same body, skip it
    if (shape1->getBody()->getID() == shape2->getBody()->getID()) return;

    // Check if the collision filtering allows collision between the two shapes
    if ((shape1->getCollideWithMaskBits() & shape2->getCollisionCategoryBits()) == 0 ||
        (shape1->getCollisionCategoryBits() & shape2->getCollideWithMaskBits()) == 0) return;

    // Compute the overlapping pair ID
    overlappingpairid pairID = IOverlappingPair::computeID(shape1, shape2);

    // Check if the overlapping pair already exists
    if (mOverlappingPairs.find(pairID) != mOverlappingPairs.end()) return;


    // Create the overlapping pair and add it into the set of overlapping pairs
    IOverlappingPair* newPair = new IOverlappingPair(shape1, shape2);
    assert(newPair != NULL);

#ifndef NDEBUG
    std::pair<std::map<overlappingpairid, IOverlappingPair*>::iterator, bool> check =
#endif
    mOverlappingPairs.insert(make_pair(pairID, newPair));

    assert(check.second);

    // Wake up the two bodies
    shape1->getBody()->setIsSleeping(false);
    shape2->getBody()->setIsSleeping(false);

}



void IContactManager::createContact(IOverlappingPair *overlappingPair, IContactPoint *contact)
{
    // Add the contact to the contact manifold set of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add the overlapping pair into the set of pairs in contact during narrow-phase
    overlappingpairid pairId = IOverlappingPair::computeID(overlappingPair->getShape1(),
                                                           overlappingPair->getShape2());
    mContactOverlappingPairs[pairId] = overlappingPair;

}

/**
 * Delete all the contact points in the currently overlapping pairs
 **/
void IContactManager::clearContactPoints()
{
    // For each overlapping pair
     for (auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); ++it)
     {
         it->second->clearContactPoints();
     }
}

void IContactManager::addProxyCollisionShape(IProxyShape *proxyShape, const IAABB &aabb)
{
    // Add the body to the broad-phase
    mBroadPhaseAlgorithm.addProxyCollisionShape(proxyShape, aabb);
    mIsCollisionShapesAdded = true;

}

void IContactManager::removeProxyCollisionShape(IProxyShape *proxyShape)
{
    // Remove all the overlapping pairs involving this proxy shape
    for ( auto it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); )
    {
      if (it->second->getShape1()->mBroadPhaseID == proxyShape->mBroadPhaseID||
          it->second->getShape2()->mBroadPhaseID == proxyShape->mBroadPhaseID)
      {
          std::map<overlappingpairid, IOverlappingPair*>::iterator itToRemove = it;
          ++it;

          // TODO : Remove all the contact manifold of the overlapping pair from the contact manifolds list of the two bodies involved

          // Destroy the overlapping pair
          itToRemove->second->~IOverlappingPair();
          itToRemove->second->mContactManifoldSet.clear();

          // Destroy the overlapping pair
          delete itToRemove->second;
          mOverlappingPairs.erase(itToRemove);
      }
      else
      {
          ++it;
      }
    }

    // Remove the body from the broad-phase
    mBroadPhaseAlgorithm.removeProxyCollisionShape(proxyShape);
}

void IContactManager::updateProxyCollisionShape(IProxyShape *shape, const IAABB &aabb, const IVector3 &displacement, bool forceReinsert)
{
    mBroadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb, displacement , forceReinsert );
}

void IContactManager::addNoCollisionPair(ICollisionBody *body1, ICollisionBody *body2)
{
    mNoCollisionPairs.insert(IOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void IContactManager::removeNoCollisionPair(ICollisionBody *body1, ICollisionBody *body2)
{
    mNoCollisionPairs.erase(IOverlappingPair::computeBodiesIndexPair(body1, body2));
}

void IContactManager::askForBroadPhaseCollisionCheck(IProxyShape *shape)
{
    mBroadPhaseAlgorithm.addMovedCollisionShape(shape->mBroadPhaseID);
}

void IContactManager::raycast(IRaycastCallback *raycastCallback, const IRay &ray, unsigned short raycastWithCategoryMaskBits) const
{
    IRaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each proxy shape hit by the ray in the broad-phase
    mBroadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}





}
