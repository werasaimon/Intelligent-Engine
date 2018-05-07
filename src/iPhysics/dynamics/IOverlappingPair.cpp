#include "IOverlappingPair.h"

namespace IPhysics
{


// Constructor
IOverlappingPair::IOverlappingPair(IProxyShape* shape1, IProxyShape* shape2, i32 nbMaxContactManifolds)
: mShape1(shape1) ,
  mShape2(shape2) ,
  mContactManifoldSet(shape1, shape2, nbMaxContactManifolds)
{

}

// Destructor
IOverlappingPair::~IOverlappingPair()
{
    mContactManifoldSet.clear();
}

/// Add new contact
void IOverlappingPair::addContact(IContactPoint* contact)
{
     mContactManifoldSet.addContactPoint(contact);
}

/// Update of repair delete contact
void IOverlappingPair::update()
{
    mContactManifoldSet.update();
}

void IOverlappingPair::update_delete_not_uset_contact()
{
   mContactManifoldSet.update_delete_not_uset_contact();
}

///Clear all contact
void IOverlappingPair::clearContactPoints()
{
    mContactManifoldSet.clear();
}


}
