#ifndef IGENERATIONCONTACTPOINTS_H
#define IGENERATIONCONTACTPOINTS_H

#include "../narrow_phase/ICollisionAlgorithm.h"
#include "../narrow_phase/ICollisionShapeInfo.h"

namespace IPhysics
{

struct IContactVertex
{
    IVector3 point1;
    IVector3 point2;
    IVector3 normal;
    scalar   penetration;


    //------------------ Constructor --------------------//
    /// Constructor
    IContactVertex(void){}

    /// Constructor
    IContactVertex(const IVector3& normal, scalar penetration,
                   const IVector3& Point1,
                   const IVector3& Point2)
        : normal(normal),
          penetration(penetration),
          point1(Point1),
          point2(Point2)
    {

    }

};




class IGenerationContactPoints
{

private:

    //-------------------- Attributes --------------------//

    IVector3  mCachedSeparatingAxis;

    const IProxyShape  *mShape1;
    const IProxyShape  *mShape2;

    ICollisionAlgorithm *mCollisionAlgorithm;

    //--------------- contact points ---------------------//
    i32                     mNbContactPoints;
    IContactVertex*         mContactPoints;


public:


    IGenerationContactPoints(const IProxyShape* shape1 ,
                             const IProxyShape* shape2 , ICollisionAlgorithm *CollisionAlgorithm)
        : mShape1(shape1) ,
          mShape2(shape2) ,
          mCollisionAlgorithm(CollisionAlgorithm),
          mCachedSeparatingAxis(0.0,1.0,0.0),
          mNbContactPoints(0)
    {

    }

    i32 getCountContactPoints() const
    {
        return mNbContactPoints;
    }

    IContactVertex *getContactPoints() const
    {
        return mContactPoints;
    }

    IVector3 getCachedSeparatingAxis() const
    {
        return mCachedSeparatingAxis;
    }

    void setCachedSeparatingAxis(const IVector3 &cachedSeparatingAxis)
    {
        mCachedSeparatingAxis = cachedSeparatingAxis;
    }


    bool ComputeCollisionAndFindContactPoints();

private:

    //***************************************************************//

        void addContact( const  IContactVertex& p_contact );
        void SwapFlip();

        /*****************************************************************
         * Method taken from the site :
         * http://www.gamedev.ru/code/articles/convex_collisions
         *****************************************************************/
        static IVector3 *CreateAxisPeturberationPoints(const IProxyShape* proxy_shape , const IVector3& xAxis ,  i32 &_NbPoints );

        //***************************************************************//

        bool FindPointsToContacts(const IVector3* SupportVertA, i32 iNumVertsA,
                                  const IVector3* SupportVertB, i32 iNumVertsB , const OutputCollisionInfo& info_collision, const IVector3 &_Normal ,
                                  const ICollisionShapeInfo &shape1Info,
                                  const ICollisionShapeInfo &shape2Info);

        //***************************************************************//

        void FaceToFaceContacts(  const IVector3* Poly , i32 iPolySize,  const IVector3* Clipper, i32 iClipperSize , const IVector3& _FaceNormal , bool flipNormal);
        void EdgeToEdgeContacts(  const IVector3& A0, const IVector3& A1, const IVector3& B0, const IVector3& B1 , const IVector3& _FaceNormal );
        void PointToEdgeContacts( const IVector3&  A, const IVector3& B0, const IVector3& B1 , const IVector3& _FaceNormal );
        void PointToFaceContacts( const IVector3&  A, const IVector3& xAxis, scalar bd , const IVector3& _FaceNormal );
        void PointToPointContacts(const IVector3&  A, const IVector3& B , const IVector3& _FaceNormal );

        //***************************************************************//



};


}

#endif // IGENERATIONCONTACTPOINTS_H
