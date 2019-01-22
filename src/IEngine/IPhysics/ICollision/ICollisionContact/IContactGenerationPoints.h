#ifndef ICONTACTGENERATIONPOINTS_H
#define ICONTACTGENERATIONPOINTS_H

#include "../ICollisionNarrowPhase/ICollisionAlgorithm.h"
#include "../ICollisionNarrowPhase/ICollisionShapeInfo.h"

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




class IContactGenerationPoints
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


    // -------------------- Methods -------------------- //

    /// Private copy-constructor
    IContactGenerationPoints(const IContactGenerationPoints& GenerationContactPoints);

    /// Private assignment operator
    IContactGenerationPoints& operator=(const IContactGenerationPoints& GenerationContactPoints);


public:


    IContactGenerationPoints(const IProxyShape* shape1 ,
                             const IProxyShape* shape2 , ICollisionAlgorithm *CollisionAlgorithm = nullptr)
        : mCachedSeparatingAxis(0.0,1.0,0.0),
          mShape1(shape1) ,
          mShape2(shape2) ,
          mCollisionAlgorithm(CollisionAlgorithm),
          mNbContactPoints(0)

    {

    }

   ~IContactGenerationPoints()
    {
        mNbContactPoints = 0;
        IFree(mContactPoints);
    }



    i32 GetCountContactPoints() const
    {
        return mNbContactPoints;
    }

    IContactVertex *GetContactPoints() const
    {
        return mContactPoints;
    }

    IVector3 GetCachedSeparatingAxis() const
    {
        return mCachedSeparatingAxis;
    }

    void SetCachedSeparatingAxis(const IVector3 &cachedSeparatingAxis)
    {
        mCachedSeparatingAxis = cachedSeparatingAxis;
    }


    bool ComputeCollisionAndFindContactPoints();

private:

    //***************************************************************//

        void AddContact( const  IContactVertex& p_contact );
        void SwapFlip();

        /*****************************************************************
         * Method taken from the site :
         * http://www.gamedev.ru/code/articles/convex_collisions
         *****************************************************************/
        static IVector3 *CreateAxisPeturberationPoints(const IProxyShape* proxy_shape , const IVector3& xAxis ,  i32 &_NbPoints );

        //***************************************************************//

        bool FindPointsToContacts(const IVector3* SupportVertA, i32 iNumVertsA,
                                  const IVector3* SupportVertB, i32 iNumVertsB , const IOutputCollisionInfo& info_collision, const IVector3 &_Normal ,
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

#endif // ICONTACTGENERATIONPOINTS_H
