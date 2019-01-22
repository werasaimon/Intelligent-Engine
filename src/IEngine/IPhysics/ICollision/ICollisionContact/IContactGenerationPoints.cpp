#include "IContactGenerationPoints.h"
#include "../../IGeometry/IQuickClipping.h"

namespace IPhysics
{

const scalar squarePersistentContactThreshold = PERSISTENT_CONTACT_DIST_THRESHOLD * PERSISTENT_CONTACT_DIST_THRESHOLD;


bool IContactGenerationPoints::ComputeCollisionAndFindContactPoints()
{

    const ICollisionShapeInfo shape1Info( mShape1->GetCollisionShape() , mShape1->GetWorldTransform() , nullptr);
    const ICollisionShapeInfo shape2Info( mShape2->GetCollisionShape() , mShape2->GetWorldTransform() , nullptr);

    mNbContactPoints = 0;
    mContactPoints = nullptr;
    mCollisionAlgorithm->mOutputCollisionInfo.Normal = mCachedSeparatingAxis;

    if ( mCollisionAlgorithm->TestCollision(shape2Info ,shape1Info) )
    {
        IOutputCollisionInfo info_collision = mCollisionAlgorithm->GetOutputCollisionInfo();

        const IVector3& SeparatonNormal = info_collision.Normal.GetUnit();

        mCachedSeparatingAxis = SeparatonNormal;


        /**/
        IVector3 SeparatonNormalA =  SeparatonNormal;
        IVector3 SeparatonNormalB = -SeparatonNormal;

        i32 iNumVertsA = 0;
        i32 iNumVertsB = 0;
        IVector3* SupportVertA = nullptr;
        IVector3* SupportVertB = nullptr;

        SupportVertA = CreateAxisPeturberationPoints(mShape1 , SeparatonNormalA , iNumVertsA );
        SupportVertB = CreateAxisPeturberationPoints(mShape2 , SeparatonNormalB , iNumVertsB );
        /**/

        // Find contact points
        bool isCollide = FindPointsToContacts(SupportVertA , iNumVertsA ,
                                              SupportVertB , iNumVertsB ,
                                              info_collision ,
                                              SeparatonNormal ,
                                              shape1Info ,
                                              shape2Info);


        IFree(SupportVertA);
        IFree(SupportVertB);

        return isCollide;

    }

    return false;
}


bool IContactGenerationPoints::FindPointsToContacts(const IVector3 *SupportVertA, i32 iNumVertsA,
                                                    const IVector3 *SupportVertB, i32 iNumVertsB,
                                                    const IOutputCollisionInfo &info_collision, const IVector3 &_Normal,
                                                    const ICollisionShapeInfo &shape1Info,
                                                    const ICollisionShapeInfo &shape2Info)
{

    if (iNumVertsA == 0 || iNumVertsB == 0)
    return false;

    mNbContactPoints = 0;
    mContactPoints = nullptr;

    const IVector3 SeparatonNormal = _Normal.GetUnit();

    /**/

    if (iNumVertsA == 1)
    {
        if (iNumVertsB == 1)
        {
            PointToPointContacts(info_collision.Point1, info_collision.Point2 , SeparatonNormal);
        }
        else if (iNumVertsB == 2)
        {
            PointToEdgeContacts(SupportVertA[0], SupportVertB[0],SupportVertB[1] , SeparatonNormal);
        }
        else
        {
            IVector3 Bn = IVector3::triNormal(SupportVertB[0],
                                              SupportVertB[1],
                                              SupportVertB[2]);

            scalar bd = Bn.Dot(SupportVertB[0]);

            PointToFaceContacts(SupportVertA[0], Bn, bd , SeparatonNormal);
        }
    }
    else if (iNumVertsA == 2)
    {
        if (iNumVertsB == 1)
        {
            PointToEdgeContacts(SupportVertB[0],
                                SupportVertA[0],
                                SupportVertA[1],
                                SeparatonNormal);

        }
        else if (iNumVertsB == 2)
        {
            EdgeToEdgeContacts(SupportVertA[0], SupportVertA[1],
                               SupportVertB[0], SupportVertB[1],
                               SeparatonNormal);

        }
        else
        {
           FaceToFaceContacts(SupportVertB, iNumVertsB, SupportVertA, iNumVertsA ,SeparatonNormal , true);
           SwapFlip();

        }
      }
    else if (iNumVertsA >= 3)
    {

        if (iNumVertsB == 1)
        {
            IVector3 An = IVector3::triNormal(SupportVertA[0],
                                              SupportVertA[1],
                                              SupportVertA[2]);
            scalar bd = An.Dot(SupportVertA[0]);
            PointToFaceContacts(SupportVertB[0], An, bd , SeparatonNormal);
        }
        else if (iNumVertsB == 2)
        {
            FaceToFaceContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB , SeparatonNormal , false);
        }
        else if (iNumVertsB >= 3)
        {

            IPlane PlaneA(SupportVertA[0],SupportVertA[1], SupportVertA[2]);
            IPlane PlaneB(SupportVertB[0],SupportVertB[1], SupportVertB[2]);

            IVector3 SupportMinmum2 = shape2Info.GetWorldSupportPointWithMargin(-PlaneA.GetNormal());
            IVector3 SupportMinmum1 = shape1Info.GetWorldSupportPointWithMargin(-PlaneB.GetNormal());

            scalar distance1 = IPlane::Distance(PlaneA,SupportMinmum2);
            scalar distance2 = IPlane::Distance(PlaneB,SupportMinmum1);

            const scalar kRelFaceTolerance = scalar(0.95); //95%
            const scalar kAbsTolerance = scalar(0.5) * LINEAR_SLOP;


            // Favor first hull face to avoid face flip-flops.
            if(  distance2 > kRelFaceTolerance * distance1 + kAbsTolerance )
            {
                // 2 = reference, 1 = incident.
                FaceToFaceContacts(SupportVertB, iNumVertsB, SupportVertA,iNumVertsA , SeparatonNormal , true );
                SwapFlip();
            }
            else
            {
                // 1 = reference, 2 = incident.
                FaceToFaceContacts(SupportVertA, iNumVertsA, SupportVertB,iNumVertsB , SeparatonNormal , false );
            }

         }
      }


  return (mNbContactPoints > 0);
}


void IContactGenerationPoints::FaceToFaceContacts(const IVector3 *Poly, i32 iPolySize,
                                                  const IVector3 *Clipper, i32 iClipperSize, const IVector3 &_FaceNormal, bool flipNormal)
{

    IQuickClipping polyClipping( Poly, iPolySize, Clipper,iClipperSize);

    const std::vector<IVector3> clipping_vertices = polyClipping.ComputeClippingVertices();

    if (!clipping_vertices.empty())
    {
        IVector3 planeNormal = IVector3::triNormal(Poly[0], Poly[1], Poly[2]);
        scalar      clipper_d = Poly[0].Dot(planeNormal);

        if(!flipNormal)
        {

            for (u32 i = 0; i < clipping_vertices.size(); i++)
            {
                IVector3 P =  clipping_vertices[i];
                scalar   d = (P.Dot(planeNormal) - clipper_d);

                if ((d) < scalar(squarePersistentContactThreshold))
                {
                    //--------------------------------------------------------------------//
                    IVector3 A = P;
                    IVector3 B = P - planeNormal * d;
                    //scalar penetration = (A - B).dot( _FaceNormal );
                    scalar penetration = (A-B).Length();
                    IContactVertex info( _FaceNormal , penetration , A, B);
                    AddContact(info);
                    //--------------------------------------------------------------------//

                }
            }

        }
        else
        {
            for (i32 i=(clipping_vertices.size())-1; i>=i32(0); i--)
            {
                IVector3 P =  clipping_vertices[i];
                scalar   d = (P.Dot(planeNormal) - clipper_d);
                if ((d) < scalar(squarePersistentContactThreshold))
                {
                    //--------------------------------------------------------------------//
                    IVector3 A = P;
                    IVector3 B = P - planeNormal * d;
                    //scalar penetration = (A - B).dot( _FaceNormal );
                    scalar penetration = (A-B).Length();
                    IContactVertex info( _FaceNormal , penetration , A, B);
                    AddContact(info);
                    //--------------------------------------------------------------------//

                }
            }

        }

    }

}


void IContactGenerationPoints::EdgeToEdgeContacts(const IVector3 &A0, const IVector3 &A1,
                                                  const IVector3 &B0, const IVector3 &B1, const IVector3 &_FaceNormal)
{

    IVector3 DA = A1 - A0;
    IVector3 DB = B1 - B0;
    IVector3 r  = A0 - B0;

    scalar a = DA.Dot(DA);
    scalar e = DB.Dot(DB);
    scalar f = DB.Dot(r);
    scalar c = DA.Dot(r);

    scalar b =  DA.Dot(DB);
    scalar denom = a * e - b * b;

    scalar TA = (b * f - c * e) / denom;
    scalar TB = (b * TA + f) / e;

    TA = IClamp(TA, scalar(0.0), scalar(1.0));
    TB = IClamp(TB, scalar(0.0), scalar(1.0));

    const IVector3 A = A0 + DA * TA;
    const IVector3 B = B0 + DB * TB;


    scalar penetration = (B - A).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
      IVector3 c_point = (A+B) * scalar(0.5);
      IContactVertex info(  _FaceNormal  , penetration , c_point , c_point);
      AddContact(info);
    }
}



void IContactGenerationPoints::PointToEdgeContacts(const IVector3 &A, const IVector3 &B0, const IVector3 &B1, const IVector3 &_FaceNormal)
{
    IVector3 B0A = A - B0;
    IVector3 BD  = B1 - B0;

    scalar t = B0A.Dot(BD) / BD.Dot(BD);
    t = IClamp(t, 0.0f, 1.0f);

    IVector3 B = B0 + t * BD;

    scalar penetration = (A - B).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
      IContactVertex info(  _FaceNormal  , penetration , A, B);
      AddContact(info);
    }
}


void IContactGenerationPoints::PointToFaceContacts(const IVector3 &A, const IVector3 &xAxis, scalar bd, const IVector3 &_FaceNormal)
{
    scalar penetration = (A.Dot(xAxis)) - bd;
    IVector3 B = A - penetration * xAxis;

    if(penetration < squarePersistentContactThreshold)
    {
      IContactVertex info( _FaceNormal , penetration, A, B);
      AddContact(info);
    }
}


void IContactGenerationPoints::PointToPointContacts(const IVector3 &A, const IVector3 &B, const IVector3 &_FaceNormal)
{
    scalar penetration = (A - B).Dot(_FaceNormal);
    if(penetration < squarePersistentContactThreshold)
    {
       IContactVertex info( _FaceNormal  , penetration , A , B);
       AddContact(info);
    }
}



IVector3 *IContactGenerationPoints::CreateAxisPeturberationPoints(const IProxyShape *proxy_shape, const IVector3 &xAxis, i32 &_NbPoints)
{

    const ICollisionShape* CollisionShape = proxy_shape->GetCollisionShape();

    IVector3 axis =  proxy_shape->GetWorldTransform().GetBasis().GetTranspose() * xAxis;
    IVector3 n0, n1;
    //Vector3::BiUnitGrammSchmidt( axis , n0 , n1 );
    IVector3::BiUnitOrthogonalVector(axis , n0 , n1);


    i32    MAX_ITERATION =  CollisionShape->mNbMaxPeturberationIteration + 1;
    scalar MIN_EPSILON = 0.082;//0.044;//mCollisionShape->mEpsilonPeturberation;//

    const scalar OFF_SET_COLLISION_CONTACT = scalar(0.02);

    IVector3 startPoint;
    IVector3 OldPoint;

    IVector3* result = nullptr;
    i32 &num_element = _NbPoints;
    for( i32 i = 0; i < MAX_ITERATION; i++ )
    {
        scalar ang = (scalar(2.0) * M_PI / scalar(MAX_ITERATION)) * scalar(i);


        IVector3 auxAxis = (axis + n0 * ICos(ang) * MIN_EPSILON
                                 + n1 * ISin(ang) * MIN_EPSILON);

        /***************************************************************/

        IVector3 spVertex  = proxy_shape->GetWorldTransform() * CollisionShape->GetLocalSupportPointWithMargin( auxAxis );


        /*-----------------------------------------------------*/
        if ((spVertex - OldPoint).LengthSquare() > OFF_SET_COLLISION_CONTACT || i == 0)
        {
            if ((spVertex - startPoint).LengthSquare() < OFF_SET_COLLISION_CONTACT && num_element > 0 ) break;


            //                    if( num_element >= 3 )
            //                    {
            //                        Vector3 NFace = Vector3::triNormal(result[0],result[1],result[2]);
            //                        if(NFace.dot(result[0] - spVertex) > OFF_SET_COLLISION_CONTACT) continue;
            //                    }

            /***************************************/
            num_element++;
            IVector3* array = (IVector3*) IRealloc(result , num_element * sizeof(IVector3));
            if (array != nullptr)
            {
                result = array;
                result[num_element - 1] = spVertex;
            }
            else
            {
                IFree(result);
            }

            /***************************************/

            OldPoint = spVertex;
            if (num_element == 1) startPoint = spVertex;
        }
    }

    return result;

}



void IContactGenerationPoints::AddContact(const IContactVertex &p_contact)
{
    mNbContactPoints++;
    IContactVertex* array = (IContactVertex*) IRealloc(mContactPoints , mNbContactPoints * sizeof(IContactVertex));
    if (array != nullptr)
    {
        mContactPoints = array;
        mContactPoints[mNbContactPoints - 1] = p_contact;
    }
    else
    {
        IFree(mContactPoints);
    }
}

void IContactGenerationPoints::SwapFlip()
{
    for (i32 i = 0; i < mNbContactPoints; ++i)
    {
        ISwap( mContactPoints[i].point1 ,
               mContactPoints[i].point2 );
    }
}



}
