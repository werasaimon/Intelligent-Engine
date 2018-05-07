#ifndef IQUICKCLIPPING_H
#define IQUICKCLIPPING_H

#include <vector>
#include "../../common/math/IMatematical.h"

namespace IPhysics
{
  using namespace std;


class IQuickClipping
{
    enum { UNKNOWN = 10, P_IS_INSIDE, Q_IS_INSIDE };
    enum { LEFT = 30, RIGHT, BEHIND };
    enum { COLLINEAR = 40, PARALLEL, SKEW, SKEW_CROSS, SKEW_NO_CROSS };


    struct IClipEdge
    {

        IClipEdge( const IVector3& _a = IVector3(0,0,0) ,
                   const IVector3& _b = IVector3(0,0,0))
            : mA(_a) , mB(_b)
        {

        }

        // isInsideLineSegment(): determine if a point is inside a segment
        //    Input:  a point P, and a collinear segment S
        //    Return: 1 = P is inside S
        //            0 = P is  not inside S
        bool isInsideLineSegment( const IVector3 &point )
        {
            if ((point - mA).dot(point - mB) > 0)  return false;
            return true;
        }


        IVector3 mA;
        IVector3 mB;
    };


    class IClipPlane : public IPlane
    {

      public:

        //-------------------- Constructor --------------------//

        IClipPlane( IVector3 _n , IVector3 _origin)
         : IPlane( _n.getUnit() , _origin )
        {

        }

        IClipPlane( const IVector3& a ,
                    const IVector3& b ,
                    const IVector3& c)
          :IPlane(a,b,c)
        {

        }


        //-------------------- Method -------------------------//

        int sideClassifyPointToEdge( const IVector3& _inVtx ) const
        {

            scalar length = IPlane::invTest(_inVtx);

            if (length > 0.0) return RIGHT;
            if (length < 0.0) return LEFT;

            return BEHIND;
        }



        bool isAtLookToPoly(const IVector3& DirectionLook ,  i32 aclass ) const
        {
            IVector3 va = DirectionLook;
            IVector3 vb = mNormal;

            scalar  v = vb.dot(va);

            if (v >= 0.0)
            {
                return (aclass != RIGHT);
            }
            else
            {
                return (aclass != LEFT);
            }
        }


        //===================================================================

        // vIntersectionLineToPlane(): find the 3D intersection of a segment and a plane
        //    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
        //    Output: *I0 = the intersect point (when it exists)
        //    Return: 0 = disjoint (no intersection)
        IVector3 vIntersectionLineToPlane( const IClipEdge& _edge , bool parallel_test = false) const
        {
           return IPlane::vIntersectionLineToPlane(ILineSegment3(_edge.mA,_edge.mB) , parallel_test );
        }


    };



    class IClipPolygon
    {
        const IVector3 *mVertices;
        const i32       mNbVertices;

    public:

        IClipPolygon(const IVector3* _vertices , const i32 _count )
            : mVertices(_vertices) ,
              mNbVertices(_count)
        {

        }

        i32 getCount() const
        {
            return mNbVertices;
        }

        const IVector3* getVertices() const
        {
            return mVertices;
        }


        IClipEdge EdgeAt( i32 &_i ) const
        {
                i32 index_a;
                i32 index_b;

                i32 index = (_i >= 0) ? _i : _i + mNbVertices;
                i32 indexation = (index >= 0 && index < mNbVertices) ? index : 0;

                if (index >= mNbVertices || index < 0)
                {
                    index_a = 0;
                    _i = 0;
                }

                index_b = (indexation >= 0) ? indexation : mNbVertices - 1;
                index_a = (index < mNbVertices && index >= 0) ? indexation - 1 : mNbVertices - 1;

                if (index_a < 0)
                {
                    _i = 0;
                    index_a = mNbVertices - 1;
                }

                return IClipEdge(mVertices[index_a],
                                 mVertices[index_b]);
        }

    };


private:


    IClipPolygon mPolygon;
    IClipPolygon mClipPolygon;

    /// Private copy-constructor
    IQuickClipping(const IQuickClipping& clipping);

    /// Private assignment operator
    IQuickClipping& operator=(const IQuickClipping& clipping);

public:

    IQuickClipping(const IVector3*  PolygonVertices , i32  CountPolygonVertices ,
                   const IVector3*  ClipVertices    , i32  CountClipVertices)
        : mPolygon(PolygonVertices,CountPolygonVertices) ,
          mClipPolygon(ClipVertices , CountClipVertices)
    {
    }

    ~IQuickClipping()
    {
    }


    //******************  Compute clipping **************************//

    std::vector<IVector3> ComputeClippingVertices() const;


private:


    //****************   Help Function ****************************//
    static i32   nextMoveIndexToEdges( bool bPlaneAIMSLookToLineA ,
                                       bool aPlaneAIMSLookToLineB ,
                                       bool isLeftClassification  ,
                                       bool exceptions ,
                                       bool isLookFaceToFace ,
                                       i32& moveIndexA ,
                                       i32& moveIndexB );

    static void   MoveIndex(i32& index , bool isLookFaceToFace );


    static bool InsidePolygonSAT(const IVector3& vIntersection, const IVector3* PolyVertices , u32 NbCount, const IClipPlane &plane);


};

}

#endif // IQUICKCLIPPING_H
