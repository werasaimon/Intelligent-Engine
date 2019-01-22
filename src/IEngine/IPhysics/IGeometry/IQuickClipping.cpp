#include "IQuickClipping.h"

namespace IPhysics
{

std::vector<IVector3> IQuickClipping::ComputeClippingVertices() const
{
    assert(mPolygon.GetCount() >= 3 && mClipPolygon.GetCount() >= 2);

    std::vector<IVector3> output_clipping(0);

    IClipPlane FacePlane(mPolygon.GetVertices()[0],
                         mPolygon.GetVertices()[1],
                         mPolygon.GetVertices()[2]);

    i32 _NbCountA =  mPolygon.GetCount();
    i32 _NbCountB =  mClipPolygon.GetCount();

    const IVector3* _axVerticesA = mPolygon.GetVertices();
    const IVector3* _axVerticesB = mClipPolygon.GetVertices();

    i32 phase = 1;
    i32 maxItns = 2 * (_NbCountA + _NbCountB);
    i32 inflag = UNKNOWN;
    i32 Iterator = 0;

    bool exceptions = (_NbCountB == 2);
    bool isLookFaceToFace = true;

    int next_index_a = 0;
    int next_index_b = 0;

    if(_NbCountB >= 3)
    {

        IVector3 NormalClipPlane = IVector3::triNormal( _axVerticesB[0],
                                                        _axVerticesB[1],
                                                        _axVerticesB[2]);

        isLookFaceToFace = (NormalClipPlane.Dot(FacePlane.GetNormal()) >= 0);
    }

    /*************************************************/

    IVector3 start_point;

    for(int i = 1; ((i <= maxItns) || (phase == 2)); i++)
    {

        if (phase == 2)   Iterator++;
        if (Iterator > (maxItns / 2)) break;

        IClipEdge EdgeA = mPolygon.EdgeAt(next_index_a);
        IClipEdge EdgeB = mClipPolygon.EdgeAt(next_index_b);

        if (!isLookFaceToFace)
        {
            ISwap(EdgeB.mA, EdgeB.mB);
        }


        IVector3 PlaneNormalA = (EdgeA.mA - EdgeA.mB).Cross(-FacePlane.GetNormal());
        IVector3 PlaneNormalB = (EdgeB.mA - EdgeB.mB).Cross(-FacePlane.GetNormal());


        IClipPlane planeA(PlaneNormalA, EdgeA.mB );
        IClipPlane planeB(PlaneNormalB, EdgeB.mB );


        //**************************************************//


        IVector3 crossPoint =  planeA.VIntersectionLineToPlane(EdgeB , true);
        IVector3 projCrossPoint =  FacePlane.ClosestPoint(crossPoint);

        bool isNotParallel = true;//PlaneNormalA.cross(PlaneNormalB).lengthSquare() > EPS;

        bool isCrossA = EdgeA.IsInsideLineSegment(projCrossPoint);
        bool isCrossB = EdgeB.IsInsideLineSegment(crossPoint);

        bool isIntersection = ( isCrossA && isCrossB );


        int a_class = planeB.SideClassifyPointToEdge(EdgeA.mB);
        int b_class = planeA.SideClassifyPointToEdge(EdgeB.mB);



        //***************** exceptions **********************//

        if((phase == 2) &&
                (isIntersection) &&
                (start_point == crossPoint) &&
                (output_clipping.size() == 1) &&
                (exceptions))
        {
            if ( b_class == LEFT )
            {
                output_clipping.push_back(EdgeB.mA);
            }
            else
            {
                output_clipping.push_back(EdgeB.mB);
            }

            break;
        }

        //**************************************************//

        if( phase == 2 && (start_point == crossPoint)  && output_clipping.size() > 1 ) break;


        //**************************************************//

        if( isIntersection )
        {
            if (phase == 1)
            {
                output_clipping.push_back((start_point = crossPoint));
                phase = 2;

            }
            else if (crossPoint != start_point && phase == 2 && isNotParallel)
            {
                output_clipping.push_back(crossPoint);
            }
        }


        //************************************************//


        const IVector3 EdgeDirA = EdgeA.mA - EdgeA.mB;
        const IVector3 EdgeDirB = EdgeB.mA - EdgeB.mB;


        bool bPlaneAIMSaLine = planeB.IsAtLookToPoly( EdgeDirA , a_class );
        bool aPlaneAIMSbLine = planeA.IsAtLookToPoly( EdgeDirB , b_class );


        /*************************************************/

        if( isIntersection )
        {
            if (a_class == RIGHT)
            {
                inflag = P_IS_INSIDE;
            }
            else if (b_class == RIGHT)
            {
                inflag = Q_IS_INSIDE;
            }
            else
            {
                inflag = UNKNOWN;
            }
        }


        /*************************************************/

        bool isLeftClassification = (a_class == LEFT);

        int lClassifyIndex = NextMoveIndexToEdges(bPlaneAIMSaLine,
                                                  aPlaneAIMSbLine,
                                                  isLeftClassification,
                                                  exceptions,
                                                  isLookFaceToFace,
                                                  next_index_a,
                                                  next_index_b);


        if( phase == 2)
        {
            if( lClassifyIndex == 1)
            {
                if ( inflag == Q_IS_INSIDE  )
                {
                    output_clipping.push_back(EdgeB.mB);
                }
            }
            else if( lClassifyIndex == 2)
            {
                if ( inflag == P_IS_INSIDE &&  _NbCountB >= 3 )
                {
                    /************************************************/
                    IVector3 NormalClipPlane = IVector3::triNormal( _axVerticesB[0],
                                                                    _axVerticesB[1],
                                                                    _axVerticesB[2]);

                    IClipPlane ClipPlane(NormalClipPlane, _axVerticesB[0]);
                    IClipEdge  ClipEdge( EdgeA.mB , EdgeA.mB + FacePlane.GetNormal());
                    IVector3 p = ClipPlane.VIntersectionLineToPlane(ClipEdge);

                    output_clipping.push_back(p);
                    /**********************************************/
                }
            }
        }

    }


    /*************************************************/

    if( phase == 1 )
    {
        int num_verticesA = _NbCountA;
        int num_verticesB = _NbCountB;

        next_index_a = (next_index_a == num_verticesA || next_index_a < num_verticesA-1) ? 0: next_index_a;
        next_index_b = (next_index_b == num_verticesB || next_index_b < num_verticesB-1) ? 0: next_index_b;


        IVector3 Apoint = _axVerticesA[next_index_a];
        IVector3 Bpoint = _axVerticesB[next_index_b];


        if (InsidePolygonSAT( Bpoint , _axVerticesA, num_verticesA , FacePlane))
        {
            for (int i = 0; i < num_verticesB; i++)
            {
                output_clipping.push_back( _axVerticesB[i] );
            }
        }
        else if ( InsidePolygonSAT(Apoint, _axVerticesB , num_verticesB , FacePlane ) && _NbCountB >= 3)
        {
            IVector3 NormalClipPlane = IVector3::triNormal( _axVerticesB[0],
                                                            _axVerticesB[1],
                                                            _axVerticesB[2]);

            IClipPlane ClipPlane(NormalClipPlane, _axVerticesB[0]);
            for (int i = 0; i < num_verticesA; i++)
            {
                IVector3 p =  ClipPlane.VIntersectionLineToPlane( IClipEdge(_axVerticesA[i] ,
                                                                            _axVerticesA[i] + FacePlane.GetNormal()) );
                output_clipping.push_back(p);
            }
        }
    }

    /*************************************************/

    return output_clipping;
}

i32 IQuickClipping::NextMoveIndexToEdges(bool bPlaneAIMSLookToLineA,
                                         bool aPlaneAIMSLookToLineB,
                                         bool isLeftClassification,
                                         bool exceptions,
                                         bool isLookFaceToFace,
                                         i32 &moveIndexA, i32 &moveIndexB)
{
    if(!exceptions)
    {

        if (aPlaneAIMSLookToLineB && bPlaneAIMSLookToLineA)
        {

            if (isLeftClassification)
            {
                MoveIndex(moveIndexA , true);
            }
            else
            {
                MoveIndex(moveIndexB , isLookFaceToFace);
            }

            return 0;
        }
        else if (bPlaneAIMSLookToLineA)
        {

            MoveIndex(moveIndexA , true);
            return 2;

        }
        else if (aPlaneAIMSLookToLineB)
        {

            MoveIndex(moveIndexB , isLookFaceToFace);
            return 1;

        }

        else
        {
            if(isLeftClassification)
            {
                MoveIndex(moveIndexA , true);
            }
            else
            {
                MoveIndex(moveIndexB , isLookFaceToFace);
            }

            return 0;
        }
    }
    else
    {
        MoveIndex(moveIndexA , true);

        return 0;
    }

    return 0;
}

void IQuickClipping::MoveIndex(i32 &index, bool isLookFaceToFace)
{
    (isLookFaceToFace)? index++ : index--;
}

bool IQuickClipping::InsidePolygonSAT(const IVector3 &vIntersection, const IVector3 *PolyVertices, i32 NbCount, const IQuickClipping::IClipPlane &plane)
{
    IVector3 vA, vB;
    IVector3 vNormalAxis;
    for (i32 i = 0; i < NbCount; i++)
    {
        vA = PolyVertices[i];
        vB = PolyVertices[(i + 1) %NbCount];

        vA = plane.ClosestPoint(vA);
        vB = plane.ClosestPoint(vB);

        if( (vIntersection - vB).Dot(vNormalAxis) > 0.f ) return false;
    }

    return true;
}



}
