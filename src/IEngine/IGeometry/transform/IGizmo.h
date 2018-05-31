#ifndef IGIZMO_H
#define IGIZMO_H

#include "IObject3D.h"
#include "../math/IColor.h"


#include <iostream>
#include <list>
using namespace std;

namespace IGeometry
{


#define EPS 0.00001f


struct DrawLineSegment
{
    DrawLineSegment( const IVector3& _point1 , const IVector3& _point2 , const IColor& _color )
     : mPoint1(_point1),
       mPoint2(_point2),
       mColor(_color)
    {
    }

    IColor    mColor;
    IVector3  mPoint1;
    IVector3  mPoint2;
};

struct DrawVertex
{
    DrawVertex( const IVector3& _position , const IColor& _color )
     : mPosition(_position),
       mColor(_color)
    {

    }

    IColor   mColor;
    IVector3 mPosition;
};

struct IDrawList
{
    std::list<DrawLineSegment> mLineSegments;
    std::list<DrawVertex>      mVertexes;


    void AddLine( const IVector3& a ,  const IVector3& b ,  const IColor& color )
    {
        mLineSegments.push_back(DrawLineSegment( a , b , color ));
    }

    void AddVertex( const IVector3& pos ,  const IColor& color )
    {
        mVertexes.push_back(DrawVertex( pos , color ));
    }

    void AddCircleAroundAxis( const IVector3& mNormalFace , float radius , const IVector3& origin , const IVector3& dirLook , const IColor& _color , bool b = false , int num = 20)
    {
        IVector3 n = IVector3(mNormalFace.x , mNormalFace.y , mNormalFace.z);
        IVector3 p = n.getOneUnitOrthogonalVector();
        IVector3 q = n.cross(p);
        p.normalize();
        q.normalize();

        IVector3 mUpVec    = IVector3(p.x,p.y,p.z);
        IVector3 mRightVec = IVector3(q.x,q.y,q.z);


        IVector3 old_rot;
        for( int i = 0; i <= num; i++ )
        {
            float ang = (2.0f * M_PI / float(num)) * float(i);
            IVector3 n0 = -mUpVec;
            IVector3 n1 =  mRightVec;
            IVector3 vertex_rot = n0 * (cos(ang) * radius) +
                                  n1 * (sin(ang) * radius);

            vertex_rot += origin;

            if( i > 0 && ( (origin - vertex_rot).dot(dirLook) >= -0.01f &&
                           (origin -    old_rot).dot(dirLook) >= -0.01f) || b )
            {
               AddLine(vertex_rot,old_rot,_color);
            }

            old_rot = vertex_rot;
        }
    }

    void clear()
    {
        mLineSegments.clear();
        mVertexes.clear();
    }
};



struct TRay
{
    IVector3 OriginPoint;
    IVector3 ClosetPoint;
    IVector3 Direction;
};




struct Context
{

    enum GizmoTransformMode { Move , Scale , Rotate };
    enum GizmoCoordinatMode { Local , World , View };

    Context()
        : mbUsing(false),
          mbEnable(true),
          mbUsingBounds(false),
          mMousePressClick(false)
    {
    }

//    IDrawList* mDrawList;

    IMatrix4x4 mMatViewCamera;
    IMatrix4x4 mMatInitModel;

    TRay mRayInit;
    TRay mRayMove;

    bool mMousePressClick;
    bool mMousePressEnable;

    bool mbUsing;
    bool mbEnable;
    bool mbUsingBounds;


    GizmoTransformMode _TransformMode_;
    GizmoCoordinatMode _CoordinatMode_;

};









class IGizmo : public IObject3D
{

  private:

     //---------------------------//
     float    length_angle;
     int      NUM;
     IVector3 old_dir;
     //---------------------------//


  public:

     IGizmo();



     void Manipulate( const Context& gContext, IDrawList* DrawList = NULL )
     {
         switch (gContext._TransformMode_)
         {
             case   Context::Move: HandleMove( gContext , DrawList , gContext._CoordinatMode_);  break;
             case  Context::Scale: HandleScale( gContext , DrawList , gContext._CoordinatMode_);  break;
             case Context::Rotate: HandleRotate( gContext , DrawList , gContext._CoordinatMode_);  break;
         }
     }


   protected:


     //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

      void HandleRotate( const Context& gContext , IDrawList* DrawList , Context::GizmoCoordinatMode Mode )
      {

          if(gContext.mMousePressClick)
          {
              NUM = 0;
              length_angle = 0;
          }


          //IMatrix4x4 RotateMatrixResult = IMatrix4x4::IDENTITY;
          mTransformMatrix =  gContext.mMatInitModel;
          IVector3  origin =  IVector3(0,0,0) * gContext.mMatInitModel;


          /**/

          IVector3 pos_cam    =  gContext.mMatViewCamera.getInverse() * IVector3(0,0,0);;
          IVector3 dir_look_n = ( gContext.mMatViewCamera.getInverse() * IVector3::Z ).normalized();
          IVector3 origin_cam = pos_cam - dir_look_n * 1.5f;

          IPlane EyeInPlaneCam(dir_look_n,origin_cam);

          IVector3 intersect_vertex = EyeInPlaneCam.vIntersectionRayToPlane( pos_cam , (origin - pos_cam).normalized() );
          //origin = intersect_vertex;
          /**/

         // intersect_vertex = origin;

          IVector3 Init_Ray_origin = gContext.mRayInit.OriginPoint;
          IVector3 Init_Ray_end    = gContext.mRayInit.ClosetPoint;
          IVector3 Init_Ray_dir    = gContext.mRayInit.Direction;


          IVector3 Ray_origin = gContext.mRayMove.OriginPoint;
          IVector3 Ray_end    = gContext.mRayMove.ClosetPoint;
          IVector3 Ray_dir    = gContext.mRayMove.Direction;




          /**/

          //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

          IMatrix3x3 rot_init_world =  IMatrix3x3::IDENTITY;
          if( Mode == Context::World )
          {
             rot_init_world = IMatrix3x3::IDENTITY;
          }
          else if( Mode == Context::Local )
          {
              IGeometry::IMatrix3x3 mm = gContext.mMatInitModel.getRotMatrix();
              mm.OrthoNormalize();
              rot_init_world = mm;
          }

         // float SignL = ISign((pos_cam - gContext.CenterCamera).dot(IVector3::Z));

          IVector3 DirX = rot_init_world * -IVector3::X;
          IVector3 DirY = rot_init_world * IVector3::Y;
          IVector3 DirZ = rot_init_world * IVector3::Z;

          DirX.normalize();
          DirY.normalize();
          DirZ.normalize();

          float min_radius = 0.1/5.f;// * Z_buffer_length / 15.f;
          float max_radius = 1.0/5.f;// * Z_buffer_length / 15.f;


          if(DrawList != NULL)
          {
              DrawList->clear();
              DrawList->AddCircleAroundAxis( DirX , max_radius , intersect_vertex , -dir_look_n , IColor(1,0,0,1)  );
              DrawList->AddCircleAroundAxis( DirY , max_radius , intersect_vertex , -dir_look_n , IColor(0,1,0,1)  );
              DrawList->AddCircleAroundAxis( DirZ , max_radius , intersect_vertex , -dir_look_n , IColor(0,0,1,1)  );
              DrawList->AddCircleAroundAxis( dir_look_n , max_radius , intersect_vertex , -dir_look_n , IColor(0,1,1,1)  );
          }
//           //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//



          float t;
          IVector3 q;

          if( IntersectRayToSphere(Init_Ray_origin, Init_Ray_dir , intersect_vertex , max_radius * 1.1 , t , q) && gContext.mMousePressEnable )
          {

              //--------------------------------------------------------------------------------------//

              IPlane planeDir( dir_look_n , intersect_vertex );

              IPlane planeYZ( DirX , intersect_vertex );
              IPlane planeXZ( DirY , intersect_vertex );
              IPlane planeXY( DirZ , intersect_vertex );

              //--------------------------------------------------------------------------------------//

              IVector3 projDir = planeDir.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );

              IVector3 projYZ = planeYZ.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );
              IVector3 projXZ = planeXZ.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );
              IVector3 projXY = planeXY.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );

              //--------------------------------------------------------------------------------------//

              IVector3 hitDir = intersect_vertex + (projDir - intersect_vertex).normalized() * max_radius;

              IVector3 hitYZ = intersect_vertex + (projYZ - intersect_vertex).normalized() * max_radius;
              IVector3 hitXZ = intersect_vertex + (projXZ - intersect_vertex).normalized() * max_radius;
              IVector3 hitXY = intersect_vertex + (projXY - intersect_vertex).normalized() * max_radius;

              //--------------------------------------------------------------------------------------//


              ILine3 RayLine(Init_Ray_origin , Init_Ray_end);
              float ClosetLengthDir = (hitDir - RayLine.ClosestPoint(hitDir)).length();
              float ClosetLengthYZ  = (hitYZ  - RayLine.ClosestPoint(hitYZ)).length();
              float ClosetLengthXZ  = (hitXZ  - RayLine.ClosestPoint(hitXZ)).length();
              float ClosetLengthXY  = (hitXY  - RayLine.ClosestPoint(hitXY)).length();

              float ClosetLength = 10000;
              IVector3 Hit;
              bool isHit = false;


              IVector3 AxisRotationSelected;

              float esp_collid = 0.03;

              if(  (intersect_vertex - hitYZ).dot(dir_look_n) < 0.001 && ClosetLengthYZ < esp_collid )
              {
                  if(ClosetLengthXZ < ClosetLength )
                  {
                      AxisRotationSelected = -DirX;
                      ClosetLength = ClosetLengthYZ;
                      Hit = hitYZ;
                      isHit = true;
                  }
              }

              if(  (intersect_vertex - hitXZ).dot(dir_look_n) < 0.001 && ClosetLengthXZ < esp_collid )
              {
                  if(ClosetLengthXZ < ClosetLength )
                  {
                      AxisRotationSelected = DirY;
                      ClosetLength = ClosetLengthXZ;
                      Hit = hitXZ;
                      isHit = true;
                  }
              }


              if(  (intersect_vertex - hitXY).dot(dir_look_n) < 0.001 && ClosetLengthXY < esp_collid )
              {
                  if(ClosetLengthXY < ClosetLength )
                  {
                      AxisRotationSelected = DirZ;
                      ClosetLength = ClosetLengthXY; DrawList->clear();
                      Hit = hitXY;
                      isHit = true;
                  }
              }


              if(  ClosetLengthDir < esp_collid && !isHit)
              {
                  if(ClosetLengthDir < ClosetLength )
                  {
                      AxisRotationSelected = dir_look_n;
                      ClosetLength = ClosetLengthDir;
                      Hit = hitDir;
                      isHit = true;
                  }
              }


              if(isHit)
              {

                  IVector3 VMouseInPlaneCam = EyeInPlaneCam.vIntersectionRayToPlane( Ray_origin , Ray_dir );
                  IVector3 VOrigin = intersect_vertex;

                  NUM++;


                  IVector3 new_dir = (VOrigin - VMouseInPlaneCam);
                  IVector3 v1 = old_dir.normalized();
                  IVector3 v2 = new_dir.normalized();
                  old_dir = new_dir;

                  float angle_vl = IVector3::AngleSigned( v1 , v2 , dir_look_n);
                  length_angle += ( NUM > 1 ) ? angle_vl : 0.0;


                  if( AxisRotationSelected.length() > 0.0001f )
                  {

                      //rotateAroundWorldPoint(mTransformMatrix.getRotMatrix().getTranspose() * AxisRotationSelected,-(length_angle),origin);
                       mTransformMatrix = mTransformMatrix * IMatrix4x4::createRotationAxis( AxisRotationSelected , length_angle );


                      if(DrawList != NULL)
                      {
                          DrawList->clear();
                          DrawList->AddLine( VMouseInPlaneCam , VOrigin , IColor(1,0,1,0) );
                      }

                  }
                  else
                  {
                    length_angle = 0;
                  }
              }

          }
          else
          {

//              if(DrawList != NULL)
//              {
//                  DrawList->clear();
//                  DrawList->AddCircleAroundAxis( DirX , max_radius , intersect_vertex , -dir_look_n , IColor(1,0,0,1)  );
//                  DrawList->AddCircleAroundAxis( DirY , max_radius , intersect_vertex , -dir_look_n , IColor(0,1,0,1)  );
//                  DrawList->AddCircleAroundAxis( DirZ , max_radius , intersect_vertex , -dir_look_n , IColor(0,0,1,1)  );
//                  DrawList->AddCircleAroundAxis( dir_look_n , max_radius , intersect_vertex , -dir_look_n , IColor(0,1,1,1)  );
//              }
          }

      }


      //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//


      void HandleScale( const Context& gContext , IDrawList* DrawList , Context::GizmoCoordinatMode Mode )
      {
          mTransformMatrix = gContext.mMatInitModel;
          IVector3 origin =  IVector3(0,0,0) * gContext.mMatInitModel;


          /**/

          IVector3 pos_cam    =  gContext.mMatViewCamera.getInverse() * IVector3(0,0,0);;
          IVector3 dir_look_n = ( gContext.mMatViewCamera.getInverse() * IVector3::Z ).normalized();
          IVector3 origin_cam = pos_cam - dir_look_n * 1.5f;

          IPlane EyeInPlaneCam(dir_look_n,origin_cam);
          IVector3 intersect_vertex = EyeInPlaneCam.vIntersectionRayToPlane( pos_cam , (origin - pos_cam).normalized() );

          //IVector3 intersect_vertex = PlaneLineIntersect( origin_cam , dir_look_n , a_ray , b_ray );
          //origin = intersect_vertex;
          /**/




          IVector3 Init_Ray_origin = gContext.mRayInit.OriginPoint;
          IVector3 Init_Ray_end    = gContext.mRayInit.ClosetPoint;
          IVector3 Init_Ray_dir    = gContext.mRayInit.Direction;


          IVector3 Ray_origin = gContext.mRayMove.OriginPoint;
          IVector3 Ray_end    = gContext.mRayMove.ClosetPoint;
          IVector3 Ray_dir    = gContext.mRayMove.Direction;


          //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

          IMatrix3x3 rot_init_world =  IMatrix3x3::IDENTITY;
          if( Mode == Context::World )
          {
             rot_init_world = IMatrix3x3::IDENTITY;
          }
          else if( Mode == Context::Local )
          {
              IGeometry::IMatrix3x3 mm = gContext.mMatInitModel.getRotMatrix();
              //mm.OrthoNormalize();
              rot_init_world = mm;
          }

          IVector3 DirX = rot_init_world * -IVector3::X;
          IVector3 DirY = rot_init_world * IVector3::Y;
          IVector3 DirZ = rot_init_world * IVector3::Z;

          DirX.normalize();
          DirY.normalize();
          DirZ.normalize();

           //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

          float min_radius = 0.1/5.f;// * Z_buffer_length / 15.f;
          float max_radius = 1.0/5.f;// * Z_buffer_length / 15.f;

          IVector3 origin_vertex = intersect_vertex;// origin;// intersect_vertex;

          IVector3 nX1 =  origin_vertex + DirX * min_radius;
          IVector3 nX2 =  origin_vertex + DirX * max_radius;

          IVector3 nY1 =  origin_vertex + DirY * min_radius;
          IVector3 nY2 =  origin_vertex + DirY * max_radius;

          IVector3 nZ1 =  origin_vertex + DirZ * min_radius;;
          IVector3 nZ2 =  origin_vertex + DirZ * max_radius;


          //==============================================================//

          if(DrawList != NULL)
          {
              DrawList->clear();
              DrawList->AddLine( nX1 , nX2 , IColor(1,0,0,0) );
              DrawList->AddLine( nY1 , nY2 , IColor(0,1,0,0) );
              DrawList->AddLine( nZ1 , nZ2 , IColor(0,0,1,0) );
          }

          //==============================================================//

              ILine3 RayLine(Init_Ray_origin , Init_Ray_end);

              if((origin_vertex - RayLine.ClosestPoint(origin_vertex)).length() < min_radius)
              {                  
                  IPlane plane_camera_look( dir_look_n , origin );

                  IVector3 a = plane_camera_look.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );
                  IVector3 b = plane_camera_look.vIntersectionRayToPlane( Ray_origin      , Ray_dir      );

                  if( (a-b).lengthSquare() > 0.0001)
                  {
                      if(DrawList != NULL)
                      {
                          DrawList->clear();
                          DrawList->AddLine( a , b , IColor(1,0,1,0) );
                      }
                  }

                  // mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation( (b - a)  );
                  mTransformMatrix = mTransformMatrix * IMatrix4x4::createScale( (b - a).length() );
              }
              else
              {

                  IColor   RGBAColor;
                  IVector3 AxisMoving = IVector3::ZERO;
                  IVector3 InitOrigin;
                  IVector3 ia,ib;
                  if( IntersectLineToLine( nX1 , nX2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
                  {
                      if( (ia - origin_vertex).dot(DirX) < max_radius && (ia - origin_vertex).dot(DirX) > min_radius )
                      {
                          AxisMoving = DirX;
                          InitOrigin = ia;
                          RGBAColor = IColor(1,0,0,1);
                      }
                  }
                  else if( IntersectLineToLine( nY1, nY2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
                  {
                      if( (ia - origin_vertex).dot(DirY) < max_radius && (ia - origin_vertex).dot(DirY) > min_radius )
                      {
                          AxisMoving = DirY;
                          InitOrigin = ia;
                          RGBAColor = IColor(0,1,0,1);
                      }
                  }
                  else  if( IntersectLineToLine( nZ1 , nZ2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
                  {
                      if( (ia - origin_vertex).dot(DirZ) < max_radius && (ia - origin_vertex).dot(DirZ) > min_radius )
                      {
                          AxisMoving = DirZ;
                          InitOrigin = ia;
                          RGBAColor = IColor(0,0,1,1);
                      }
                  }


                  if( AxisMoving.lengthSquare() > 0.0001 )
                  {

                      IVector3 init_ia;
                      IVector3 init_ib;
                      if( IntersectLineToLine( origin_vertex , origin_vertex + AxisMoving , Init_Ray_origin , Init_Ray_end , &init_ia , &init_ib , true ) )
                      {
                          if( IntersectLineToLine( origin_vertex , origin_vertex + AxisMoving , Ray_origin , Ray_end , &ia , &ib , true ) )
                          {
                              IVector3 RelativOffset = ia - init_ia;
                              float Sign = ISign(RelativOffset.dot(AxisMoving));
                             // mTransformMatrix = mTransformMatrix * IMatrix4x4::createScaleAroundAxis(RelativOffset.normalized() , RelativOffset.length() * Sign).getTranspose();
                              if( RelativOffset.lengthSquare() > 0.00001 )
                              {
                                  if(DrawList != NULL)
                                  {
                                      DrawList->clear();
                                      DrawList->AddLine( origin - AxisMoving * 1000.f,
                                                         origin + AxisMoving * 1000.f, RGBAColor );
                                  }

                              }

                              IVector3 ScaleAxis = mTransformMatrix.getRotMatrix().getTranspose() * AxisMoving;
                              scaleAroundWorldPoint( ScaleAxis.normalized() , RelativOffset.length() * Sign * 5.f , origin );

                          }
                      }

                  }
              }

      }


    //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//


     void HandleMove( const Context& gContext , IDrawList* DrawList ,  Context::GizmoCoordinatMode Mode )
     {

         mTransformMatrix = gContext.mMatInitModel;

         /**/

         IVector3 origin     =   IVector3(0,0,0) * gContext.mMatInitModel;
         IVector3 pos_cam    =   gContext.mMatViewCamera.getInverse() * IVector3(0,0,0);;
         IVector3 dir_look_n = ( gContext.mMatViewCamera.getInverse() * IVector3::Z ).normalized();
         IVector3 origin_cam =   pos_cam - dir_look_n * 1.5f;

         IPlane EyeInPlaneCam(dir_look_n,origin_cam);
         IVector3 intersect_vertex = EyeInPlaneCam.vIntersectionRayToPlane( pos_cam , (origin - pos_cam).normalized() );

         //origin = intersect_vertex;
         /**/

         IVector3 Init_Ray_origin = gContext.mRayInit.OriginPoint;
         IVector3 Init_Ray_end    = gContext.mRayInit.ClosetPoint;
         IVector3 Init_Ray_dir    = gContext.mRayInit.Direction;


         IVector3 Ray_origin = gContext.mRayMove.OriginPoint;
         IVector3 Ray_end    = gContext.mRayMove.ClosetPoint;
         IVector3 Ray_dir    = gContext.mRayMove.Direction;

         //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

         IMatrix3x3 rot_init_world =  IMatrix3x3::IDENTITY;
         if( Mode == Context::World )
         {
            rot_init_world = IMatrix3x3::IDENTITY;
         }
         else if( Mode == Context::Local )
         {
             IGeometry::IMatrix3x3 mm = gContext.mMatInitModel.getRotMatrix();
             //mm.OrthoNormalize();
             rot_init_world = mm;
         }

         IVector3 DirX = rot_init_world * -IVector3::X;
         IVector3 DirY = rot_init_world *  IVector3::Y;
         IVector3 DirZ = rot_init_world *  IVector3::Z;

         DirX.normalize();
         DirY.normalize();
         DirZ.normalize();

          //=--=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-//

         float min_radius = 0.1/5.f;// * Z_buffer_length / 15.f;
         float max_radius = 1.0/5.f;// * Z_buffer_length / 15.f;

         IVector3 origin_vertex = intersect_vertex;// origin;// intersect_vertex;

         IVector3 nX1 =  origin_vertex + DirX * min_radius;
         IVector3 nX2 =  origin_vertex + DirX * max_radius;

         IVector3 nY1 =  origin_vertex + DirY * min_radius;
         IVector3 nY2 =  origin_vertex + DirY * max_radius;

         IVector3 nZ1 =  origin_vertex + DirZ * min_radius;;
         IVector3 nZ2 =  origin_vertex + DirZ * max_radius;


         //==============================================================//


         if(DrawList != NULL)
         {
             DrawList->clear();
             DrawList->AddLine( nX1 , nX2 , IColor(1,0,0,0) );
             DrawList->AddLine( nY1 , nY2 , IColor(0,1,0,0) );
             DrawList->AddLine( nZ1 , nZ2 , IColor(0,0,1,0) );
         }


         //==============================================================//
         ILine3 RayLine(Init_Ray_origin , Init_Ray_end);

         if((origin_vertex - RayLine.ClosestPoint(origin_vertex)).length() < min_radius)
         {
             IPlane plane_camera_look( dir_look_n , origin );

             IVector3 a = plane_camera_look.vIntersectionRayToPlane( Init_Ray_origin , Init_Ray_dir );
             IVector3 b = plane_camera_look.vIntersectionRayToPlane( Ray_origin      , Ray_dir      );


             if( (a-b).lengthSquare() > 0.0001)
             {
                 if(DrawList != NULL)
                 {
                     DrawList->clear();
                     DrawList->AddLine( a , b , IColor(1,0,1,0) );
                 }
             }


             // mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation( (b - a)  );
             mTransformMatrix = IMatrix4x4::createTranslation( b - a) * mTransformMatrix;
         }
         else
         {

             IColor   RGBAColor;
             IVector3 AxisMoving = IVector3::ZERO;
             IVector3 InitOrigin;
             IVector3 ia,ib;
             if( IntersectLineToLine( nX1 , nX2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
             {
                 if( (ia - origin_vertex).dot(DirX) < max_radius && (ia - origin_vertex).dot(DirX) > min_radius )
                 {
                     AxisMoving = DirX;
                     InitOrigin = ia;
                     RGBAColor = IColor(1,0,0,1);
                 }
             }
             else if( IntersectLineToLine( nY1, nY2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
             {
                 if( (ia - origin_vertex).dot(DirY) < max_radius && (ia - origin_vertex).dot(DirY) > min_radius )
                 {
                     AxisMoving = DirY;
                     InitOrigin = ia;
                     RGBAColor = IColor(0,1,0,1);
                 }
             }
             else  if( IntersectLineToLine( nZ1 , nZ2 , Init_Ray_origin , Init_Ray_end , &ia , &ib ))
             {
                 if( (ia - origin_vertex).dot(DirZ) < max_radius && (ia - origin_vertex).dot(DirZ) > min_radius )
                 {
                     AxisMoving = DirZ;
                     InitOrigin = ia;
                     RGBAColor = IColor(0,0,1,1);
                 }
             }


             if( AxisMoving.lengthSquare() > 0.0001 )
             {

                 IVector3 init_ia;
                 IVector3 init_ib;
                 if( IntersectLineToLine( origin , origin + AxisMoving , Init_Ray_origin , Init_Ray_end , &init_ia , &init_ib , true ) )
                 {
                     if( IntersectLineToLine( origin , origin + AxisMoving , Ray_origin , Ray_end , &ia , &ib , true ) )
                     {
                         IVector3 RelativOffset = ia - init_ia;
                         //mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation( RelativOffset );
                         //mTransformMatrix = IMatrix4x4::createTranslation( RelativOffset ) * mTransformMatrix;
                         if( RelativOffset.lengthSquare() > 0.00001 )
                         {
                             if(DrawList != NULL)
                             {
                                 DrawList->clear();
                                 DrawList->AddLine( origin - AxisMoving * 1000.f,
                                                    origin + AxisMoving * 1000.f, RGBAColor );
                             }

                         }

                         translateWorld( RelativOffset );

                     }
                 }
             }
         }


     }


     //====================================== Geometry Function ===============================================//

     // Intersects ray r = p + td, |d| = 1, with sphere s and, if intersecting,
     // returns t value of intersection and intersection point q
     static bool IntersectRayToSphere(IVector3 rayOrigin, IVector3 rayDir , IVector3 center ,  float radius , float &t, IVector3 &q);

     static bool IntersectLineToLine( IVector3  p1 , IVector3  p2, IVector3  p3 , IVector3  p4, IVector3 *pa = NULL , IVector3 *pb = NULL , bool is_realse = false);


};

}

#endif // IGIZMO_H
