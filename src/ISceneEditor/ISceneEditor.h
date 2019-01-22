#ifndef ISceneEditor_H
#define ISceneEditor_H

#include "ISceneCompare.h"

#include "IGizmo/IUGizmo.h"

//#include "Component/CComponentObject.h"
//#include "Component/CComponentMeshModel.h"
//#include "Component/CComponentGeometricWorld.h"

//#include "UserInterface/UIPropertiesObject.h"
//#include "UserInterface/UIPropertiesTransform.h"
//#include "UserInterface/UIPropertiesMeshModel.h"
//#include "UserInterface/UInterface.h"


#include <vector>
#include <memory>
#include <GL/gl.h>

#include "IVivwer.h"
#include "IEngineComponent/IComponentMesh3.h"
#include "IEngineComponent/IScene3D.h"

#include "ISceneConservationCheckPoint.h"



class IGroupTransform : public IEngine::IObjectTransform
{

    std::map< IEngineComponent::IComponent3D* , IEngine::IMatrix4x4 > m_GroupSVGMatrix;

    public:

      IGroupTransform(){}


      void AddMatrix( IEngineComponent::IComponent3D* obj_key , const IEngine::IMatrix4x4& _matrix )
      {
         m_GroupSVGMatrix[obj_key] = _matrix;
         mTransformMatrix = InterpolationMatrix();
      }

      void InsertMatrix( IEngineComponent::IComponent3D* obj_key , const IEngine::IMatrix4x4& _matrix )
      {
         m_GroupSVGMatrix.insert( std::make_pair(obj_key , _matrix) );
         mTransformMatrix = InterpolationMatrix();
      }

      void Clear()
      {
         m_GroupSVGMatrix.clear();
      }


      IEngine::IMatrix4x4 &TransformMatrix()
      {
          return mTransformMatrix;
      }


      IEngine::IMatrix4x4 InterpolationMatrix() const
      {
          IEngine::IMatrix4x4 InterpolateMatrix = IEngine::IMatrix4x4::ZERO;
          for (auto item : m_GroupSVGMatrix)
          {
              InterpolateMatrix = item.second + InterpolateMatrix;
          }
          return InterpolateMatrix / float(m_GroupSVGMatrix.size());
      }



      std::map<IEngineComponent::IComponent3D *, IEngine::IMatrix4x4> &GroupSVGMatrix()
      {
          return m_GroupSVGMatrix;
      }
};



class ISceneEditor : public  ISceneCompare
{

    private:

        //----- Scene Viewport Ptr -------------//
        IVivwer *mPtrViewportScene;

        //----- Gizmo Manipulator Ptr ----------//
        IuGizmo::IGizmo* mPtrGizmoManipulator;

        //----- Scene Components Compare -------//
        IEngineComponent::IScene3D *mSceneWorld;


        //---- Gizmo Manipulator Control -------//
        IGroupTransform mGizmoGroup;


        float mWidth;
        float mHeight;



public:


      void onSelectedComponent(IEngineComponent::IComponent3D *_component)
      {
            mGizmoGroup.Clear();
            mGizmoGroup.AddMatrix(_component,_component->GetTransformHierarchy());
            mPtrGizmoManipulator->SetEditMatrix( mGizmoGroup.getTransformMatrix() );
      }

      struct ContexSaveScene
      {

           bool isSaveComponent;
           bool isSaveHierarchyNode;
           bool isDeleteComponent;

           ContexSaveScene(bool _isSaveComponent = true ,
                           bool _isSaveHierarchyNode = true ,
                           bool _isDeleteComponent = false)
            : isSaveComponent(_isSaveComponent) ,
              isSaveHierarchyNode(_isSaveHierarchyNode) ,
              isDeleteComponent(_isDeleteComponent)
           {

           }
      };

      void SaveSceneState(ContexSaveScene _ContexSave)
      {
          this->ChaekSaveCurrentSceneState(_ContexSave);
      }

      void PrevScene()
      {
          this->PrevStoreCurrentSceneState();
      }

      void NextScene()
      {
          this->NextStoreCurrentSceneState();
      }

      //-------------------------------------------------------//

      void setGizmoManipulator(IuGizmo::IGizmo *_GizmoManipulator)
      {
          mPtrGizmoManipulator = _GizmoManipulator;
      }

      void AddComponent( IEngineComponent::IComponent3D *_Component )
      {
          mSceneWorld->AddComponent(_Component);
          onSelectedComponent(_Component);
      }

private:

      unsigned int mMaximum_count_save_scene = 0;
      unsigned int mIterator_position_save_scene = 0;
      unsigned int mMaximumCountOfFixationPoints = 5;
      std::vector<ISceneConservationCheckPoint> mSceneSaveCheckPoints;
      void RemoveCheckPointRealase();
      void CheckSave(ISceneConservationCheckPoint SceneSaveCheckPoint);
      //void ChaekSaveCurrentSceneStateDelete();
      void ChaekSaveCurrentSceneState(ContexSaveScene _ContexSave);
      void PrevStoreCurrentSceneState();
      void NextStoreCurrentSceneState();


public:

    ISceneEditor(IVivwer *_PtrViewportScene);


    virtual bool initialization();
    virtual void render(float FrameTime);
    virtual void update();
    virtual void resize( float width , float height );

    virtual void mouseMove( float x , float y  , int button);
    virtual void mousePress( float x , float y , int button );
    virtual void mouseReleasePress( float x , float y , int button );
    virtual void mouseWheel( float delta );

    virtual void realaseKeyboard( int key );
    virtual void keyboard(int key );
    virtual void destroy();



    void RemoveSelectedComponent()
    {

        ContexSaveScene contex_save;
        contex_save.isSaveComponent = false;
        contex_save.isSaveHierarchyNode = false;
        contex_save.isDeleteComponent = true;

        ChaekSaveCurrentSceneState(contex_save);

        for( auto item : mGizmoGroup.GroupSVGMatrix() )
        {
            auto SelectedComponent = item.first;
            mSceneWorld->EraseComponent(SelectedComponent);
            SelectedComponent = nullptr;
        }

        mGizmoGroup.Clear();

    }


//    //-----------------------//

//        void CheckMove()
//        {
////            mGizmoManipulator = mGizmoMove;
////            mGizmoManipulator->SetLocation( mGizmoLocationMode );
////            mGizmoManipulator->SetEditMatrix( mGizmoGroup.TransformMatrix() );
////            mGizmoManipulator->SetDisplayScale( 2.f );
////            mGizmoManipulator->SetScreenDimension( mWidth, mHeight );
//        }

//        void CheckRotate()
//        {
////            mGizmoManipulator = mGizmoRotate;
////            mGizmoManipulator->SetLocation( mGizmoLocationMode );
////            mGizmoManipulator->SetEditMatrix( mGizmoGroup.TransformMatrix() );
////            mGizmoManipulator->SetDisplayScale( 2.f );
////            mGizmoManipulator->SetScreenDimension( mWidth, mHeight );
//        }

//        void CheckScale()
//        {
////            mGizmoManipulator = mGizmoScale;
////            mGizmoManipulator->SetLocation( mGizmoLocationMode );
////            mGizmoManipulator->SetEditMatrix( mGizmoGroup.TransformMatrix() );
////            mGizmoManipulator->SetDisplayScale( 2.f );
////            mGizmoManipulator->SetScreenDimension( mWidth, mHeight );
//        }

//        void CheckLocal()
//        {
////            mGizmoManipulator->SetDisplayScale( 2.f );
////            mGizmoManipulator->SetLocation( mGizmoLocationMode = IuGizmo::IGizmo::LOCATE_LOCAL );
//        }

//        void CheckWorld()
//        {
////            mGizmoManipulator->SetDisplayScale( 2.f );
////            mGizmoManipulator->SetLocation( mGizmoLocationMode = IuGizmo::IGizmo::LOCATE_WORLD );
//        }

//        //-----------------------//




        IEngine::IMatrix4x4 &getGizmo_transform_matrix() //const
        {
            return  mGizmoGroup.TransformMatrix();
        }
};



#endif // ISceneEditor_H
