#ifndef ISCENECONSERVATIONCHECKPOINT_H
#define ISCENECONSERVATIONCHECKPOINT_H

#include "IEngineComponent/IComponentMesh3.h"
#include "IEngineComponent/IComponentCamera.h"

class SaveStateElement
{
    public:

      SaveStateElement() {}
      virtual ~SaveStateElement() {}
      virtual void ReturnStoreState() {}
};

class SaveStateElementComponentMesh3 : public SaveStateElement
{
        IEngineComponent::IComponentMesh3 *mPtrComponentMesh;
        IEngine::IMesh3 mCopyMesh;

    public:

        SaveStateElementComponentMesh3(IEngineComponent::IComponent3D *_Component)
        : mPtrComponentMesh( static_cast<IEngineComponent::IComponentMesh3*>(_Component) )
        {
            mCopyMesh.Copy(*mPtrComponentMesh->GetMesh());
        }

        ~SaveStateElementComponentMesh3()
        {
            mPtrComponentMesh = nullptr;
        }

        virtual void ReturnStoreState()
        {
            mPtrComponentMesh->Mesh()->Copy( mCopyMesh );
        }
};

class SaveStateElementCamera : public SaveStateElement
{
        IEngineComponent::IComponentCamera *mPtrComponentCamera;
        IEngine::IQCamera mCopyCamera;

    public:

        SaveStateElementCamera(IEngineComponent::IComponent3D *_Component)
          : mPtrComponentCamera( static_cast<IEngineComponent::IComponentCamera*>(_Component) )
        {
            mCopyCamera = *mPtrComponentCamera->GetCamera();
        }

        ~SaveStateElementCamera()
        {
            std::cout << " ~SaveStateElementCamera " << std::endl;
            mPtrComponentCamera = nullptr;
        }

        virtual void ReturnStoreState()
        {
            *mPtrComponentCamera->Camera() = mCopyCamera;
        }
};




class SaveComponentHierarchyCheckPoint
{

    public:

       IEngineComponent::IComponent3D *mComponentNode;
       IEngineComponent::IHierarchyNode *mParentNode;
       std::vector<IEngineComponent::IHierarchyNode*> mChildrenNode;


    public:

        SaveComponentHierarchyCheckPoint(){}

        SaveComponentHierarchyCheckPoint(IEngineComponent::IComponent3D *component)
        : mComponentNode(component)
        {
            assert(mComponentNode);

            mParentNode = nullptr;
            if( component->GetParent() )
            {
                mParentNode = component->GetParent();
            }

            mChildrenNode.clear();
            for (auto* child = component->GetFirstChild(); child != nullptr; child = component->GetNextSibling())
            {
                mChildrenNode.push_back(child);
            }

        }

        void Release()
        {
            mComponentNode = nullptr;
            mParentNode = nullptr;
            mChildrenNode.clear();
        }


        IEngineComponent::IComponent3D *ComponentNode()
        {
            return mComponentNode;
        }


        void RestoreHierarchy()
        {
            if(mParentNode)
            {
                mParentNode->AddChild(mComponentNode);
            }

            for( auto child_item : mChildrenNode )
            {
                mComponentNode->AddChild(child_item);
            }
        }

        void RestoreHierarchyRemove()
        {
            if(mParentNode)
            {
                mParentNode->RemoveChild(mComponentNode);
            }

            for( auto child_item : mChildrenNode )
            {
                mComponentNode->RemoveChild(child_item);
            }
        }
};


class ISceneConservationCheckPoint
{
        friend class ISceneEditor;

    private:

        std::vector<SaveComponentHierarchyCheckPoint>   mCheckDeleteNodes;
        std::vector<SaveComponentHierarchyCheckPoint>   mCheckCopyNodes;

        std::map< IEngineComponent::IComponent3D* , IEngine::IMatrix4x4 > mSaveStoreGroupSVGMatrixCopy;
        //std::map< IEngineComponent::IComponent3D* , IEngine::IMatrix4x4 > mSaveStoreGroupSVGMatrix;
        std::map< IEngineComponent::IComponent3D* , SaveStateElement* >   mSaveStoreElements;


    public:

      ISceneConservationCheckPoint()
      {
      }

      void Realase()
      {
          for( auto item : mSaveStoreElements )
          {
              delete item.second;
              item.second = nullptr;
          }

          mCheckDeleteNodes.clear();
          mCheckCopyNodes.clear();
          mSaveStoreElements.clear();
         // mSaveStoreGroupSVGMatrix.clear();
          mSaveStoreGroupSVGMatrixCopy.clear();
      }
};




#endif // ISCENECONSERVATIONCHECKPOINT_H
