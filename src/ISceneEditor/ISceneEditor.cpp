#include "ISceneEditor.h"

#include "IUtilityOpenGLDraw.hpp"



ISceneEditor::ISceneEditor(IVivwer *_PtrViewportScene)
: mPtrViewportScene(_PtrViewportScene)
{
   assert(_PtrViewportScene);
}

bool ISceneEditor::initialization()
{
       assert(mPtrViewportScene);

        mWidth  = mPtrViewportScene->mWidth;
        mHeight = mPtrViewportScene->mHeight;


       //-------------------------------------------------------------------------//

       mSceneWorld = IEngineComponent::IScene3D::Create("wera");

       //-------------------------------------------------------------------------//


    return true;
}

void ISceneEditor::render(float FrameTime)
{
    //-------------------------- Render ---------------------------//

    mPtrViewportScene->BeginLookSceneOpenGL();



    glLineWidth(1);
    IUtilityOpenGLDraw::DrawGrid();

    glPushMatrix();
    glTranslatef(mPtrViewportScene->mCamera->Camera()->GetAt().x,
                 mPtrViewportScene->mCamera->Camera()->GetAt().y,
                 mPtrViewportScene->mCamera->Camera()->GetAt().z);
    IUtilityOpenGLDraw::DrawCube();
    glPopMatrix();


    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);


    for( auto it = mSceneWorld->GetComponentsBeginIterator(); it != mSceneWorld->GetComponentsEndIterator(); ++it )
    {
        if( (*it)->GetType() == IEngineComponent::ComponentType::COMPONENT_MESH3 )
        {
            IEngineComponent::IComponentMesh3 *item = static_cast<IEngineComponent::IComponentMesh3*>(*it);
            item->Mesh()->setTransformMatrix(item->GetTransformHierarchy());
            IUtilityOpenGLDraw::DrawComponentMeshLine(item);
            IUtilityOpenGLDraw::DrawComponentMeshTriangle(item);
        }
    }


    glLineWidth(5);
    if (mPtrGizmoManipulator)
    {
        if(!mGizmoGroup.GroupSVGMatrix().empty())
        {
            mPtrGizmoManipulator->Draw();
        }
    }


}



void ISceneEditor::update()
{

   if(mPtrGizmoManipulator)
   {
       if(!mGizmoGroup.GroupSVGMatrix().empty())
       {
           for( auto item : mGizmoGroup.GroupSVGMatrix() )
           {
               IEngine::IVector3 worldPoint =  mGizmoGroup.getTransformMatrix().GetTranslation();
               IEngine::IMatrix4x4 m = IEngine::IMatrix4x4::CreateTranslation( worldPoint) * mPtrGizmoManipulator->GetEditMatrix() *
                                       IEngine::IMatrix4x4::CreateTranslation(-worldPoint) * item.second;

               if(item.first->GetParent())
               {
                   IEngineComponent::IComponent3D *object_parent = static_cast<IEngineComponent::IComponent3D*>(item.first->GetParent());
                   item.first->SetTransform( object_parent->GetTransformHierarchy().GetInverse() * m);
               }
               else
               {
                   item.first->SetTransform(m);
               }
           }
       }
   }

}

void ISceneEditor::resize(float width, float height)
{
   mWidth  = width;
   mHeight = height;
}

void ISceneEditor::mouseMove(float x, float y, int button)
{
    if(button){}
    mUtilMouse.mouseX = x;
    mUtilMouse.mouseY = y;
}

void ISceneEditor::mousePress(float x, float y, int button )
{
    mUtilMouse.mouseX = x;
    mUtilMouse.mouseY = y;
    mUtilMouse.mMouseButton = button;


    if (mPtrGizmoManipulator && button == Qt::MouseButton::LeftButton)
    {

        if ( mPtrGizmoManipulator->isOnMouseDownSelected() && !mGizmoGroup.GroupSVGMatrix().empty())
        {
            //....
        }
        else
        {

            IEngine::IVector3 rayOrigin;
            IEngine::IVector3 rayDir;

            mPtrGizmoManipulator->BuildRay( mUtilMouse.mouseX , mUtilMouse.mouseY , rayOrigin , rayDir );
            IEngine::IRay _ray_cast( rayOrigin , rayDir );


            auto mSelectedComponent = mSceneWorld->RayCastingComponentBruteForce(_ray_cast);
            if(mSelectedComponent != nullptr)
            {
                if( !mUtilKeybard.KeyPressed(Qt::Key_J) )
                {
                    mGizmoGroup.Clear();
                }

                mGizmoGroup.InsertMatrix(mSelectedComponent , mSelectedComponent->GetTransformHierarchy());
                mPtrGizmoManipulator->SetEditMatrix(  mGizmoGroup.getTransformMatrix() );

            }
            else
            {
                mSelectedComponent = nullptr;
                mGizmoGroup.Clear();
            }

        }
    }
}


void ISceneEditor::mouseReleasePress(float x, float y, int button)
{
    mUtilMouse.mouseX = x;
    mUtilMouse.mouseY = y;
    mUtilMouse.mMouseButton = button;

    if(  mUtilMouse.mMouseButton == Qt::MouseButton::LeftButton )
    {
        if (mPtrGizmoManipulator)
        {
            if(mPtrGizmoManipulator->isTransformationChange())
            {
                ContexSaveScene context_save;
                context_save.isSaveComponent = false;
                context_save.isSaveHierarchyNode = false;
                ChaekSaveCurrentSceneState(context_save);
            }


            for( auto it = mGizmoGroup.GroupSVGMatrix().begin(); it != mGizmoGroup.GroupSVGMatrix().end(); ++it )
            {
                (*it).second = (*it).first->GetTransformHierarchy();
            }
        }
    }
}



void ISceneEditor::mouseWheel(float delta)
{

}

void ISceneEditor::realaseKeyboard(int key)
{

}

void ISceneEditor::keyboard(int key)
{
    if( key == Qt::Key_D)
    {
        std::cout << "key_D delete" << std::endl;

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
        // mSelectedComponent = nullptr;
    }


    if( key == Qt::Key_S )
    {
        std::cout << "key_S Save-Scene " << std::endl;
        ContexSaveScene _ContexSave { true , false };
        ChaekSaveCurrentSceneState(_ContexSave);
    }



    if( key == Qt::Key_Z )
    {
        std::cout << "key_Z Prev Scene  " << std::endl;
        PrevStoreCurrentSceneState();
    }

    if( key == Qt::Key_X )
    {
        std::cout << "key_X Next  Scene " << std::endl;
        NextStoreCurrentSceneState();
    }

    /**/

}

void ISceneEditor::destroy()
{
   mGizmoGroup.Clear();
   //mSceneWorld->RemoveAllNodes();
}


//-------------------------------------------------------------//


void ISceneEditor::RemoveCheckPointRealase()
{
    for( auto item : mSceneSaveCheckPoints.begin()->mCheckDeleteNodes )
    {
        mSceneWorld->DestroyComponent(item.ComponentNode());
    }

    mSceneSaveCheckPoints.begin()->Realase();
    mSceneSaveCheckPoints.erase(mSceneSaveCheckPoints.begin());
    mIterator_position_save_scene--;
}


void ISceneEditor::CheckSave(ISceneConservationCheckPoint SceneSaveCheckPoint)
{


    if(mIterator_position_save_scene > mMaximumCountOfFixationPoints)
    {
        RemoveCheckPointRealase();
    }

    if( mSceneSaveCheckPoints.size() > mIterator_position_save_scene && mSceneSaveCheckPoints.size() > 0)
    {
        mSceneSaveCheckPoints[mIterator_position_save_scene] = SceneSaveCheckPoint;
    }
    else
    {
        mSceneSaveCheckPoints.push_back(SceneSaveCheckPoint);
    }

    mMaximum_count_save_scene = mIterator_position_save_scene++;

}


//void ISceneEditor::ChaekSaveCurrentSceneStateDelete()
//{
//    ISceneConservationCheckPoint SceneSaveCheckPoint;
//    SceneSaveCheckPoint.Realase();
//    SceneSaveCheckPoint. mSaveStoreGroupSVGMatrix = mGizmoGroup.GroupSVGMatrix();

//    for( auto item : mGizmoGroup.GroupSVGMatrix() )
//    {
//        auto SelectedComponent = item.first;
//        SceneSaveCheckPoint.mCheckDeleteNodes.push_back( SaveComponentHierarchyCheckPoint(SelectedComponent) );
//        SelectedComponent = nullptr;
//    }

//    CheckSave(SceneSaveCheckPoint);
//}


void ISceneEditor::ChaekSaveCurrentSceneState(ContexSaveScene _ContexSave)
{
    ISceneConservationCheckPoint SceneSaveCheckPoint;
    SceneSaveCheckPoint.Realase();
    //SceneSaveCheckPoint.mSaveStoreGroupSVGMatrix = mGizmoGroup.GroupSVGMatrix();



    if( _ContexSave.isDeleteComponent )
    {
        for( auto item : mGizmoGroup.GroupSVGMatrix() )
        {
            auto SelectedComponent = item.first;
            SceneSaveCheckPoint.mCheckDeleteNodes.push_back( SaveComponentHierarchyCheckPoint(SelectedComponent) );
            SelectedComponent = nullptr;
        }

        CheckSave(SceneSaveCheckPoint);
        return;
    }

    //=========================================================//

    for( auto item : mGizmoGroup.GroupSVGMatrix() )
    {
        auto SelectedComponent = item.first;
        SceneSaveCheckPoint.mSaveStoreGroupSVGMatrixCopy[SelectedComponent] = SelectedComponent->GetTransform();
    }

    //=========================================================//

    if( _ContexSave.isSaveHierarchyNode )
    {
        for( auto item : mGizmoGroup.GroupSVGMatrix() )
        {
            auto SelectedComponent = item.first;
            SceneSaveCheckPoint.mCheckCopyNodes.push_back( SaveComponentHierarchyCheckPoint(SelectedComponent) );
            SelectedComponent = nullptr;
        }
    }

    //=========================================================//

    if( _ContexSave.isSaveComponent )
    {
        for( auto item : mGizmoGroup.GroupSVGMatrix() )
        {
            auto SelectedComponent = item.first;

            if( SelectedComponent->GetType() == IEngineComponent::ComponentType::COMPONENT_MESH3 )
            {
                SceneSaveCheckPoint.mSaveStoreElements[SelectedComponent] = new SaveStateElementComponentMesh3(SelectedComponent);
            }
            else if ( SelectedComponent->GetType() == IEngineComponent::ComponentType::COMPONENT_CAMERA )
            {
                SceneSaveCheckPoint.mSaveStoreElements[SelectedComponent] = new SaveStateElementCamera(SelectedComponent);
            }

            SelectedComponent = nullptr;
        }
    }

    //=========================================================//

    CheckSave(SceneSaveCheckPoint);
}


void ISceneEditor::PrevStoreCurrentSceneState()
{
    if(mIterator_position_save_scene > 0)
    {
        mIterator_position_save_scene--;
        ISceneConservationCheckPoint SceneStoreCheckPoint = mSceneSaveCheckPoints[mIterator_position_save_scene];

        //=========================================================//

//        for( auto item : SceneStoreCheckPoint.mSaveStoreGroupSVGMatrix )
//        {
//            item.first->SetTransform(item.second);
//        }

        for( auto item : SceneStoreCheckPoint.mSaveStoreGroupSVGMatrixCopy )
        {
            item.first->SetTransform(item.second);
        }

        //=========================================================//

        for( auto item : SceneStoreCheckPoint.mSaveStoreElements )
        {
            item.second->ReturnStoreState();
        }

        //=========================================================//

        for( auto item : SceneStoreCheckPoint.mCheckDeleteNodes )
        {
            mSceneWorld->AddComponent(item.ComponentNode());
        }

        for( auto item : SceneStoreCheckPoint.mCheckDeleteNodes )
        {
            item.RestoreHierarchy();
        }

        //=========================================================//

        for( auto item : SceneStoreCheckPoint.mCheckCopyNodes )
        {
            mSceneWorld->AddComponent(item.ComponentNode());
        }

        for( auto item : SceneStoreCheckPoint.mCheckCopyNodes )
        {
            item.RestoreHierarchy();
        }

        //=========================================================//

        mGizmoGroup.GroupSVGMatrix() = SceneStoreCheckPoint.mSaveStoreGroupSVGMatrixCopy;
        mGizmoGroup.setTransformMatrix(mGizmoGroup.InterpolationMatrix());

        //=========================================================//
    }


}



void ISceneEditor::NextStoreCurrentSceneState()
{
    if(mIterator_position_save_scene <= mMaximum_count_save_scene)
    {
        ISceneConservationCheckPoint SceneStoreCheckPoint = mSceneSaveCheckPoints[mIterator_position_save_scene];
        mIterator_position_save_scene++;

        //=========================================================//

//        for( auto item : SceneStoreCheckPoint.mSaveStoreGroupSVGMatrix )
//        {
//            item.first->SetTransform(item.second);
//        }


        for( auto item : SceneStoreCheckPoint.mSaveStoreGroupSVGMatrixCopy )
        {
            item.first->SetTransform(item.second);
        }

        //=========================================================//

        for( auto item : SceneStoreCheckPoint.mSaveStoreElements )
        {
            item.second->ReturnStoreState();
        }

        //=========================================================//

        mGizmoGroup.GroupSVGMatrix() = SceneStoreCheckPoint.mSaveStoreGroupSVGMatrixCopy;
        mGizmoGroup.setTransformMatrix(mGizmoGroup.InterpolationMatrix());


        if(!SceneStoreCheckPoint.mCheckDeleteNodes.empty())
        {
            for( auto item : SceneStoreCheckPoint.mCheckDeleteNodes )
            {
                auto SelectedComponent = item.ComponentNode();
                mSceneWorld->EraseComponent(SelectedComponent);
                SelectedComponent = nullptr;
            }

            mGizmoGroup.Clear();
        }

        //=========================================================//

//        for( auto item : SceneStoreCheckPoint.mCheckCopyNodes )
//        {
//            mSceneWorld->AddComponent(item.ComponentNode());
//        }

        for( auto item : SceneStoreCheckPoint.mCheckCopyNodes )
        {
            item.RestoreHierarchyRemove();
        }

        //=========================================================//
    }
}




