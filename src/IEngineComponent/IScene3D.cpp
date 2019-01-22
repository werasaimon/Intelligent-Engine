#include "IScene3D.h"
#include "IComponentMesh3.h"

namespace IEngineComponent
{

    IScene3D::IScene3D()
      : IHierarchyScene()
    {

    }

    unsigned int IScene3D::ComputeNextAvailableBodyID()
    {
        // Compute the body ID
        unsigned int bodyID;
        if (!mFreeComponentsIDs.empty())
        {
            bodyID = mFreeComponentsIDs.back();
            mFreeComponentsIDs.pop_back();
        }
        else
        {
            bodyID = mCurrentBodyID;
            mCurrentBodyID++;
        }

        return bodyID;
    }



    IScene3D *IScene3D::Create(const char *id)
    {
        IScene3D* scene = new IScene3D();
        //scene->SetId(id);
        return scene;
    }

    std::set<IComponent3D*>::iterator IScene3D::GetComponentsBeginIterator()
    {
        return mComponents.begin();
    }

    std::set<IComponent3D*>::iterator IScene3D::GetComponentsEndIterator()
    {
        return mComponents.end();
    }


    IComponentMesh3 *IScene3D::CreateComponentMesh3( IEngine::IMesh3 *Mesh )
    {
        // Get the next available body ID
        unsigned int bodyID = ComputeNextAvailableBodyID();

        // Largest index cannot be used (it is used for invalid index)
        assert(bodyID < std::numeric_limits<unsigned int>::max());

        // Create the collision body
        IComponentMesh3* Component = new IComponentMesh3( Mesh , bodyID );

        assert(Component != NULL);

        if(!Component->GetParent())
        {
            IHierarchyScene::AddNode(Component);
        }
        else
        {
            Component->GetParent()->AddChild(Component);
        }

        // Add the collision body to the world
        mComponents.insert(Component);

        // Return the pointer to the rigid body
        return Component;
    }


    void IScene3D::AddComponent(IComponent3D *Component)
    {
        // Get the next available body ID
        unsigned int bodyID = ComputeNextAvailableBodyID();

        // Largest index cannot be used (it is used for invalid index)
        assert(bodyID < std::numeric_limits<unsigned int>::max());

        assert(Component != NULL);

        if(!Component->GetParent())
        {
            IHierarchyScene::AddNode(Component);
        }
        else
        {
            Component->GetParent()->AddChild(Component);
        }

        // Add the collision body to the world
        mComponents.insert(Component);
    }

    void IScene3D::DestroyComponent(IComponent3D *Component)
    {
        assert(Component);

        unsigned int id = Component->GetId();
        //unsigned int id = Component->mID;

        // Add the body ID to the list of free IDs
        mFreeComponentsIDs.push_back(id);


        RemoveNode(Component);
        if(Component->GetParent())
        {
           Component->GetParent()->RemoveChild(Component);
        }
        Component->RemoveAllChildren();


        // Call the destructor of the collision body
        delete Component;
        //Component = nullptr;

        // Remove the collision body from the list of bodies
        mComponents.erase(Component);
    }

    void IScene3D::EraseComponent(IComponent3D *Component)
    {
        unsigned int id = Component->GetId();
        //unsigned int id = Component->mID;

        // Add the body ID to the list of free IDs
        mFreeComponentsIDs.push_back(id);


        RemoveNode(Component);
        if(Component->GetParent())
        {
           Component->GetParent()->RemoveChild(Component);
        }
        Component->RemoveAllChildren();



        // Remove the collision body from the list of bodies
        mComponents.erase(Component);
    }


    IComponent3D *IScene3D::RayCastingComponentBruteForce(IEngine::IRay &_Ray)
    {
        float maxFriction = 0;
        IComponent3D *res = nullptr;
        for( auto item : mComponents )
        {
            auto component = static_cast<IComponent3D*>(item);
            bool isIntersection = component->GetAxisAlignedBoxTransform().IntersectsRay(_Ray);
            _Ray.maxFraction = _Ray.maxFraction;

            if( isIntersection )
            {
                if( res == nullptr )
                {
                    res = component;
                    maxFriction = _Ray.maxFraction;
                }
                else
                {
                    if( _Ray.maxFraction < maxFriction )
                    {
                        res = component;
                        maxFriction = _Ray.maxFraction;
                    }
                }

            }
        }

        return res;
    }



//    IComponent3D *IScene3D::RayCastingComponentBruteForce(IEngine::IRay &_Ray)
//    {

//        float maxFriction = 0;
//        IComponent3D *res = nullptr;
//        std::vector<IEngine::INode*> list_old_nodes = ListOldNodes();
//        for(std::size_t i = 0; i < list_old_nodes.size(); ++i)
//        {
//            auto component = static_cast<IComponent3D*>(list_old_nodes[i]);
//            bool isIntersection = component->GetAxisAlignedBoxTransform().IntersectsRay(_Ray);
//            _Ray.maxFraction = _Ray.maxFraction;

//            if( isIntersection )
//            {
//                if( res == nullptr )
//                {
//                    res = component;
//                    maxFriction = _Ray.maxFraction;
//                }
//                else
//                {
//                    if( _Ray.maxFraction < maxFriction )
//                    {
//                        res = component;
//                        maxFriction = _Ray.maxFraction;
//                    }
//                }

//            }
//        }

//        return res;
//    }


}

