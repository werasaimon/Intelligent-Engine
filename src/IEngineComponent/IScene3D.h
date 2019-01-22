#ifndef ISCENE3D_H
#define ISCENE3D_H

#include "../IEngineComponent/IHierarchy/IHierarchyScene.h"
#include "IComponent3D.h"

namespace IEngineComponent
{

class IComponentMesh3;

class IScene3D : public IHierarchyScene
{

    private:

      /// Current Component ID
      unsigned int  mCurrentBodyID;

      /// Current Component Free IDs
      std::vector<unsigned int>  mFreeComponentsIDs;

      /// Current Components
      std::set<IComponent3D*>  mComponents;


    public:

            /**
             * Creates a new empty scene.
             *
             * @param id ID of the new scene, or NULL to use an empty string for the ID (default).
             *
             * @return The newly created empty scene.
             * @script{create}
             */
            static IScene3D* Create(const char* id = NULL);



            /// Return an iterator to the beginning of the bodies of the physics world
            std::set<IComponent3D*>::iterator GetComponentsBeginIterator();

            /// Return an iterator to the end of the bodies of the physics world
            std::set<IComponent3D*>::iterator GetComponentsEndIterator();



            IComponentMesh3 *CreateComponentMesh3( IEngine::IMesh3 *Mesh );



            /**
             * @Create one of the scene components
             * @script{Create}
             */
            void AddComponent( IComponent3D* component );


            /**
             * @Destroy one of the scene components
             * @script{destroy}
             */
            void DestroyComponent(IComponent3D* Component);



            void EraseComponent(IComponent3D* Component);


            /**
             * @return Raytrace select one of the scene components
             * @script{getting}
             */
            IComponent3D* RayCastingComponentBruteForce( IEngine::IRay &_Ray );



            std::set<IComponent3D*> GetComponents() const
            {
                return mComponents;
            }


    protected:

            /**
            * Constructor.
            */
           IScene3D();

           /**
            * Hidden copy constructor.
            */
           IScene3D(const IScene3D& copy);

           /**
            * Hidden copy assignment operator.
            */
           IScene3D& operator=(const IScene3D&);


           unsigned int ComputeNextAvailableBodyID();
};



}

#endif // ISCENE3D_H
