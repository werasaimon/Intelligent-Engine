#ifndef IOBJECTTRANSFORM_H
#define IOBJECTTRANSFORM_H


#include "../ICommon/IMatematical.h"

namespace IEngine
{

// Class Object3D
// This class represent a generic 3D object on the scene.
class IObjectTransform
{

    protected:

    // -------------------- Attributes -------------------- //

        // Transformation matrix that convert local-space
        // coordinates to world-space coordinates
        IMatrix4x4 mTransformMatrix;



    public:

        // -------------------- Methods -------------------- //

        // Constructor
        IObjectTransform();

        // Destructor
        virtual ~IObjectTransform();

        // Return the transform matrix
        const IMatrix4x4 &getTransformMatrix() const;

        // Set to the transform matrix
        void  setTransformMatrix( const IMatrix4x4& matrix );



        // Set to the identity transform
        void setToIdentity();

        // Return the origin of object in world-space
        IVector3 getOrigin() const;

        // Translate the object in world-space
        void translateWorld(const IVector3& v);

        // Translate the object in local-space
        void translateLocal(const IVector3& v);

        // Rotate the object in world-space
        void rotateWorld(const IVector3& axis, float angle);

        // Rotate the object in local-space
        void rotateLocal(const IVector3& axis, float angle);

        /**/
        // Rotate around a world-space point
        void rotateAroundWorldPoint(const IVector3& axis, float angle, const IVector3& point);

        // Rotate around a local-space point
        void rotateAroundLocalPoint(const IVector3& axis, float angle, const IVector3& worldPoint);
        /**/


        /**/
        // Scale around a world-space point
        void scaleAroundWorldPoint(const IVector3& axis, float scale, const IVector3& point);

        // Scale around a local-space point
        void scaleAroundLocalPoint(const IVector3& axis, float scale, const IVector3& worldPoint);
        /**/

};

// Return the transform matrix
inline const IMatrix4x4 &IObjectTransform::getTransformMatrix() const
{
    return mTransformMatrix;
}


inline void  IObjectTransform::setTransformMatrix( const IMatrix4x4& matrix )
{
    mTransformMatrix = matrix;
}


// Set to the identity transform
inline void IObjectTransform::setToIdentity()
{
    mTransformMatrix.SetToIdentity();
}

 // Return the origin of object in world-space
inline IVector3 IObjectTransform::getOrigin() const
{
    return mTransformMatrix * IVector3(0.0, 0.0, 0.0);
}


//// Translate the object in world-space
inline void IObjectTransform::translateWorld(const IVector3& v)
{
    mTransformMatrix = IMatrix4x4::CreateTranslation(v) * mTransformMatrix;
}

// Translate the object in local-space
inline void IObjectTransform::translateLocal(const IVector3& v)
{
    mTransformMatrix = mTransformMatrix * IMatrix4x4::CreateTranslation(v);
}

// Rotate the object in world-space
inline void IObjectTransform::rotateWorld(const IVector3& axis, float angle)
{
    mTransformMatrix = IMatrix4x4::CreateRotationAxis(axis, angle) * mTransformMatrix;
}

// Rotate the object in local-space
inline void IObjectTransform::rotateLocal(const IVector3& axis, float angle)
{
    mTransformMatrix = mTransformMatrix * IMatrix4x4::CreateRotationAxis(axis, angle);
}


/**/
// Rotate the object around a world-space point
inline void IObjectTransform::rotateAroundWorldPoint(const IVector3& axis, float angle, const IVector3& worldPoint)
{
    mTransformMatrix =   IMatrix4x4::CreateTranslation( worldPoint)  *
                         IMatrix4x4::CreateRotationAxis(axis, angle) *
                         IMatrix4x4::CreateTranslation(-worldPoint)  * mTransformMatrix;
}



// Rotate the object around a local-space point
inline void IObjectTransform::rotateAroundLocalPoint(const IVector3& axis, float angle, const IVector3& worldPoint)
{
    // Convert the world point into the local coordinate system
    IVector3 localPoint = mTransformMatrix.GetInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * IMatrix4x4::CreateTranslation(localPoint)
                                        * IMatrix4x4::CreateRotationAxis(axis, angle)
                                        * IMatrix4x4::CreateTranslation(-localPoint);
}

// Scale around a world-space point
inline void IObjectTransform::scaleAroundWorldPoint(const IVector3 &axis, float scale, const IVector3 &worldPoint)
{
    mTransformMatrix =   IMatrix4x4::CreateTranslation( worldPoint) * IMatrix4x4::CreateScaleAroundAxis(axis, scale)
                       * IMatrix4x4::CreateTranslation(-worldPoint) * mTransformMatrix;
}

// Scale around a local-space poin
inline void IObjectTransform::scaleAroundLocalPoint(const IVector3 &axis, float scale, const IVector3 &worldPoint)
{
    // Convert the world point into the local coordinate system
    IVector3 localPoint = mTransformMatrix.GetInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * IMatrix4x4::CreateTranslation(localPoint)
                                        * IMatrix4x4::CreateScaleAroundAxis(axis, scale)
                                        * IMatrix4x4::CreateTranslation(-localPoint);
}
/**/

}

#endif // IOBJECTTRANSFORM_H
