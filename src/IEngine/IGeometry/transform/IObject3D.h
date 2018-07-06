#ifndef IOBJECT3D_H
#define IOBJECT3D_H

#include "../math/IMatematical.h"

namespace IGeometry
{

// Class Object3D
// This class represent a generic 3D object on the scene.
class IObject3D
{

    protected:

    // -------------------- Attributes -------------------- //

        // Transformation matrix that convert local-space
        // coordinates to world-space coordinates
        IMatrix4x4 mTransformMatrix;



    public:

        // -------------------- Methods -------------------- //

        // Constructor
        IObject3D();

        // Destructor
        virtual ~IObject3D();

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
inline const IMatrix4x4 &IObject3D::getTransformMatrix() const
{
    return mTransformMatrix;
}


inline void  IObject3D::setTransformMatrix( const IMatrix4x4& matrix )
{
    mTransformMatrix = matrix;
}


// Set to the identity transform
inline void IObject3D::setToIdentity()
{
    mTransformMatrix.setToIdentity();
}

 // Return the origin of object in world-space
inline IVector3 IObject3D::getOrigin() const
{
    return mTransformMatrix * IVector3(0.0, 0.0, 0.0);
}


//// Translate the object in world-space
inline void IObject3D::translateWorld(const IVector3& v)
{
    mTransformMatrix = IMatrix4x4::createTranslation(v) * mTransformMatrix;
}

// Translate the object in local-space
inline void IObject3D::translateLocal(const IVector3& v)
{
    mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation(v);
}

// Rotate the object in world-space
inline void IObject3D::rotateWorld(const IVector3& axis, float angle)
{
    mTransformMatrix = IMatrix4x4::createRotationAxis(axis, angle) * mTransformMatrix;
}

// Rotate the object in local-space
inline void IObject3D::rotateLocal(const IVector3& axis, float angle)
{
    mTransformMatrix = mTransformMatrix * IMatrix4x4::createRotationAxis(axis, angle);
}


/**/
// Rotate the object around a world-space point
inline void IObject3D::rotateAroundWorldPoint(const IVector3& axis, float angle, const IVector3& worldPoint)
{
    mTransformMatrix =   IMatrix4x4::createTranslation( worldPoint)  *
                         IMatrix4x4::createRotationAxis(axis, angle) *
                         IMatrix4x4::createTranslation(-worldPoint)  * mTransformMatrix;
}



// Rotate the object around a local-space point
inline void IObject3D::rotateAroundLocalPoint(const IVector3& axis, float angle, const IVector3& worldPoint)
{
    // Convert the world point into the local coordinate system
    IVector3 localPoint = mTransformMatrix.getInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation(localPoint)
                                        * IMatrix4x4::createRotationAxis(axis, angle)
                                        * IMatrix4x4::createTranslation(-localPoint);
}

// Scale around a world-space point
inline void IObject3D::scaleAroundWorldPoint(const IVector3 &axis, float scale, const IVector3 &worldPoint)
{
    mTransformMatrix =   IMatrix4x4::createTranslation( worldPoint) * IMatrix4x4::createScaleAroundAxis(axis, scale)
                       * IMatrix4x4::createTranslation(-worldPoint) * mTransformMatrix;
}

// Scale around a local-space poin
inline void IObject3D::scaleAroundLocalPoint(const IVector3 &axis, float scale, const IVector3 &worldPoint)
{
    // Convert the world point into the local coordinate system
    IVector3 localPoint = mTransformMatrix.getInverse() * worldPoint;

    mTransformMatrix = mTransformMatrix * IMatrix4x4::createTranslation(localPoint)
                                        * IMatrix4x4::createScaleAroundAxis(axis, scale)
                                        * IMatrix4x4::createTranslation(-localPoint);
}
/**/

}

#endif // IOBJECT3D_H
