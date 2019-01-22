#ifndef ICOLLISIONSHAPEHULL_H
#define ICOLLISIONSHAPEHULL_H

#include "ICollisionShapeConvex.h"
#include "../../IGeometry/IQuickConvexHull.h"



namespace IPhysics
{

    namespace
    {
        static IQuickHull<scalar>  QuickHullAlgorithm;
    }


struct ICalcModelHull
{

    //------------ Attribute -----------//
     IConvexHull<scalar> mConvexHull;

 public:

    ICalcModelHull( const IVector3 *axVertices , uint NbCount )
    {
        mConvexHull = QuickHullAlgorithm.getConvexHull( axVertices , NbCount , true, false);
    }


    ICalcModelHull( std::vector<IVector3> Vertices )
    {
        mConvexHull = QuickHullAlgorithm.getConvexHull( Vertices , true, false);
    }


    ~ICalcModelHull()
    {
        mConvexHull.getIndexBuffer().clear();
    }

};


struct ITriangleIndex
{
    u32 index_a;
    u32 index_b;
    u32 index_c;
};



struct IHullDescriptor
{
    u32             NbVertices;
    const IVector3 *Vertices=nullptr;

    u32   NbIndexes;
    lu32 *Indexes=nullptr;


    IHullDescriptor(ICalcModelHull* clacHull) :
        NbVertices(clacHull->mConvexHull.getVertexBuffer().size()) , Vertices(clacHull->mConvexHull.getVertexBuffer().data()) ,
        NbIndexes(clacHull->mConvexHull.getIndexBuffer().size()) , Indexes(clacHull->mConvexHull.getIndexBuffer().data())
    {


    }
};



class ICollisionShapeHull: public ICollisionShapeConvex
{

private:

    //-------------------- Attributes --------------------//
    u32             mNbVertices;
    const IVector3 *mVertices=nullptr;

    u32   mNbIndexes;
    lu32 *mIndexes=nullptr;


protected :

    /// Private copy-constructor
    ICollisionShapeHull(const ICollisionShapeHull& shape);

    /// Private assignment operator
    ICollisionShapeHull& operator=(const ICollisionShapeHull& shape);




    /// Return a local support interval ( minimum , maximum )
    void GetIntervalLocal(const IVector3 &xAxis, scalar &min, scalar &max) const;


    /// Return a local support point in a given direction without the object margin
    virtual IVector3 GetLocalSupportPointWithMargin(const IVector3& direction ) const;

    /// Return true if a point is inside the collision shape
    virtual bool TestPointInside(const IVector3& localPoint, IProxyShape* proxyShape) const;

    /// Raycast method with feedback information
    virtual bool Raycast(const IRay& ray, IRaycastInfo& raycastInfo, IProxyShape* proxyShape) const;

    /// Return the number of bytes used by the collision shape
    virtual size_t GetSizeInBytes() const;






public:

    ICollisionShapeHull(const IHullDescriptor &_HullDescriptor , scalar margin = OBJECT_MARGIN );

    virtual ~ICollisionShapeHull();


    /// Set the scaling vector of the collision shape
    virtual void SetLocalScaling(const IVector3& scaling);

    /// Return the local bounds of the shape in x, y and z directions
    virtual void GetLocalBounds(IVector3& min, IVector3& max) const;

    /// Return the local inertia tensor of the collision shape
    virtual void ComputeLocalInertiaTensor(IMatrix3x3& tensor, scalar mass) const;


};



}


#endif // ICOLLISIONSHAPEHULL_H
