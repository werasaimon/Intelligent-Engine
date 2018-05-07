#ifndef IPLANE_H
#define IPLANE_H

#include "IVector3D.h"
#include "IQuaternion.h"
#include "ILine3D.h"
#include "ILineSegment3D.h"

namespace IMath
{

//-------------------------------------------------------------------------------
//-- Classes --------------------------------------------------------------------
//-------------------------------------------------------------------------------
template<class T>
class IPlane
{

protected:

    IVector3D<T> mNormal;
    T             mOffset;


public:
    // constructor/destructor

    // Set this plane from a normal and a signed distance from its origin.
    IPlane( const IVector3D<T>& normal = IVector3D<T>(1,0,0) , T offset = T(0) )
      : mNormal(normal) ,
        mOffset(offset)
    {

    }

    // Set this plane from a normal and a point on the plane.
    IPlane(const IVector3D<T>& _normal, const IVector3D<T>& _point)
    {
        mNormal = _normal;
        mOffset = _normal.dot(_point);
    }

    IPlane( const IVector3D<T>& p0,
             const IVector3D<T>& p1,
             const IVector3D<T>& p2 )
    {
       Set( p0, p1, p2 );
    }


    IPlane( T a, T b, T c, T d )
    {
       Set( a, b, c, d );
    }

    // copy operations
    IPlane(const IPlane& other)
     : mNormal( other.mNormal ),
       mOffset( other.mOffset )
    {

    }





    SIMD_INLINE ~IPlane() {}


    // ---------------------------------------------------------------------------
    // Assigment operator
    //-----------------------------------------------------------------------------
    IPlane& operator=(const IPlane& other)
    {
        // if same object
        if ( this == &other )
            return *this;

        mNormal = other.mNormal;
        mOffset = other.mOffset;

        return *this;
    }


    // accessors
    SIMD_INLINE const IVector3D<T>& GetNormal() const { return mNormal; }
    SIMD_INLINE T GetOffset() const { return mOffset; }

    // ---------------------------------------------------------------------------
    // Returns the two endpoints
    //-----------------------------------------------------------------------------
    void Get( IVector3D<T>& normal, T& offset ) const
    {
        normal = mNormal;
        offset = mOffset;
    }

    // ---------------------------------------------------------------------------
    // Are two IPlane's equal?
    //----------------------------------------------------------------------------
    bool operator==( const IPlane&  plane ) const
    {
        return (plane.mNormal == mNormal &&
                plane.mOffset == mOffset);
    }

    // ---------------------------------------------------------------------------
    // Are two IPlane's not equal?
    //----------------------------------------------------------------------------
    bool operator!=( const IPlane&  plane ) const
    {
        return !(plane.mNormal == mNormal &&
                 plane.mOffset == mOffset);
    }

    // manipulators
    SIMD_INLINE void Set( const IVector3D<T>& n, T d )
    {
        Set( n.x, n.y, n.z, d );
    }

    // ---------------------------------------------------------------------------
    // Sets the parameters
    //-----------------------------------------------------------------------------
    void Set( T a, T b, T c, T d )
    {
        // normalize for cheap distance checks
        T lensq = a*a + b*b + c*c;
        // length of normal had better not be zero
        assert( !iIsZero( lensq ) );

        // recover gracefully
        if ( iIsZero( lensq ) )
        {
            mNormal = IVector3D<T>::X;
            mOffset = 0.0f;
        }
        else
        {
            T recip = 1.0/iSqrt(lensq);
            mNormal.setAllValues( a*recip, b*recip, c*recip );
            mOffset = d*recip;
        }
    }



    // ---------------------------------------------------------------------------
    // Sets the parameters
    //-----------------------------------------------------------------------------
    void Set( const IVector3D<T>& p0, const IVector3D<T>& p1, const IVector3D<T>& p2 )
    {
        mNormal = (IVector3D<T>::triNormal(p0,p1,p2));
        mOffset = mNormal.dot(p0);
    }

    // ---------------------------------------------------------------------------
    // Transforms plane into new space
    //-----------------------------------------------------------------------------
    IPlane Transform( T scale, const IQuaternion<T>& rotate, const IVector3D<T>& translate ) const
    {
        IPlane<T> plane;

        // get rotation matrix
        IMatrix3x3<T>  rotmatrix = rotate.getMatrix();

        // transform to get normal
        plane.mNormal = rotmatrix*mNormal/scale;

        // transform to get offset
        IVector3D<T> newTrans = translate*rotmatrix;
        plane.mOffset = -newTrans.dot( mNormal )/scale + mOffset;

        return plane;
    }

    // ---------------------------------------------------------------------------
    // Transforms plane into new space
    //-----------------------------------------------------------------------------
    IPlane Transform( T scale, const IMatrix3x3<T>& rotmatrix, const IVector3D<T>& translate ) const
    {
        IPlane<T> plane;

        // transform to get normal
        plane.mNormal = rotmatrix*mNormal/scale;

        // transform to get offset
        IVector3D<T> newTrans = translate*rotmatrix;
        plane.mOffset = -newTrans.dot( mNormal )/scale + mOffset;

        return plane;
    }

    // distance
    static T Distance( const IPlane& plane, const IVector3D<T>& point )
    {
        return ( plane.Test( point ) );
    }

    // ---------------------------------------------------------------------------
    // Returns the closest point on plane to point
    //-----------------------------------------------------------------------------
    IVector3D<T> ClosestPoint( const IVector3D<T>& point ) const
    {
        return point - Test(point)*mNormal;
    }

    // result of plane test
    SIMD_INLINE T Test( const IVector3D<T>& point ) const
    {
        return mNormal.dot(point) - mOffset;
    }

    // result of plane test
    SIMD_INLINE T invTest( const IVector3D<T>& point ) const
    {
        return mOffset - mNormal.dot(point);
    }


    // vIntersectionLineToPlane(): find the 3D intersection of a segment and a plane
    //    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
    //    Output: *I0 = the intersect point (when it exists)
    //    Return: 0 = disjoint (no intersection)
    IVector3D<T> vIntersectionLineToPlane( const ILineSegment3D<T>& _edge , bool parallel_test) const
    {
        IVector3D<T> N = mNormal;
        IVector3D<T> P = _edge.GetEndpoint0();
        IVector3D<T> W = _edge.GetDirection();

        T  d =  invTest(P);
        T  e =  N.dot(W);

        if(parallel_test)
        if( IAbs(e) < MACHINE_EPSILON  ) return P;

        T param = d/e;
        return P + W * param;
    }


    //----------[ output operator ]----------------------------
     /**
     * Provides output to standard output stream.
     */
     friend std::ostream& operator <<(std::ostream& oss, const IPlane<T>& rhs)
     {
         oss << "(" << "normal: " << rhs.mNormal << " offset: " << rhs.mOffset << ")";
         return oss;
     }

     /**
     * Gets string representation.
     */
     std::string toString() const
     {
         std::ostringstream oss;
         oss << *this;
         return oss.str();
     }



private:
};



//--------------------------------------
// Typedef shortcuts for Plane
//-------------------------------------
/// Three dimensional Plane of floats
typedef IPlane<float> IPlanef;
/// Three dimensional Plane of doubles
typedef IPlane<double> IPlaned;
/// Three dimensional Plane of ints
typedef IPlane<int> IPlanei;


}


#endif // IPLANE_H
