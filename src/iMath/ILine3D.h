#ifndef ILINE3D_H
#define ILINE3D_H

#include "IVector3D.h"
#include "IQuaternion.h"
#include "IMatrix3x3.h"


namespace IMath
{


template<class T>
class ILine3D
{
public:
      // constructor/destructor
      ILine3D()
      : mOrigin( 0.0f, 0.0f, 0.0f ),
        mDirection( 1.0f, 0.0f, 0.0f )
      {

      }

      ILine3D( const IVector3D<T>& origin, const IVector3D<T>& direction )
      : mOrigin( origin ),
        mDirection( direction )
      {
          mDirection.normalize();
      }

      // copy operations
      ILine3D(const ILine3D& other)
       : mOrigin( other.mOrigin ),
         mDirection( other.mDirection )
      {

      }


      ~ILine3D() {}


      // ---------------------------------------------------------------------------
      // Assigment operator
      //-----------------------------------------------------------------------------
      ILine3D& operator=(const ILine3D& other)
      {
         // if same object
         if ( this == &other )
          return *this;

          mOrigin = other.mOrigin;
          mDirection = other.mDirection;
          mDirection.normalize();

          return *this;
      }

      // ---------------------------------------------------------------------------
      // Returns the two endpoints
      //-----------------------------------------------------------------------------
      void Get( IVector3D<T>& origin, IVector3D<T>& direction ) const
      {
          origin    = mOrigin;
          direction = mDirection;

      }

      // manipulators
      void Set( const IVector3D<T>& origin, const IVector3D<T>& direction )
      {
          mOrigin = origin;
          mDirection = direction;
          mDirection.normalize();
      }


      // accessors
      SIMD_INLINE IVector3D<T> GetOrigin() const { return mOrigin; }
      SIMD_INLINE IVector3D<T> GetDirection() const { return mDirection; }



      // comparison
      bool operator==( const ILine3D& line ) const
      {
          return (line.mOrigin == mOrigin && line.mDirection == mDirection);
      }

      bool operator!=( const ILine3D& line ) const
      {
          return !(line.mOrigin == mOrigin && line.mDirection == mDirection);
      }


      // ---------------------------------------------------------------------------
      // Transforms ray into new space
      //-----------------------------------------------------------------------------
      ILine3D Transform(T scale, const IQuaternion<T>& quad, const IVector3D<T>& translate) const
      {
          ILine3D<T>     line;
          IMatrix3x3<T>  rotate = quad.getMatrix();

          line.mDirection = rotate * mDirection;
          line.mDirection *= scale;

          line.mOrigin =  rotate * mOrigin;
          line.mOrigin *= scale;
          line.mOrigin += translate;

          return line;
      }

      // ---------------------------------------------------------------------------
      // Transforms ray into new space
      //-----------------------------------------------------------------------------
      ILine3D Transform(T scale, const IMatrix3x3<T>&  rotate, const IVector3D<T>& translate) const
      {
          ILine3D<T> line;

          line.mDirection  = rotate * mDirection;
          line.mDirection *= scale;

          line.mOrigin  = rotate * mOrigin;
          line.mOrigin *= scale;
          line.mOrigin += translate;

          return line;

      }

      // ---------------------------------------------------------------------------
      // Returns the distance squared between lines.
      //-----------------------------------------------------------------------------
      static T DistanceSquared( const ILine3D& line0, const ILine3D& line1,  T& s_c, T& t_c )
      {
           IVector3D<T> w0 = line0.mOrigin - line1.mOrigin;
           T a = line0.mDirection.dot( line0.mDirection );
           T b = line0.mDirection.dot( line1.mDirection );
           T c = line1.mDirection.dot( line1.mDirection );
           T d = line0.mDirection.dot( w0 );
           T e = line1.mDirection.dot( w0 );
           T denom = a*c - b*b;
           if ( IsZero(denom) )
           {
               s_c = 0.0f;
               t_c = e/c;
               IVector3D<T> wc = w0 - t_c*line1.mDirection;
               return wc.dot(wc);
           }
           else
           {
               s_c = ((b*e - c*d)/denom);
               t_c = ((a*e - b*d)/denom);
               IVector3D<T> wc = w0 + s_c*line0.mDirection
                                     - t_c*line1.mDirection;
               return wc.dot(wc);
       }
      }

      static T Distance( const ILine3D& line0, const ILine3D& line1,  T& s_c, T& t_c )
      {
          return ISqrt( DistanceSquared( line0, line1, s_c, t_c ) );
      }

      // ---------------------------------------------------------------------------
      // Returns the distance squared between line and point.
      //-----------------------------------------------------------------------------
      static T DistanceSquared( const ILine3D& line, const IVector3D<T>& point,   T &t_c )
      {
          IVector3D<T> w = point - line.mOrigin;
          T vsq = line.mDirection.dot(line.mDirection);
          T proj = w.dot(line.mDirection);
          t_c = proj/vsq;

          return w.dot(w) - t_c*proj;
      }

      static T Distance( const ILine3D& line, const IVector3D<T>& point,  T &t_c )
      {
          return ISqrt( DistanceSquared( line, point, t_c ) );
      }


      //----------------------------------------------------------------------------
      // @ ClosestPoints()
      // ---------------------------------------------------------------------------
      // Returns the closest points between two lines
      //-----------------------------------------------------------------------------
      static void ClosestPoints( const ILine3D& line0,  const ILine3D& line1 , IVector3D<T>& point0, IVector3D<T>& point1 )
      {
          // compute intermediate parameters
             IVector3D<T> w0 = line0.mOrigin - line1.mOrigin;
             T a = line0.mDirection.dot( line0.mDirection );
             T b = line0.mDirection.dot( line1.mDirection );
             T c = line1.mDirection.dot( line1.mDirection );
             T d = line0.mDirection.dot( w0 );
             T e = line1.mDirection.dot( w0 );

             T denom = a*c - b*b;

             if ( IsZero(denom) )
             {
                 point0 = line0.mOrigin;
                 point1 = line1.mOrigin + (e/c)*line1.mDirection;
             }
             else
             {
                 point0 = line0.mOrigin + ((b*e - c*d)/denom)*line0.mDirection;
                 point1 = line1.mOrigin + ((a*e - b*d)/denom)*line1.mDirection;
         }
      }

      // ---------------------------------------------------------------------------
      // Returns the closest point on line to point.
      //-----------------------------------------------------------------------------
      IVector3D<T> ClosestPoint( const IVector3D<T>& point ) const
      {
          IVector3D<T> w = point - mOrigin;
          T vsq  = mDirection.dot(mDirection);
          T proj = w.dot(mDirection);

          return mOrigin + (proj/vsq)*mDirection;
      }


      //----------[ output operator ]----------------------------
      /**
      * Provides output to standard output stream.
      */
      friend std::ostream& operator <<(std::ostream& oss, const ILine3D<T>& rhs)
      {
    	  oss << "(" << "origin: " << rhs.mOrigin << " direction: " << rhs.mDirection << ")";
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



  protected:

      IVector3D<T> mOrigin;
      IVector3D<T> mDirection;

};


/// Two dimensional Line3 of floats
typedef class ILine3D<float> Line3f;
/// Two dimensional Line3 of doubles
typedef class ILine3D<double> Line3d;
/// Two dimensional Line3 of ints
typedef class ILine3D<int> Line3i;

}
#endif // ILINE3D_H
