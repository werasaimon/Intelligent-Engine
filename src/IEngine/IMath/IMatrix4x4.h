 /********************************************************************************
 *
 * IMatrix4x4.h
 *
 * IMath : 3d_math library,
 * Copyright (c)  *
 * Created on: 3 July. 2018 г.
 * Author: werasaimon                                     *
 *********************************************************************************
 *                                                                               *
 * This software is provided 'as-is', without any express or implied warranty.   *
 * In no event will the authors be held liable for any damages arising from the  *
 * use of this software.                                                         *
 *                                                                               *
 * Permission is granted to anyone to use this software for any purpose,         *
 * including commercial applications, and to alter it and redistribute it        *
 * freely, subject to the following restrictions:                                *
 *                                                                               *
 * 1. The origin of this software must not be misrepresented; you must not claim *
 *    that you wrote the original software. If you use this software in a        *
 *    product, an acknowledgment in the product documentation would be           *
 *    appreciated but is not required.                                           *
 *                                                                               *
 * 2. Altered source versions must be plainly marked as such, and must not be    *
 *    misrepresented as being the original software.                             *
 *                                                                               *
 * 3. This notice may not be removed or altered from any source distribution.    *
 *                                                                               *
 ********************************************************************************/

#ifndef IMATRIX4X4_H_
#define IMATRIX4X4_H_

#include "ILorentzVector.h"
#include "IMatrix3x3.h"


namespace IMath
{

/**
 * Class for matrix 4x4
 * @note Data stored in this matrix are in column major order. This arrangement suits OpenGL.
 * If you're using row major matrix, consider using fromRowMajorArray as way for construction
 * Matrix4<T> instance.
 */
template<class T>
class IMatrix4x4
{
public:


    //! Specifies the typename of the scalar components.
    using ScalarType = T;

    //! Specifies the number of vector components.
    static const std::size_t components = 4*4;

private:

    //-------------------- Attributes --------------------//

    union
    {
        T            mData[16];
        IVector4D<T> mRows[4];
    };

 public:

    //--------------------------[ constructors ]-------------------------------
    /**
     *Creates identity matrix
     */
    SIMD_INLINE IMatrix4x4()
    {
        // Initialize all values in the matrix to zero
        setAllValues(1.0, 0.0, 0.0, 0.0 ,
                     0.0, 1.0, 0.0, 0.0 ,
                     0.0, 0.0, 1.0, 0.0 ,
                     0.0, 0.0, 0.0, 1.0);
    }

    /**
     * Copy matrix values from array (these data must be in column
     * major order!)
     */
    SIMD_INLINE IMatrix4x4(const T * dt)
    {
        std::memcpy(mData, dt, sizeof(T) * 16);
    }

    /**
     * Copy constructor.
     * @param src Data source for new created instance of IMatrix4x4.
     */
    SIMD_INLINE IMatrix4x4(const IMatrix4x4<T>& src)
    {
        std::memcpy(mData, src.mData, sizeof(T) * 16);
    }

    /**
     * Copy constructor.
     * @param src Data source for new created instance of IMatrix4x4.
     */
    SIMD_INLINE IMatrix4x4(const IMatrix3x3<T>& src)
    {
        mRows[0][0]  = src[0][0];
        mRows[1][0]  = src[1][0];
        mRows[2][0]  = src[2][0];
        mRows[3][0]  = 0.0;

        mRows[0][1]  = src[0][1];
        mRows[1][1]  = src[1][1];
        mRows[2][1]  = src[2][1];
        mRows[3][1]  = 0.0;

        mRows[0][2]  = src[0][2];
        mRows[1][2]  = src[1][2];
        mRows[2][2]  = src[2][2];
        mRows[3][2]  = 0.0;

        mRows[0][3]  = 0.0;
        mRows[1][3]  = 0.0;
        mRows[2][3]  = 0.0;
        mRows[3][3]  = 1.0;

    }

    // Constructor
    SIMD_INLINE IMatrix4x4<T>(T value)
    {
        setAllValues(value, value, value, value,
                     value, value, value, value,
                     value, value, value, value,
                     value, value, value, value);
    }

    // Constructor
    SIMD_INLINE IMatrix4x4<T>(T a1, T a2, T a3, T a4,
                               T b1, T b2, T b3, T b4,
                               T c1, T c2, T c3, T c4,
                               T d1, T d2, T d3, T d4)
    {
        // Initialize the matrix with the values
        setAllValues(a1, a2, a3, a4 ,
                     b1, b2, b3, b4 ,
                     c1, c2, c3, c4 ,
                     d1, d2, d3, d4);
    }


    // Constructor with arguments
    SIMD_INLINE IMatrix4x4( T data[4][4] )
    {
        setAllValues(data[0][0], data[0][1], data[0][2], data[0][3],
                     data[1][0], data[1][1], data[1][2], data[1][3],
                     data[2][0], data[2][1], data[2][2], data[2][3],
                     data[3][0], data[3][1], data[3][2], data[3][3]);
    }


    /**
     * Copy casting constructor.
     * @param src Data source for new created instance of IMatrix4x4.
     */
    template<class FromT>
    SIMD_INLINE IMatrix4x4(const IMatrix4x4<FromT>& src)
    {
        for (int i = 0; i < 16; i++)
        {
            mData[i] = static_cast<T>(src.mData[i]);
        }
    }



   //---------------------- Methods ---------------------//

    /**
    * Resets matrix to be identity matrix
    */
    SIMD_INLINE void setToIdentity()
    {
    	// Initialize all values in the matrix to identity
    	setAllValues(1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, 0.0, 0.0, 1.0);
    }



    /**
    * Resets matrix to zero
    */
    SIMD_INLINE void setToZero()
    {
      // Initialize all values in the matrix to zero
      mRows[0].setToZero();
      mRows[1].setToZero();
      mRows[2].setToZero();
      mRows[3].setToZero();
    }



    /**
     * Resets matrix to be value matrix
     */
    SIMD_INLINE void setAllValues(T a1, T a2, T a3, T a4,
                                  T b1, T b2, T b3, T b4,
                                  T c1, T c2, T c3, T c4,
                                  T d1, T d2, T d3, T d4)
    {
        mRows[0][0] = a1; mRows[0][1] = a2; mRows[0][2] = a3; mRows[0][3] = a4;
        mRows[1][0] = b1; mRows[1][1] = b2; mRows[1][2] = b3; mRows[1][3] = b4;
        mRows[2][0] = c1; mRows[2][1] = c2; mRows[2][2] = c3; mRows[2][3] = c4;
        mRows[3][0] = d1; mRows[3][1] = d2; mRows[3][2] = d3; mRows[3][3] = d4;
    }


    /**
     * normalized matrix to be value matrix3x3
     */
    void OrthoNormalize()
    {
       mRows[0].normalize();
       mRows[1].normalize();
       mRows[2].normalize();
    }


    //---------------------[ Equality operators ]------------------------------

    /**
     * Equality test operator
     * @param rhs Right hand side argument of binary operator.
     * @note Test of equality is based of threshold MACHINE_EPSILON value. To be two
     * values equal, must satisfy this condition all elements of matrix
     * | lhs[i] - rhs[i] | < MACHINE_EPSILON,
     * same for y-coordinate, z-coordinate, and w-coordinate.
     */
    SIMD_INLINE bool operator==(const IMatrix4x4<T>& rhs) const
    {
        for (int i = 0; i < 16; i++)
        {
            if (IAbs(mData[i] - rhs.mData[i]) >= MACHINE_EPSILON )
                return false;
        }
        return true;
    }

    /**
     * Inequality test operator
     * @param rhs Right hand side argument of binary operator.
     * @return not (lhs == rhs) :-P
     */
    SIMD_INLINE bool operator!=(const IMatrix4x4<T>& rhs) const
    {
        return !(*this == rhs);
    }





    //-------------[ conversion data ]-----------------------------

    /**
     * Conversion to pointer operator
     * @return Pointer to internally stored (in management of class IMatrix4x4<T>)
     * used for passing IMatrix4x4<T> values to gl*[fd]v functions.
     */
    SIMD_INLINE operator T*()
    {
        return (T*) &mRows[0][0];
    }

    /**
     * Conversion to pointer operator
     * @return Constant Pointer to internally stored (in management of class IMatrix4x4<T>)
     * used for passing IMatrix4x4<T> values to gl*[fd]v functions.
     */
    SIMD_INLINE operator const T*() const
    {
        return (const T*) &mRows[0][0];
    }


    /**
    * Conversion to pointer operator
    */
    SIMD_INLINE const T* getData() const
    {
        return &mRows[0][0];
    }

   /**
    * Conversion to pointer operator
    */
    SIMD_INLINE T* getData()
    {
        return &mRows[0][0];
    }


    /// Overloaded operator to read element of the matrix.
    SIMD_INLINE const IVector4D<T>& operator[](int row) const
    {
       return mRows[row];
    }

    /// Overloaded operator to read/write element of the matrix.
    SIMD_INLINE IVector4D<T>& operator[](int row)
    {
       return mRows[row];
    }



    /// Return a column
    SIMD_INLINE IVector4D<T> getColumn(int i) const
    {
        assert(i>= 0 && i<4);
        return IVector4D<T> (mRows[0][i], mRows[1][i], mRows[2][i] , mRows[3][i]);
    }

    /// Return a row
    SIMD_INLINE IVector4D<T> getRow(int i) const
    {
        assert(i>= 0 && i<4);
        return mRows[i];
    }



    //---------------------[ assignment operations ]---------------------------------

    /**
     * Copy operator
     * @param rhs Right hand side argument of binary operator.
     */
    SIMD_INLINE IMatrix4x4<T>& operator=(const IMatrix4x4<T>& rhs)
    {
        std::memcpy(mData, rhs.mData, sizeof(T) * 16);
        return *this;
    }

    /**
     * Copy casting operator
     * @param rhs Right hand side argument of binary operator.
     */
    template<class FromT>
    SIMD_INLINE IMatrix4x4<T>& operator=(const IMatrix4x4<FromT>& rhs)
    {
        for (int i = 0; i < 16; i++)
        {
            mData[i] = static_cast<T>(rhs.mData[i]);
        }
        return *this;
    }

    /**
     * Copy operator
     * @param rhs Right hand side argument of binary operator.
     */
    SIMD_INLINE IMatrix4x4<T>& operator=(const T* rhs)
    {
        std::memcpy(mData, rhs, sizeof(T) * 16);
        return *this;
    }


    /**
     * Sets rotation part (matrix 3x3) of matrix.
     *
     * @param m Rotation part of matrix
     */
    SIMD_INLINE void setRotation(const IMatrix3x3<T>& m)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                mRows[i][j]  = m[i][j];
            }
        }
    }

    /**
     * Sets position part (vector3) of matrix.
     *
     * @param m Position part of matrix
     */
    SIMD_INLINE void setPosition(const IVector3D<T>& v)
    {
        mRows[3][0]  = v.x;
        mRows[3][1]  = v.y;
        mRows[3][2]  = v.z;
        mRows[3][3]  = 1.0;
    }



    //--------------------[ matrix with matrix operations ]---------------------
    /**
     * Addition operator
     * @param rhs Right hand side argument of binary operator.
     */
     SIMD_INLINE IMatrix4x4<T> operator+(const IMatrix4x4<T>& rhs) const
     {
        IMatrix4x4<T> ret;
        for (int i = 0; i < 4; i++)   ret.mRows[i] = mRows[i] + rhs.mRows[i];
        return ret;
     }

     /**
      * Subtraction operator
      * @param rhs Right hand side argument of binary operator.
      */
      SIMD_INLINE IMatrix4x4<T> operator-(const IMatrix4x4<T>& rhs) const
      {
         IMatrix4x4<T> ret;
         for (int i = 0; i < 4; i++)   ret.mRows[i] = mRows[i] - rhs.mRows[i];
         return ret;
      }






    //--------------------[ matrix with scalar operations ]---------------------

    /**
    * Addition operator
    * @param rhs Right hand side argument of binary operator.
    */
    SIMD_INLINE IMatrix4x4<T> operator+(T rhs) const
    {
       IMatrix4x4<T> ret;
       for (int i = 0; i < 4; i++) ret.mRows[i] = mRows[i] + rhs;
       return ret;
    }

    /**
    * Subtraction operator
    * @param rhs Right hand side argument of binary operator.
    */
    SIMD_INLINE IMatrix4x4<T> operator-(T rhs) const
    {
      IMatrix4x4<T> ret;
      for (int i = 0; i < 4; i++) ret.mRows[i] = mRows[i] - rhs;
      return ret;
    }


    /// Overloaded operator for multiplication with a number
    friend SIMD_INLINE IMatrix4x4<T>  operator*(T nb, const IMatrix4x4<T>& matrix)
    {
        return IMatrix4x4<T>(matrix.mRows[0][0] * nb, matrix.mRows[0][1] * nb, matrix.mRows[0][2] * nb, matrix.mRows[0][3] * nb,
                              matrix.mRows[1][0] * nb, matrix.mRows[1][1] * nb, matrix.mRows[1][2] * nb, matrix.mRows[1][3] * nb,
                              matrix.mRows[2][0] * nb, matrix.mRows[2][1] * nb, matrix.mRows[2][2] * nb, matrix.mRows[2][3] * nb,
                              matrix.mRows[3][0] * nb, matrix.mRows[3][1] * nb, matrix.mRows[3][2] * nb, matrix.mRows[3][3] * nb);
    }

    /// Overloaded operator for multiplication with a matrix
    friend SIMD_INLINE IMatrix4x4<T>  operator*(const IMatrix4x4<T>& matrix, T nb)
    {
        return nb * matrix;
    }


    /// Overloaded operator for invert multiplication with a number
    friend SIMD_INLINE IMatrix4x4<T>  operator/(T nb, const IMatrix4x4<T>& matrix)
    {
        return IMatrix4x4<T>(matrix.mRows[0][0] / nb, matrix.mRows[0][1] / nb, matrix.mRows[0][2] / nb, matrix.mRows[0][3] / nb,
                              matrix.mRows[1][0] / nb, matrix.mRows[1][1] / nb, matrix.mRows[1][2] / nb, matrix.mRows[1][3] / nb,
                              matrix.mRows[2][0] / nb, matrix.mRows[2][1] / nb, matrix.mRows[2][2] / nb, matrix.mRows[2][3] / nb,
                              matrix.mRows[3][0] / nb, matrix.mRows[3][1] / nb, matrix.mRows[3][2] / nb, matrix.mRows[3][3] / nb);
    }

    /// Overloaded operator for invert multiplication with a matrix
    friend SIMD_INLINE IMatrix4x4<T>  operator/(const IMatrix4x4<T>& matrix, T nb)
    {
        return nb / matrix;
    }



    /**
     * Vector multiplication operator.
     *
     * Multiplies the matrix `rhs` on the left by the row vector `lhs`,
     * returning the resulting vector.
     *
     * @param lhs The matrix.
     * @param rhs The row vector.
     * @return The vector `lhs` multiplied by the matrix `rhs` on the right.
     */
    friend SIMD_INLINE IVector3D<T> operator*(const IVector3D<T>& rhs , const IMatrix4x4<T>& lhs)
    {
    	return lhs * rhs;
    }

    /**
     * Vector multiplication operator.
     *
     * Multiplies the matrix `rhs` on the left by the row vector `lhs`,
     * returning the resulting vector.
     *
     * @param lhs The matrix.
     * @param rhs The row vector.
     * @return The vector `lhs` multiplied by the matrix `rhs` on the right.
     */
    friend SIMD_INLINE IVector4D<T> operator*(const IVector4D<T>& rhs , const IMatrix4x4<T>& lhs)
    {
    	return lhs * rhs;
    }

    //--------------------[ multiply operators ]--------------------------------
    /**
    * Multiplication operator
    * @param rhs Right hand side argument of binary operator.
    */
    SIMD_INLINE IVector4D<T> operator*(const IVector4D<T>& rhs) const
    {
        IVector4D<T> u =IVector4D<T>(mRows[0][0]*rhs.x + mRows[0][1]*rhs.y + mRows[0][2]*rhs.z + rhs.w*mRows[0][3],
                                       mRows[1][0]*rhs.x + mRows[1][1]*rhs.y + mRows[1][2]*rhs.z + rhs.w*mRows[1][3],
                                       mRows[2][0]*rhs.x + mRows[2][1]*rhs.y + mRows[2][2]*rhs.z + rhs.w*mRows[2][3],
                                       mRows[3][0]*rhs.x + mRows[3][1]*rhs.y + mRows[3][2]*rhs.z + rhs.w*mRows[3][3]);

        return u;
    }


    /**
    * Multiplication operator
    * @param rhs Right hand side argument of binary operator.
    */
    SIMD_INLINE ILorentzVector<T> operator*(const ILorentzVector<T>& rhs) const
    {
       ILorentzVector<T> u = ILorentzVector<T>(mRows[0][0]*rhs.x + mRows[0][1]*rhs.y + mRows[0][2]*rhs.z + rhs.t*mRows[0][3],
                                                 mRows[1][0]*rhs.x + mRows[1][1]*rhs.y + mRows[1][2]*rhs.z + rhs.t*mRows[1][3],
                                                 mRows[2][0]*rhs.x + mRows[2][1]*rhs.y + mRows[2][2]*rhs.z + rhs.t*mRows[2][3],
                                                 mRows[3][0]*rhs.x + mRows[3][1]*rhs.y + mRows[3][2]*rhs.z + rhs.t*mRows[3][3]);

        return u;
    }


    /**
    * Multiplication operator
    * @param rhs Right hand side argument of binary operator.
    */
    SIMD_INLINE IVector3D<T> operator*(const IVector3D<T>& rhs) const
    {

        IVector3D<T> Point;
        Point.x = ( rhs.x * mRows[0][0] + rhs.y * mRows[0][1] + rhs.z * mRows[0][2] + mRows[3][0]);
        Point.y = ( rhs.x * mRows[1][0] + rhs.y * mRows[1][1] + rhs.z * mRows[1][2] + mRows[3][1]);
        Point.z = ( rhs.x * mRows[2][0] + rhs.y * mRows[2][1] + rhs.z * mRows[2][2] + mRows[3][2]);

        T w = mRows[0][3]*rhs.x +
              mRows[1][3]*rhs.y +
              mRows[2][3]*rhs.z +
              mRows[3][3];
         return Point / w;

    }


//    SIMD_INLINE IVector3D<T> operator*(const IVector3D<T>& rhs) const
//    {
//        IVector3D<T> u =IVector3D<T>(rhs.x * mRows[0][0] + rhs.y * mRows[1][0] + rhs.z * mRows[2][0] + mRows[3][0],
//                                     rhs.x * mRows[0][1] + rhs.y * mRows[1][1] + rhs.z * mRows[2][1] + mRows[3][1],
//                                     rhs.x * mRows[0][2] + rhs.y * mRows[1][2] + rhs.z * mRows[2][2] + mRows[3][2]);

//        float w = mRows[3][0]*rhs.x +
//                  mRows[3][1]*rhs.y +
//                  mRows[3][2]*rhs.z +
//                  mRows[3][3];

//        return u/w;
//    }


    /**
     * Matrix multiplication operator.
     *
     * @note Matrix multiplication is not commutative.
     *
     * @param lhs The left hand side matrix.
     * @param rhs The right hand side matrix.
     * @return The matrix equal to the product `lhs` x `rhs`.
     */
    SIMD_INLINE IMatrix4x4<T> operator*(IMatrix4x4<T> rhs) const
    {
        IMatrix4x4<T> w;
    	for(int i = 0; i < 4; i++)
    	{
    		for (int j = 0; j < 4; j++)
    		{
    			T n = 0;
    			for (int k = 0; k < 4; k++)
    			{
    				n += rhs.mRows[i][k] * mRows[k][j];
    			}
    			w.mRows[i][j] = n;
    		}
    	}

    	return w;
    }




    //---------------------------[ misc operations ]----------------------------

    /**
    * Computes of matrix_3x3
    * @return recombinate of matrix_3x3
    */
    SIMD_INLINE IMatrix3x3<T> getRotMatrix() const
    {
        IMatrix3x3<T> M;

        M[0][0] = mRows[0][0];
        M[1][0] = mRows[1][0];
        M[2][0] = mRows[2][0];

        M[0][1] = mRows[0][1];
        M[1][1] = mRows[1][1];
        M[2][1] = mRows[2][1];

        M[0][2] = mRows[0][2];
        M[1][2] = mRows[1][2];
        M[2][2] = mRows[2][2];

        return M;
    }


    /**
     * Computes determinant of matrix
     * @return Determinant of matrix
     * @note This function does 3 * 4 * 6 mul, 3 * 6 add.
     */
    SIMD_INLINE T getDeterminant() const
    {

        return  + mRows[3][0] * mRows[2][1] * mRows[1][2] * mRows[0][3] - mRows[2][0] * mRows[3][1] * mRows[1][2] * mRows[0][3]
                - mRows[3][0] * mRows[1][1] * mRows[2][2] * mRows[0][3] + mRows[1][0] * mRows[3][1] * mRows[2][2] * mRows[0][3]

                + mRows[2][0] * mRows[1][1] * mRows[3][2] * mRows[0][3] - mRows[1][0] * mRows[2][1] * mRows[3][2] * mRows[0][3]
                - mRows[3][0] * mRows[2][1] * mRows[0][2] * mRows[1][3] + mRows[2][0] * mRows[3][1] * mRows[0][2] * mRows[1][3]

                + mRows[3][0] * mRows[0][1] * mRows[2][2] * mRows[1][3] - mRows[0][0] * mRows[3][1] * mRows[2][2] * mRows[1][3]
                - mRows[2][0] * mRows[0][1] * mRows[3][2] * mRows[1][3] + mRows[0][0] * mRows[2][1] * mRows[3][2] * mRows[1][3]

                + mRows[3][0] * mRows[1][1] * mRows[0][2] * mRows[2][3] - mRows[1][0] * mRows[3][1] * mRows[0][2] * mRows[2][3]
                - mRows[3][0] * mRows[0][1] * mRows[1][2] * mRows[2][3] + mRows[0][0] * mRows[3][1] * mRows[1][2] * mRows[2][3]

                + mRows[1][0] * mRows[0][1] * mRows[3][2] * mRows[2][3] - mRows[0][0] * mRows[1][1] * mRows[3][2] * mRows[2][3]
                - mRows[2][0] * mRows[1][1] * mRows[0][2] * mRows[3][3] + mRows[1][0] * mRows[2][1] * mRows[0][2] * mRows[3][3]

                + mRows[2][0] * mRows[0][1] * mRows[1][2] * mRows[3][3] - mRows[0][0] * mRows[2][1] * mRows[1][2] * mRows[3][3]
                - mRows[1][0] * mRows[0][1] * mRows[2][2] * mRows[3][3] + mRows[0][0] * mRows[1][1] * mRows[2][2] * mRows[3][3];

    }

    /**
     * Computes inverse matrix
     * @return Inverse matrix of this matrix.
     * @note This is a little bit time consuming operation
     * (16 * 6 * 3 mul, 16 * 5 add + det() + mul() functions)
     */
    SIMD_INLINE IMatrix4x4<T> getInverse() const
    {

        // Compute the determinant of the matrix
        T determinant = getDeterminant();

        // Check if the determinant is equal to zero
        //assert(IAbs(determinant) > MACHINE_EPSILON);

        IMatrix4x4<T> ret;

                  ret.mRows[0][0] = + mRows[2][1] * mRows[3][2] * mRows[1][3] - mRows[3][1] * mRows[2][2] * mRows[1][3] + mRows[3][1] * mRows[1][2] * mRows[2][3]
                                    - mRows[1][1] * mRows[3][2] * mRows[2][3] - mRows[2][1] * mRows[1][2] * mRows[3][3] + mRows[1][1] * mRows[2][2] * mRows[3][3];

                  ret.mRows[1][0] = + mRows[3][0] * mRows[2][2] * mRows[1][3] - mRows[2][0] * mRows[3][2] * mRows[1][3] - mRows[3][0] * mRows[1][2] * mRows[2][3]
                                    + mRows[1][0] * mRows[3][2] * mRows[2][3] + mRows[2][0] * mRows[1][2] * mRows[3][3] - mRows[1][0] * mRows[2][2] * mRows[3][3];

                  ret.mRows[2][0] = + mRows[2][0] * mRows[3][1] * mRows[1][3] - mRows[3][0] * mRows[2][1] * mRows[1][3] + mRows[3][0] * mRows[1][1] * mRows[2][3]
                                    - mRows[1][0] * mRows[3][1] * mRows[2][3] - mRows[2][0] * mRows[1][1] * mRows[3][3] + mRows[1][0] * mRows[2][1] * mRows[3][3];

                  ret.mRows[3][0] = + mRows[3][0] * mRows[2][1] * mRows[1][2] - mRows[2][0] * mRows[3][1] * mRows[1][2] - mRows[3][0] * mRows[1][1] * mRows[2][2]
                                    + mRows[1][0] * mRows[3][1] * mRows[2][2] + mRows[2][0] * mRows[1][1] * mRows[3][2] - mRows[1][0] * mRows[2][1] * mRows[3][2];



                  ret.mRows[0][1] = + mRows[3][1] * mRows[2][2] * mRows[0][3] - mRows[2][1] * mRows[3][2] * mRows[0][3] - mRows[3][1] * mRows[0][2] * mRows[2][3]
                                    + mRows[0][1] * mRows[3][2] * mRows[2][3] + mRows[2][1] * mRows[0][2] * mRows[3][3] - mRows[0][1] * mRows[2][2] * mRows[3][3];

                  ret.mRows[1][1] = + mRows[2][0] * mRows[3][2] * mRows[0][3] - mRows[3][0] * mRows[2][2] * mRows[0][3] + mRows[3][0] * mRows[0][2] * mRows[2][3]
                                    - mRows[0][0] * mRows[3][2] * mRows[2][3] - mRows[2][0] * mRows[0][2] * mRows[3][3] + mRows[0][0] * mRows[2][2] * mRows[3][3];

                  ret.mRows[2][1] = + mRows[3][0] * mRows[2][1] * mRows[0][3] - mRows[2][0] * mRows[3][1] * mRows[0][3] - mRows[3][0] * mRows[0][1] * mRows[2][3]
                                    + mRows[0][0] * mRows[3][1] * mRows[2][3] + mRows[2][0] * mRows[0][1] * mRows[3][3] - mRows[0][0] * mRows[2][1] * mRows[3][3];

                  ret.mRows[3][1] = + mRows[2][0] * mRows[3][1] * mRows[0][2] - mRows[3][0] * mRows[2][1] * mRows[0][2] + mRows[3][0] * mRows[0][1] * mRows[2][2]
                                    - mRows[0][0] * mRows[3][1] * mRows[2][2] - mRows[2][0] * mRows[0][1] * mRows[3][2] + mRows[0][0] * mRows[2][1] * mRows[3][2];



                  ret.mRows[0][2] = + mRows[1][1] * mRows[3][2] * mRows[0][3] - mRows[3][1] * mRows[1][2] * mRows[0][3] + mRows[3][1] * mRows[0][2] * mRows[1][3]
                                    - mRows[0][1] * mRows[3][2] * mRows[1][3] - mRows[1][1] * mRows[0][2] * mRows[3][3] + mRows[0][1] * mRows[1][2] * mRows[3][3];

                  ret.mRows[1][2] = + mRows[3][0] * mRows[1][2] * mRows[0][3] - mRows[1][0] * mRows[3][2] * mRows[0][3] - mRows[3][0] * mRows[0][2] * mRows[1][3]
                                    + mRows[0][0] * mRows[3][2] * mRows[1][3] + mRows[1][0] * mRows[0][2] * mRows[3][3] - mRows[0][0] * mRows[1][2] * mRows[3][3];

                  ret.mRows[2][2] = + mRows[1][0] * mRows[3][1] * mRows[0][3] - mRows[3][0] * mRows[1][1] * mRows[0][3] + mRows[3][0] * mRows[0][1] * mRows[1][3]
                                    - mRows[0][0] * mRows[3][1] * mRows[1][3] - mRows[1][0] * mRows[0][1] * mRows[3][3] + mRows[0][0] * mRows[1][1] * mRows[3][3];

                  ret.mRows[3][2] = + mRows[3][0] * mRows[1][1] * mRows[0][2] - mRows[1][0] * mRows[3][1] * mRows[0][2] - mRows[3][0] * mRows[0][1] * mRows[1][2]
                                    + mRows[0][0] * mRows[3][1] * mRows[1][2] + mRows[1][0] * mRows[0][1] * mRows[3][2] - mRows[0][0] * mRows[1][1] * mRows[3][2];



                  ret.mRows[0][3] = + mRows[2][1] * mRows[1][2] * mRows[0][3] - mRows[1][1] * mRows[2][2] * mRows[0][3] - mRows[2][1] * mRows[0][2] * mRows[1][3]
                                    + mRows[0][1] * mRows[2][2] * mRows[1][3] + mRows[1][1] * mRows[0][2] * mRows[2][3] - mRows[0][1] * mRows[1][2] * mRows[2][3];

                  ret.mRows[1][3] = + mRows[1][0] * mRows[2][2] * mRows[0][3] - mRows[2][0] * mRows[1][2] * mRows[0][3] + mRows[2][0] * mRows[0][2] * mRows[1][3]
                                    - mRows[0][0] * mRows[2][2] * mRows[1][3] - mRows[1][0] * mRows[0][2] * mRows[2][3] + mRows[0][0] * mRows[1][2] * mRows[2][3];

                  ret.mRows[2][3] = + mRows[2][0] * mRows[1][1] * mRows[0][3] - mRows[1][0] * mRows[2][1] * mRows[0][3] - mRows[2][0] * mRows[0][1] * mRows[1][3]
                                    + mRows[0][0] * mRows[2][1] * mRows[1][3] + mRows[1][0] * mRows[0][1] * mRows[2][3] - mRows[0][0] * mRows[1][1] * mRows[2][3];

                  ret.mRows[3][3] = + mRows[1][0] * mRows[2][1] * mRows[0][2] - mRows[2][0] * mRows[1][1] * mRows[0][2] + mRows[2][0] * mRows[0][1] * mRows[1][2]
                                    - mRows[0][0] * mRows[2][1] * mRows[1][2] - mRows[1][0] * mRows[0][1] * mRows[2][2] + mRows[0][0] * mRows[1][1] * mRows[2][2];

        return ret / determinant;
        /**/
    }



    /**
    * Transpose matrix.
    */
    SIMD_INLINE IMatrix4x4<T> getTranspose() const
    {
        IMatrix4x4<T> ret;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                ret.mRows[i][j] = mRows[j][i];
            }
        }
        return ret;
    }


    /// Return the matrix with absolute values
    SIMD_INLINE IMatrix4x4<T> getAbsoluteMatrix() const
    {
        return IMatrix4x4<T>(IAbs(mRows[0][0]), IAbs(mRows[0][1]), IAbs(mRows[0][2]), IAbs(mRows[0][3]),
                             IAbs(mRows[1][0]), IAbs(mRows[1][1]), IAbs(mRows[1][2]), IAbs(mRows[1][3]),
                             IAbs(mRows[2][0]), IAbs(mRows[2][1]), IAbs(mRows[2][2]), IAbs(mRows[2][3]),
                             IAbs(mRows[3][0]), IAbs(mRows[3][1]), IAbs(mRows[3][2]), IAbs(mRows[3][3]));
    }


    /// Return the trace of the matrix
    SIMD_INLINE T getTrace() const
    {
        // Compute and return the trace
        return (mRows[0][0] + mRows[1][1] + mRows[2][2] + mRows[3][3]);
    }


    /**
    * Return the coords of the matrix
    */
    SIMD_INLINE IVector3D<T> getCoords() const
    {
        // Compute and return the coords
        return IVector3D<T>(mRows[3][0] , mRows[3][1] , mRows[3][2]);
    }






    /**
     * Linear interpolation of two matrices
     * @param fact Factor of interpolation. For translation from positon
     * of this matrix (lhs) to matrix rhs, values of factor goes from 0.0 to 1.0.
     * @param rhs Second Matrix for interpolation
     * @note However values of fact parameter are reasonable only in interval
     * [0.0 , 1.0], you can pass also values outside of this interval and you
     * can get result (extrapolation?)
     */
    SIMD_INLINE IMatrix4x4<T> lerp(T fact, const IMatrix4x4<T>& rhs) const
    {
        IMatrix4x4<T> ret = (*this) + (rhs - (*this)) * fact;
        return ret;
    }




    //--------------------------------------------------------------------//


    /// Overloaded operator for addition with assignment
    SIMD_INLINE IMatrix4x4<T>& operator+=(const IMatrix4x4<T>& matrix)
    {
        mRows[0] += matrix.mRows[0];
        mRows[1] += matrix.mRows[1];
        mRows[2] += matrix.mRows[2];
        mRows[3] += matrix.mRows[3];

        return *this;
    }

    /// Overloaded operator for substraction with assignment
    SIMD_INLINE IMatrix4x4<T>& operator-=(const IMatrix4x4<T>& matrix)
    {
        mRows[0] -= matrix.mRows[0];
        mRows[1] -= matrix.mRows[1];
        mRows[2] -= matrix.mRows[2];
        mRows[3] -= matrix.mRows[3];

        return *this;
    }

    /// Overloaded operator for multiplication with a number with assignment
    SIMD_INLINE IMatrix4x4<T>& operator*=(T nb)
    {

        mRows[0] *= nb;
        mRows[1] *= nb;
        mRows[2] *= nb;
        mRows[3] *= nb;

        return *this;
    }

    /// Overloaded operator for invert multiplication with a number with assignment
    SIMD_INLINE IMatrix4x4<T> &operator/=(T nb)
    {
        mRows[0] /= nb;
        mRows[1] /= nb;
        mRows[2] /= nb;
        mRows[3] /= nb;

        return *this;
    }


    /// Convert in OpenGL Matrix Model
    SIMD_INLINE IMatrix4x4<T> convertInOpenGLFormat() const
    {
        IMatrix4x4<T> m(*this);
        m.setPosition(getCoords());
        m.setRotation(getRotMatrix().getTranspose());
        return m;
    }


    ///---------------------------[ Pulgins ] -----------------------------------///



    /// Return a skew-symmetric matrix using a given vector that can be used
    /// to compute cross product with another vector using matrix multiplication
    static SIMD_INLINE IMatrix4x4<T> computeSkewSymmetricMatrixForCrossProduct(const IVector4D<T>& vector)
    {
        return IMatrix4x4<T>(0       , -vector.z,  vector.y,  vector.t,
                             vector.z , 0        , -vector.x,  vector.y,
                            -vector.y , vector.x , 0        , -vector.x,
                            -vector.t ,-vector.y , vector.x , 0);
    }




    /// Return a symmetric matrix using a given vector that can be used
    /// to compute dot product with another vector using matrix multiplication
     static SIMD_INLINE IMatrix4x4<T> computeSymmetricMatrix(const IVector4D<T>& vector)
     {
         return IMatrix4x4<T>(0       ,  vector.z,  vector.y,  vector.t,
                              vector.z , 0        ,  vector.x,  vector.y,
                              vector.y , vector.x , 0        ,  vector.x,
                              vector.t , vector.y , vector.x , 0);
     }






     /**
      * Returns a scaling matrix that scales
      *
      * @param scaleFactors Scale .
      * @return Scaling matrix.
      */
     static SIMD_INLINE IMatrix4x4<T> createScale( const T& _scale )
     {
         static IMatrix4x4 res;

         T _x = T(1. + _scale);
         T _y = T(1. + _scale);
         T _z = T(1. + _scale);

         res.mRows[0] = IVector4D<T>(_x , 0.f, 0.f, 0.f);
         res.mRows[1] = IVector4D<T>(0.f, _y , 0.f, 0.f);
         res.mRows[2] = IVector4D<T>(0.f, 0.f, _z , 0.f);
         res.mRows[3] = IVector4D<T>(0.f, 0.f, 0.f, 1.f);

         return res;
     }


     /**
      * Returns a scaling matrix that scales
      *
      * @param scaleFactors Scale .
      * @return Scaling matrix.
      */
     static SIMD_INLINE IMatrix4x4<T> createScale( const IVector3D<T>& _scale )
     {
         static IMatrix4x4 res;

         T _x = T(1. + _scale.x);
         T _y = T(1. + _scale.y);
         T _z = T(1. + _scale.z);

         res.mRows[0] = IVector4D<T>(_x , 0.f, 0.f, 0.f);
         res.mRows[1] = IVector4D<T>(0.f, _y , 0.f, 0.f);
         res.mRows[2] = IVector4D<T>(0.f, 0.f, _z , 0.f);
         res.mRows[3] = IVector4D<T>(0.f, 0.f, 0.f, 1.f);

         return res;
     }


     /**
      * Returns a scaling around axis matrix that scales
      * @return axis to scaling matrix.
      */
     static SIMD_INLINE IMatrix4x4<T>  createScaleAroundAxis( const IVector3D<T> n , T _scale )
     {
         static IMatrix4x4<T> M;
         T gamma = (_scale);

         M.mRows[0][0]= 1.0+((gamma - 0.0)*((n.x * n.x)));
         M.mRows[1][0]=     ((gamma - 0.0)*((n.y * n.x)));
         M.mRows[2][0]=     ((gamma - 0.0)*((n.z * n.x)));
         M.mRows[3][0]= 0;

         M.mRows[0][1]=     ((gamma - 0.0)*((n.x * n.y)));
         M.mRows[1][1]= 1.0+((gamma - 0.0)*((n.y * n.y)));
         M.mRows[2][1]=     ((gamma - 0.0)*((n.z * n.y)));
         M.mRows[3][1]= 0;

         M.mRows[0][2]=      ((gamma - 0.0)*((n.x * n.z)));
         M.mRows[1][2]=      ((gamma - 0.0)*((n.y * n.z)));
         M.mRows[2][2]=  1.0+((gamma - 0.0)*((n.z * n.z)));
         M.mRows[3][2]= 0;

         M.mRows[0][3]= 0;
         M.mRows[1][3]= 0;
         M.mRows[2][3]= 0;
         M.mRows[3][3]= 1.0;

         return M;
     }


     /*****************************************************
      *  Help info to web site:  https://arxiv.org/pdf/1103.0156.pdf
      *****************************************************/

     /**
      * class TLorentzRotation
             \ingroup Physics
         The TLorentzRotation class describes Lorentz transformations including
         Lorentz boosts and rotations (see TRotation)
         ~~~
                     | xx  xy  xz  xt |
                     |                |
                     | yx  yy  yz  yt |
            lambda = |                |
                     | zx  zy  zz  zt |
                     |                |
                     | tx  ty  tz  tt |
         ~~~
         ### Declaration
         By default it is initialized to the identity matrix, but it may also be
         intialized by an other TLorentzRotation,
         by a pure TRotation or by a boost:
          TLorentzRotation l; // l is
         initialized as identity
          TLorentzRotation m(l); // m = l
          TRotation r;
          TLorentzRotation lr(r);
          TLorentzRotation lb1(bx,by,bz);
          TVector3 b;
          TLorentzRotation lb2(b);
         The Matrix for a Lorentz boosts is:
         ~~~
          | 1+gamma'*bx*bx  gamma'*bx*by   gamma'*bx*bz  gamma*bx |
          |  gamma'*by*bx  1+gamma'*by*by  gamma'*by*bz  gamma*by |
          |  gamma'*bz*bx   gamma'*bz*by  1+gamma'*bz*bz gamma*bz |
          |    gamma*bx       gamma*by       gamma*bz     gamma   |
         ~~~
         with the boost vector b=(bx,by,bz) and gamma=1/Sqrt(1-beta*beta)
         and gamma'=(gamma-1)/beta*beta.
         ### Access to the matrix components/Comparisons
         Access to the matrix components is possible through the member functions
         XX(), XY() .. TT(),
         through the operator (int,int):
      */

     static SIMD_INLINE IMatrix4x4<T> createLorentzBoost( const IVector3D<T> &vel )
     {


        static IMatrix4x4<T> M;


        const IVector3D<T> n = vel.getUnit();
        const T             v = vel.length();

        const T c = LIGHT_MAX_VELOCITY_C;

         //boost this Lorentz vector

          T gamma = 1.0 * ISqrt( 1.0 - (v*v) / (c*c) );

        // T bgamma = gamma * gamma / (1.0 + gamma);
          T bgamma = (gamma - 1.0);


        M[0][0] = 1.0+((bgamma)*((n.x * n.x)));
        M[1][0] =     ((bgamma)*((n.y * n.x)));
        M[2][0] =     ((bgamma)*((n.z * n.x)));
        M[3][0] = (v*n.x*gamma);

        M[0][1] =     ((bgamma)*((n.x * n.y)));
        M[1][1] = 1.0+((bgamma)*((n.y * n.y)));
        M[2][1] =     ((bgamma)*((n.z * n.y)));
        M[3][1] = (v*n.y*gamma);

        M[0][2] =      ((bgamma)*((n.x * n.z)));
        M[1][2] =      ((bgamma)*((n.y * n.z)));
        M[2][2] =  1.0+((bgamma)*((n.z * n.z)));
        M[3][2] = (v*n.z*gamma);

        M[0][3] =  v*n.x*gamma/(c*c);
        M[1][3] =  v*n.y*gamma/(c*c);
        M[2][3] =  v*n.z*gamma/(c*c);
        M[3][3] =  gamma;


         return M;
    }



     static SIMD_INLINE IMatrix4x4<T> createLorentzBoost( const IVector3D<T> vel , T gamma )
     {

         static IMatrix4x4<T> M;

         const IVector3D<T> n = vel.getUnit();
    	 const T             v = vel.length();

    	 // T bgamma = gamma * gamma / (1.0 + gamma);
    	 T bgamma = (gamma - 1.0);

    	 const T c = LIGHT_MAX_VELOCITY_C;

    	 M[0][0] = 1.0+((bgamma)*((n.x * n.x)));
    	 M[1][0] =     ((bgamma)*((n.y * n.x)));
    	 M[2][0] =     ((bgamma)*((n.z * n.x)));
    	 M[3][0] = (v*n.x*gamma);

    	 M[0][1] =     ((bgamma)*((n.x * n.y)));
    	 M[1][1] = 1.0+((bgamma)*((n.y * n.y)));
    	 M[2][1] =     ((bgamma)*((n.z * n.y)));
    	 M[3][1] = (v*n.y*gamma);

    	 M[0][2] =      ((bgamma)*((n.x * n.z)));
    	 M[1][2] =      ((bgamma)*((n.y * n.z)));
    	 M[2][2] =  1.0+((bgamma)*((n.z * n.z)));
    	 M[3][2] = (v*n.z*gamma);

    	 M[0][3] =  v*n.x*gamma/(c*c);
    	 M[1][3] =  v*n.y*gamma/(c*c);
    	 M[2][3] =  v*n.z*gamma/(c*c);
    	 M[3][3] =  gamma;  

    	 return M;
     }



       /// Creates translation matrix
       /**
        * Creates translation matrix.
        * @param x X-direction translation
        * @param y Y-direction translation
        * @param z Z-direction translation
        * @param w for W-coordinate translation (implicitly set to 1)
        */
       static SIMD_INLINE IMatrix4x4<T> createTranslation(T x, T y, T z, T w = 1)
       {
           IMatrix4x4<T> ret;
           ret.mRows[3][0] = x;
           ret.mRows[3][1] = y;
           ret.mRows[3][2] = z;
           ret.mRows[3][3] = w;

           return ret;
       }


       /// Creates translation matrix
       /**
        * Creates translation matrix.
        * @param x X-direction translation
        * @param y Y-direction translation
        * @param z Z-direction translation
        * @param w for W-coordinate translation (implicitly set to 1)
        */
       static SIMD_INLINE IMatrix4x4<T> createTranslation(const IVector3D<T> & v, T w = 1)
       {
           IMatrix4x4<T> ret;
           ret.mRows[3][0] = v.x;
           ret.mRows[3][1] = v.y;
           ret.mRows[3][2] = v.z;
           ret.mRows[3][3] = w;

           return ret;
       }


       /**
        * Creates rotation matrix by rotation around axis.
        * @param xDeg Angle (in degrees) of rotation around axis X.
        * @param yDeg Angle (in degrees) of rotation around axis Y.
        * @param zDeg Angle (in degrees) of rotation around axis Z.
        */
       static SIMD_INLINE IMatrix4x4<T> createRotationAroundAxis(T xDeg, T yDeg, T zDeg)
       {
           T xRads(/*IDegreesToRadians*/(xDeg));
           T yRads(/*IDegreesToRadians*/(yDeg));
           T zRads(/*IDegreesToRadians*/(zDeg));

           IMatrix4x4<T> ma, mb, mc;
           float ac = ICos(xRads);
           float as = ISin(xRads);
           float bc = ICos(yRads);
           float bs = ISin(yRads);
           float cc = ICos(zRads);
           float cs = ISin(zRads);

           ma.mRows[1][1] = ac;
           ma.mRows[2][1] = as;
           ma.mRows[1][2] = -as;
           ma.mRows[2][2] = ac;

           mb.mRows[0][0] = bc;
           mb.mRows[2][0] = -bs;
           mb.mRows[0][2] = bs;
           mb.mRows[2][2] = bc;

           mc.mRows[0][0] = cc;
           mc.mRows[1][0] = cs;
           mc.mRows[0][1] = -cs;
           mc.mRows[1][1] = cc;

           IMatrix4x4<T> ret = ma * mb * mc;

           return ret;
       }


       // Return a 4x4 rotation of axis to matrix
       static SIMD_INLINE IMatrix4x4<T> createRotationAxis(const IVector3D<T>& axis, T angle)
       {

           //angle = angle / 180.0f * (float)M_PI;

           T cosA = ICos(angle);
           T sinA = ISin(angle);
           IMatrix4x4<T> rotationMatrix;

           rotationMatrix.mRows[0][0] = cosA + (1-cosA) * axis.x * axis.x;
           rotationMatrix.mRows[1][0] = (1-cosA) * axis.x * axis.y - axis.z * sinA;
           rotationMatrix.mRows[2][0] = (1-cosA) * axis.x * axis.z + axis.y * sinA;
           rotationMatrix.mRows[3][0] = 0.f;

           rotationMatrix.mRows[0][1] = (1-cosA) * axis.x * axis.y + axis.z * sinA;
           rotationMatrix.mRows[1][1] = cosA + (1-cosA) * axis.y * axis.y;
           rotationMatrix.mRows[2][1] = (1-cosA) * axis.y * axis.z - axis.x * sinA;
           rotationMatrix.mRows[3][1] = 0.f;

           rotationMatrix.mRows[0][2] = (1-cosA) * axis.x * axis.z - axis.y * sinA;
           rotationMatrix.mRows[1][2] = (1-cosA) * axis.y * axis.z + axis.x * sinA;
           rotationMatrix.mRows[2][2] = cosA + (1-cosA) * axis.z * axis.z;
           rotationMatrix.mRows[3][2] = 0.f;

           rotationMatrix.mRows[0][3] = 0.f;
           rotationMatrix.mRows[1][3] = 0.f;
           rotationMatrix.mRows[2][3] = 0.f;
           rotationMatrix.mRows[3][3] = 1.f;

           return rotationMatrix;
       }


        // Return a 4x4 rotation of quaternion to matrix
       static  SIMD_INLINE IMatrix4x4<T>& createRotation(const IQuaternion<T>& Quat)
       {
           static IMatrix4x4<T> M;

           T D1, D2, D3, D4, D5, D6, D7, D8, D9; //Dummy variables to hold precalcs

           D1 = (Quat.v.x * Quat.v.x) * 2.0f;
           D2 = (Quat.v.y * Quat.v.y) * 2.0f;
           D3 = (Quat.v.z * Quat.v.z) * 2.0f;

           T RTimesTwo = Quat.w * 2.0f;
           D4 = Quat.v.x * RTimesTwo;
           D5 = Quat.v.y * RTimesTwo;
           D6 = Quat.v.z * RTimesTwo;

           D7 = (Quat.v.x * Quat.v.y) * 2.0f;
           D8 = (Quat.v.x * Quat.v.z) * 2.0f;
           D9 = (Quat.v.y * Quat.v.z) * 2.0f;

           M.mRows[0][0] = 1.0f - D2 - D3;
           M.mRows[1][0] = D7 - D6;
           M.mRows[2][0] = D8 + D5;
           M.mRows[3][0] = 0;

           M.mRows[0][1] = D7 + D6;
           M.mRows[1][1] = 1.0f - D1 - D3;
           M.mRows[2][1] = D9 - D4;
           M.mRows[3][1] = 0;

           M.mRows[0][2] = D8 - D5;
           M.mRows[1][2] = D9 + D4;
           M.mRows[2][2] = 1.0f - D1 - D2;
           M.mRows[3][2] = 0;

           M.mRows[0][3] = 0;
           M.mRows[1][3] = 0;
           M.mRows[2][3] = 0;
           M.mRows[3][3] = 1;

           return M;
       }


       /**
        * Creates new view matrix to look from specified position @a eyePos to specified position @a centerPos
        * @param eyePos A position of camera
        * @param centerPos A position where camera looks-at
        * @param upDir Direction of up vector
        * @return Resulting view matrix that looks from and at specific position.
        */
       static SIMD_INLINE IMatrix4x4<T> createLookAt(const IVector3D<T>& eyePos, const IVector3D<T>& centerPos, const IVector3D<T>& upDir)
       {

            IVector3D<T> forward = centerPos - eyePos;
           if (IAbs(forward.x) < MACHINE_EPSILON &&
               IAbs(forward.y) < MACHINE_EPSILON &&
               IAbs(forward.z) < MACHINE_EPSILON)
           {
               return IMatrix4x4<T>::IDENTITY;
           }

           forward.normalize();
           IVector3D<T> side = cross(forward, upDir).normalized();
           IVector3D<T> upVector = cross(side, forward);

           IMatrix4x4<T> m;
           m.setToIdentity();

           m.mRows[0][0] = side.x;
           m.mRows[1][0] = side.y;
           m.mRows[2][0] = side.z;
           m.mRows[3][0] = 0.0f;
           m.mRows[0][1] = upVector.x;
           m.mRows[1][1] = upVector.y;
           m.mRows[2][1] = upVector.z;
           m.mRows[3][1] = 0.0f;
           m.mRows[0][2] = -forward.x;
           m.mRows[1][2] = -forward.y;
           m.mRows[2][2] = -forward.z;
           m.mRows[3][2] = 0.0f;
           m.mRows[0][3] = 0.0f;
           m.mRows[1][3] = 0.0f;
           m.mRows[2][3] = 0.0f;
           m.mRows[3][3] = 1.0f;

           return m * IMatrix4x4<T>::createTranslation(-eyePos);

       }



       /*!
           Multiplies this matrix by another that applies an orthographic
           projection for a window with lower-left corner (\a left, \a bottom),
           upper-right corner (\a right, \a top), and the specified \a nearPlane
           and \a farPlane clipping planes.
           \sa frustum(), perspective()
       */
       static SIMD_INLINE IMatrix4x4<T> createOrtho(T left, T right, T bottom, T top, T nearPlane, T farPlane)
       {

         // Bail out if the projection volume is zero-sized.
          if (left == right || bottom == top || nearPlane == farPlane)
              return IMatrix4x4<T>::IDENTITY;

          // Construct the projection.
          T width = right - left;
          T invheight = top - bottom;
          T clip = farPlane - nearPlane;
//      #ifndef I_NO_VECTOR3D
//          if (clip == 2.0f && (nearPlane + farPlane) == 0.0f) {
//              // We can express this projection as a translate and scale
//              // which will be more efficient to modify with further
//              // transformations than producing a "General" matrix.
//              translate(QVector3D
//                  (-(left + right) / width,
//                   -(top + bottom) / invheight,
//                   0.0f));
//              scale(QVector3D
//                  (2.0f / width,
//                   2.0f / invheight,
//                   -1.0f));
//              return;
//          }
//      #endif
          IMatrix4x4 m;
          m.setToIdentity();

          m.mRows[0][0] = 2.0f / width;
          m.mRows[1][0] = 0.0f;
          m.mRows[2][0] = 0.0f;
          m.mRows[3][0] = -(left + right) / width;

          m.mRows[0][1] = 0.0f;
          m.mRows[1][1] = 2.0f / invheight;
          m.mRows[2][1] = 0.0f;
          m.mRows[3][1] = -(top + bottom) / invheight;

          m.mRows[0][2] = 0.0f;
          m.mRows[1][2] = 0.0f;
          m.mRows[2][2] = -2.0f / clip;
          m.mRows[3][2] = -(nearPlane + farPlane) / clip;;

          m.mRows[0][3] = 0.0f;
          m.mRows[1][3] = 0.0f;
          m.mRows[2][3] = 0.0f;
          m.mRows[3][3] = 1.0f;

          // Apply the projection.
          return m;

       }



       /*!
           Multiplies this matrix by another that applies a perspective
           frustum projection for a window with lower-left corner (\a left, \a bottom),
           upper-right corner (\a right, \a top), and the specified \a nearPlane
           and \a farPlane clipping planes.
           \sa ortho(), perspective()
       */
       static SIMD_INLINE IMatrix4x4<T> createFrustum(T left, T right, T bottom, T top, T nearPlane, T farPlane)
       {
         // Bail out if the projection volume is zero-sized.
         if (left == right || bottom == top || nearPlane == farPlane)
             return IMatrix4x4<T>::IDENTITY;

         // Construct the projection.
         IMatrix4x4<T> m;
         m.setToIdentity();
         T width = right - left;
         T invheight = top - bottom;
         T clip = farPlane - nearPlane;
         m.mRows[0][0] = 2.0f * nearPlane / width;
         m.mRows[1][0] = 0.0f;
         m.mRows[2][0] = (left + right) / width;
         m.mRows[3][0] = 0.0f;
         m.mRows[0][1] = 0.0f;
         m.mRows[1][1] = 2.0f * nearPlane / invheight;
         m.mRows[2][1] = (top + bottom) / invheight;
         m.mRows[3][1] = 0.0f;
         m.mRows[0][2] = 0.0f;
         m.mRows[1][2] = 0.0f;
         m.mRows[2][2] = -(nearPlane + farPlane) / clip;
         m.mRows[3][2] = -2.0f * nearPlane * farPlane / clip;
         m.mRows[0][3] = 0.0f;
         m.mRows[1][3] = 0.0f;
         m.mRows[2][3] = -1.0f;
         m.mRows[3][3] = 0.0f;

         // Apply the projection.
         return m;
       }


       /*!
           Multiplies this matrix by another that applies a perspective
           projection. The vertical field of view will be \a verticalAngle degrees
           within a window with a given \a aspectRatio that determines the horizontal
           field of view.
           The projection will have the specified \a nearPlane and \a farPlane clipping
           planes which are the distances from the viewer to the corresponding planes.
           \sa ortho(), frustum()
       */
       static SIMD_INLINE IMatrix4x4<T> createPerspective(T verticalAngle, T aspectRatio, T nearPlane, T farPlane)
       {
           // Bail out if the projection volume is zero-sized.
           if (nearPlane == farPlane || aspectRatio == 0.0f) return IMatrix4x4<T>::IDENTITY;

           // Construct the projection.
           IMatrix4x4<T> m;
           T radians = (verticalAngle / 2.0f) * M_PI / 180.0f;
           T sine = ISin(radians);

           if (sine == 0.0f) return IMatrix4x4<T>::IDENTITY;

           T cotan = ICos(radians) / sine;
           T clip = farPlane - nearPlane;

           m.mRows[0][0] = cotan / aspectRatio;
           m.mRows[1][0] = 0.0f;
           m.mRows[2][0] = 0.0f;
           m.mRows[3][0] = 0.0f;

           m.mRows[0][1] = 0.0f;
           m.mRows[1][1] = cotan;
           m.mRows[2][1] = 0.0f;
           m.mRows[3][1] = 0.0f;

           m.mRows[0][2] = 0.0f;
           m.mRows[1][2] = 0.0f;
           m.mRows[2][2] = -(nearPlane + farPlane) / clip;
           m.mRows[3][2] = -(2.0f * nearPlane * farPlane) / clip;

           m.mRows[0][3] = 0.0f;
           m.mRows[1][3] = 0.0f;
           m.mRows[2][3] = -1.0f;
           m.mRows[3][3] = 0.0f;

           return m;
       }





       /*!
           Multiplies this matrix by another that performs the scale and bias
           transformation used by OpenGL to transform from normalized device
           coordinates (NDC) to viewport (window) coordinates. That is it maps
           points from the cube ranging over [-1, 1] in each dimension to the
           viewport with it's near-lower-left corner at (\a left, \a bottom, \a nearPlane)
           and with size (\a width, \a height, \a farPlane - \a nearPlane).
           This matches the transform used by the fixed function OpenGL viewport
           transform controlled by the functions glViewport() and glDepthRange().
        */
       static SIMD_INLINE IMatrix4x4<T> createViewport(T left, T bottom, T width, T height, T nearPlane, T farPlane)
       {
           const T w2 = width / 2.0f;
           const T h2 = height / 2.0f;

           IMatrix4x4<T> m;
           m.mRows[0][0] = w2;
           m.mRows[1][0] = 0.0f;
           m.mRows[2][0] = 0.0f;
           m.mRows[3][0] = left + w2;

           m.mRows[0][1] = 0.0f;
           m.mRows[1][1] = h2;
           m.mRows[2][1] = 0.0f;
           m.mRows[3][1] = bottom + h2;

           m.mRows[0][2] = 0.0f;
           m.mRows[1][2] = 0.0f;
           m.mRows[2][2] = (farPlane - nearPlane) / 2.0f;
           m.mRows[3][2] = (nearPlane + farPlane) / 2.0f;

           m.mRows[0][3] = 0.0f;
           m.mRows[1][3] = 0.0f;
           m.mRows[2][3] = 0.0f;
           m.mRows[3][3] = 1.0f;

           return m;
       }




    //----------[ output operator ]----------------------------
    /**
    * Output to stream operator
    * @param lhs Left hand side argument of operator (commonly ostream instance).
    * @param rhs Right hand side argument of operator.
    * @return Left hand side argument - the ostream object passed to operator.
    */
    friend std::ostream& operator <<(std::ostream& lhs, const IMatrix4x4<T>& rhs)
    {
        for (int i = 0; i < 4; i++)
        {
            lhs << "|\t";
            for (int j = 0; j < 4; j++)
            {
                lhs << rhs[i][j]  << "\t";
            }
            lhs << "|" << std::endl;
        }
        return lhs;
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



public:
    /**
     * The multiplicitive identity matrix.
     */
    static const IMatrix4x4<T> IDENTITY;

    /**
     * The additive identity matrix.
     */
    static const IMatrix4x4<T> ZERO;

};

template<class T> const IMatrix4x4<T> IMatrix4x4<T>::IDENTITY = IMatrix4x4<T>(1.0, 0.0, 0.0, 0.0,
																				  0.0, 1.0, 0.0, 0.0,
																				  0.0, 0.0, 1.0, 0.0,
																				  0.0, 0.0, 0.0, 1.0);

template<class T> const IMatrix4x4<T> IMatrix4x4<T>::ZERO = IMatrix4x4<T>(0.0, 0.0, 0.0, 0.0,
																			 0.0, 0.0, 0.0, 0.0,
																			 0.0, 0.0, 0.0, 0.0,
																			 0.0, 0.0, 0.0, 0.0);





template<class T>  SIMD_INLINE IMatrix4x4<T> operator ^ (const IVector4D<T> lhs , const IVector4D<T> rhs )
{
    return IMatrix4x4<T>( lhs.x * rhs.x , lhs.x * rhs.y, lhs.x * rhs.z, lhs.x * rhs.w ,
			               lhs.y * rhs.x , lhs.y * rhs.y, lhs.y * rhs.z, lhs.y * rhs.w ,
						   lhs.z * rhs.x , lhs.z * rhs.y, lhs.z * rhs.z, lhs.z * rhs.w ,
						   lhs.w * rhs.x , lhs.w * rhs.y, lhs.w * rhs.z, lhs.w * rhs.w);
}

template<class T>  SIMD_INLINE IMatrix4x4<T> operator ^ (const ILorentzVector<T> lhs , const ILorentzVector<T> rhs )
{
    return IMatrix4x4<T>( lhs.x * rhs.x , lhs.x * rhs.y, lhs.x * rhs.z, lhs.x * rhs.t ,
                          lhs.y * rhs.x , lhs.y * rhs.y, lhs.y * rhs.z, lhs.y * rhs.t ,
                          lhs.z * rhs.x , lhs.z * rhs.y, lhs.z * rhs.z, lhs.z * rhs.t ,
                          lhs.t * rhs.x , lhs.t * rhs.y, lhs.t * rhs.z, lhs.t * rhs.t);
}



//--------------------------------------
// Typedef shortcuts for Matrix4x4
//-------------------------------------
/// Matrix 4x4 of floats
typedef IMatrix4x4<float> IMatrix4x4f;
/// Matrix 4x4 of doubles
typedef IMatrix4x4<double> IMatrix4x4d;
/// Matrix 4x4 of int
typedef IMatrix4x4<int> IMatrix4x4i;



} /* namespace  */

#endif /* IMATRIX4X4_H_ */
