/********************************************************************************
 *
 * IQuaternion.h
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


#ifndef IQUATERNION_H_
#define IQUATERNION_H_


// Libraries
#include <cmath>
#include "IMatrix4x4.h"

namespace IMath
{

    /**
 * Quaternion class implementing some quaternion algebra operations.
 * Quaternion is kind of complex number it consists of its real part (w)
 * and its complex part v. This complex part has three elements, so we
 * can express it as xi + yj + zk . Note that coordinates of (x,y,z) are
 * hold inside v field.
 */
    template<class T> class IQuaternion
    {


     public:

        static_assert(std::is_floating_point<T>::value, "quaternions can only be used with floating point types");

        //! Specifies the typename of the scalar components.
        using ScalarType = T;

        //! Specifies the number of quaternion components. This is just for the internal template interface.
        static const std::size_t components = 4;


    private:

        // -------------------- Attributes -------------------- //


        /**
     * Real part of IQuaternion.
     */
        T w;

        /**
     * Imaginary part of IQuaternion.
     */
        IVector3D<T> v;





    public:

        //--------------------------[ constructors ]------------------------------- //

        /**
    * IQuaternion constructor, sets IQuaternion to (0 + 0i + 0j + 0k).
    */
        SIMD_INLINE IQuaternion()
            : w(0), v(0, 0, 0)
        {
        }

        /**
    * Copy constructor.
    */
        SIMD_INLINE IQuaternion(const IQuaternion<T>& q)
            : w(q.w), v(q.v)
        {
        }

        /**
     * Copy casting constructor.
     */
        template<class FromT>
        SIMD_INLINE IQuaternion(const IQuaternion<FromT>& q)
            : w(static_cast<T>(q.w)), v(q.v)
        {
        }

        /**
    * Creates IQuaternion object from real part w_ and complex part v_.
    * @param w_ Real part of IQuaternion.
    * @param v_ Complex part of IQuaternion (xi + yj + zk).
    */
        SIMD_INLINE IQuaternion(T w_, const IVector3D<T>& v_)
            : w(w_), v(v_)
        {
        }

        /**
    * Creates IQuaternion object from real part w_ and complex part v_.
    * @param w_ Real part of IQuaternion.
    * @param v_ Complex part of IQuaternion (xi + yj + zk).
    */
        SIMD_INLINE IQuaternion(const IVector3D<T>& v_, T w_)
            : w(w_), v(v_)
        {
        }

        /**
    * Creates IQuaternion object from value (w_ + xi + yj + zk).
    * @param x Complex coefficient for i complex constant.
    * @param y Complex coefficient for j complex constant.
    * @param z Complex coefficient for k complex constant.
    * @param w_ Real part of IQuaternion.
    */
        SIMD_INLINE IQuaternion(T _x, T _y, T _z, T _w)
            : w(_w), v(_x, _y, _z)
        {
        }


        /// Constructor which convert Euler angles (in radians) to a quaternion
        SIMD_INLINE IQuaternion(T angleX, T angleY, T angleZ)
        {
            initWithEulerAngles(angleX, angleY, angleZ);
        }

        /// Constructor which convert Euler angles (in radians) to a quaternion
        SIMD_INLINE IQuaternion(const IVector3D<T>& eulerAngles)
        {
            initWithEulerAngles(eulerAngles.x, eulerAngles.y, eulerAngles.z);
        }

        /// Create a unit quaternion from a rotation matrix
        SIMD_INLINE IQuaternion(const IMatrix3x3<T>& matrix)
        {

            T trace = matrix[0].x + matrix[1].y + matrix[2].z;
            T temp[4];

            if (trace > T(0.0))
            {
                T s = ISqrt(trace + T(1.0));
                temp[3]=(s * T(0.5));
                s = T(0.5) / s;

                temp[0]=((matrix[2].y - matrix[1].z) * s);
                temp[1]=((matrix[0].z - matrix[2].x) * s);
                temp[2]=((matrix[1].x - matrix[0].y) * s);
            }
            else
            {
                int i = matrix[0].x < matrix[1].y ?
                            (matrix[1].y < matrix[2].z ? 2 : 1) :
                    (matrix[0].x < matrix[2].z ? 2 : 0);

                int j = (i + 1) % 3;
                int k = (i + 2) % 3;

                T s = ISqrt(matrix[i][i] - matrix[j][j] - matrix[k][k] + T(1.0));
                temp[i] = s * T(0.5);
                s = T(0.5) / s;

                temp[3] = (matrix[k][j] - matrix[j][k]) * s;
                temp[j] = (matrix[j][i] + matrix[i][j]) * s;
                temp[k] = (matrix[k][i] + matrix[i][k]) * s;
            }

            setAllValues(temp[0],temp[1],temp[2],temp[3]);

        }

        /// Create a unit quaternion from a transform matrix
        SIMD_INLINE IQuaternion(const IMatrix4x4<T>& matrix)
            : IQuaternion( matrix.getRotMatrix() )
        {

        }

        //---------------------- Methods ---------------------//


        /// Set all the values
        SIMD_INLINE void setAllValues(T newX, T newY, T newZ, T newW)
        {
            v.x = newX;
            v.y = newY;
            v.z = newZ;
            w = newW;
        }

        /// Set the quaternion to zero
        SIMD_INLINE void setToZero()
        {
            v.x = 0;
            v.y = 0;
            v.z = 0;
            w = 0;
        }

        /// Set to the identity quaternion
        SIMD_INLINE void setToIdentity()
        {
            v.x = 0;
            v.y = 0;
            v.z = 0;
            w = 1;
        }

        SIMD_INLINE T R_component_1() const { return(w);   }
        SIMD_INLINE T R_component_2() const { return(v.x); }
        SIMD_INLINE T R_component_3() const { return(v.y); }
        SIMD_INLINE T R_component_4() const { return(v.z); }


        SIMD_INLINE IVector3D<T> getV() const { return v; }
        SIMD_INLINE            T getW() const { return w; }


        /// Spinor operators : quantum mechanics
        SIMD_INLINE IQuaternion<T> getConjugateSpinor()  const { return Quaternion(w,  v.x,  v.y,  v.z); }
        SIMD_INLINE IQuaternion<T> getXConjugateSpinor() const { return Quaternion(w,  v.x, -v.y, -v.z); }
        SIMD_INLINE IQuaternion<T> getYConjugateSpinor() const { return Quaternion(w, -v.x,  v.y, -v.z); }
        SIMD_INLINE IQuaternion<T> getZConjugateSpinor() const { return Quaternion(w, -v.x, -v.y,  v.z); }


        SIMD_INLINE T getAngle() const { return T(2)*logRotor().length(); }



        /// Initialize the quaternion using Euler angles
        SIMD_INLINE void initWithEulerAngles(T angleX, T angleY, T angleZ)
        {
            T angle = angleX * T(0.5);
            const T sinX = ISin(angle);
            const T cosX = ICos(angle);

            angle = angleY * T(0.5);
            const T sinY = ISin(angle);
            const T cosY = ICos(angle);

            angle = angleZ * T(0.5);
            const T sinZ = ISin(angle);
            const T cosZ = ICos(angle);

            const T cosYcosZ = cosY * cosZ;
            const T sinYcosZ = sinY * cosZ;
            const T cosYsinZ = cosY * sinZ;
            const T sinYsinZ = sinY * sinZ;

            v.x = sinX * cosYcosZ - cosX * sinYsinZ;
            v.y = cosX * sinYcosZ + sinX * cosYsinZ;
            v.z = cosX * cosYsinZ - sinX * sinYcosZ;
            w   = cosX * cosYcosZ + sinX * sinYsinZ;

            // Normalize the quaternion
            normalize();
        }



        /// Quaternion using Euler angles
        SIMD_INLINE IVector3D<T> getEulerAngles() const
        {
            // Euler angles in radians
            T pitch;
            T yaw;
            T roll;

            IQuaternion<T> q(*this);

            T sqw = q.w * q.w;
            T sqx = q.v.x * q.v.x;
            T sqy = q.v.y * q.v.y;
            T sqz = q.v.z * q.v.z;

            // If quaternion is normalised the unit is one, otherwise it is the correction factor
            T unit = sqx + sqy + sqz + sqw;
            T val = q.v.x * q.v.y + q.v.z * q.w;

            if (val > T(0.4999) * unit)                                // 0.4999f OR 0.5f - EPSILON
            {
                // Singularity at north pole
                pitch = 2.f * IAtan2(q.v.x, q.w);           // Yaw
                yaw = M_PI * 0.5f;                         // Pitch
                roll = 0.f;                                // Roll
            }
            else if (val < -T(0.4999) * unit)                          // -0.4999f OR -0.5f + EPSILON
            {
                // Singularity at south pole
                pitch = -2.f * IAtan2(q.v.x, q.w);          // Yaw
                yaw = -M_PI * 0.5f;                        // Pitch
                roll = 0.f;                                // Roll
            }
            else
            {
                pitch = IAtan2(2.f * q.v.y * q.w - 2.f * q.v.x * q.v.z,  sqx - sqy - sqz + sqw);     // Yaw
                yaw   = IArcSin(2.f * val / unit);                                                   // Pitch
                roll  = IAtan2(2.f * q.v.x * q.w - 2.f * q.v.y * q.v.z, -sqx + sqy - sqz + sqw);     // Roll
            }

            return IVector3D<T>(pitch,yaw,roll);
        }


        /// Quaternion using Euler angles
        SIMD_INLINE IVector3D<T> GetXYZ_EulerAngles() const
        {
            T pitch;
            T yaw;
            T roll;
            T val = 2.0f * (v.y * w - v.z * v.x);
            if (val  >= 0.99995f)
            {
                pitch = 2.0f * IAtan2(v.x, w);
                yaw = (0.5f  * M_PI);
                roll = 0.0f;
            }
            else if (val  <= -0.99995f)
            {
                pitch = 2.0f * IAtan2(v.x, w);
                yaw = -(0.5f * M_PI);
                roll = 0.0f;
            }
            else
            {
                yaw =   IArcSin (val);
                pitch = IAtan2 (2.0f * (v.x * w + v.z * v.y), 1.0f - 2.0f * (v.x * v.x + v.y * v.y));
                roll =  IAtan2 (2.0f * (v.z * w + v.x * v.y), 1.0f - 2.0f * (v.y * v.y + v.z * v.z));
            }
            return IVector3D<T>(pitch, yaw, roll);
        }



        /// Quaternion using camera look direction
        SIMD_INLINE IQuaternion<T> lookRotation(IVector3D<T>& lookAt, IVector3D<T>& upDirection)
        {
            IVector3D<T> forward = lookAt.getUnit();
            IVector3D<T> right   = upDirection.getUnit().cross(forward);
            IVector3D<T> up      = forward.cross(right);

            IQuaternion<T> ret;
            ret.w = ISqrt(1.0f + right.x + up.y + forward.z) * 0.5f;
            T w4_recip = 1.0f / (4.0f * ret.w);
            ret.v.x = (forward.y - up.z) * w4_recip;
            ret.v.y = (right.z - forward.x) * w4_recip;
            ret.v.z = (up.x - right.y) * w4_recip;

            return ret;
        }


        //--------------------- operators -------------------------//

        /**
  * Copy operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator=(const IQuaternion<T>& rhs)
        {
            v = rhs.v;
            w = rhs.w;
            return *this;
        }

        /**
  * Copy convert operator
  * @param rhs Right hand side argument of binary operator.
  */
        template<class FromT>
        SIMD_INLINE IQuaternion<T>& operator=(const IQuaternion<FromT>& rhs)
        {
            v = rhs.v;
            w = static_cast<T>(rhs.w);
            return *this;
        }


        /**
  * Addition operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T> operator+(const IQuaternion<T>& rhs) const
        {
            const IQuaternion<T>& lhs = *this;
            return IQuaternion<T>(lhs.w + rhs.w, lhs.v + rhs.v);
        }


        /**
  * Subtraction operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T> operator-(const IQuaternion<T>& rhs) const
        {
            const IQuaternion<T>& lhs = *this;
            return IQuaternion<T>(lhs.w - rhs.w, lhs.v - rhs.v);
        }


        /**
  * Multiplication operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T> operator*(T rhs) const
        {
            return IQuaternion<T>(w * rhs, v * rhs);
        }


        /**
  * Multiplication invert operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T> operator/(T rhs) const
        {
            return IQuaternion<T>(w / rhs, v / rhs);
        }



        /**
  * Multiplication operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T> operator*(const IQuaternion<T>& rhs) const
        {
            //      const IQuaternion<T>& lhs = *this;
            //      return IQuaternion<T>(lhs.w * rhs.v.x + lhs.v.x * rhs.w   + lhs.v.y * rhs.v.z - lhs.v.z * rhs.v.y,
            //                             lhs.w * rhs.v.y - lhs.v.x * rhs.v.z + lhs.v.y * rhs.w   + lhs.v.z * rhs.v.x,
            //                             lhs.w * rhs.v.z + lhs.v.x * rhs.v.y - lhs.v.y * rhs.v.x + lhs.v.z * rhs.w  ,
            //                             lhs.w * rhs.w   - lhs.v.x * rhs.v.x - lhs.v.y * rhs.v.y - lhs.v.z * rhs.v.z);


            return IQuaternion<T> (w * rhs.w - v.dot(rhs.v), w * rhs.v + rhs.w * v + v.cross(rhs.v));
        }


        SIMD_INLINE IVector3D<T> nVidia_dot(const IVector3D<T>& v) const
        {
            // nVidia SDK implementation
            IVector3D<T> uv, uuv;
            IVector3D<T> qvec(v.x, v.y, v.z);
            uv = qvec.cross(v);
            uuv = qvec.cross(uv);
            uv *= (2.0f * w);
            uuv *= 2.0f;
            return v + uv + uuv;
        }



        /**
  * Addition operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator+=(const IQuaternion<T>& rhs)
        {
            w += rhs.w;
            v += rhs.v;
            return *this;
        }

        /**
  * Subtraction operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator-=(const IQuaternion<T>& rhs)
        {
            w -= rhs.w;
            v -= rhs.v;
            return *this;
        }


        /**
  * Multiplication operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator*=(T rhs)
        {
            w *= rhs;
            v *= rhs;
            return *this;
        }


        /**
  * Multiplication invert operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator/=(T rhs)
        {
            w /= rhs;
            v /= rhs;
            return *this;
        }


        /**
  * Multiplication operator
  * @param rhs Right hand side argument of binary operator.
  */
        SIMD_INLINE IQuaternion<T>& operator*=(const IQuaternion<T>& rhs)
        {
            IQuaternion q = (*this) * rhs;
            v = q.v;
            w = q.w;
            return *this;
        }



        /**
  * Equality test operator
  * @param rhs Right hand side argument of binary operator.
  * @note Test of equality is based of threshold EPSILON value. To be two
  * values equal, must satisfy this condition | lhs - rhs | < EPSILON,
  * for all IQuaternion coordinates.
  */
        SIMD_INLINE bool operator==(const IQuaternion<T>& rhs) const
        {
            const IQuaternion<T>& lhs = *this;
            return (IAbs(lhs.w - rhs.w) < MACHINE_EPSILON) && lhs.v == rhs.v;
        }

        /**
  * Inequality test operator
  * @param rhs Right hand side argument of binary operator.
  * @return not (lhs == rhs) :-P
  */
        SIMD_INLINE bool operator!=(const IQuaternion<T>& rhs) const
        {
            return !(*this == rhs);
        }

        //---------------------- Methods ---------------------//

        SIMD_INLINE IQuaternion<T> cross(const IQuaternion<T>& Q) const
        {
            return IQuaternion<T>(0, -v.z*Q.v.y+v.y*Q.v.z,
                                  v.z*Q.v.x-v.x*Q.v.z,
                                  -v.y*Q.v.x+v.x*Q.v.y);
        }

        SIMD_INLINE IQuaternion<T> dot(const IQuaternion<T>& Q) const
        {
            return v.x*Q.v.x+v.y*Q.v.y+v.z*Q.v.z;
        }


        SIMD_INLINE IQuaternion<T> commutator(const IQuaternion<T>& Q) const
        {
            return IQuaternion<T>(0, -2*v.z*Q.v.y+2*v.y*Q.v.z,
                                  2*v.z*Q.v.x-2*v.x*Q.v.z,
                                  -2*v.y*Q.v.x+2*v.x*Q.v.y);
        }


        SIMD_INLINE IQuaternion<T> pow(const T t) const
        {
            return (this->log() * t).exp();
        }

        SIMD_INLINE IQuaternion<T> pow(const IQuaternion<T>& Q) const
        {
            return (this->log() * Q).exp();
        }

        //-------------[ unary operations ]--------------------------

        /**
  * Unary negate operator
  * @return negated IQuaternion
  */
        SIMD_INLINE IQuaternion<T> operator-() const
        {
            return IQuaternion<T>(-w, -v);
        }

        /**
  * Unary conjugate operator
  * @return conjugated IQuaternion
  */
        SIMD_INLINE IQuaternion<T> operator~() const
        {
            return IQuaternion<T>(w, -v);
        }

        /**
  * Get lenght of IQuaternion.
  * @return Length of IQuaternion.
  */
        SIMD_INLINE T length() const
        {
            return (T) ISqrt(w * w + v.lengthSquare());
        }

        /**
  * Return square of length.
  * @return length ^ 2
  * @note This method is faster then length(). For comparison
  * of length of two IQuaternion can be used just this value, instead
  * of more expensive length() method.
  */
        SIMD_INLINE T lengthSquare() const
        {
            return w * w + v.lengthSquare();
        }

        /**
  * Normalize IQuaternion
  */
        SIMD_INLINE void normalize()
        {
            T len = length();
            w /= len;
            v /= len;
        }

        /**
  * Inverse IQuaternion
  */
        SIMD_INLINE IQuaternion<T> getInverse() const
        {
            T lengthSquareQuaternion = lengthSquare();

            assert (lengthSquareQuaternion > MACHINE_EPSILON);

            // Compute and return the inverse quaternion
            return IQuaternion<T>(-v.x / lengthSquareQuaternion,
                                  -v.y / lengthSquareQuaternion,
                                  -v.z / lengthSquareQuaternion,
                                  w   / lengthSquareQuaternion);
        }


        /**
  * Unit IQuaternion
  */
        SIMD_INLINE IQuaternion<T> getUnit() const
        {
            T lengthQuaternion = length();

            // Check if the length is not equal to zero
            assert (lengthQuaternion > MACHINE_EPSILON);

            // Compute and return the unit quaternion
            return IQuaternion<T>(v.x / lengthQuaternion,
                                  v.y / lengthQuaternion,
                                  v.z / lengthQuaternion,
                                  w / lengthQuaternion);
        }


        /**
  * Conjugate IQuaternion
  */
        SIMD_INLINE IQuaternion<T> getConjugate() const
        {
            return IQuaternion<T>(-v, w);
        }


        /**
  * Creates IQuaternion for eulers angles.
  * @param x Rotation around x axis (in degrees).
  * @param y Rotation around y axis (in degrees).
  * @param z Rotation around z axis (in degrees).
  * @return IQuaternion object representing transformation.
  */
        static SIMD_INLINE IQuaternion<T> fromEulerAngles(T x, T y, T z)
        {
            IQuaternion<T> ret = fromAxisRot(IVector3D<T>(1, 0, 0), x) *
                                 fromAxisRot(IVector3D<T>(0, 1, 0), y) *
                                 fromAxisRot(IVector3D<T>(0, 0, 1), z);
            return ret;
        }


        /**
  * Creates IQuaternion for eulers angles.
  * @param x Rotation around x axis (in degrees).
  * @param y Rotation around y axis (in degrees).
  * @param z Rotation around z axis (in degrees).
  * @return IQuaternion object representing transformation.
  */
        static SIMD_INLINE IQuaternion<T> fromEulerAngles(const IVector3D<T>& euler_angles)
        {
            IQuaternion<T> ret = fromAxisRot(IVector3D<T>(1, 0, 0), euler_angles.x) *
                                 fromAxisRot(IVector3D<T>(0, 1, 0), euler_angles.y) *
                                 fromAxisRot(IVector3D<T>(0, 0, 1), euler_angles.z);
            return ret;
        }

        /**
  * Creates IQuaternion as rotation around axis.
  * @param axis Unit vector expressing axis of rotation.
  * @param angleDeg Angle of rotation around axis (in degrees).
  */
        static SIMD_INLINE IQuaternion<T> fromAxisRot(const IVector3D<T> axis, T angleDeg)
        {
            T angleRad = /*IDegreesToRadians*/(angleDeg);
            T sa2 = ISin(angleRad / 2);
            T ca2 = ICos(angleRad / 2);
            return IQuaternion<T>( ca2 , sa2 * axis );
        }

        /**
  * Converts IQuaternion into rotation matrix.
  * @return Rotation matrix expressing this IQuaternion.
  */
        SIMD_INLINE IMatrix3x3<T> getMatrix() const
        {
            IMatrix3x3<T> ret;


            T nQ = v.x*v.x + v.y*v.y + v.z*v.z + w*w;
            T s = 0.0;

            if(nQ > 0.0)
            {
                s = T(2.0) / nQ; /// normalize quaternion
            }

            // Computations used for optimization (less multiplications)
            T xs  = v.x*s;
            T ys  = v.y*s;
            T zs  = v.z*s;
            T wxs = w*xs;
            T wys = w*ys;
            T wzs = w*zs;
            T xxs = v.x*xs;
            T xys = v.x*ys;
            T xzs = v.x*zs;
            T yys = v.y*ys;
            T yzs = v.y*zs;
            T zzs = v.z*zs;

            // Create the matrix corresponding to the quaternion
            ret = IMatrix3x3<T>( (1.0) - yys - zzs, xys-wzs, xzs + wys ,
                                 xys + wzs, T(1.0) - xxs - zzs, yzs-wxs,
                                 xzs-wys, yzs + wxs, T(1.0) - xxs - yys );


            return ret;
        }

        /**
  * Converts IQuaternion into transformation matrix.
  * @note This method performs same operation as rotMatrix()
  * conversion method. But returns Matrix of 4x4 elements.
  * @return Transformation matrix expressing this IQuaternion.
  */
        SIMD_INLINE IMatrix4x4<T> transform() const
        {
            IMatrix4x4<T> ret;

            T nQ = v.x*v.x + v.y*v.y + v.z*v.z + w*w;
            T s = 0.0;

            if(nQ > 0.0)
            {
                s = T(2.0) / nQ; /// normalize quaternion
            }

            // Computations used for optimization (less multiplications)
            T xs  = v.x*s;
            T ys  = v.y*s;
            T zs  = v.z*s;
            T wxs = w*xs;
            T wys = w*ys;
            T wzs = w*zs;
            T xxs = v.x*xs;
            T xys = v.x*ys;
            T xzs = v.x*zs;
            T yys = v.y*ys;
            T yzs = v.y*zs;
            T zzs = v.z*zs;

            // Create the matrix corresponding to the quaternion
            ret = IMatrix4x4<T>( T(1.0) - yys - zzs, xys-wzs, xzs + wys, T(0.0),
                                 xys + wzs, T(1.0) - xxs - zzs, yzs-wxs, T(0.0),
                                 xzs-wys, yzs + wxs, T(1.0) - xxs - yys, T(0.0),
                                 T(0.0),T(0.0),T(0.0),T(1.0));

            return ret;

        }


        /**
  * Linear interpolation of two IQuaternions
  * @param fact Factor of interpolation. For translation from position
  * of this vector to IQuaternion rhs, values of factor goes from 0.0 to 1.0.
  * @param rhs Second IQuaternion for interpolation
  * @note However values of fact parameter are reasonable only in interval
  * [0.0 , 1.0], you can pass also values outside of this interval and you
  * can get result (extrapolation?)
  */
        SIMD_INLINE IQuaternion<T> lerp(T fact, const IQuaternion<T>& rhs) const
        {
            return IQuaternion<T>((1 - fact) * w + fact * rhs.w, v.lerp(fact, rhs.v));
        }


        /**
  * Computes spherical interpolation between IQuaternions (this, q2)
  * using coefficient of interpolation r (in [0, 1]).
  *
  * @param r The ratio of interpolation form this (r = 0) to q2 (r = 1).
  * @param q2 Second IQuaternion for interpolation.
  * @return Result of interpolation.
  */
        SIMD_INLINE IQuaternion<T> slerp(T r, const IQuaternion<T>& q2) const
        {
            IQuaternion<T> ret;
            T cosTheta = w * q2.w + v.x * q2.v.x + v.y * q2.v.y + v.z * q2.v.z;
            T theta = (T) IArcCos(cosTheta);
            if (IAbs(theta) < MACHINE_EPSILON)
            {
                ret = *this;
            }
            else
            {
                T sinTheta = (T) ISqrt(1.0 - cosTheta * cosTheta);
                if (IAbs(sinTheta) < MACHINE_EPSILON)
                {
                    ret.w = 0.5 * w + 0.5 * q2.w;
                    ret.v = v.lerp(0.5, q2.v);
                }
                else
                {
                    T rA = (T) ISin((1.0 - r) * theta) / sinTheta;
                    T rB = (T) ISin(r * theta) / sinTheta;

                    ret.w = w * rA + q2.w * rB;
                    ret.v.x = v.x * rA + q2.v.x * rB;
                    ret.v.y = v.y * rA + q2.v.y * rB;
                    ret.v.z = v.z * rA + q2.v.z * rB;
                }
            }
            return ret;
        }



        /// Return logarithm of Quaternion.
        SIMD_INLINE IQuaternion<T> log() const
        {
            IQuaternion<T>  Result;
            const T b = ISqrt(v.x*v.x + v.y*v.y + v.z*v.z);
            if(IAbs(b) <= MACHINE_EPSILON*IAbs(w))
            {
                if(w<0.0)
                {
                    //INFOTOCERR << " Error: Infinitely many solutions for log of a negative scalar (w=" << w << ")." << endl;
                    //throw(InfinitelyManySolutions);
                }
                Result.w = ILog(w);
            }
            else
            {
                const T _v = IAtan2(b, w);
                const T _f = _v/b;
                Result.w   = ILog(w*w+b*b)/2.0;
                // Result.w = std::log(w/std::cos(v)); // Not nice for unit vectors [w=cos(v)=0]
                Result.v.x = _f*v.x;
                Result.v.y = _f*v.y;
                Result.v.z = _f*v.z;
            }
            return Result;
        }

        /// Return logarithm of a rotor.
        SIMD_INLINE IQuaternion<T> logRotor() const
        {
            /// This function is just like the standard log function, except
            /// that a negative scalar does not raise an exception; instead, the
            /// scalar pi is returned.
            IQuaternion<T> Result;
            const T b = ISqrt(v.x*v.x + v.y*v.y + v.z*v.z);

            if(IAbs(b) <= MACHINE_EPSILON*IAbs(w))
            {
                if(w<0.0)
                {
                    //        INFOTOCERR
                    //             << "\nWarning: Infinitely many solutions for log of a negative scalar (w=" << w << "); "
                    //             << "arbitrarily returning the one in the x direction." << endl;
                    Result.v.x = M_PI;
                    if(IAbs(w+1)>MACHINE_EPSILON)
                    {
                        Result.w = ILog(-w);
                    }
                }
                else
                {
                    Result.w = ILog(w);
                }
            }
            else
            {
                const T _v = IAtan2(b, w);
                const T _f = _v/b;
                Result.w = ILog(w*w+b*b)/2.0;
                // Result.w = std::log(w/std::cos(v)); // Not nice for unit vectors [w=cos(v)=0]
                Result.v.x = _f*v.x;
                Result.v.y = _f*v.y;
                Result.v.z = _f*v.z;
            }

            return Result;
        }

        /// Return exponent of Quaternion.
        SIMD_INLINE IQuaternion<T>  exp() const
        {
            IQuaternion<T>  Result;
            const T b = ISqrt(v.x*v.x + v.y*v.y + v.z*v.z);
            if(IAbs(b)<=MACHINE_EPSILON*IAbs(w))
            {
                Result.w = IExp(w);
            }
            else
            {
                const T e = IExp(w);
                const T f = ISin(b)/b; // Note: b is never 0.0 at this point
                Result.w   = e*ICos(b);
                Result.v.x = e*f*v.x;
                Result.v.y = e*f*v.y;
                Result.v.z = e*f*v.z;
            }
            return Result;
        }

        //----------[ output operator ]----------------------------
        /**
  * Provides output to standard output stream.
  */
        friend std::ostream& operator <<(std::ostream& oss, const IQuaternion<T>& q)
        {
            oss << "(" << "Re: " << q.w << " Im: " << q.v << ")";
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


        //---------------------------------- Plugins ----------------------------------------//

        /// The angle between the vectors is simple to find: the dot product gives its cosine.
        /// The needed axis is also simple to find: it’s the cross product of the two vectors.
        static SIMD_INLINE IQuaternion<T> rotationBetweenSlerp(  IVector3D<T> start ,  IVector3D<T> dest )
        {

            //start.normalize();
            //dest.normalize();

            T cosTheta = start.dot(dest);
            IVector3D<T> rotationAxis;

            if (cosTheta < -1 + 0.0001f)
            {
                // special case when vectors in opposite directions:
                // there is no "ideal" rotation axis
                // So guess one; any will do as long as it's perpendicular to start
                rotationAxis = IVector3D<T>::Z.cross(start);
                if (rotationAxis.length2() < T(0.0001) )
                {
                    // bad luck, they were parallel, try again!
                    rotationAxis = IVector3D<T>::X.cross(start);
                }

                rotationAxis.normalize();
                static IQuaternion<T> q;
                return q.fromAxisRot(rotationAxis,T(180.0f));
            }

            rotationAxis = start.cross(dest);
            //rotationAxis = rotationAxis.normalize();



            T s = ISqrt( (1+cosTheta)*2 );
            T invs = 1.0 / s;

            return IQuaternion<T>
                    (
                        rotationAxis.x * invs,
                        rotationAxis.y * invs,
                        rotationAxis.z * invs,
                        s * 0.5f
                        );

        }


        //------------------------------- friendship ----------------------------//
        friend class IMatrix3x3<T>;
        friend class IMatrix4x4<T>;



    public:
        /**
     * The multiplicitive identity quaternion
     */
        static const IQuaternion<T> IDENTITY;
        /**
     * The additive identity quaternion.
     */
        static const IQuaternion<T> ZERO;

    };

    template<class T> const IQuaternion<T> IQuaternion<T>::IDENTITY(0.0, 0.0, 0.0, 1.0);
    template<class T> const IQuaternion<T> IQuaternion<T>::ZERO(0.0, 0.0, 0.0, 0.0);



    //--------------------------------------
    // Typedef shortcuts for Quaternion
    //-------------------------------------
    /// Octonion of floats
    typedef IQuaternion<float> IQuaternionf;
    /// Octonionof doubles
    typedef IQuaternion<double> IQuaterniond;
    /// Octonion of int
    typedef IQuaternion<double> IQuaternioni;

} /* namespace  */



#endif /* IQUATERNION_H_ */
