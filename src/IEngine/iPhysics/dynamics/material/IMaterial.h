#ifndef IMATERIAL_H_
#define IMATERIAL_H_


#include "../../common/math/IMatematical.h"

namespace IPhysics
{


// Class Material
/**
 * This class contains the material properties of a rigid body that will be use for
 * the dynamics simulation like the friction coefficient or the bounciness of the rigid
 * body.
 */
class IMaterial
{

    private :

        // -------------------- Attributes -------------------- //

        /// Friction coefficient (positive value)
        scalar mFrictionCoefficient;

        /// Rolling resistance factor (positive value)
        scalar mRollingResistance;

        /// Bounciness during collisions (between 0 and 1) where 1 is for a very bouncy body
        scalar mBounciness;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        IMaterial();

        /// Copy-constructor
        IMaterial(const IMaterial& material);

        /// Destructor
        ~IMaterial();

        /// Return the bounciness
        scalar getBounciness() const;

        /// Set the bounciness.
        void setBounciness(scalar bounciness);

        /// Return the friction coefficient
        scalar getFrictionCoefficient() const;

        /// Set the friction coefficient.
        void setFrictionCoefficient(scalar frictionCoefficient);

        /// Return the rolling resistance factor
        scalar getRollingResistance() const;

        /// Set the rolling resistance factor
        void setRollingResistance(scalar rollingResistance);

        /// Overloaded assignment operator
        IMaterial& operator=(const IMaterial& material);
};

// Return the bounciness
/**
 * @return Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
SIMD_INLINE scalar IMaterial::getBounciness() const
{
    return mBounciness;
}

// Set the bounciness.
/// The bounciness should be a value between 0 and 1. The value 1 is used for a
/// very bouncy body and zero is used for a body that is not bouncy at all.
/**
 * @param bounciness Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
SIMD_INLINE void IMaterial::setBounciness(scalar bounciness)
{
    assert(bounciness >= scalar(0.0) && bounciness <= scalar(1.0));
    mBounciness = bounciness;
}

// Return the friction coefficient
/**
 * @return Friction coefficient (positive value)
 */
SIMD_INLINE scalar IMaterial::getFrictionCoefficient() const
{
    return mFrictionCoefficient;
}

// Set the friction coefficient.
/// The friction coefficient has to be a positive value. The value zero is used for no
/// friction at all.
/**
 * @param frictionCoefficient Friction coefficient (positive value)
 */
SIMD_INLINE void IMaterial::setFrictionCoefficient(scalar frictionCoefficient)
{
    assert(frictionCoefficient >= scalar(0.0));
    mFrictionCoefficient = frictionCoefficient;
}

// Return the rolling resistance factor. If this value is larger than zero,
// it will be used to slow down the body when it is rolling
// against another body.
/**
 * @return The rolling resistance factor (positive value)
 */
SIMD_INLINE scalar IMaterial::getRollingResistance() const
{
    return mRollingResistance;
}

// Set the rolling resistance factor. If this value is larger than zero,
// it will be used to slow down the body when it is rolling
// against another body.
/**
 * @param rollingResistance The rolling resistance factor
 */
SIMD_INLINE void IMaterial::setRollingResistance(scalar rollingResistance)
{
    assert(rollingResistance >= 0);
    mRollingResistance = rollingResistance;
}

// Overloaded assignment operator
SIMD_INLINE IMaterial& IMaterial::operator=(const IMaterial& material)
{

    // Check for self-assignment
    if (this != &material)
    {
        mFrictionCoefficient = material.mFrictionCoefficient;
        mBounciness = material.mBounciness;
        mRollingResistance = material.mRollingResistance;
    }

    // Return this material
    return *this;
}

}

#endif // IMATERIAL_H_
