#ifndef IJOINT_H
#define IJOINT_H

#include "../../body/IRigidBody.h"

namespace IPhysics
{



enum JointType { BALLSOCKETJOINT,
                 SLIDERJOINT,
                 HINGEJOINT ,
                 FIXEDJOINT ,
                 DISTANCEJOINT ,
                 AXISJOINT};



// Class declarations
struct IConstraintSolverData;
class  IJoint;

struct IJointInfo
{

    public:

        // -------------------- Attributes -------------------- //

        /// First rigid body of the IJoint
        IRigidBody* body1;

        /// Second rigid body of the IJoint
        IRigidBody* body2;

        /// Type of the IJoint
        JointType type;

        /// Position correction technique used for the constraint (used for joints).
        /// By default, the BAUMGARTE technique is used
        JointsPositionCorrectionTechnique positionCorrectionTechnique;

        /// True if the two bodies of the joint are allowed to collide with each other
        bool isCollisionEnabled;

        /// Constructor
        IJointInfo(JointType constraintType)
                      : body1(NULL), body2(NULL), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true)
        {
        }

        /// Constructor
        IJointInfo(IRigidBody* rigidBody1, IRigidBody* rigidBody2, JointType constraintType)
                      : body1(rigidBody1), body2(rigidBody2), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true)
        {
        }

        /// Destructor
        virtual ~IJointInfo() {}

};



// Class IJoint
/**
 * This abstract class represents a IJoint between two bodies.
 */
class IJoint //: public BlockAlloc<IJoint>
{

    protected:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first body of the IJoint
        IRigidBody* const mBody1;

        /// Pointer to the second body of the IJoint
        IRigidBody* const mBody2;

        /// Type of the IJoint
        const JointType mType;

        /// Body 1 index in the velocity array to solve the constraint
        u32 mIndexBody1;

        /// Body 2 index in the velocity array to solve the constraint
        u32 mIndexBody2;

        /// Position correction technique used for the constraint (used for joints)
        JointsPositionCorrectionTechnique mPositionCorrectionTechnique;

        /// True if the two bodies of the constraint are allowed to collide with each other
        bool mIsCollisionEnabled;

        /// True if the IJoint has already been added into an island
        bool mIsAlreadyInIsland;


        /****************************/
        scalar biasFactor;
        scalar softness; //0.05f;0.0f;
        /***************************/

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        IJoint(const IJoint& constraint);

        /// Private assignment operator
        IJoint& operator=(const IJoint& constraint);

        /// Return true if the IJoint has already been added into an island
        bool isAlreadyInIsland() const;

        /// Return the number of bytes used by the IJoint
        virtual size_t getSizeInBytes() const = 0;

        /// Initialize before solving the IJoint
        virtual void initBeforeSolve( const IConstraintSolverData& constraintSolverData ) = 0;

        /// Warm start the IJoint (apply the previous impulse at the beginning of the step)
        virtual void warmstart( const IConstraintSolverData& constraintSolverData ) = 0;

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint( const IConstraintSolverData& constraintSolverData ) = 0;


        /// Solve the position constraint
        virtual void solvePositionConstraint( const IConstraintSolverData& constraintSolverData ) = 0;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        IJoint(const IJointInfo& jointInfo);

        /// Destructor
        virtual ~IJoint();

        /// Return the reference to the body 1
        IRigidBody* getBody1() const;

        /// Return the reference to the body 2
        IRigidBody* getBody2() const;

        /// Return true if the constraint is active
        bool isActive() const;

        /// Return the type of the constraint
        JointType getType() const;

        /// Return true if the collision between the two bodies of the IJoint is enabled
        bool isCollisionEnabled() const;

        /// set false if the collision between the two bodies of the IJoint is enabled
        void setIsCollisionEnabled(bool isCollisionEnabled);

        /// set stoffnes constraint-joint
        void setSoftness(const scalar &value);


        // -------------------- Friendship -------------------- //
        friend class IDynamicsWorld;
        friend class IIsland;
        friend class IConstraintSolver;



};



// Return the reference to the body 1
/**
 * @return The first body involved in the IJoint
 */
SIMD_INLINE IRigidBody* IJoint::getBody1() const
{
    return mBody1;
}

// Return the reference to the body 2
/**
 * @return The second body involved in the IJoint
 */

SIMD_INLINE IRigidBody* IJoint::getBody2() const
{
    return mBody2;
}

// Return true if the IJoint is active
/**
 * @return True if the IJoint is active
 */
SIMD_INLINE bool IJoint::isActive() const
{
    return (mBody1->isActive() &&
            mBody2->isActive());
}

// Return the type of the IJoint
/**
 * @return The type of the IJoint
 */
SIMD_INLINE JointType IJoint::getType() const
{
    return mType;
}

// Return true if the collision between the two bodies of the IJoint is enabled
/**
 * @return True if the collision is enabled between the two bodies of the IJoint
 *              is enabled and false otherwise
 */
SIMD_INLINE bool IJoint::isCollisionEnabled() const
{
    return mIsCollisionEnabled;
}


SIMD_INLINE void IJoint::setIsCollisionEnabled(bool isCollisionEnabled)
{
    mIsCollisionEnabled = isCollisionEnabled;
}


// Return true if the IJoint has already been added into an island
SIMD_INLINE bool IJoint::isAlreadyInIsland() const
{
    return mIsAlreadyInIsland;
}




}

#endif // IJOINT_H
