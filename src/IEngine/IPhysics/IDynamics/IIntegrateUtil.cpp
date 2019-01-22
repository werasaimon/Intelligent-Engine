#include "IIntegrateUtil.h"


namespace IPhysics
{

    IIntegrateUtil::IIntegrateUtil()
    {

    }

    ITransform IIntegrateUtil::IntegrateTransform(const ITransform &curTrans, const IVector3 &linvel, const IVector3 &angvel, scalar timeStep)
    {
        //Exponential map
        //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
        IVector3 axis;
        scalar fAngle = angvel.Length();

        if (fAngle < scalar(0.001))
        {
            // use Taylor's expansions of sync function
            axis = angvel * (scalar(0.5) * timeStep -
                    (timeStep * timeStep * timeStep) *
                    (scalar(0.020833333333)) * fAngle * fAngle);
        }
        else
        {
            axis = angvel * (ISin(scalar(0.5) * fAngle * timeStep) / fAngle);
        }


        IQuaternion dorn(axis, ICos(fAngle * timeStep * scalar(0.5)));
        IQuaternion predictedOrn = dorn * IQuaternion(curTrans.GetBasis());
        predictedOrn.Normalize();

        return ITransform( curTrans.GetPosition() + linvel * timeStep ,  predictedOrn.GetRotMatrix(), curTrans.GetTime() );

    }

    void IIntegrateUtil::Damping(IVector3 &Velocity, const scalar &min_damping, const scalar &damping)
    {
        if( Velocity.LengthSquare()  < min_damping )
        {
            if( Velocity.Length() > damping )
            {
                Velocity -= ( Velocity.GetUnit() * damping);
            }
            else
            {
                Velocity  = IVector3(0,0,0);
            }
        }
    }




}

