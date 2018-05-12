#ifndef SOURCE_ENGIE_SCALAR_H_
#define SOURCE_ENGIE_SCALAR_H_


namespace IPhysics
{

#if defined(IS_DOUBLE_PRECISION_ENABLED)   // If we are compiling for double precision
    typedef double scalar;
#else                                   // If we are compiling for single precision
    typedef float scalar;
#endif

}



#endif /* SOURCE_ENGIE_SCALAR_H_ */
