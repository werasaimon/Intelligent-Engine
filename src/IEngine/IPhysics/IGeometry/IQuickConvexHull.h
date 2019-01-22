#ifndef IQUICKCONVEXHULL_H
#define IQUICKCONVEXHULL_H

#include "QuickHull/ConvexHull.hpp"
#include "QuickHull/QuickHull.hpp"

namespace IPhysics
{

   //-------------------------------------------------------------//
    template<class T> using IQuickHull  = quickhull::QuickHull<T>;
    template<class T> using IConvexHull = quickhull::ConvexHull<T>;
   //-------------------------------------------------------------//

}

#endif // IQUICKCONVEXHULL_H
