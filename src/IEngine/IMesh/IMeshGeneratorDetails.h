#ifndef IMESHGENERATORDETAILS_H
#define IMESHGENERATORDETAILS_H


#include "IMeshModel.h"
#include <algorithm>


namespace IEngine
{



    namespace MeshGenerator
    {


    static const auto pi        = M_PI;
    static const auto pi_2      = pi*float(2);
    static const auto pi_0_5    = pi*float(0.5);


    using VertexIndex = std::size_t;



    void AddTriangulatedQuad(
        IMeshModel& mesh,
        bool alternateGrid,
        std::uint32_t u, std::uint32_t v,
        VertexIndex i0, VertexIndex i1,
        VertexIndex i2, VertexIndex i3,
        VertexIndex indexOffset = 0
    );


    } // /namespace MeshGenerator

}

#endif // IMESHGENERATORDETAILS_H
