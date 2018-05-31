#include "IMeshCreateBox.h"

namespace IGeometry
{

IMeshCreateBox::IMeshCreateBox(const IVector3 &halfSize)
: mExtentSize(halfSize)
{
    init(halfSize);
}

void IMeshCreateBox::init(const IVector3 &halfSize)
{
    const float size = 1.0;

    // Vertices
    const IVector3 vertices[] =
    {
        // front
        IVector3(-size, size, size),
        IVector3( size, size, size),
        IVector3( size,-size, size),
        IVector3(-size,-size, size),
        // back
        IVector3( size, size,-size),
        IVector3(-size, size,-size),
        IVector3(-size,-size,-size),
        IVector3( size,-size,-size),
        // top
        IVector3(-size, size,-size),
        IVector3( size, size,-size),
        IVector3( size, size, size),
        IVector3(-size, size, size),
        // bottom
        IVector3( size,-size,-size),
        IVector3(-size,-size,-size),
        IVector3(-size,-size, size),
        IVector3( size,-size, size),
        // left
        IVector3(-size, size,-size),
        IVector3(-size, size, size),
        IVector3(-size,-size, size),
        IVector3(-size,-size,-size),
        // right
        IVector3( size, size, size),
        IVector3( size, size,-size),
        IVector3( size,-size,-size),
        IVector3( size,-size, size)
    };


    // textures UV
    const IVector2 vUVs[] =
    {
        // front
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
        // back
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
        // top
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
        // bottom
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
        // left
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
        // right
        IVector2(0.0f,1.0f),
        IVector2(1.0f,1.0f),
        IVector2(1.0f,0.0f),
        IVector2(0.0f,0.0f),
    };


    // Normals
    const IVector3 normals[] =
    {
        IVector3(0.0f, 0.0f, 1.0f),
        IVector3(0.0f, 0.0f, 1.0f),
        IVector3(0.0f, 0.0f, 1.0f),
        IVector3(0.0f, 0.0f, 1.0f),
        // back
        IVector3(0.0f, 0.0f,-1.0f),
        IVector3(0.0f, 0.0f,-1.0f),
        IVector3(0.0f, 0.0f,-1.0f),
        IVector3(0.0f, 0.0f,-1.0f),
        // top
        IVector3(0.0f, 1.0f, 0.0f),
        IVector3(0.0f, 1.0f, 0.0f),
        IVector3(0.0f, 1.0f, 0.0f),
        IVector3(0.0f, 1.0f, 0.0f),
        // bottom
        IVector3(0.0f,-1.0f, 0.0f),
        IVector3(0.0f,-1.0f, 0.0f),
        IVector3(0.0f,-1.0f, 0.0f),
        IVector3(0.0f,-1.0f, 0.0f),
        // left
        IVector3(-1.0f, 0.0f, 0.0f),
        IVector3(-1.0f, 0.0f, 0.0f),
        IVector3(-1.0f, 0.0f, 0.0f),
        IVector3(-1.0f, 0.0f, 0.0f),
        // right
        IVector3(1.0f, 0.0f, 0.0f),
        IVector3(1.0f, 0.0f, 0.0f),
        IVector3(1.0f, 0.0f, 0.0f),
        IVector3(1.0f, 0.0f, 0.0f)
    };

    const IColor colors[] =
    {
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),

        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),

        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),

        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),

        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),

        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 ),
        IColor( 1 , 1 , 1 , 1 )

    };


    // indices
    const uint indices[] =
    {
        0, 3, 1,  1, 3, 2,  // front
        4, 7, 5,  5, 7, 6,  // back
        8,11, 9,  9,11,10,  // top
        12,15,13, 13,15,14, // bottom
        16,19,17, 17,19,18, // left
        20,23,21, 21,23,22  // right
    };



    //-------------------- init geometry -----------------------//

    for (uint i = 0; i < sizeof(vertices) / sizeof(IVector3); ++i)
    {
        mVertices.push_back( vertices[i] * halfSize );
        mUVs.push_back( vUVs[i] );
    }

    for (uint i = 0; i < sizeof(normals) / sizeof(IVector3); ++i)
    {
        mNormals.push_back( normals[i] );
    }

    for (uint i = 0; i < (sizeof(colors)/sizeof(IColor)); ++i)
    {
        mColors.push_back(colors[i]);
    }


    for (uint i = 0; i < (sizeof(indices)/sizeof(uint)); ++i)
    {
        mIndicess.push_back(indices[i]);
    }

    for (uint i = 0; i < (sizeof(indices)/sizeof(uint)); i+=3)
    {
        uint a = indices[i + 0];
        uint b = indices[i + 1];
        uint c = indices[i + 2];
        std::vector<uint> indx;
        indx.push_back(a);
        indx.push_back(b);
        indx.push_back(c);
        mIndices.push_back(indx);
    }
}


}
