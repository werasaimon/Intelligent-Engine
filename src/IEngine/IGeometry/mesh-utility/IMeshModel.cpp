#include "IMeshModel.h"

namespace IGeometry
{


// Constructor
IMeshModel::IMeshModel()
    //: IGeometryModel()
{

}

// Destructor
IMeshModel::~IMeshModel()
{
   destroy();
}


// Destroy the IIMeshModel
void IMeshModel::destroy()
{
    /// - clear all elements - ///
    mVertices.clear();
    mUVs.clear();
    mNormals.clear();
    mTangents.clear();

    mColors.clear();
    mTextures.clear();

    mIndices.clear();
    mIndicess.clear();

}

// Compute the normals of the IMeshModel
void IMeshModel::calculateNormals()
{

    mNormals = std::vector<IVector3>(getNbVertices(), IVector3(0, 0, 0));

    // For each triangular face
    for (uint i=0; i<getNbFaces(); i++)
    {

        // Get the three vertices index of the current face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Compute the normal of the face
        IVector3 p = getVertex(v1);
        IVector3 q = getVertex(v2);
        IVector3 r = getVertex(v3);
        IVector3 normal = (q-p).cross(r-p).normalized();

        // Add the face surface normal to the sum of normals at
        // each vertex of the face
        mNormals[v1] += normal;
        mNormals[v2] += normal;
        mNormals[v3] += normal;
    }

    // Normalize the normal at each vertex
    for (uint i=0; i<getNbVertices(); i++)
    {
        assert(mNormals[i].length() > 0);
        mNormals[i] = mNormals[i].normalized();
    }
}

// Compute the tangents of the IMeshModel
void IMeshModel::calculateTangents()
{

    mTangents = std::vector<IVector3>(getNbVertices(), IVector3(0, 0, 0));

    // For each face
    for (uint i=0; i<getNbFaces(); i++)
    {

        // Get the three vertices index of the face
        uint v1 = getVertexIndexInFace(i, 0);
        uint v2 = getVertexIndexInFace(i, 1);
        uint v3 = getVertexIndexInFace(i, 2);

        assert(v1 < getNbVertices());
        assert(v2 < getNbVertices());
        assert(v3 < getNbVertices());

        // Get the vertices positions
        IVector3 p = getVertex(v1);
        IVector3 q = getVertex(v2);
        IVector3 r = getVertex(v3);

        // Get the texture coordinates of each vertex
        IVector2 uvP = getUV(v1);
        IVector2 uvQ = getUV(v2);
        IVector2 uvR = getUV(v3);

        // Get the three edges
        IVector3 edge1 = q - p;
        IVector3 edge2 = r - p;
        IVector2 edge1UV = uvQ - uvP;
        IVector2 edge2UV = uvR - uvP;

        float cp = edge1UV.y * edge2UV.x - edge1UV.x * edge2UV.y;

        // Compute the tangent
        if (cp != 0.0f)
        {
            float factor = 1.0f / cp;
            IVector3 tangent = (edge1 * -edge2UV.y + edge2 * edge1UV.y) * factor;
            tangent.normalize();
            mTangents[v1] += tangent;
            mTangents[v2] += tangent;
            mTangents[v3] += tangent;
        }
    }
}


// Calculate the bounding box of the IMeshModel
void IMeshModel::calculateLocalBoundingBox(IVector3& min, IVector3& max) const
{

    // If the IMeshModel contains vertices
    if (!mVertices.empty())
    {

        min = mVertices[0];
        max = mVertices[0];

        std::vector<IVector3>::const_iterator  it(mVertices.begin());

        // For each vertex of the IMeshModel
        for (; it != mVertices.end(); ++it)
        {

            if( (*it).x < min.x ) min.x = (*it).x;
            else if ( (*it).x > max.x ) max.x = (*it).x;

            if( (*it).y < min.y ) min.y = (*it).y;
            else if ( (*it).y > max.y ) max.y = (*it).y;

            if( (*it).z < min.z ) min.z = (*it).z;
            else if ( (*it).z > max.z ) max.z = (*it).z;
        }
    }
    else
    {
        std::cerr << "Error : Impossible to calculate the bounding box of the IMeshModel because there" <<
                    "are no vertices !" << std::endl;
        assert(false);
    }
}

void IMeshModel::InitBoundingBox()
{
    IVector3 min;
    IVector3 max;

    // Calculation axis-aligned bounding box
    calculateLocalBoundingBox(min,max);

    // Initilization  axis-aligned bounding box
    InitAxisAlignedBoundingBox(min,max);

}




// Calculate the bounding box of the IMeshModel
void IMeshModel::calculateBoundingBox(IVector3& min, IVector3& max) const
{

    // If the IMeshModel contains vertices
    if (!mVertices.empty())
    {

        min = mVertices[0];
        max = mVertices[0];

        std::vector<IVector3>::const_iterator  it(mVertices.begin());

        // For each vertex of the IMeshModel
        for (; it != mVertices.end(); ++it)
        {

            if( (*it).x < min.x ) min.x = (*it).x;
            else if ( (*it).x > max.x ) max.x = (*it).x;

            if( (*it).y < min.y ) min.y = (*it).y;
            else if ( (*it).y > max.y ) max.y = (*it).y;

            if( (*it).z < min.z ) min.z = (*it).z;
            else if ( (*it).z > max.z ) max.z = (*it).z;
        }
    }
    else
    {
        std::cerr << "Error : Impossible to calculate the bounding box of the IMeshModel because there" <<
                    "are no vertices !" << std::endl;
        assert(false);
    }
}

// Scale of vertices of the IMeshModel using a given factor
void IMeshModel::scaleVertices(float factor)
{
    // For each vertex
    for (uint i=0; i<getNbVertices(); i++)
    {
        mVertices.at(i) *= factor;
    }
}


}
