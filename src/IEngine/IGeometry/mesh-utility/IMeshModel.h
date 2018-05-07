#ifndef IMESHMODEL_H
#define IMESHMODEL_H


// Libraries
#include <string>
#include <vector>
#include <map>

#include "../math/IMatematical.h"
#include "../math/IColor.h"
#include "../transform/IObject3D.h"
#include "ITexture2D.h"

namespace IGeometry
{



typedef unsigned int uint;


// Class Mesh
// This class represents a 3D triangular mesh
// object that can be loaded from an OBJ file for instance.
class IMeshModel : public IObject3D
{


    protected:

    // -------------------- Attributes -------------------- //

        // A triplet of vertex indices for each triangle
        std::vector<std::vector<uint> > mIndices;
        std::vector<uint>               mIndicess;

        // Vertices coordinates (local space)
        std::vector<IVector3> mVertices;

        // Normals coordinates
        std::vector<IVector3> mNormals;

        // Tangents coordinates
        std::vector<IVector3> mTangents;

        // Color for each vertex
        std::vector<IColor>   mColors;

        // UV texture coordinates
        std::vector<IVector2> mUVs;

        // Textures of the mesh (one for each part of the mesh)
        std::map<uint, ITexture2D> mTextures;




    public:


        //-------------------- Methods --------------------//

        // Constructor
        IMeshModel();

        // Destructor
       ~IMeshModel();




        // Destroy the IMeshModel
        void destroy();

        // Compute the normals of the IMeshModel
        void calculateNormals();

        // Compute the tangents of the IMeshModel
        void calculateTangents();

        // Calculate the bounding box of the IMeshModel
        void calculateBoundingBox(IVector3& min, IVector3& max) const;

        // Scale of vertices of the IMeshModel using a given factor
        void scaleVertices(float factor);



        // Return the number of triangles
        uint getNbFaces(uint part = 0) const;

        // Return the number of vertices
        uint getNbVertices() const;

        // Return the number of vertices UV coords
        uint getNbUVsVertices() const;

        // Return the number of normals
        uint getNbNormals() const;

        // Return the number of tangents
        uint getNbTangents() const;

        // Return the number of parts in the IMeshModel
        uint getNbParts() const;

        // Return the number of indiciess
        uint getNbIndicess() const;



        // Return a reference to the vertices
        const std::vector<IVector3>& getVertices() const;

        // Set the vertices of the IMeshModel
        void setVertices(std::vector<IVector3>& vertices);

        // Return a reference to the normals
        const std::vector<IVector3>& getNormals() const;

        // set the normals of the IMeshModel
        void setNormals(std::vector<IVector3>& normals);

        // Return a reference to the UVs
        const std::vector<IVector2>& getUVs() const;

        // Set the UV texture coordinates of the IMeshModel
        void setUVs(std::vector<IVector2>& uvs);

        // Return a reference to the vertex indices
        const std::vector<uint>& getIndices(uint part = 0) const;

        // Set the vertices indices of the IMeshModel
        void setIndices(std::vector<std::vector<uint> >& indices);

        // Return the coordinates of a given vertex
        const IVector3& getVertex(uint i) const;

        // Set the coordinates of a given vertex
        void setVertex(uint i, const IVector3& vertex);

        // Return the coordinates of a given normal
        const IVector3& getNormal(uint i) const;

        // Set the coordinates of a given normal
        void setNormal(uint i, const IVector3& normal);

        // Return the color of a given vertex
        const IColor& getColor(uint i) const;

        // Set the color of a given vertex
        void setColor(uint i, const IColor& color);

        // Set a color to all the vertices
        void setColorToAllVertices(const IColor& color);

        // Return the UV of a given vertex
        const IVector2& getUV(uint i) const;

        // Set the UV of a given vertex
        void setUV(uint i, const IVector2& uv);

        // Return the vertex index of the ith (i=0,1,2) vertex of a given face
        uint getVertexIndexInFace(uint faceIndex, uint i, uint part = 0) const;



        // Return true if the IMeshModel has normals
        bool hasNormals() const;

        // Return true if the IMeshModel has tangents
        bool hasTangents() const;

        // Return true if the IMeshModel has vertex colors
        bool hasColors() const;

        // Return true if the IMeshModel has UV texture coordinates
        bool hasUVTextureCoordinates() const;

        // Return true if the IMeshModel has a texture for a given part of the IMeshModel and if it
        // also have texture coordinates
        bool hasTextureForPart(uint part = 0) const;

        // Return true if the IMeshModel has a texture (and texture coordinates) for at least one
        // part of the IMeshModel
        bool hasTexture() const;



        // Return a pointer to the vertices data
        void* getVerticesPointer();

        // Return a pointer to the normals data
        void* getNormalsPointer();

        // Return a pointer to the colors data
        void* getColorsPointer();

        // Return a pointer to the tangents data
        void* getTangentsPointer();

        // Return a pointer to the UV texture coordinates data
        void* getUVTextureCoordinatesPointer();

        // Return a pointer to the vertex indicies data
        void* getIndicesPointer(uint part = 0);

        // Return a pointer to the vertex indiciess data
        void* getIndicessPointer();



        // Return a reference to a texture of the IMeshModel
        ITexture2D &getTexture(uint part = 0);

        // Set a texture to a part of the IMeshModel
        void setTexture(ITexture2D &texture, uint part = 0);




        // Return a index to the vertex indicies data
        const std::vector<std::vector<uint> > getIndices() const { return mIndices; }

        // Return a index to the vertex indiciess data
        const std::vector<uint>& getIndicess() const { return mIndicess; }



        //---------------- friendship ----------------------//


};

// Return the number of triangles
inline uint IMeshModel::getNbFaces(uint part) const
{
    return mIndices[part].size() / 3;
}


// Return the number of vertices
inline uint IMeshModel::getNbVertices() const
{
    return mVertices.size();
}


// Return the number of vertices UV coords
inline uint IMeshModel::getNbUVsVertices() const
{
    return mUVs.size();
}

// Return the number of normals
inline uint IMeshModel::getNbNormals() const
{
    return mNormals.size();
}

// Return the number of tangents
inline uint IMeshModel::getNbTangents() const
{
    return mTangents.size();
}

// Return the number of parts in the IMeshModel
inline uint IMeshModel::getNbParts() const
{
    return mIndices.size();
}


// Return the number of indicess
inline uint IMeshModel::getNbIndicess() const
{
    return mIndicess.size();
}

// Return a reference to the vertices
inline const std::vector<IVector3>& IMeshModel::getVertices() const
{
    return mVertices;
}

// Set the vertices of the IMeshModel
inline void IMeshModel::setVertices(std::vector<IVector3>& vertices)
{
    mVertices = vertices;
}

// Return a reference to the normals
inline const std::vector<IVector3>& IMeshModel::getNormals() const
{
    return mNormals;
}

// set the normals of the IMeshModel
inline void IMeshModel::setNormals(std::vector<IVector3>& normals)
{
    mNormals = normals;
}

// Return a reference to the UVs
inline const std::vector<IVector2>& IMeshModel::getUVs() const
{
    return mUVs;
}

// Set the UV texture coordinates of the IMeshModel
inline void IMeshModel::setUVs(std::vector<IVector2>& uvs)
{
    mUVs = uvs;
}

// Return a reference to the vertex indices
inline const std::vector<uint>& IMeshModel::getIndices(uint part) const
{
    return mIndices[part];
}

// Set the vertices indices of the IMeshModel
inline void IMeshModel::setIndices(std::vector<std::vector<uint> >& indices)
{
    mIndices = indices;
}

// Return the coordinates of a given vertex
inline const IVector3& IMeshModel::getVertex(uint i) const
{
    assert(i < getNbVertices());
    return mVertices[i];
}

// Set the coordinates of a given vertex
inline void IMeshModel::setVertex(uint i, const IVector3& vertex)
{
    assert(i < getNbVertices());
    mVertices[i] = vertex;
}

// Return the coordinates of a given normal
inline const IVector3& IMeshModel::getNormal(uint i) const
{
    assert(i < getNbVertices());
    return mNormals[i];
}

// Set the coordinates of a given normal
inline void IMeshModel::setNormal(uint i, const IVector3& normal)
{
    assert(i < getNbVertices());
    mNormals[i] = normal;
}

// Return the color of a given vertex
inline const IColor& IMeshModel::getColor(uint i) const
{
    assert(i < getNbVertices());
    return mColors[i];
}

// Set the color of a given vertex
inline void IMeshModel::setColor(uint i, const IColor& color)
{

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size())
    {

        // Create the color array with the same size
        mColors = std::vector<IColor>(mVertices.size());
    }

    mColors[i] = color;
}

// Set a color to all the vertices
inline void IMeshModel::setColorToAllVertices(const IColor& color)
{

    // If the color array does not have the same size as
    // the vertices array
    if (mColors.size() != mVertices.size())
    {

        // Create the color array with the same size
        mColors = std::vector<IColor>(mVertices.size());
    }

    for (size_t v=0; v<mVertices.size(); v++)
    {
        mColors[v] = color;
    }
}

// Return the UV of a given vertex
inline const IVector2& IMeshModel::getUV(uint i) const
{
    assert(i < getNbVertices());
    return mUVs[i];
}

// Set the UV of a given vertex
inline void IMeshModel::setUV(uint i, const IVector2& uv)
{
    assert(i < getNbVertices());
    mUVs[i] = uv;
}

// Return the vertex index of the ith (i=0,1,2) vertex of a given face
inline uint IMeshModel::getVertexIndexInFace(uint faceIndex, uint i, uint part) const
{
    return (mIndices[part])[faceIndex*3 + i];
}

// Return true if the IMeshModel has normals
inline bool IMeshModel::hasNormals() const
{
    return mNormals.size() == mVertices.size();
}

// Return true if the IMeshModel has tangents
inline bool IMeshModel::hasTangents() const
{
    return mTangents.size() == mVertices.size();
}

// Return true if the IMeshModel has vertex colors
inline bool IMeshModel::hasColors() const
{
    return mColors.size() == mVertices.size();
}

// Return true if the IMeshModel has UV texture coordinates
inline bool IMeshModel::hasUVTextureCoordinates() const
{
    return mUVs.size() == mVertices.size();
}

// Return true if the IMeshModel has a texture for a given part of the IMeshModel and if it
// also have texture coordinates
inline bool IMeshModel::hasTextureForPart(uint part) const
{
    return hasUVTextureCoordinates() && mTextures.count(part);
}

// Return true if the IMeshModel has a texture (and texture coordinates) for at least one
// part of the IMeshModel
inline bool IMeshModel::hasTexture() const
{
    return hasUVTextureCoordinates() && (mTextures.size() > 0);
}

// Return a pointer to the vertices data
inline void* IMeshModel::getVerticesPointer()
{
    return &(mVertices[0]);
}

// Return a pointer to the normals data
inline void* IMeshModel::getNormalsPointer()
{
    return &(mNormals[0]);
}

// Return a pointer to the colors data
inline void* IMeshModel::getColorsPointer()
{
    return &(mColors[0]);
}

// Return a pointer to the tangents data
inline void* IMeshModel::getTangentsPointer()
{
    return &(mTangents[0]);
}

// Return a pointer to the UV texture coordinates data
inline void* IMeshModel::getUVTextureCoordinatesPointer()
{
    return &(mUVs[0]);
}

// Return a pointer to the vertex indicies data
inline void* IMeshModel::getIndicesPointer(uint part)
{
    return &(mIndices[part])[0];
}


// Return a pointer to the vertex indiciess data
inline void* IMeshModel::getIndicessPointer()
{
    return &(mIndices[0]);
}

// Return a reference to a texture of the IMeshModel
inline ITexture2D& IMeshModel::getTexture(uint part)
{
    return mTextures[part];
}

// Set a texture to a part of the IMeshModel
inline void IMeshModel::setTexture(ITexture2D& texture, uint part)
{
    mTextures[part] = texture;
}


}

#endif // IMESHMODEL_H
