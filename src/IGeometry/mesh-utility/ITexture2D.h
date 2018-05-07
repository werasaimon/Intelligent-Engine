#ifndef ITEXTURE2D_H
#define ITEXTURE2D_H

#include <assert.h>


namespace IGeometry
{

typedef unsigned int uint;

// Class Texture2D
// This class represents a 2D texture
class ITexture2D
{


    friend class  TextureReader;

    private:

        // -------------------- Attributes -------------------- //

        // OpenGL texture ID
        uint mID;

        // Layer of the texture
        uint mLayer;

        // Width
        int mWidth;

        // Height
        int mHeight;




    public:

        // -------------------- Methods -------------------- //

        // Constructor
        ITexture2D(uint _ID = 0);

        // Constructor
        ITexture2D(int width, int height, int internalFormat, int format, int type);

        // Destructor
        ~ITexture2D();

        // Create the texture
        void create(int width, int height, int internalFormat, int format, int type, void* data = nullptr);

        // Create Depth the texture
        void createDepth( int width, int height );


        // Destroy the texture
        void destroy();

        // Bind the texture
        void bind() const;

        // Unbind the texture
        void unbind() const;

        // Get the OpenGL texture ID
        int getID() const;

        // Get the layer of the texture
        int getLayer() const;

        // Set the layer of the texture
        void setLayer(int layer);

        // Get the width
        int getWidth() const;

        // Get the height
        int getHeight() const;


        operator uint();

        void setID(const uint &iD);

};



// Get the OpenGL texture ID
inline int ITexture2D::getID() const {
    return mID;
}

// Get the layer of the texture
inline int ITexture2D::getLayer() const {
    return mLayer;
}

// Set the layer of the texture
inline void ITexture2D::setLayer(int layer) {
    mLayer = layer;
}

// Get the width
inline int ITexture2D::getWidth() const {
    return mWidth;
}



inline ITexture2D::operator uint()
{
    return mID;
}

// Get the height
inline int ITexture2D::getHeight() const {
    return mHeight;
}



}

#endif // ITEXTURE2D_H
