#ifndef ISCENE_H
#define ISCENE_H

#include <Qt>
#include <set>



class IUtilityKeyboard
{
  private:

    int  mPressedKeys = 0;

  public:


    inline void SaveKeyPressed( int key )
    {
        mPressedKeys |= ( 1 << key );
       //mPressedKeys |= key;
    }

    inline void SaveKeyReleased( int key )
    {
        mPressedKeys &= ~( 1 << key );
        //mPressedKeys &= ~key;
    }

    inline bool KeyPressed( int key )
    {
        return (mPressedKeys & ( 1 << key ));
       //return (mPressedKeys & key)? true : false;
    }

};


class IUtilityMouse
{
    public:

        int mouseX;
        int mouseY;
        int mMouseButton;


        inline void SaveKeyPressed( int key )
        {
            mMouseButton |= ( 1 << key );
            //mPressedKeys |= key;
        }

        inline void SaveKeyReleased( int key )
        {
            mMouseButton &= ~( 1 << key );
            //mPressedKeys &= ~key;
        }

        inline bool KeyPressed( int key )
        {
            return (mMouseButton & ( 1 << key ));
            //return (mPressedKeys & key)? true : false;
        }
};



class ISceneCompare
{

protected:

    IUtilityKeyboard mUtilKeybard;
    IUtilityMouse mUtilMouse;


public:

    void SaveKeyPressed(int key) { mUtilKeybard.SaveKeyPressed(key); }
    void SaveKeyReleased(int key) { mUtilKeybard.SaveKeyReleased(key); }
    void SaveMouseKeyPressed(int key) { mUtilMouse.SaveKeyPressed(key); }
    void SaveMouseKeyReleased(int key) { mUtilMouse.SaveKeyReleased(key); }

    ISceneCompare()
    {

    }

    virtual ~ISceneCompare()
    {
    }

    virtual bool initialization() = 0;
    virtual void render(float FrameTime) = 0;
    virtual void update() = 0;
    virtual void resize( float width , float height ) = 0;

    virtual void mouseMove( float x , float y  , int button) = 0;
    virtual void mousePress( float x , float y , int button) = 0;
    virtual void mouseReleasePress( float x , float y , int button ) = 0;
    virtual void mouseWheel( float delta ) = 0;

    virtual void realaseKeyboard( int key ) = 0;
    virtual void keyboard(int key ) = 0;
    virtual void destroy() = 0;

    ///===========================================///

};

#endif // ISCENE_H
