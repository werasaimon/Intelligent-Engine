#ifndef SCENE_H
#define SCENE_H



class UnitKey
{
  protected:

    bool mKeys[256];
    int  mMouseButton;

  public:

    void specialKeyboardDown(int key)
    {
         mKeys[key] = true;
    }

    void specialKeyboardUp(int key)
    {
         mKeys[key] = false;
    }

    void NullAllKey()
    {
        for (int i = 0; i < 256; ++i) mKeys[i] = false;
    }

    bool keyDown( int key )
    {
        return mKeys[key];
    }

};



class Scene : public  UnitKey
{
public:
    Scene();

   // virtual ~Scene();

    virtual bool initialization() = 0;
    virtual void render(float FrameTime) = 0;
    virtual void update() = 0;
    virtual void resize( float width , float height ) = 0;

    virtual void mouseMove( float x , float y  , int button) = 0;
    virtual void mousePress( float x , float y , int button ) = 0;
    virtual void mouseReleasePress( float x , float y , int button ) = 0;
    virtual void mouseWheel( float delta ) = 0;

    virtual void keyboard(int key ) = 0;
    virtual void destroy() = 0;


};




#endif // SCENE_H
