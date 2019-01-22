#ifndef IUTILITYOPENGLDRAW_HPP
#define IUTILITYOPENGLDRAW_HPP

#include <GL/gl.h>
#include <GL/glu.h>

#include "IEngineComponent/IComponentLib.h"

namespace IUtilityOpenGLDraw
{

    void DrawCube()
    {
        glBegin(GL_LINE_LOOP);
        // top
        glColor3f(1.0f, 0.0f, 0.0f);
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // front
        glColor3f(0.0f, 1.0f, 0.0f);
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // right
        glColor3f(0.0f, 0.0f, 1.0f);
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // left
        glColor3f(0.0f, 0.0f, 0.5f);
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // bottom
        glColor3f(0.5f, 0.0f, 0.0f);
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);

        glEnd();

        glBegin(GL_LINE_LOOP);
        // back
        glColor3f(0.0f, 0.5f, 0.0f);
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);

        glEnd();
    }




    void DrawGrid()
    {
        int size = 10;
        int step = 1;



        glPushMatrix();
        glColor3f(1,1,1);
        glBegin(GL_LINES);
        for(int i=-size;i<=size;i += step)
        {
            glVertex3f(i, 0.f, -size); // Vertex 2
            glVertex3f(i, 0.f,  size); // Vertex 1
            glVertex3f(-size, 0.f, i); // Vertex 4
            glVertex3f( size, 0.f, i); // Vertex 3
        }
        glEnd();
        glPopMatrix();
    }


    void DrawComponentMeshTriangle( const IEngineComponent::IComponentMesh3* ComponentMesh )
    {

        glPushMatrix();
        //glMultMatrixf(ComponentMesh->GetTransformHierarchy());
        glMultMatrixf(ComponentMesh->GetMesh()->getTransformMatrix());
        for (IEngine::Index3i t : ComponentMesh->GetMesh()->Triangles())
        {
            glBegin(GL_TRIANGLES);
            glColor3f(0.4,0.4,0.4);
            glVertex3fv(ComponentMesh->GetMesh()->GetVertex(t[0]));
            glVertex3fv(ComponentMesh->GetMesh()->GetVertex(t[1]));
            glVertex3fv(ComponentMesh->GetMesh()->GetVertex(t[2]));
            glEnd();
        }
        glPopMatrix();


        IEngine::IVector3 HalfSize =  ComponentMesh->GetAxisAlignedBoxTransform().HalfSize();
        IEngine::IVector3 center   =  ComponentMesh->GetAxisAlignedBoxTransform().Center();
        glPushMatrix();
        glTranslatef(center.x,center.y,center.z);
        glScalef(HalfSize.x,HalfSize.y,HalfSize.z);
        DrawCube();
        glPopMatrix();
    }

    void DrawComponentMeshLine( const IEngineComponent::IComponentMesh3* ComponentMesh )
    {

       glPushMatrix();
       //glMultMatrixf(ComponentMesh->GetTransformHierarchy());
       glMultMatrixf(ComponentMesh->GetMesh()->getTransformMatrix());
       glLineWidth(3);
       for (IEngine::Index4i t : ComponentMesh->GetMesh()->Edges())
       {

           glBegin(GL_LINES);
           glColor3f(1,1,1);
           glVertex3fv(ComponentMesh->GetMesh()->GetVertex(t[0]));
           glVertex3fv(ComponentMesh->GetMesh()->GetVertex(t[1]));
           glEnd();
       }
       glLineWidth(1);
       glPopMatrix();


       IEngine::IVector3 HalfSize =  ComponentMesh->GetAxisAlignedBoxTransform().HalfSize();
       IEngine::IVector3 center   =  ComponentMesh->GetAxisAlignedBoxTransform().Center();
       glPushMatrix();
       glTranslatef(center.x,center.y,center.z);
       glScalef(HalfSize.x,HalfSize.y,HalfSize.z);
       DrawCube();
       glPopMatrix();
    }




}


#endif // IUTILITYOPENGLDRAW_HPP
