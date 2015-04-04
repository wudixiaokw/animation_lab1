

#ifndef __lab0__ControPt__
#define __lab0__ControPt__

#include <iostream>
#include<GLUT/GLUT.h>


class ControPtQuat{
public:
    GLfloat x ,y ,z ,angle, xOri , yOri , zOri;

public:
    void setCoord(GLfloat xAxis, GLfloat yAxis, GLfloat zAxis);
    void setOrien(GLfloat angle, GLfloat xAxis, GLfloat yAxis, GLfloat zAxis);
    
};

#endif /* defined(__lab0__ControPt__) */
