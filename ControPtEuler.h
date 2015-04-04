

#ifndef __lab0__ControPtEuler__
#define __lab0__ControPtEuler__

#include <iostream>
#include <GLUT/GLUT.h>

class ControPtEuler{
public:
    GLfloat x, y ,z;
    GLfloat xOri, yOri, zOri;
    
public:
    void setCood(GLfloat xAxis, GLfloat yAxis, GLfloat zAxis);
    void setOri(GLfloat xDegree, GLfloat yDegree, GLfloat zDegree);
};

#endif /* defined(__lab0__ControPtEuler__) */
