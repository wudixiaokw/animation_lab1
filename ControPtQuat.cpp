

#include "ControPtQuat.h"

void ControPtQuat::setCoord(GLfloat xAxis, GLfloat yAxis, GLfloat zAxis){
    x = xAxis;
    y = yAxis;
    z = zAxis;
}

void ControPtQuat::setOrien(GLfloat angle, GLfloat xAxis, GLfloat yAxis, GLfloat zAxis){
    this->angle = angle;
    xOri = xAxis;
    yOri = yAxis;
    zOri = zAxis;
}