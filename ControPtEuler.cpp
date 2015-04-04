

#include "ControPtEuler.h"

void ControPtEuler::setCood(GLfloat xAxis, GLfloat yAxis, GLfloat zAxis){
    x = xAxis;
    y = yAxis;
    z = zAxis;
}

void ControPtEuler::setOri(GLfloat xDegree, GLfloat yDegree, GLfloat zDegree){
    xOri = xDegree;
    yOri = yDegree;
    zOri = zDegree;
}