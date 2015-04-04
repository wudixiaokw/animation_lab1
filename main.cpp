
#include "main.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GLUT/GLUT.h>
#include <math.h>

// two representations of controlpoint
#include "ControPtQuat.h"
#include "ControPtEuler.h"

//================================
// global variables
//================================
// screen size
int g_screenWidth  = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;

// angle for rotation
int g_angle = 0;

//one degree
const float DEGREE = 3.1415926 / 180 ;

//set control points with quaternion representation
ControPtQuat conPtQuat[9] = {
    {-3.2,-0.5, -5.0, 45 * DEGREE, 0.0, 0.0, -1.0},
    {-3.0, 0.0, -5.0, 30 * DEGREE, 0.0, 0.0, -1.0}, {-2.0, 1.0, -5.0, 0, 0.0, 0.0, -1.0},
    {-1.0, 0.5, -5.0, -45 * DEGREE, 0.0, 0.0, -1.0}, {0.0, -0.8, -5.0, 20 * DEGREE, 0.0, 0.0, -1.0},
    {1.2, 0.0, -5.0, 45 * DEGREE, 0.0, 0.0, -1.0}, {2.2, 1.2, -5.0, 0, 0.0, 0.0, -1.0},
    {3.0, 0.0, -5.0, 45*DEGREE, 0.0, 0.0, -1.0}, {3.2, 1.0, -5.0, 10*DEGREE, 0.0, 0.0, -1.0}
};

//set control points with Euler representation
ControPtEuler conPtEul[9] = {
    {-3.2,-0.5, -5.0, 0.0, 0.0, 45 * DEGREE},
    {-3.0, 0.0, -5.0, 0.0, 0.0, 30 * DEGREE}, {-2.0, 1.0, -5.0, 0.0, 0.0, 0.0},
    {-1.0, 0.5, -5.0, 0.0, 0.0, -45 * DEGREE}, {0.0, -0.8, -5.0, 0.0, 0.0, 20 * DEGREE},
    {1.2, 0.0, -5.0, 0.0, 0.0, 45 * DEGREE}, {2.2, 1.2, -5.0, 0.0, 0.0, 0.0},
    {3.0, 0.0, -5.0, 0.0, 0.0, -45 * DEGREE}, {3.2, 1.0, -5.0, 0.0, 0.0, 10 * DEGREE}
};

//set frames between adjecent points
int frameNum = 60;


//the number of control points
int conNum = 9;



//4X4 matrix
typedef GLfloat Matrix4x4 [4][4];

//ultimate transform matrix
Matrix4x4 ultTranMat;

//================================
// init
//================================
void init( void ) {
	// init something before main loop...
    //    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
}


//================================
// identify matrix
//================================
void matIdent(Matrix4x4 tempMat){
    for (int row=0; row<4; row++) {
        for (int col=0; col<4; col++) {
            tempMat[row][col] = (row == col);
        }
    }
}

//================================
// matrix multiplication
//================================
void matMult(Matrix4x4 matA, Matrix4x4 matB){
    Matrix4x4 matTemp;
    for(int row=0; row<4; row++)
        for (int col=0; col<4; col++) {
            matTemp[row][col] = matA[row][0] * matB[0][col] + matA[row][1] * matB[1][col]
            + matA[row][2] * matB[2][col] + matA[row][3] * matB[3][col];
        }
    
    for(int row=0; row<4; row++)
        for (int col=0; col<4; col++){
            matB[row][col] = matTemp[row][col];
        }
}

//================================
// quarternion transform matrix
//================================
void quatMatTrans(ControPtQuat conPt){
    matIdent(ultTranMat);
    ultTranMat[0][0] = 1 - 2 * conPt.yOri * conPt.yOri - 2 * conPt.zOri * conPt.zOri;
    ultTranMat[0][1] = 2 * conPt.xOri * conPt.yOri - 2 * conPt.angle * conPt.zOri;
    ultTranMat[0][2] = 2 * conPt.xOri * conPt.zOri + 2 * conPt.angle * conPt.yOri;
    ultTranMat[0][3] = conPt.x;
    ultTranMat[1][0] = 2 * conPt.xOri * conPt.yOri + 2 * conPt.angle * conPt.zOri;
    ultTranMat[1][1] = 1 - 2 * conPt.xOri * conPt.xOri - 2 * conPt.zOri * conPt.zOri;
    ultTranMat[1][2] = 2 * conPt.yOri * conPt.zOri - 2 * conPt.angle * conPt.xOri;
    ultTranMat[1][3] = conPt.y;
    ultTranMat[2][0] = 2 * conPt.xOri * conPt.zOri - 2 * conPt.angle * conPt.yOri;
    ultTranMat[2][1] = 2 * conPt.yOri * conPt.zOri + 2 * conPt.angle * conPt.xOri;
    ultTranMat[2][2] = 1 - 2 * conPt.xOri * conPt.xOri - 2 * conPt.yOri * conPt.yOri;
    ultTranMat[2][3] = conPt.z;
    
}

//================================
// Euler transform matrix
//================================
void eulMatTrans(ControPtEuler conPt){
    Matrix4x4 temMat;
    matIdent(ultTranMat);
    matIdent(temMat);
    
    //roatation about Z-axis
    temMat[0][0] = cosf(conPt.zOri);
    temMat[0][1] = -sinf(conPt.zOri);
    temMat[1][0] = sinf(conPt.zOri);
    temMat[1][1] = cosf(conPt.zOri);
    matMult(temMat, ultTranMat);
    
    //roatation about y-axis
    matIdent(temMat);
    temMat[0][0] = cosf(conPt.yOri);
    temMat[0][1] = -sinf(conPt.yOri);
    temMat[1][0] = sinf(conPt.yOri);
    temMat[1][1] = cosf(conPt.yOri);
    matMult(temMat, ultTranMat);
    
    //roatation about x-axis
    matIdent(temMat);
    temMat[0][0] = cosf(conPt.xOri);
    temMat[0][1] = -sinf(conPt.xOri);
    temMat[1][0] = sinf(conPt.xOri);
    temMat[1][1] = cosf(conPt.xOri);
    matMult(temMat, ultTranMat);
    
    //translate
    matIdent(temMat);
    temMat[0][3] = conPt.x;
    temMat[1][3] = conPt.y;
    temMat[2][3] = conPt.z;
    matMult(temMat, ultTranMat);
    
}

//================================
// matrix transpose
//================================
void matT(void){
    Matrix4x4 tempMat;
    for (int row=0; row<4; row++) {
        for (int col=0; col<4; col++) {
            tempMat[col][row] = ultTranMat[row][col];
        }
    }
    
    for (int row=0; row<4; row++) {
        for (int col=0; col<4; col++) {
            ultTranMat[row][col] = tempMat[row][col];
        }
    }
}

//================================
// update
//================================
void update( void ) {
	// do something before rendering...
    //set the interpolating value
    GLfloat t;
    
    //t power 2
    GLfloat douT;
    
    //t power 3
    GLfloat triT;
    
    //the x,y,z axis coordinates of interpolating point
    GLfloat interX, interY, interZ;
    
    //the orientation of interpolating point
    GLfloat interAngel, interOriX, interOriY, interOriZ;
    
    //sum = w^2 + x^2 + y^2 + z^2
    GLfloat sum;
    
    //selected control point's order num
    int selPT;
    
    //the frameindex during the second curve
    int secFrameIndex;
    
    //tempary points between control points
    ControPtQuat temPTQuar;
    ControPtEuler temPTEul;
    
    
    //the phase between second control point and penultimate control point
    if (g_frameIndex< frameNum * (conNum-3)) { //Quarternion & Catmul-Rom
        
        //figure out the current frame is between which two points
        selPT = g_frameIndex / frameNum;
        
        //compute the interpolating values
        t = (GLfloat)(g_frameIndex - frameNum * selPT) / (GLfloat)frameNum;
        
        // t squared and t to the power of 3
        douT = t*t;
        triT = douT * t;
        
        // computer interpolation of every single attribute
        interX = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT].x
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT +1 ].x
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].x
        + (triT - douT)* conPtQuat[selPT+3].x;
        
        interX = interX/2;
        
        interY = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT ].y
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].y
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].y
        + (triT - douT)* conPtQuat[selPT+3].y;
        interY = interY/2;
        
        interZ = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT].z
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].z
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].z
        + (triT - douT)* conPtQuat[selPT+3].z;
        interZ = interZ/2;
        
        interAngel =(triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT].angle
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].angle
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].angle
        + (triT - douT)* conPtQuat[selPT+3].angle;
        interAngel = interAngel/2;
        
        interOriX = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT].xOri
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].xOri
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].xOri
        + (triT - douT)* conPtQuat[selPT+3].xOri;
        interOriX = interOriX/2;
        
        interOriY = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT ].yOri
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].yOri
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].yOri
        + (triT - douT)* conPtQuat[selPT+3].yOri;
        interOriY = interOriY/2;
        
        interOriZ = (triT * (-1) + douT * 2 + t * (-1))* conPtQuat[selPT].zOri
        + (triT * 3 + douT * (-5) + 2) * conPtQuat[selPT+1].zOri
        + (triT *(-3) + douT * 4 + t) * conPtQuat[selPT + 2].zOri
        + (triT - douT)* conPtQuat[selPT+3].zOri;
        interOriZ = interOriZ/2;
        
        sum = interAngel * interAngel + interOriX * interOriX
        + interOriY * interOriY + interOriZ * interOriZ;
        
        interAngel = interAngel /(sqrtf(sum));
        interOriX = interOriX /(sqrtf(sum));
        interOriY = interOriY/(sqrtf(sum));
        interOriZ = interOriZ / (sqrtf(sum));
        
        //interpolate point
        temPTQuar.setCoord(interX, interY, interZ);
        temPTQuar.setOrien(interAngel,interOriX, interOriY, interOriZ);
        
        //transformation matrix
        quatMatTrans(temPTQuar);
        
    }else if(g_frameIndex < 2*(frameNum * (conNum-3))){ //second curve Quarternion & B-Spline
        
        //compute current phase's framindex
        secFrameIndex = g_frameIndex - frameNum * (conNum - 3);
        
        //figure out the current frame is between which two points
        selPT = secFrameIndex / frameNum;
        
        //compute the interpolating values
        t = (GLfloat)(secFrameIndex - frameNum * selPT) / (GLfloat)frameNum;
        
        douT = t*t;
        triT = douT * t;
        
        interX = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].x
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].x
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].x
        + triT * conPtQuat[selPT+3].x;
        
        interX = interX/6;
        
        interY = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].y
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].y
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].y
        + triT * conPtQuat[selPT+3].y;
        interY = interY/6;
        
        interZ = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].z
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].z
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].z
        + triT * conPtQuat[selPT+3].z;
        interZ = interZ/6;
        
        interAngel =(triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].angle
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].angle
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].angle
        + triT * conPtQuat[selPT+3].angle;
        interAngel = interAngel/6;
        
        interOriX = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].xOri
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].xOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].xOri
        + triT * conPtQuat[selPT+3].xOri;
        interOriX = interOriX/6;
        
        interOriY = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].yOri
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].yOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].yOri
        + triT * conPtQuat[selPT+3].yOri;
        interOriY = interOriY/6;
        
        interOriZ = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtQuat[selPT].zOri
        + (triT * 3 + douT * (-6) + 4) * conPtQuat[selPT +1 ].zOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtQuat[selPT + 2].zOri
        + triT * conPtQuat[selPT+3].zOri;
        interOriZ = interOriZ/6;
        
        sum = interAngel * interAngel + interOriX * interOriX
        + interOriY * interOriY + interOriZ * interOriZ;
        
        interAngel = interAngel /(sqrtf(sum));
        interOriX = interOriX /(sqrtf(sum));
        interOriY = interOriY/(sqrtf(sum));
        interOriZ = interOriZ / (sqrtf(sum));
        
        temPTQuar.setCoord(interX, interY, interZ);
        temPTQuar.setOrien(interAngel,interOriX, interOriY, interOriZ);
        
        quatMatTrans(temPTQuar);
        
    }else if(g_frameIndex < 3*(frameNum * (conNum-3))){    //third curve Euler Angles & Catmul-Rom
        //compute current phase's framindex
        secFrameIndex = g_frameIndex - 2 * frameNum * (conNum - 3);
        
        //figure out the current frame is between which two points
        selPT = secFrameIndex / frameNum;
        
        //compute the interpolating values
        t = (GLfloat)(secFrameIndex - frameNum * selPT) / (GLfloat)frameNum;
        
        douT = t*t;
        triT = douT * t;
        
        interX = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT].x
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT +1 ].x
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].x
        + (triT - douT)* conPtEul[selPT+3].x;
        
        interX = interX/2;
        
        interY = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT ].y
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT+1].y
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].y
        + (triT - douT)* conPtEul[selPT+3].y;
        interY = interY/2;
        
        interZ = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT].z
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT+1].z
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].z
        + (triT - douT)* conPtEul[selPT+3].z;
        interZ = interZ/2;
        
        interOriX = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT].xOri
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT+1].xOri
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].xOri
        + (triT - douT)* conPtEul[selPT+3].xOri;
        interOriX = interOriX/2;
        
        interOriY = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT ].yOri
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT+1].yOri
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].yOri
        + (triT - douT)* conPtEul[selPT+3].yOri;
        interOriY = interOriY/2;
        
        interOriZ = (triT * (-1) + douT * 2 + t * (-1))* conPtEul[selPT].zOri
        + (triT * 3 + douT * (-5) + 2) * conPtEul[selPT+1].zOri
        + (triT *(-3) + douT * 4 + t) * conPtEul[selPT + 2].zOri
        + (triT - douT)* conPtEul[selPT+3].zOri;
        interOriZ = interOriZ/2;
        
        
        temPTEul.setCood(interX, interY, interZ);
        temPTEul.setOri(interOriX, interOriY, interOriZ);
        
        eulMatTrans(temPTEul);
        
        
    }else if(g_frameIndex < 4*(frameNum * (conNum-3))){  //Fouth curve Euler angle & B-spline
        secFrameIndex = g_frameIndex - 3 * frameNum * (conNum - 3);
        
        //figure out the current frame is between which two points
        selPT = secFrameIndex / frameNum;
        
        //compute the interpolating values
        t = (GLfloat)(secFrameIndex - frameNum * selPT) / (GLfloat)frameNum;
        
        douT = t*t;
        triT = douT * t;
        
        interX = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].x
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].x
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].x
        + triT * conPtEul[selPT+3].x;
        
        interX = interX/6;
        
        interY = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].y
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].y
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].y
        + triT * conPtEul[selPT+3].y;
        interY = interY/6;
        
        interZ = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].z
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].z
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].z
        + triT * conPtEul[selPT+3].z;
        interZ = interZ/6;
        
        interOriX = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].xOri
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].xOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].xOri
        + triT * conPtEul[selPT+3].xOri;
        interOriX = interOriX/6;
        
        interOriY = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].yOri
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].yOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].yOri
        + triT * conPtEul[selPT+3].yOri;
        interOriY = interOriY/6;
        
        interOriZ = (triT * (-1) + douT * 3 + t * (-3) + 1)* conPtEul[selPT].zOri
        + (triT * 3 + douT * (-6) + 4) * conPtEul[selPT +1 ].zOri
        + (triT *(-3) + douT * 3 + t * 3 + 1) * conPtEul[selPT + 2].zOri
        + triT * conPtEul[selPT+3].zOri;
        interOriZ = interOriZ/6;
        
        temPTEul.setCood(interX, interY, interZ);
        temPTEul.setOri(interOriX, interOriY, interOriZ);
        
        eulMatTrans(temPTEul);
        
        
    }else{ //start over again
        g_frameIndex =0;
    }
    
    
    //transpose the ultimate transfomation matrix
    matT();
    
    
}

//================================
// render
//================================
void render( void ) {
	// clear buffer
	glClearColor (0.0, 0.0, 0.0, 1.0);
	glClearDepth (1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
    
	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
    
    /*
     // light source attributes
     GLfloat LightAmbient[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
     GLfloat LightDiffuse[]	= { 0.3f, 0.3f, 0.3f, 1.0f };
     GLfloat LightSpecular[]	= { 0.4f, 0.4f, 0.4f, 1.0f };
     GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };
     
     glLightfv(GL_LIGHT0, GL_AMBIENT , LightAmbient );
     glLightfv(GL_LIGHT0, GL_DIFFUSE , LightDiffuse );
     glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
     glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
     */
    
	// surface material attributes
	GLfloat material_Ka[]	= { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[]	= { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[]	= { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[]	= { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se		= 10;
    
	glMaterialfv(GL_FRONT, GL_AMBIENT	, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE	, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR	, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION	, material_Ke);
	glMaterialf (GL_FRONT, GL_SHININESS	, material_Se);
    
	// modelview matrix
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
    glMultMatrixf(*ultTranMat);
    
	// render objects
	glutSolidTeapot(0.2);
    
    // control points
    glLoadIdentity();
    glColor3f(1.0, 0.0, 0.0);
    glPointSize(5);
    glBegin(GL_POINTS);
    for (int i=1; i<conNum-1; i++) {
        glVertex3f(conPtQuat[i].x, conPtQuat[i].y, conPtQuat[i].z);
    }
    glEnd();
    
	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
    
	// swap back and front buffers
	glutSwapBuffers();
    glFlush();
}

//================================
// keyboard input
//================================
void keyboard( unsigned char key, int x, int y ) {
    
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape( int w, int h ) {
	// screen size
	g_screenWidth  = w;
	g_screenHeight = h;
	
	// viewport
	glViewport( 0, 0, (GLsizei)w, (GLsizei)h );
    
	// projection matrix
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w/(GLfloat)h, 1.0, 2000.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer( int value ) {
	// increase frame index
	g_frameIndex++;
    
	update();
	
	// render
	glutPostRedisplay();
    
	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc( 16, timer, 0 );
}

//================================
// main
//================================
int main( int argc, char** argv ) {
	// create opengL window
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB |GLUT_DEPTH );
	glutInitWindowSize( 1200, 800 );
	glutInitWindowPosition( 100, 100 );
	glutCreateWindow( argv[0] );
    
	// init
	init();
	
	// set callback functions
	glutDisplayFunc( render );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyboard );
	glutTimerFunc( 16, timer, 0 );
	
	// main loop
	glutMainLoop();
    
	return 0;
}