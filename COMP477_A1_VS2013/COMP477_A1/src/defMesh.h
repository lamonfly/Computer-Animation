/*
 * Mesh with skeleton attached
 * You could put attachment here, or create an attachment class to manage weights
 */
#ifndef MESH_H
#define MESH_H

#include "./glm.h"
#include "./skeleton.h"
#include "splitstring.h"

class DefMesh
{
public:
    std::vector<float> weights;
    Skeleton mySkeleton;
    GLMmodel * pmodel;
    
    std::vector<float*> remember;

	float interval = 0.01f;
    
    float * pcopy;
    float * ncopy;
    GLuint mode;
    DefMesh(); 
    void glDraw(int type);
    void loadWeights();
    void updateVertices();
	void loadKeyFrame(int n);
	void animate(int interpolation, int frame_index);
	void loadAnimation(string animFile);
	void saveAnimation(string animFile);

private:
	void matrixInterpolation(int frame_index);
	void eulerInterpolation(int frame_index);
	void lerpInterpolation(int frame_index);
	void slerpInterpolation(int frame_index);
};
#endif
