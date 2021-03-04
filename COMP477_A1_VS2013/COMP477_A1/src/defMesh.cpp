#include "defMesh.h"
#include <fstream>
#include <iostream>

DefMesh::DefMesh()
{
    pmodel = NULL;
    if (!pmodel) {	/* load up the model */

    char meshFile[] = "model/cheb.obj";
    pmodel = glmReadOBJ(meshFile);
    if (!pmodel) {
        return;
    }
        //glmUnitize(pmodel);
        glmFacetNormals(pmodel);
        glmVertexNormals(pmodel, 0);
        glmFacetNormals(pmodel);
        pcopy=new float[3*(pmodel->numvertices+1)];
        ncopy=new float[3*(pmodel->numnormals+1)];
        memcpy(pcopy, pmodel->vertices, sizeof(float)*3*(pmodel->numvertices+1));
        memcpy(ncopy, pmodel->normals, sizeof(float)*3*(pmodel->numnormals+1));
        
        for(int i=0; i<pmodel->numvertices; ++i)
            remember.push_back(new float[16]);
    }
    mySkeleton.loadSkeleton("./model/skeleton.out");
    loadWeights();
}

void DefMesh::updateVertices()
{
    for(int i=1; i<=pmodel->numvertices; ++i)
    {
        float m[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        for(int j=1; j<mySkeleton.joints.size(); ++j)
        {
            float t[16];
            scalar(weights[(i-1)*17+j-1], mySkeleton.joints[j].global_t, t);
            add(m, t, m);
        }
        float v[]={pcopy[i*3], pcopy[i*3+1], pcopy[i*3+2]};
        multv(m, v, v);
        
        pmodel->vertices[i*3]=v[0];pmodel->vertices[i*3+1]=v[1];pmodel->vertices[i*3+2]=v[2];
        
        memcpy(remember[i-1], m, sizeof(float)*16);
        
        //pmodel->normals[i*3]=n[0];pmodel->normals[i*3+1]=n[1];pmodel->normals[i*3+2]=n[2];
    }
    
    for(int i=0; i<pmodel->numtriangles; ++i)
    {
        GLMtriangle* triangle = &(pmodel->triangles[i]);
        
        for(int j=0; j<3; ++j)
        {
            int nin=triangle->nindices[j];
            int vin=triangle->vindices[j]-1;
            
            float n[]={ncopy[nin*3], ncopy[nin*3+1], ncopy[nin*3+2]};
            multv(remember[vin],n ,n, 0.0);
            
            pmodel->normals[nin*3]=n[0];pmodel->normals[nin*3+1]=n[1];pmodel->normals[nin*3+2]=n[2];
        }
    }
    
}

void DefMesh::loadWeights()
{
    std::string file("model/weights.out");
    
    std::ifstream reader(file);
    
    float f;
    reader >> f;
    
    while(!reader.eof())
    {
        weights.push_back(f);
        reader >> f;
    }
    reader.close();
    
    for(int i=0; i<pmodel->numvertices; ++i)
    {
        float sum=0;
        for(int j=0; j<17; ++j)
            sum+=weights[i*17+j];
        for(int j=0; j<17; ++j)
            weights[i*17+j]/=sum;
    }
}

void DefMesh::glDraw(int type)
{
    
    switch(type){
    case 0:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); break;
    case 1:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); break;
    case 2:
        mySkeleton.glDrawSkeleton(); return;
    
    }
    glColor3f(0.5, 0.5, 0.5);
    mode = GLM_NONE;
    mode = mode | GLM_SMOOTH;
    
    glPushMatrix();
    glScalef(2,2,2);
    glTranslatef(-0.5, -0.5, -0.5);
    glmDraw(pmodel, mode);
    glPopMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    mySkeleton.glDrawSkeleton();
}

void DefMesh::loadKeyFrame(int n) {
	std::vector<quaternion> frame = mySkeleton.key_frame.at(n);

	for (unsigned i = 0; i < frame.size(); i++) {
		mySkeleton.joints[i].quat = frame.at(i);
		float* newMatrix = frame.at(i).matrix();

		for (unsigned j = 0; j < 16; j++) {
			mySkeleton.joints[i].local_t[j] = newMatrix[j];
		}
	}

	mySkeleton.updateGlobal();
	updateVertices();
}

void DefMesh::animate(int interpolation, int frame_index) {
	switch(interpolation)
	{
		case 1:
			matrixInterpolation(frame_index);
			break;
		case 2:
			eulerInterpolation(frame_index);
			break;
		case 3:
			lerpInterpolation(frame_index);
			break;
		case 4:
			slerpInterpolation(frame_index);
			break;
		default:
			break;
	}
}

void DefMesh::matrixInterpolation(int frame_index) {
	float num = frame_index * interval;
	int iF = floor(num);
	num = num - iF;

	//For every quat
	for (unsigned j = 0; j < mySkeleton.key_frame[iF].size(); j++) {
		//initial frame
		float* mI = mySkeleton.key_frame[iF][j].normalized().matrix();
		//final frame
		float* mF = mySkeleton.key_frame[iF + 1][j].normalized().matrix();
		//new frame
		float mN[16];

		//Compile new frame
		for (unsigned k = 0; k < 3; k++) {
			mN[k * 4] = (mF[k * 4] - mI[k * 4]) * num + mI[k * 4];
			mN[k * 4 + 1] = (mF[k * 4 + 1] - mI[k * 4 + 1]) * num + mI[k * 4 + 1];
			mN[k * 4 + 2] = (mF[k * 4 + 2] - mI[k * 4 + 2]) * num + mI[k * 4 + 2];
		}

		//Normalize Vector so it goes around
		float* temp_q = matrixToQuat(mN);
		quaternion qN = quaternion(Vec3(temp_q[0], temp_q[1], temp_q[2]), temp_q[3]);
		qN = qN.normalized();
		float* endM = qN.matrix();

		//Set new frame
		for (unsigned k = 0; k < 3; k++) {
			mySkeleton.joints[j].local_t[k * 4] = endM[k * 4];
			mySkeleton.joints[j].local_t[k * 4 + 1] = endM[k * 4 + 1];
			mySkeleton.joints[j].local_t[k * 4 + 2] = endM[k * 4 + 2];
		}
	}

	mySkeleton.updateGlobal();
	updateVertices();
}

void DefMesh::eulerInterpolation(int frame_index) {
	float num = frame_index * interval;
	int iF = floor(num);
	num = num - iF;

	//For every quat
	for (unsigned j = 0; j < mySkeleton.key_frame[iF].size(); j++) {
		//initial frame
		Vec3 vI = mySkeleton.key_frame[iF][j].normalized().euler();
		//final frame
		Vec3 vF = mySkeleton.key_frame[iF + 1][j].normalized().euler();
		//new frame
		Vec3 vN;

		//Compile new frame (bank, attitude, heading)
		vN.x = (vF.x - vI.x) * num + vI.x;
		vN.y = (vF.y - vI.y) * num + vI.y;
		vN.z = (vF.z - vI.z) * num + vI.z;
		float* mN = eulerToMatrix(vN);

		//Set new frame
		for (unsigned k = 0; k < 3; k++) {
			mySkeleton.joints[j].local_t[k * 4] = mN[k * 4];
			mySkeleton.joints[j].local_t[k * 4 + 1] = mN[k * 4 + 1];
			mySkeleton.joints[j].local_t[k * 4 + 2] = mN[k * 4 + 2];
		}
	}

	mySkeleton.updateGlobal();
	updateVertices();
}

void DefMesh::lerpInterpolation(int frame_index) {
	float num = frame_index * interval;
	int iF = floor(num);
	num = num - iF;

	for (unsigned j = 0; j < mySkeleton.key_frame[iF].size(); j++) {
		//initial frame
		quaternion qI = mySkeleton.key_frame[iF][j].normalized();
		//final frame
		quaternion qF = mySkeleton.key_frame[iF + 1][j].normalized();
		//new frame
		quaternion vN = (qF - qI) * num + qI;

		//Compile new frame
		float* mN = vN.normalized().matrix();

		//Set new frame
		for (unsigned k = 0; k < 3; k++) {
			mySkeleton.joints[j].local_t[k * 4] = mN[k * 4];
			mySkeleton.joints[j].local_t[k * 4 + 1] = mN[k * 4 + 1];
			mySkeleton.joints[j].local_t[k * 4 + 2] = mN[k * 4 + 2];
		}
	}

	mySkeleton.updateGlobal();
	updateVertices();
}

void DefMesh::slerpInterpolation(int frame_index) {
	float num = frame_index * interval;
	int iF = floor(num);
	num = num - iF;

	for (unsigned j = 0; j < mySkeleton.key_frame[iF].size(); j++) {
		//initial frame
		quaternion qI = mySkeleton.key_frame[iF][j].normalized();
		//final frame
		quaternion qF = mySkeleton.key_frame[iF + 1][j].normalized();
		//new frame
		quaternion vN;

		//Compile new frame
		double dot = qI.dot(qF);
		if (dot < 0.0f) {
			qF = quaternion(Vec3(-qF.values[0], -qF.values[1], -qF.values[2]), -qF.values[3]);
			dot = -dot;
		}

		if (dot > 0.9995) {
			vN = (qF - qI) * num + qI;
		}
		else {
			double t0 = acos(dot);
			double t = t0 * num;
			double st = sin(t);
			double st0 = sin(t0);

			double sI = cos(t) - dot * st / st0;
			double sF = st / st0;

			vN = (qI * sI) + (qF * sF);
		}

		float* mN = vN.normalized().matrix();

		//Set new frame
		for (unsigned k = 0; k < 3; k++) {
			mySkeleton.joints[j].local_t[k * 4] = mN[k * 4];
			mySkeleton.joints[j].local_t[k * 4 + 1] = mN[k * 4 + 1];
			mySkeleton.joints[j].local_t[k * 4 + 2] = mN[k * 4 + 2];
		}
	}

	mySkeleton.updateGlobal();
	updateVertices();
}

void DefMesh::loadAnimation(string animFile) {
	mySkeleton.key_frame.clear();
	string line;
	ifstream myFile(animFile.c_str());

	if (myFile.is_open()) {

		while (getline(myFile, line)) {
			vector<quaternion> temp_q;
			splitstring split(line);
			vector<string> s = split.split(' ');

			for (unsigned i = 0; i < 17; i++) {
				float w = std::atof(s[4 * i].c_str());
				float x = std::atof(s[4 * i + 1].c_str());
				float y = std::atof(s[4 * i + 2].c_str());
				float z = std::atof(s[4 * i + 3].c_str());

				quaternion q = quaternion(Vec3(x, y, z), w);
				temp_q.push_back(q);
			}

			mySkeleton.key_frame.push_back(temp_q);
		}
		myFile.close();
	}

	cout << "Loaded " << mySkeleton.key_frame.size() << " new frames" << endl;
}

void DefMesh::saveAnimation(string animFile) {
	ofstream myFile(animFile.c_str());

	if (myFile.is_open()) {
		for (unsigned i = 0; i < mySkeleton.key_frame.size(); i++) {
			//myFile << i << " ";

			for (unsigned j = 0; j < mySkeleton.key_frame[i].size(); j++) {
				myFile << mySkeleton.key_frame[i][j].values[3] << " ";
				myFile << mySkeleton.key_frame[i][j].values[0] << " ";
				myFile << mySkeleton.key_frame[i][j].values[1] << " ";
				myFile << mySkeleton.key_frame[i][j].values[2] << " ";
			}

			myFile << "\n";
		}

		myFile.close();
	}

	cout << "Saved " << mySkeleton.key_frame.size() << " frames" << endl;
}