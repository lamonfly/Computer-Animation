#include "skeleton.h"
#include "splitstring.h"
#include <cmath>

/*
 * Load skeleton file
 */
void Skeleton::loadSkeleton(std::string skelFileName)
{
    std::string strBone;
    std::ifstream skelFile(skelFileName.c_str());
    if (skelFile.is_open())
    {
        while ( std::getline(skelFile, strBone)) { //Read a line to build a bone
            std::vector<std::string> boneParams;
            splitstring splitStr(strBone);
            boneParams = splitStr.split(' ');
            Joint temp;
            temp.position.x = std::atof(boneParams[1].c_str());
            temp.position.y = std::atof(boneParams[2].c_str());
            temp.position.z = std::atof(boneParams[3].c_str());
            temp.parentID = std::atoi(boneParams[4].c_str());
            if (std::atoi(boneParams[0].c_str()) != joints.size())
            {
                std::cout<<"[Warning!!!] Bone index not match\n";
            }
            joints.push_back(temp);
        }
    }
}

/*
 * Load Animation
 */
void Skeleton::loadAnimation(std::string skelFileName)
{
}


/*
 * Draw skeleton with OpenGL
 */
void Skeleton::glDrawSkeleton()
{
    //Rigging skeleton
    glDisable(GL_DEPTH_TEST);
    
    glPushMatrix();
    glTranslatef(-0.9,-0.9,-0.9);
	glScalef(1.8,1.8,1.8);
	glPointSize(6);
	glColor3f(1,0,0);
    updateScreenCoord();
    
    for (unsigned i=0; i<joints.size(); i++)
    {
        glPushMatrix();
        if (joints[i].isPicked)
            glColor3f(1.0, 0.0, 0.0);
        else if (joints[i].isHovered)
            glColor3f(0.7, 0.7, 0.7);
        else
            glColor3f(0.3, 0.3, 0.3);

        glMultMatrixf(joints[i].global_t);
        
        glTranslated(joints[i].position.x, joints[i].position.y, joints[i].position.z);
        glutSolidSphere(0.01, 15, 15);
        glTranslated(-joints[i].position.x, -joints[i].position.y, -joints[i].position.z);
        
        if (joints[i].parentID !=-1)
        {
            glColor3f(0.7, 0.3, 0.3);
            glBegin(GL_LINES);
                glVertex3d(joints[i].position.x, joints[i].position.y, joints[i].position.z);
                glVertex3d(joints[joints[i].parentID].position.x,
                        joints[joints[i].parentID].position.y,
                        joints[joints[i].parentID].position.z);
            glEnd();
        }
        glPopMatrix();
    }
    glPopMatrix();
    
    glEnable(GL_DEPTH_TEST);
}

void Skeleton::updateScreenCoord()
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLdouble winX, winY, winZ;

    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    for (unsigned i=0; i<joints.size(); i++)
    {
        double mm[16];
        mult(modelview, joints[i].global_t, mm);
        
        gluProject((GLdouble)joints[i].position.x, (GLdouble)joints[i].position.y, (GLdouble)joints[i].position.z,
                mm, projection, viewport,
                &winX, &winY, &winZ );
        joints[i].screenCoord.x = winX;
        joints[i].screenCoord.y = (double)glutGet(GLUT_WINDOW_HEIGHT)-winY;
    }
}
void Skeleton::checkHoveringStatus(int x, int y)
{
    double distance = 0.0f;
    double minDistance = 1000.0f;
    int hoveredJoint = -1;
    for(unsigned i=0; i < joints.size(); i++)
    {
        joints[i].isHovered = false;
        distance = sqrt((x - joints[i].screenCoord.x)*(x - joints[i].screenCoord.x) 
                + (y - joints[i].screenCoord.y)*(y - joints[i].screenCoord.y));
        if (distance > 50) continue;
        if (distance < minDistance)
        {
            hoveredJoint = i;
            minDistance = distance;
        }
    }
    if (hoveredJoint != -1) joints[hoveredJoint].isHovered = true;
}

void Skeleton::release()
{
    hasJointSelected = false;
    for (unsigned i=0; i<joints.size(); i++)
    {
        joints[i].isPicked = false;
    }
}

void Skeleton::addRotation(float* q)
{
    for(int i=0; i<joints.size(); ++i)
    {
        if(joints[i].isPicked)
        {
			mult(q, joints[i].local_t, joints[i].local_t);
            break;
        }
    }
    
}

void Skeleton::updateGlobal()
{
    for(int i=0; i<joints.size(); ++i)
    {
        if(joints[i].parentID!=-1)
        {
            float g[16];
            int p=joints[i].parentID;
            
            float p_pos[3];
            p_pos[0]=joints[p].position.x;
            p_pos[1]=joints[p].position.y;
            p_pos[2]=joints[p].position.z;
            
            
            float tr[16];
            
            trans(joints[p].global_t, p_pos, g);
            
            mult(g, joints[i].local_t, g);
            
            p_pos[0]=-p_pos[0];
            p_pos[1]=-p_pos[1];
            p_pos[2]=-p_pos[2];
            
            trans(g, p_pos, joints[i].global_t);
        }
    }
}

void Skeleton::selectOrReleaseJoint()
{
    bool hasHovered=false;
    for (unsigned i=0; i<joints.size(); i++)
    {
        joints[i].isPicked = false;
        if (joints[i].isHovered)
        {
            hasHovered = true;
            joints[i].isPicked = true;
            hasJointSelected = true;
        }
    }
    if (!hasHovered)    //Release joint
    if (!hasHovered)    //Release joint
        hasJointSelected = false;
}

void Skeleton::add_keyframe() {
	std::vector<quaternion> s;
	for (unsigned i = 0; i < joints.size(); i++)
	{
		s.push_back(joints[i].quat);
	}
	key_frame.push_back(s);
}

