#include "simpleMath.h"

double dot2(Vec2 a, Vec2 b)
{
    return a.x*b.x + a.y*b.y;
};

double dot3(Vec3 a, Vec3 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
};

double dot4(Vec4 a, Vec4 b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
};

void axisToMat(float* a, float ang, float* m)
{
    float t=1-cos(ang);
    float x=a[0];
    float y=a[1];
    float z=a[2];
    float c=cos(ang);
    float s=sin(ang);
    
    m[0]=t*x*x + c;
    m[1]=t*x*y + z*s;
    m[2]=t*x*z - y*s;
    m[3]=0.0;
    m[4]=t*x*y - z*s;
    m[5]=t*y*y + c;
    m[6]=t*y*z + x*s;
    m[7]=0.0;
    m[8]=t*x*z + y*s;
    m[9]=t*y*z - x*s;
    m[10]=t*z*z + c;
    m[11]=0.0;
    m[12]=m[13]=m[14]=0.0;
    m[15]=1.0;
}

void trans(float* m, float* v, float* r)
{
    float p[16];
    
    for(int i=0; i<16; ++i)
        p[i]=0.0;
    p[0]=p[5]=p[10]=p[15]=1.0;
    
    p[12]=v[0];
    p[13]=v[1];
    p[14]=v[2];
    
    mult(m, p, r);
}

void mult(float* m1, float* m2, float* r)
{
    float p[16];
    
    for(int i=0; i<4; ++i)
    {
        for(int j=0; j<4; ++j)
        {
            p[i*4+j]=0;
            for(int x=0; x<4; ++x)
                p[i*4+j]+=m1[j+4*x]*m2[i*4+x];
        }
    }
    
    memcpy(r, p, sizeof(float)*16);
}

void mult(double* m1, float* m2, double* r)
{
    double p[16];
    
    for(int i=0; i<4; ++i)
    {
        for(int j=0; j<4; ++j)
        {
            p[i*4+j]=0;
            for(int x=0; x<4; ++x)
                p[i*4+j]+=m1[j+4*x]*m2[i*4+x];
        }
    }
    
    memcpy(r, p, sizeof(double)*16);
}

void multv(float *m, float *v, float *r, float w)
{
    float t[3];
    
    t[0]=m[0]*v[0]+m[4]*v[1]+m[8]*v[2]+m[12]*w;
    t[1]=m[1]*v[0]+m[5]*v[1]+m[9]*v[2]+m[13]*w;
    t[2]=m[2]*v[0]+m[6]*v[1]+m[10]*v[2]+m[14]*w;
    
    memcpy(r, t, sizeof(float)*3);
}

void scalar(float s,float* m, float *r)
{
    for(int i=0; i<16; ++i)
        r[i]=m[i]*s;
}

void add(float* s,float* m, float *r)
{
    for(int i=0; i<16; ++i)
        r[i]=m[i]+s[i];
}

float* eulerToMatrix(Vec3 euler) {
	float sa = sin(euler.y);
	float ca = cos(euler.y);
	float sb = sin(euler.x);
	float cb = cos(euler.x);
	float sh = sin(euler.z);
	float ch = cos(euler.z);

	float p[16];

	p[0] = ch * ca;
	p[1] = sh * ca;
	p[2] = -sa;
	p[4] = -(-ch * sa * sb + sh * cb);
	p[5] = sh * sa * sb + ch * cb;
	p[6] = ca * sb;
	p[8] = ch * sa * cb + sh * sb;
	p[9] = -(-sh * sa * cb + ch * sb);
	p[10] = ca * cb;
	p[3] = p[7] = p[11] = p[12] = p[13] = p[14] = 0;
	p[15] = 1;

	return p;
}

float* matrixToQuat(float* m) {
	float q[4];

	q[3] = sqrt(1.0 + m[0] + m[5] + m[10]) / 2.0;
	double w4 = (4.0 * q[3]);
	q[0] = (m[6] - m[9]) / w4;
	q[1] = (m[8] - m[2]) / w4;
	q[2] = (m[1] - m[4]) / w4;

	return q;
}
