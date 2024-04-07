#include "surf.h"
#include "vertexrecorder.h"
using namespace std;

namespace
{
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

// DEBUG HELPER
Surface quad() { 
	Surface ret;
	ret.VV.push_back(Vector3f(-1, -1, 0));
	ret.VV.push_back(Vector3f(+1, -1, 0));
	ret.VV.push_back(Vector3f(+1, +1, 0));
	ret.VV.push_back(Vector3f(-1, +1, 0));

	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));
	ret.VN.push_back(Vector3f(0, 0, 1));

	ret.VF.push_back(Tup3u(0, 1, 2));
	ret.VF.push_back(Tup3u(0, 2, 3));
	return ret;
}

const double pi = 3.14159265358979323846;

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
	//surface = quad();
    unsigned lenth = profile.size();

    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
    // 旋转曲面
    for(unsigned i = 0; i < steps; i++){
        double theta = 2*pi/steps*i;
        Matrix4f M = Matrix4f::rotateY(theta);
        Matrix4f M_inverse_T = M.inverse().transposed();
        for(unsigned j = 0; j <lenth; j++){
            Vector4f P = Vector4f(profile[j].V, 1);
            Vector4f N = Vector4f(profile[j].N, 1);
            surface.VV.push_back((M*P).xyz());
            surface.VN.push_back((-(M_inverse_T*N).normalized()).xyz());
        }
        for(unsigned k = 0; k<lenth-1; k++){
            // 构成三角形
            unsigned temp = (i+1)%(steps);
            surface.VF.push_back(Tup3u(i*lenth+k, i*lenth+k+1, temp*lenth+k));
            surface.VF.push_back(Tup3u(i*lenth+k+1, temp*lenth+k+1, temp*lenth+k));
        }
    }

    cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;
 
    return surface;
}

bool CheckWeird(const Curve& sweep)
{
    unsigned sweep_size=sweep.size();
    double const eps=1.0e-6;
    if((sweep[0].T == sweep[sweep_size-1].T) && (sweep[0].V==sweep[sweep_size-1].V) 
            && (sweep[0].N != sweep[sweep_size-1].N))
        return false;//不闭合返回false
    return true;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;
	//surface = quad();

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.
    // 广义圆柱体
    unsigned lenth = profile.size();
    unsigned steps = sweep.size();
    //插值旋转
    Quat4f P0=Quat4f::IDENTITY;
    Quat4f P1=Quat4f::IDENTITY;
    if(!CheckWeird(sweep))
    {
        float theta = -acos(Vector3f::dot(sweep[0].N,sweep[steps-1].N));
        P1.setAxisAngle(theta,sweep[0].T);
        
    }
    Curve sweep_trans;
    float t=0,delta=1.0/steps;
    for(unsigned i=0;i<steps;i++)
    {
        CurvePoint point;
        Quat4f Slerp = Quat4f::slerp(P0,P1,t).normalized();
        point.V = sweep[i].V;
        point.N = (Slerp*Quat4f(sweep[i].N)*Slerp.inverse()).xyz(); //利用四元数进行插值旋转
        point.B = (Slerp*Quat4f(sweep[i].B)*Slerp.inverse()).xyz();
        point.T = (Slerp*Quat4f(sweep[i].T)*Slerp.inverse()).xyz();
        sweep_trans.push_back(point);
        t+=delta;

    }
    for(unsigned i = 0; i < steps; i++){
        Vector4f N = Vector4f(sweep_trans[i].N, 0);
        Vector4f B = Vector4f(sweep_trans[i].B, 0);
        Vector4f T = Vector4f(sweep_trans[i].T, 0);
        Vector4f V = Vector4f(sweep_trans[i].V, 1);
        Matrix4f M = Matrix4f(N, B, T, V, true);
        Matrix4f M_inverse_T = M.inverse().transposed();
        for(unsigned j = 0; j < lenth; j++){
            Vector4f P = Vector4f(profile[j].V, 1);
            Vector4f N = Vector4f(profile[j].N, 1);
            surface.VV.push_back((M*P).xyz());
            surface.VN.push_back((-(M_inverse_T*N).normalized()).xyz());
        }

        for(unsigned k = 0; k<lenth-1; k++){
            // 构成三角形
            unsigned temp = (i+1)%(steps);
            surface.VF.push_back(Tup3u(i*lenth+k, i*lenth+k+1, temp*lenth+k));
            surface.VF.push_back(Tup3u(i*lenth+k+1, temp*lenth+k+1, temp*lenth+k));
        }
    }
    
    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

void recordSurface(const Surface &surface, VertexRecorder* recorder) {
	const Vector3f WIRECOLOR(0.4f, 0.4f, 0.4f);
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
		recorder->record(surface.VV[surface.VF[i][0]], surface.VN[surface.VF[i][0]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][1]], surface.VN[surface.VF[i][1]], WIRECOLOR);
		recorder->record(surface.VV[surface.VF[i][2]], surface.VN[surface.VF[i][2]], WIRECOLOR);
    }
}

void recordNormals(const Surface &surface, VertexRecorder* recorder, float len)
{
	const Vector3f NORMALCOLOR(0, 1, 1);
    for (int i=0; i<(int)surface.VV.size(); i++)
    {
		recorder->record_poscolor(surface.VV[i], NORMALCOLOR);
		recorder->record_poscolor(surface.VV[i] + surface.VN[i] * len, NORMALCOLOR);
    }
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (int i=0; i<(int)surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (int i=0; i<(int)surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (int i=0; i<(int)surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
