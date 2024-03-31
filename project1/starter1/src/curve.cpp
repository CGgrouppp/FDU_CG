#include "curve.h"
#include "vertexrecorder.h"
using namespace std;

const float c_pi = 3.14159265358979323846f;

namespace
{
// Approximately equal to.  We don't want to use == because of
// precision issues with floating point.
inline bool approx(const Vector3f& lhs, const Vector3f& rhs)
{
	const float eps = 1e-8f;
	return (lhs - rhs).absSquared() < eps;
}


}


Curve evalBezier(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4 || P.size() % 3 != 1)
	{
		cerr << "evalBezier must be called with 3n+1 control points." << endl;
		exit(0);
	}

	// TODO:
	// You should implement this function so that it returns a Curve
	// (e.g., a vector< CurvePoint >).  The variable "steps" tells you
	// the number of points to generate on each piece of the spline.
	// At least, that's how the sample solution is implemented and how
	// the SWP files are written.  But you are free to interpret this
	// variable however you want, so long as you can control the
	// "resolution" of the discretized spline curve with it.

	// Make sure that this function computes all the appropriate
	// Vector3fs for each CurvePoint: V,T,N,B.
	// [NBT] should be unit and orthogonal.

	// Also note that you may assume that all Bezier curves that you
	// receive have G1 continuity.  Otherwise, the TNB will not be
	// be defined at points where this does not hold.
	
	int P_size = P.size();
	int segments = P_size/3;
	float t,delta;
	delta = 1.0/steps;
	t = 0;
	Curve curves;
	Matrix4f M_bez={1,-3,3,-1,
								0,3,-6,3,
								0,0,3,-3,
								0,0,0,1};
	Vector3f B0={0,0,1};
	Vector4f MT;
	Vector4f MT_dao;
	
	for(int i=0;i<segments;i++)
	{
		t = 0;
		Vector3f P1,P2,P3,P4;
		P1 = P[i*3];
		P2 = P[i*3+1];
		P3 = P[i*3+2];
		P4 = P[i*3+3];
		
		if(i == 0)
		{
			CurvePoint C;
			C.V = P1;
			C.T =(3* (P2-P1)).normalized();
			C.N = Vector3f::cross(B0,C.T).normalized();
			C.B = Vector3f::cross(C.T,C.N).normalized();
			curves.push_back(C);
		}
		for(unsigned j=0;j<steps;j++)
		{
			t+=delta;
			CurvePoint NewC;
			Vector4f T(1,t,t*t,t*t*t);
			Vector4f T_dao(0,1,2*t,3*t*t);
			MT=M_bez*T;
			MT_dao=M_bez*T_dao;
			NewC.V = P1*MT.x()+P2*MT.y()+P3*MT.z()+P4*MT.w();
			NewC.T = (P1*MT_dao.x()+P2*MT_dao.y()+P3*MT_dao.z()+P4*MT_dao.w()).normalized();
			NewC.N = Vector3f::cross(curves.back().B,NewC.T).normalized();
			NewC.B = Vector3f::cross(NewC.T,NewC.N).normalized();
			curves.push_back(NewC);
		}
	}

	cerr << "\t>>> evalBezier has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	cerr << "\t>>> Steps (type steps): " << steps << endl;
	cerr << "\t>>> Returning empty curve." << endl;

	// Right now this will just return this empty curve.
	return curves;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps)
{
	// Check
	if (P.size() < 4)
	{
		cerr << "evalBspline must be called with 4 or more control points." << endl;
		exit(0);
	}

	// TODO:
	// It is suggested that you implement this function by changing
	// basis from B-spline to Bezier.  That way, you can just call
	// your evalBezier function.

	int P_size=P.size();
	Matrix4f M_B(1.0/6, -1.0/2, 1.0/2, -1.0/6,
							2.0/3, 0, -1.0, 1.0/2,
							1.0/6, 1.0/2, 1.0/2, -1.0/2,
							0, 0, 0, 1.0/6);
	Matrix4f M_bez={1,-3,3,-1,
								0,3,-6,3,
								0,0,3,-3,
								0,0,0,1};
	Matrix4f Multi=M_B*(M_bez.inverse());
	vector<Vector3f> P_trans;
	for(int i=3;i<P_size;i++)
	{
		Matrix4f temp(Vector4f(P[i-3],0),
								 Vector4f(P[i-2],0),
								 Vector4f(P[i-1],0),
								 Vector4f(P[i],0)); //构造4*4矩阵，方便相乘
		Matrix4f P_trans_raw=temp*Multi; //得到新的点，但是w轴多余，只取xyz
		if(i==3)
		{
			P_trans.push_back(P_trans_raw.getCol(0).xyz());
		}
		P_trans.push_back(P_trans_raw.getCol(1).xyz());
		P_trans.push_back(P_trans_raw.getCol(2).xyz());
		P_trans.push_back(P_trans_raw.getCol(3).xyz());
	}
	vector<CurvePoint> curves=evalBezier(P_trans,steps);

	cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

	cerr << "\t>>> Control points (type vector< Vector3f >): " << endl;
	for (int i = 0; i < (int)P.size(); ++i)
	{
		cerr << "\t>>> " << P[i] << endl;
	}

	cerr << "\t>>> Steps (type steps): " << steps << endl;
	cerr << "\t>>> Returning empty curve." << endl;

	// Return an empty curve right now.
	return curves;
}

Curve evalCircle(float radius, unsigned steps)
{
	// This is a sample function on how to properly initialize a Curve
	// (which is a vector< CurvePoint >).

	// Preallocate a curve with steps+1 CurvePoints
	Curve R(steps + 1);

	// Fill it in counterclockwise
	for (unsigned i = 0; i <= steps; ++i)
	{
		// step from 0 to 2pi
		float t = 2.0f * c_pi * float(i) / steps;

		// Initialize position
		// We're pivoting counterclockwise around the y-axis
		R[i].V = radius * Vector3f(cos(t), sin(t), 0);

		// Tangent vector is first derivative
		R[i].T = Vector3f(-sin(t), cos(t), 0);

		// Normal vector is second derivative
		R[i].N = Vector3f(-cos(t), -sin(t), 0);

		// Finally, binormal is facing up.
		R[i].B = Vector3f(0, 0, 1);
	}

	return R;
}

void recordCurve(const Curve& curve, VertexRecorder* recorder)
{
	const Vector3f WHITE(1, 1, 1);
	for (int i = 0; i < (int)curve.size() - 1; ++i)
	{
		recorder->record_poscolor(curve[i].V, WHITE);
		recorder->record_poscolor(curve[i + 1].V, WHITE);
	}
}
void recordCurveFrames(const Curve& curve, VertexRecorder* recorder, float framesize)
{
	Matrix4f T;
	const Vector3f RED(1, 0, 0);
	const Vector3f GREEN(0, 1, 0);
	const Vector3f BLUE(0, 0, 1);
	
	const Vector4f ORGN(0, 0, 0, 1);
	const Vector4f AXISX(framesize, 0, 0, 1);
	const Vector4f AXISY(0, framesize, 0, 1);
	const Vector4f AXISZ(0, 0, framesize, 1);

	for (int i = 0; i < (int)curve.size(); ++i)
	{
		T.setCol(0, Vector4f(curve[i].N, 0));
		T.setCol(1, Vector4f(curve[i].B, 0));
		T.setCol(2, Vector4f(curve[i].T, 0));
		T.setCol(3, Vector4f(curve[i].V, 1));
 
		// Transform orthogonal frames into model space
		Vector4f MORGN  = T * ORGN;
		Vector4f MAXISX = T * AXISX;
		Vector4f MAXISY = T * AXISY;
		Vector4f MAXISZ = T * AXISZ;

		// Record in model space
		recorder->record_poscolor(MORGN.xyz(), RED);
		recorder->record_poscolor(MAXISX.xyz(), RED);

		recorder->record_poscolor(MORGN.xyz(), GREEN);
		recorder->record_poscolor(MAXISY.xyz(), GREEN);

		recorder->record_poscolor(MORGN.xyz(), BLUE);
		recorder->record_poscolor(MAXISZ.xyz(), BLUE);
	}
}

