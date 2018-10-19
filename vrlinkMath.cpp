#include "StdAfx.h"
#include "cveddecl.h"
#include "vector3d.h"
#include <matrix/vlTaitBryan.h>
#include <vlpi/entityTypes.h>
#include "vrlinkMath.h"

void TaitBran2Quaternion(double i, double j, double k, double *w, double *x, double *y, double *z)
{
	DtTaitBryan aTB(k, j, i);
	DtQuaternion aQ(aTB);
	*w = aQ.w();
	*x = aQ.x();
	*y = aQ.y();
	*z = aQ.z();
}
void Quaternion2Taitbran(double w, double x, double y, double z, double *i, double *j, double *k)
{
	DtQuaternion q_vrlink(w, x, y, z);
	DtTaitBryan ori = q_vrlink; //this call might have performance overhead
	*k = ori.psi();
	*j = ori.theta();
	*i = ori.phi();
}