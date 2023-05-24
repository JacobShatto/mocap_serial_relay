#include <math.h>       // cos, sin

struct Quat {
    float w;
    float x;
    float y;
    float z;
};

Quat quatInv(Quat Q1) {
    Q1.x = -1 * Q1.x;
    Q1.y = -1 * Q1.y;
    Q1.z = -1 * Q1.z;    

    return Q1;
};

Quat quatMultiply(Quat Q1, Quat Q2) {
    Quat Q3;

    // w component
    Q3.w = Q1.w*Q2.w - Q1.x*Q2.x - Q1.y*Q2.y - Q1.z*Q2.z;

    // x component
    Q3.x = Q1.w*Q2.x + Q1.x*Q2.w + Q1.y*Q2.z - Q1.z*Q2.y;

    // y component
    Q3.y = Q1.w*Q2.y + Q1.y*Q2.w + Q1.z*Q2.x - Q1.x*Q2.z;

    // z component
    Q3.z = Q1.w*Q2.z + Q1.z*Q2.w + Q1.x*Q2.y - Q1.y*Q2.x;
 
    return Q3;   
};

Quat quatMultiply3(Quat Q1, Quat Q2, Quat Q3) {
    Quat Q4;

    Q4 = quatMultiply( quatMultiply(Q1,Q2), Q3);

    return Q4;
};

// Assuming R_z(\Psi) * R_y(\theta) * R_x(\phi)
//            Yaw         Pitch         Roll
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
Quat EulerToQuat(float roll, float pitch, float yaw) {
    Quat q;

	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}
