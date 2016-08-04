#ifndef TUPLE_H
#define TUPLE_H

// 2-tuple (x, y)
struct Points2D {
 Points2D( double x, double y):_x(x),_y(y)
 {
 }
 double _x , _y;
};

// 3-tuple (SE(2))
struct Points3D {
 Points3D( double x, double y, double z):_x(x),_y(y),_z(z)
 {
 }
 double _x , _y, _z;
};

// 5-tuple (T^6)
struct Points5D {
 Points5D( double q1, double q2, double q3, double q4, double q5):_q1(q1),_q2(q2),_q3(q3),_q4(q4),_q5(q5)
 {
 }
 double _q1 , _q2, _q3, _q4, _q5;
};

#endif
