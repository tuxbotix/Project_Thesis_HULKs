#pragma once

#include "NaoPoseInfo.hpp"

namespace GeomUtils
{

// The function dist3D_Segment_to_Segment is from Dan Sunday's website:
//   http://geomalgorithms.com/a07-_distance.html
// with some modifications.  The inputs of type Segment were replaced by
// point pairs of type Vector3<T> and the algebraic operator calls were
// replaced accordingly.  The distance is now returned (with other parameters)
// as arguments to the function.  The SMALL_NUM macro was replaced by a
// 'const' declaration.  The modified code computes the closest points.  See
// the revised document (as of 2014/11/05)
//   http://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf
// that describes an algorithm that is robust, particularly for nearly
// segments, and that uses floating-point arithmetic.  An example in this PDF
// shows that there is a problem with the logic of Sunday's algorithm when
// D < SMALL_NUM and the search is started on the s=0 edge. Specifically,
// the closest points are not found correctly--the closest point on the
// first segment occurs when s=1.  No contact information is at his website,
// so we are unable to report the problem to him.

// template <typename T>
// T dist3D_Segment_to_Segment(
//     const Vector3<T> &P0, const Vector3<T> &P1,
//     const Vector3<T> &Q0, const Vector3<T> &Q1
//     /*T &distance, T &s, T &t, Vector3<T> closest[2]*/)
// {
//     const T SMALL_NUM = __FLT_EPSILON__;
//     Vector3<T> u = P1 - P0;
//     Vector3<T> v = Q1 - Q0;
//     Vector3<T> w = P0 - Q0;
//     T a = u.dot(u); // always >= 0
//     T b = u.dot(v);
//     T c = v.dot(v); // always >= 0
//     T d = u.dot(w);
//     T e = v.dot(w);
//     T D = a * c - b * b; // always >= 0
//     T sc, sN, sD = D;    // sc = sN / sD, default sD = D >= 0
//     T tc, tN, tD = D;    // tc = tN / tD, default tD = D >= 0
//     T distance = 0;

//     // compute the line parameters of the two closest points
//     if (D < SMALL_NUM)
//     {             // the lines are almost parallel
//         sN = 0.0; // force using point P0 on segment S1
//         sD = 1.0; // to prevent possible division by 0.0 later
//         tN = e;
//         tD = c;
//     }
//     else
//     { // get the closest points on the infinite lines
//         sN = (b * e - c * d);
//         tN = (a * e - b * d);
//         if (sN < 0.0)
//         { // sc < 0 => the s=0 edge is visible
//             sN = 0.0;
//             tN = e;
//             tD = c;
//         }
//         else if (sN > sD)
//         { // sc > 1  => the s=1 edge is visible
//             sN = sD;
//             tN = e + b;
//             tD = c;
//         }
//     }

//     if (tN < 0.0)
//     { // tc < 0 => the t=0 edge is visible
//         tN = 0.0;
//         // recompute sc for this edge
//         if (-d < 0.0)
//             sN = 0.0;
//         else if (-d > a)
//             sN = sD;
//         else
//         {
//             sN = -d;
//             sD = a;
//         }
//     }
//     else if (tN > tD)
//     { // tc > 1  => the t=1 edge is visible
//         tN = tD;
//         // recompute sc for this edge
//         if ((-d + b) < 0.0)
//             sN = 0;
//         else if ((-d + b) > a)
//             sN = sD;
//         else
//         {
//             sN = (-d + b);
//             sD = a;
//         }
//     }
//     // finally do the division to get sc and tc
//     sc = (std::abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
//     tc = (std::abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

//     // get the difference of the two closest points
//     s = sc;
//     t = tc;
//     Vector3<T> closest0 = (1.0 - sc) * P0 + sc * P1;
//     Vector3<T> closest1 = (1.0 - tc) * Q0 + tc * Q1;
//     Vector3<T> diff = closest0 - closest1;
//     distance = diff.norm();
//     return distance;
// }

// The function dist3D_Segment_to_Segment is from Dan Sunday's website:
//   http://geomalgorithms.com/a07-_distance.html
// with some modifications.  The inputs of type Segment were replaced by
// point pairs of type Vector3<T> and the algebraic operator calls were
// replaced accordingly.  The distance is now returned (with other parameters)
// as arguments to the function.  The SMALL_NUM macro was replaced by a
// 'const' declaration.  The modified code computes the closest points.  See
// the revised document (as of 2014/11/05)
//   http://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf
// that describes an algorithm that is robust, particularly for nearly
// segments, and that uses floating-point arithmetic.  An example in this PDF
// shows that there is a problem with the logic of Sunday's algorithm when
// D < SMALL_NUM and the search is started on the s=0 edge. Specifically,
// the closest points are not found correctly--the closest point on the
// first segment occurs when s=1.  No contact information is at his website,
// so we are unable to report the problem to him.

template <typename T>
std::pair<Vector3<T>, Vector3<T>> closestPtRayToRay(
    const Vector3<T> &P0, const Vector3<T> &P1,
    const Vector3<T> &Q0, const Vector3<T> &Q1)
{
    const T SMALL_NUM = __FLT_EPSILON__;
    Vector3<T> u = P1 - P0;
    Vector3<T> v = Q1 - Q0;
    Vector3<T> w = P0 - Q0;
    T a = u.dot(u); // always >= 0
    T b = u.dot(v);
    T c = v.dot(v); // always >= 0
    T d = u.dot(w);
    T e = v.dot(w);
    T D = a * c - b * b; // always >= 0
    T sc, tc;

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM)
    { // the lines are almost parallel
        sc = 0.0;
        tc = (b > c ? d / b : e / c); // use the largest denominator
    }
    else
    {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    // // finally do the division to get sc and tc
    // sc = (std::abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    // tc = (std::abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    return {P0 + sc * u, Q0 + tc * v};
}

template <typename T>
T closestPtRayToRay(
    const Vector3<T> &P0, const Vector3<T> &P1,
    const Vector3<T> &Q0, const Vector3<T> &Q1,
    std::pair<Vector3<T>, Vector3<T>> &closestPts)
{
    closestPts = closestPtRayToRay(P0, P1, Q0, Q1);
    return (closestPts.first - closestPts.second).norm();
}

// dist_Point_to_Line(): get the distance of a point to a line
//     Input:  a Point P and a Line L (in any dimension)
//     Return: the shortest distance from P to L
template <typename T>
T distPointToLine(const Vector3<T> &lP0, const Vector3<T> &lP1, const Vector3<T> &Pt)
{
    Vector3<T> v = lP1 - lP0;
    Vector3<T> w = Pt - lP0;

    double c1 = (double)w.dot(v);
    double c2 = (double)v.dot(v);
    double b = c1 / c2;

    Vector3<T> Pb = lP0 + (float)b * v;
    return (Pt - Pb).norm();
}

} // namespace GeomUtils
