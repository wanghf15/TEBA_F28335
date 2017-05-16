/*****************************************************************************/
/* math.h     v5.0.0B3                                                         */
/* Copyright (c) 1996-2007 Texas Instruments Incorporated                    */
/*****************************************************************************/

#ifndef _MATH
#define _MATH

#ifdef __cplusplus
//----------------------------------------------------------------------------
// <cmath> IS RECOMMENDED OVER <math.h>.  <math.h> IS PROVIDED FOR 
// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
//----------------------------------------------------------------------------
#include <cmath>
using std::modf;
using std::asin;
using std::acos;
using std::atan;
using std::atan2;
using std::ceil;
using std::cos;
using std::cosh;
using std::exp;
using std::fabs;
using std::floor;
using std::fmod;
using std::frexp;
using std::ldexp;
using std::log;
using std::log10;
using std::pow;
using std::sin;
using std::sinh;
using std::tan;
using std::tanh;
using std::sqrt;

#else /* !__cplusplus */

#include <linkage.h>
#include <float.h>
#define HUGE_VAL   DBL_MAX
#define HUGE_VALL  LDBL_MAX

/***************************************************************/
/* FUNCTION DEFINITIONS.                                       */
/***************************************************************/
         double modf(double x, double *y); 
         double far_modf(double x, far double *y); 
         double asin(double x);
         double acos(double x);
         double atan(double x);
         double atan2(double y, double x);
__inline double ceil(double x)  { double y; return (modf(x, &y) > 0 ? y+1:y); }
         double cos(double x);
         double cosh(double x);
         double exp(double x);
         double fabs(double x);
__inline double floor(double x) { double y; return (modf(x, &y) < 0 ? y-1:y); }
         double fmod(double x, double y);
         double frexp(double x, int *exp);
         double far_frexp(double x, far int *exp);
         double ldexp(double x, int exp);
         double log(double x);
         double log10(double x);
         double pow(double x, double y);
         double sin(double x);
         double sinh(double x);
         double tan(double x);
         double tanh(double x);
         double sqrt(double x);

#endif /* __cplusplus */
#endif /* _MATH */
