//
//  solve_polynom.hpp
//  ComputerVisionProject4
//
//  Created by HJKBD on 4/10/17.
//  Copyright © 2017 HJKBD. All rights reserved.
//

#ifndef solve_polynom_hpp
#define solve_polynom_hpp

#include "global.h"
#include <math.h>

// The is to solve second order polynomial equation a*x^2 + b*x + c = 0
//@para a The second order coefficient
//@para b The first order coefficient
//@para c The zero order coefficient
//@para x1 The first root
//@para x2 The second root
//@return the number of roots found
int solve_deg2(double a, double b, double c, double & x1, double & x2);


//This is to calculate the form a*x^3 + b * x^2 + c * x + d = 0
//@para a
//@para b
//@para c
//@parac d
int solve_deg3(double a, double b, double c, double d,
               double & x0, double & x1, double & x2);

//This is to calculate the form a*x^4 + b * x^3 + c * x^2 + d*x + e = 0
//@para a
//@para b
//@para c
//@para d
//@para e
//@return the number of solutions
int solve_deg4(double a, double b, double c, double d, double e,
               double & x0, double & x1, double & x2, double & x3);

#endif /* solve_polynom_hpp */
