//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: StabController.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-May-2024 01:17:58
//

#ifndef STABCONTROLLER_H
#define STABCONTROLLER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void StabController(const double x[13], const double Kv[15],
                           const double param[4], const double setpoint[3],
                           double u[3]);

#endif
//
// File trailer for StabController.h
//
// [EOF]
//
