#ifndef FRICMODELSIM_H
#define FRICMODELSIM_H

// 这里声明您的函数
void mdlInitializeSizesFri();
void mdlUpdate(double t, double x[], double u, double A, double B, double f, double sys[]);
void mdlOutputsFri(double t, double x[], double u, double C, double *sys);
double signum(double value);

#endif // FRICMODELSIM_H
