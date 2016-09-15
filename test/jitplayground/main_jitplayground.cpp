#include <cmath>
#include <stdio.h>

#include <iostream>

using namespace std;

int main (void)
{
    cout << "Starting test <jit>" << endl;
    cout << "This test is x64 and GCC only" << endl;

#if defined (__GNUC__) && __x86_64

    double sin_a, cos_a, a = 0.5;
    asm ("fldl %2;"
         "fsincos;"
         "fstpl %1;"
         "fstpl %0;" : "=m"(sin_a), "=m"(cos_a) : "m"(a));
    printf("sin(29°) = %lf, cos(29°) = %lf\n", sin_a, cos_a);

#endif

    double sin_b;
    double cos_b;
    double b = 0.5;

    sin_b = sin(b);
    cos_b = cos(b);

    double sum = sin_b + cos_b;
    double diff = cos_b - sin_b;


    printf("V: sin(29°) = %lf, cos(29°) = %lf\n", sin_b, cos_b);
    printf("V: sum = %lf, diff = %lf\n", sum, diff);

    cout << "Test <jit> PASSED" << endl;

}
