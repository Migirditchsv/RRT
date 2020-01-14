/* C++ environment setup/test
Sam Migirditch*/

#include <iostream>
#include <cmath>

// Global function declare
int foo(int x, float y);


int main(int argc, int **argv)
{
    int maxVal = 1000;
    int x = 2;
    double y = 3.4;
    int ans =2.0 * x;

    while (ans < maxVal)
    {
        ans += y;
    }

    return(ans);
}