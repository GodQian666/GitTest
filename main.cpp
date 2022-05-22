#include "include/swap.h"
using namespace std;

int main(){
    int a = 105;
    int b = 200;
    int *p;
    int *pp = (int *)0xdddddffa6c;
    p = &a;
    *p = 999;

    swap(a, b);
    cout << pp << endl;
    cout << sizeof(p) << endl;
    cout << &a << "\t" << *(&a + 1) << endl;
    cout << p <<  "\t" << &p <<  "\t" << *(p+1) << endl;

    return 0;

}

