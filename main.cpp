#include "include/swap.h"
using namespace std;

int main(){
    int a = 105;
    int b = 200;
    int * p;
    p = &a;
    *p = 999;

    swap(a, b);
    cout << sizeof(*p) << endl;
    cout << &a << "\t" << *(&a + 1) << endl;
    cout << p <<  "\t" << &p <<  "\t" << *(p+1) << endl;

    return 0;

}

