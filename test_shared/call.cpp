#include <iostream>
#include "fun.hpp"
extern "C"
{
    int calladd(int x,int y)
    {
        return add(x,y);
    }
}