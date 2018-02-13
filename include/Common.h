#pragma once
#ifndef __Common_H
#define __Common_H

namespace GS
{

#define DELETE_NEW_OBJ(obj) if(NULL!=obj){delete obj;obj=NULL;}

enum
{
    RET_FAILED = -1,
    RET_SUCESS = 0
};

struct Size
{
    unsigned int width;
    unsigned int height;
    Size(){}
    Size(unsigned int w,unsigned int h):width(w),height(h){}
    Size(const Size &size):width(size.width),height(size.height){}
};

struct RGB
{
    float r;
    float g;
    float b;
    RGB(){}
    RGB(float red, float green, float blue):r(red),g(green),b(blue){}
    RGB& operator=(const RGB &rhs)
    {
        r = rhs.r;
        g = rhs.g;
        b = rhs.b;
        return *this;
    }
};

}
#endif
