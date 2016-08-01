#pragma once

#define BEGIN_INCREON_SPACE namespace Increon {
#define END_INCREON_SPACE }

#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <string>
    
BEGIN_INCREON_SPACE

typedef unsigned char       uchar;
typedef unsigned short      ushort;
typedef unsigned int        uint;
typedef unsigned long 		ulong;			// 64-bit machine long is 8 bytes

typedef unsigned char       uint8;
typedef unsigned short      uint16;
typedef unsigned int        uint32;
typedef unsigned long long  uint64;

struct Color3b {
    unsigned char r, g, b;
	
	Color3b() : r(0), g(0), b(0) {}
	Color3b(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {}
};

struct Color3f {
    float r, g, b;
	
	Color3f() : r(0), g(0), b(0) {}
	Color3f(float r, float g, float b) : r(r), g(g), b(b) {}
};

END_INCREON_SPACE