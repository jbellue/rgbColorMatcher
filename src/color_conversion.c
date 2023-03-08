#include "color_conversion.h"

#include <stdint.h>
#include <math.h>

typedef struct CIELab {
    float L, a, b;
} CIELab;

typedef struct XYZ {
    float X, Y, Z;
} XYZ;

XYZ rgbToXYZ(const rgb_color c);
CIELab xyzToCIELab(const XYZ color);

XYZ rgbToXYZ(const rgb_color c) {
    float var_R = (float)c.r / 255;
    float var_G = (float)c.g / 255;
    float var_B = (float)c.b / 255;

    if ( var_R > 0.04045 ) {
        var_R = pow(( var_R + 0.055 ) / 1.055, 2.4);
    }
    else {
        var_R = var_R / 12.92;
    }

    if ( var_G > 0.04045 ) {
        var_G = pow(( var_G + 0.055 ) / 1.055, 2.4);
    }
    else {
        var_G = var_G / 12.92;
    }

    if ( var_B > 0.04045 ) {
        var_B = pow(( var_B + 0.055 ) / 1.055, 2.4);
    }
    else {
        var_B = var_B / 12.92;
    }

    var_R = var_R * 100;
    var_G = var_G * 100;
    var_B = var_B * 100;

    const float X = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805;
    const float Y = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722;
    const float Z = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505;
    return (XYZ){X, Y, Z};
}

CIELab xyzToCIELab(const XYZ color) {
    float var_X = (float)color.X / 95.047;
    float var_Y = (float)color.Y / 100;
    float var_Z = (float)color.Z / 108.883;

    if (var_X > 0.008856) {
        var_X = pow(var_X, (float)1/3 );
    }
    else {
        var_X = ( 7.787 * var_X ) + ( (float)16/116 );
    }

    if (var_Y > 0.008856) {
        var_Y = pow(var_Y, (float)1/3 );
    }
    else {
        var_Y = ( 7.787 * var_Y ) + ( (float)16/116 );
    }

    if (var_Z > 0.008856) {
        var_Z = pow(var_Z, (float)1/3 );
    }
    else {
        var_Z = ( 7.787 * var_Z ) + ( (float)16/116 );
    }

    const float CIEL = ( 116 * var_Y ) - 16;
    const float CIEa = 500 * ( var_X - var_Y );
    const float CIEb = 200 * ( var_Y - var_Z );
    return (CIELab){CIEL, CIEa, CIEb};
}


float colorDistance(const rgb_color c1, const rgb_color c2) {
    const XYZ xyz1 = rgbToXYZ(c1);
    const CIELab CIELab1 = xyzToCIELab(xyz1);

    const XYZ xyz2 = rgbToXYZ(c2);
    const CIELab CIELab2 = xyzToCIELab(xyz2);

    return sqrt(
        pow(CIELab1.L - CIELab2.L, 2) +
        pow(CIELab1.a - CIELab2.a, 2) +
        pow(CIELab1.b - CIELab2.b, 2)
    );
}
