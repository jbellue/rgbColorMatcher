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

float xyzChannelConversion(const int value) {
    const float ret = (float)value / 255;
    if ( ret > 0.04045 ) {
        return pow(( ret + 0.055 ) / 1.055, 2.4) * 100;
    }
    return ret / 0.1292;
}

XYZ rgbToXYZ(const rgb_color c) {
    const float var_R = xyzChannelConversion(c.r);
    const float var_G = xyzChannelConversion(c.g);
    const float var_B = xyzChannelConversion(c.b);

    const float X = var_R * 0.4124 + var_G * 0.3576 + var_B * 0.1805;
    const float Y = var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722;
    const float Z = var_R * 0.0193 + var_G * 0.1192 + var_B * 0.9505;
    return (XYZ){X, Y, Z};
}

float CIELabChannelConversion(const float value) {
    if (value > 0.008856) {
        return pow(value, (float)1/3 );
    }
    return ( 7.787 * value ) + ( (float)16/116 );
}

CIELab xyzToCIELab(const XYZ color) {
    const float var_X = CIELabChannelConversion((float)color.X / 95.047 );
    const float var_Y = CIELabChannelConversion((float)color.Y / 100    );
    const float var_Z = CIELabChannelConversion((float)color.Z / 108.883);

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
