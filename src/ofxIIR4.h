// The filter is a c++ port of the work presented in the paper:
//S. Skogstad, K. Nymoen, and M. Høvin, “Filtering Motion Capture Data for Real- Time Applications,” Proc. Int. Conf. New Interfaces Music. Expr. - NIME’13, pp. 142– 147, 2013.
//
// more information about the filter and the work can be found:
//http://www.uio.no/english/research/groups/fourms/projects/sma/subprojects/mocapfilters/
//
/*
In the IIR4 patch we have implemented a range of real-time IIR filters applicable for motion controllers, more specifically, low-pass filters and low-pass differentiators of degree 1 and 2. The latter can be used to extract velocity and acceleration data directly from position data. All filters have a group delay of 2 samples or less and have better low-delay performance than what currently established filter design methods can create. See the below recommended cut-off frequencies to use when filtering MoCap data (free-hand motion) and [1] for more information.
5 Hz
Heavy filtering: Fast and rapid motion may be heavily smoothed out. However, the data will contain the main features of normal controlled hand motion.

10 Hz
Medium filtering: Most features of normal and medium rapid motion will be kept in the filtered data. However, some of the higher frequencies will be partially distorted.

15 Hz
Light filtering: All main features of both rapid and normal motion are kept. Only the most extreme parts of the data may be partially blurred.

To convert normalized frequency (pb) to hertz, multiply by half the sample frequency. Or refer the below table.

pb =            0.1       0.2       0.3        0.4       0.5         
Fs =   30Hz :   1.5 Hz,   3.0 Hz,   4.5 Hz,    6.0 Hz,   7.5 Hz,  
Fs =   50Hz :   2.5 Hz,   5.0 Hz,   7.5 Hz,   10.0 Hz,  12.5 Hz,  
Fs =   60Hz :   3.0 Hz,   6.0 Hz,   9.0 Hz,   12.0 Hz,  15.0 Hz,  
Fs =  100Hz :   5.0 Hz,   10.0 Hz,  15.0 Hz,  20.0 Hz,  25.0 Hz,  
Fs =  120Hz :   6.0 Hz,   12.0 Hz,  18.0 Hz,  24.0 Hz,  30.0 Hz,  
Fs =  200Hz :  10.0 Hz,   20.0 Hz,  30.0 Hz,  40.0 Hz,  50.0 Hz,  
*/

#pragma once

#include "ofMain.h"

/*
0: no filtering
1: pb 0.1 low-pass filter
2: pb 0.2 low-pass filter
3: pb 0.3 low-pass filter
4: pb 0.4 low-pass filter
5: pb 0.5 low-pass filter
6: pb 0.1 low-pass differentiator
7: pb 0.2 low-pass differentiator
8: pb 0.3 low-pass differentiator
9: pb 0.4 low-pass differentiator
10: pb 0.5 low-pass differentiator
11: pb 0.1 low-pass double differentiator
12: pb 0.2 low-pass double differentiator
13: pb 0.3 low-pass double differentiator
14: pb 0.4 low-pass double differentiator
15: pb 0.5 low-pass double differentiator
*/




class ofxIIR4{
public:
    void init(float _b0, float _b1, float _b2, float _b3, float _b4, float _a0, float _a1, float _a2, float _a3, float _a4);
    float calc(float _x);
    
    
    
    // coefficients
    float a0;
    float a1;
    float a2;
    float a3;
    float a4;
    
    float b0;
    float b1;
    float b2;
    float b3;
    float b4;
    
    // data stored
    float x_b0;
    float x_b1;
    float x_b2;
    float x_b3;
    float x_b4;
    
    float x_a1;
    float x_a2;
    float x_a3;
    float x_a4;
    
    float y;
    
    int preset;
    
private:
    float presets [16][10] = {
        {1, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0.1400982208, -0.0343775491, 0.0454003083, 0.0099732061, 0.0008485135, 1, -1.9185418203, 1.5929378702, -0.5939699187, 0.0814687111},
        {0.1526249789, 0.0333481282, 0.0777551903, 0.0667145281, 0.0138945068, 1, -1.7462227354, 1.7354077932, -0.8232679111, 0.1793463694},
        {0.1851439645, 0.1383283833, 0.1746892243, 0.1046627716, 0.0464383730, 1, -1.2982434912, 1.4634092217, -0.7106501488, 0.2028836637},
        {0.2680960849, 0.5174415712, 0.5839923942, 0.3748650443, 0.1199394960, 1, 0.0324610402, 0.7694515981, -0.0071430949, 0.0714586993},
        {0.3730569536, 0.8983119412, 0.9660856693, 0.5189611913, 0.1099005390, 1, 0.8053107424, 0.8110594452, 0.2371869724, 0.0849291749},
        {-0.1543174259, 0.1742393427, -0.0178886989, -0.0022975713, 0.0002643535, 1, -2.057495, 1.858706, -0.801785, 0.131076},
        {0.1973679432, -0.0056567353, -0.0321850947, -0.1099445540, -0.0495815592, 1, -0.987078, 0.777486, -0.220684, 0.028134},
        {0.2712475020, 0.1323672597, -0.0487267360, -0.1783422292, -0.1765457966, 1, -0.291948, 0.510465, -0.015578, 0.000284},
        {-0.3453135426, -0.1914474803, 0.0747940184, 0.2519130203, 0.2100539842, 1, -0.094497, 0.735455, -0.136289, 0.075249},
        {-0.4565845496, -0.3412669800, 0.1339011848, 0.4087476487, 0.2552026961, 1, 0.405357, 0.786120, -0.016588, 0.103079},
        {-0.0738989849, 0.1351624829, -0.0512998379, -0.0072918334, -0.0026718267, 1, -1.628287, 1.418759, -0.622342, 0.108528},
        {-0.0795571277, 0.1390709784, -0.0479192600, -0.0031459045, -0.0084486862, 1, -1.571029, 1.459213, -0.717374, 0.148801},
        {0.1099156485, -0.1289124440, -0.0372667405, 0.0216082189, 0.0346553170, 1, -0.827495, 0.811078, -0.353088, 0.065989},
        {0.1395922313, -0.1222824625, -0.0923902820, -0.0067409738, 0.0818214870, 1, -0.363318, 0.635959, -0.298413, 0.067731},
        {0.2032072414, -0.0963599828, -0.2483531617, -0.0270426939, 0.1685485970, 1, 0.615841, 0.520733, -0.003834, 0.018098}
    };


};
class ofxIIR4Vec3f{
public:
    void init(float _b0, float _b1, float _b2, float _b3, float _b4, float _a0, float _a1, float _a2, float _a3, float _a4);
    ofVec3f calc(ofVec3f _x);
    
    ofxIIR4 x;
    ofxIIR4 y;
    ofxIIR4 z;
};

class ofxIIR4Vec2f{
public:
    void init(float _b0, float _b1, float _b2, float _b3, float _b4, float _a0, float _a1, float _a2, float _a3, float _a4);
    ofVec2f calc(ofVec2f _x);
    
    ofxIIR4 x;
    ofxIIR4 y;
};

