#include "filter.h"

LowPass::LowPass(float f0, float fs){
    // f0: cutoff frequency (Hz)
    // fs: sample frequency (Hz)
    // adaptive: boolean flag, if set to 1, the code will automatically set
    // the sample frequency based on the time history.
    omega0 = 6.28318530718*f0;
    dt = 1.0/fs;
    tn1 = -dt;
    for(int k = 0; k < 1+1; k++){
    x[k] = 0;
    y[k] = 0;        
    }
    setCoef();
}

void LowPass::setCoef(){    
    float alpha = omega0*dt;

    a[0] = -(alpha - 2.0)/(alpha+2.0);
    b[0] = alpha/(alpha+2.0);
    b[1] = alpha/(alpha+2.0);        

}

float LowPass::filt(float xn){
    // Provide me with the current raw value: x
    // I will give you the current filtered value: y

    y[0] = 0;
    x[0] = xn;
    // Compute the filtered values
    for(int k = 0; k < 1; k++){
    y[0] += a[k]*y[k+1] + b[k]*x[k];
    }
    y[0] += b[1]*x[1];

    // Save the historical values
    for(int k = 1; k > 0; k--){
    y[k] = y[k-1];
    x[k] = x[k-1];
    }

    // Return the filtered value    
    return y[0];
}
