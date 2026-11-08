#include "PMRcon.h"
#include <math.h>


PMR::PMR(float k_p, float k_r, float T_s, float w_0, float xi, const int harmonics[], int m)
{
    // set T_s
    this->T_s = (T_s > 0.0f) ? T_s : 0.01f;

    // set PMR parameters
    this->k_p = k_p;
    this->k_r = k_r;
    
    // validate parameters
    if (m < 0 || m > PMR_MAX_MODES) m = 0;
    if (m > 0 && (w_0 <= 0.0f || xi < 0.0f || harmonics == nullptr)) m = 0;

    // configure modes
    this->m = m;

    for (int i = 0; i < m; i++)
    {
        mode[i].omega = harmonics[i]*w_0;
        mode[i].xi    = xi;
        mode[i].k_r   = k_r;
        update_coefficients(i);
    }
    
    // set default values
    u_max =  255.0f;
    u_min = -255.0f;

    // reset method
    reset();
}

// compute
float PMR::compute(float r, float y)
{
    // calculates error
    float e = r -y;

    // proportional control action
    float P = k_p*e;

    // resonant control action
    float R = 0.0f;

    for (int i = 0; i < m; i++)
    {
        res_mode &h = mode[i];
        float u_h = h.c1*e +h.c2*h.e1 +h.c3*h.e2 -h.c4*h.u1 -h.c5*h.u2;

        // update previous values
        h.e2 = h.e1;
        h.e1 = e;
        h.u2 = h.u1;
        h.u1 = u_h;
        R   += u_h;
    }
    
    // control signal
    float u = P +R;

    // saturation
    if (u > u_max) u = u_max;
    if (u < u_min) u = u_min;
    
    return u;
}

void PMR::update_coefficients(int i)
{
    res_mode &h = mode[i];
    float omega = h.omega;
    float xi    = h.xi;

    // prewarping coefficient
    float alpha = omega/tanf(omega*T_s/2.0f);

    // denominator coefficients
    float a_0 = alpha*alpha +2.0f*xi*omega*alpha +omega*omega;
    float a_1 = -2.0f*alpha*alpha +2.0f*omega*omega;
    float a_2 = alpha*alpha -2.0f*xi*omega*alpha +omega*omega;

    // numerator coefficients
    float b_0 =  h.k_r*alpha;
    float b_1 =  0.0f;
    float b_2 = -h.k_r*alpha;

    // discrete controller coefficients
    h.c1 = b_0/a_0;
    h.c2 = b_1/a_0;
    h.c3 = b_2/a_0;
    h.c4 = a_1/a_0;
    h.c5 = a_2/a_0;
}

// reset
void PMR::reset()
{
    for (int i = 0; i < m; i++)
    {
        mode[i].u1 = 0.0f;
        mode[i].u2 = 0.0f;
        mode[i].e1 = 0.0f;
        mode[i].e2 = 0.0f;
    }
}

// set methods
void PMR::set_tuning(float k_p, float k_r)
{   
    if (k_p < 0.0f || k_r < 0.0f) return;
    this->k_p = k_p;
    this->k_r = k_r;
    for (int i = 0; i < m; i++)
    {
        mode[i].k_r = k_r;
        update_coefficients(i);
    }
}
void PMR::set_limit(float u_min, float u_max)
{
    if (u_min >= u_max) return;
    this->u_min = u_min;
    this->u_max = u_max;
}

// get methods
float PMR::get_kp() { return k_p; }
float PMR::get_kr() { return k_r; }
int   PMR::get_modes() {return m; }