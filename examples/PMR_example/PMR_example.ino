#include <PMRcon.h>

const float T_s = 0.01f; // 10 ms
float r = 0.0f; // setpoint
float y = 0.0f; // output
float u = 0.0f; // control input

// reference parameters
float t   = 0.0f; // time
uint8_t A = 50;   // amplitude
uint8_t T = 5;    // reference period
uint8_t cont = 0;

// plant parameters
float w_n  = 10.0f; // natural frequency (rad/s)
float zeta = 0.7f; // damping ratio
float v    = 0.0f;

// PMR parameters
const float k_p = 10.0f;
const float k_r = 5.0f;
const float w_0 = 2.0f*PI/T;
const float xi  = 0.001f;
const int m = 4;
const int harmonics[m] = {1, 3, 5, 7};

// PMR
// PMR pmr(k_p, k_r, T_s, w_0);                // default
PMR pmr(k_p, k_r, T_s, w_0, xi, harmonics, m); // explicit


void reference(){
    t += T_s;

    float tau = fmodf(t+T/4.0, T);
    r         = (4.0f*A/T)*fabsf(tau -T/2.0f) -A;
}

void setup() {
    Serial.begin(115200);
    // pmr.set_limit(-1000.0f, 1000.0f); // set saturation limit
}

void loop() {
    static uint32_t previous = 0; 

    if (millis() -previous >= T_s*1000){
        previous += T_s*1000;

        // square-wave reference
        reference();

        // PMR control signal
        u = pmr.compute(r, y);

        // second-order plant simulated output
        v += T_s*(w_n*w_n*(u -y) -2*zeta*w_n*v);
        y += T_s*v;

        // Serial Plotter: r, y, e, u
        if (++cont >= 10)
        {
            Serial.print(r, 2);
            Serial.print(" ");
            Serial.print(y, 2);
            Serial.print(" ");
            Serial.print(r-y, 2);
            Serial.print(" ");
            Serial.println(u, 2);
            cont = 0;
        }
    }
}