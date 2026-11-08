#ifndef PMRcon_h
#define PMRcon_h
#ifndef PMR_MAX_MODES
#define PMR_MAX_MODES 10
#endif


class PMR {
    public:
        // constructor
        PMR(float k_p, float k_r, float T_s, float w_0, float xi = 0.0f, const int harmonics[] = nullptr, int m = 0);

        // configuration
        void set_tuning(float k_p, float k_r);
        void set_limit(float u_min, float u_max);

        // compute PMR control signal
        float compute(float r, float y);

        // reset control signals
        void reset();

        // get methods
        float get_kp();
        float get_kr();
        int   get_modes();

    private:
        struct res_mode
        {
            float xi, omega, k_r;
            float c1, c2, c3, c4, c5;
            float u1, u2, e1, e2;
        };
        res_mode mode[PMR_MAX_MODES];

        float k_p, k_r, T_s;
        float u_min, u_max;
        int m;
        void update_coefficients(int i);
};
#endif