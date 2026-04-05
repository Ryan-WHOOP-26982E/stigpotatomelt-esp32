class IMU {
    public:
        IMU();
        float get_rpm(int target_rpm);
        bool get_inverted();
        void poll();
        void init();
        void trim(bool increase, int target_rpm);
        float get_accel_1_g();
        float get_accel_2_g();
        float z_accel_buffer = 0.0;
        float accel_g_ema = 0.0f;   // EMA-filtered centripetal g reading for RPM calc
        float get_trim(int target_rpm);
    private:
        void set_z_offset();
        void get_accel_correction(int target_rpm);
        int current_target_rpm;
};