#include "../melty_config.h"

class LED {
    public:
        LED();

        void leds_on_ready();
        void leds_on_low_battery();
        void leds_on_controller_stale();
        void leds_on_no_controller();

        void leds_on_gradient(int color);
        void leds_off();
    private:
        void set_leds(bool on);
};
