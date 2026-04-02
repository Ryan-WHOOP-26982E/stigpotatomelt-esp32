// controller.h - SBUS receiver interface for Potatomelt

typedef struct ctrl_state {
    // failsafes - both must be true for the robot to do anything other than blink
    bool connected;
    bool alive;

    // master spin switch
    bool spin_requested;

    // translation commands and spin direction
    int translate_forback; // -512 to 512
    int translate_lr;      // reserved for future use
    int turn_lr;           // -512 to 512
    bool reverse_spin;
    int target_rpm;

    float translate_trim;

    // edge-detector outputs: go true once on button press, then back to false
    bool trim_left;
    bool trim_right;
};

typedef struct prev_state {
    bool reverse_spin_pressed;
    bool increase_translate_pressed;
    bool decrease_translate_pressed;
    bool trim_left_pressed;
    bool trim_right_pressed;
    bool spin_target_rpm_changed;
    long last_trim_at;
};

// Call once in setup()
void ctrl_init();

bool is_connected();

// Call every loop iteration. Returns pointer to the current control state.
// The upd8 parameter is kept for API compatibility but is unused with SBUS.
ctrl_state* ctrl_update(bool upd8);
