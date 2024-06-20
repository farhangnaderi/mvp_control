#ifndef Q_LEARNING_H
#define Q_LEARNING_H

#include <unordered_map>
#include <vector>

// Define the State structure
struct State {
    double error_surge, error_z, error_roll;
    double error_sway, error_pitch, error_yaw;
    double d_error_surge, d_error_z, d_error_roll;
    double d_error_sway, d_error_pitch, d_error_yaw;
    double i_error_surge, i_error_z, i_error_roll;
    double i_error_sway, i_error_pitch, i_error_yaw;

    bool operator==(const State &other) const {
        return error_surge == other.error_surge && error_z == other.error_z && error_roll == other.error_roll &&
               error_sway == other.error_sway && error_pitch == other.error_pitch && error_yaw == other.error_yaw &&
               d_error_surge == other.d_error_surge && d_error_z == other.d_error_z && d_error_roll == other.d_error_roll &&
               d_error_sway == other.d_error_sway && d_error_pitch == other.d_error_pitch && d_error_yaw == other.d_error_yaw &&
               i_error_surge == other.i_error_surge && i_error_z == other.i_error_z && i_error_roll == other.i_error_roll &&
               i_error_sway == other.i_error_sway && i_error_pitch == other.i_error_pitch && i_error_yaw == other.i_error_yaw;
    }
};

namespace std {
    template <>
    struct hash<State> {
        std::size_t operator()(const State &s) const {
            return hash<double>()(s.error_surge) ^ hash<double>()(s.error_z) ^ hash<double>()(s.error_roll) ^
                   hash<double>()(s.error_sway) ^ hash<double>()(s.error_pitch) ^ hash<double>()(s.error_yaw) ^
                   hash<double>()(s.d_error_surge) ^ hash<double>()(s.d_error_z) ^ hash<double>()(s.d_error_roll) ^
                   hash<double>()(s.d_error_sway) ^ hash<double>()(s.d_error_pitch) ^ hash<double>()(s.d_error_yaw) ^
                   hash<double>()(s.i_error_surge) ^ hash<double>()(s.i_error_z) ^ hash<double>()(s.i_error_roll) ^
                   hash<double>()(s.i_error_sway) ^ hash<double>()(s.i_error_pitch) ^ hash<double>()(s.i_error_yaw);
        }
    };
}

// Define the Action enumeration
enum Action {
    INCREASE_KP_SURGE, DECREASE_KP_SURGE,
    INCREASE_KI_SURGE, DECREASE_KI_SURGE,
    INCREASE_KD_SURGE, DECREASE_KD_SURGE,
    INCREASE_KP_Z, DECREASE_KP_Z,
    INCREASE_KI_Z, DECREASE_KI_Z,
    INCREASE_KD_Z, DECREASE_KD_Z,
    INCREASE_KP_ROLL, DECREASE_KP_ROLL,
    INCREASE_KI_ROLL, DECREASE_KI_ROLL,
    INCREASE_KD_ROLL, DECREASE_KD_ROLL,
    INCREASE_KP_SWAY, DECREASE_KP_SWAY,
    INCREASE_KI_SWAY, DECREASE_KI_SWAY,
    INCREASE_KD_SWAY, DECREASE_KD_SWAY,
    INCREASE_KP_PITCH, DECREASE_KP_PITCH,
    INCREASE_KI_PITCH, DECREASE_KI_PITCH,
    INCREASE_KD_PITCH, DECREASE_KD_PITCH,
    INCREASE_KP_YAW, DECREASE_KP_YAW,
    INCREASE_KI_YAW, DECREASE_KI_YAW,
    INCREASE_KD_YAW, DECREASE_KD_YAW,
    ACTION_SPACE_SIZE
};

// Function prototypes
Action choose_action(const State& state);
void update_q_table(const State& state, Action action, double reward, const State& next_state);

// Q-learning parameters
extern double alpha;
extern double gamma_discount;
extern double epsilon;

// Q-table
extern std::unordered_map<State, std::vector<double>> q_table;

#endif // Q_LEARNING_H