#include "q_learning.h"
#include <algorithm>
#include <cstdlib>
#include <ctime>

// Q-learning parameters
double alpha = 0.05;
double gamma_discount = 0.98; // Discount factor
double epsilon = 0.3; // Exploration rate

// Q-table
std::unordered_map<State, std::vector<double>> q_table;

// Function to initialize Q-values for a state
void initialize_state(const State& state) {
    if (q_table.find(state) == q_table.end()) {
        q_table[state] = std::vector<double>(ACTION_SPACE_SIZE, 0.0); // Initialize with zeros or any other value
    }
}

Action choose_action(const State& state) {
    // Ensure state is initialized
    initialize_state(state);

    if (static_cast<double>(rand()) / RAND_MAX < epsilon) {
        // Exploration: choose a random action
        return static_cast<Action>(rand() % ACTION_SPACE_SIZE);
    } else {
        // Exploitation: choose the best action based on Q-values
        return static_cast<Action>(std::distance(q_table[state].begin(), std::max_element(q_table[state].begin(), q_table[state].end())));
    }
}

void update_q_table(const State& state, Action action, double reward, const State& next_state) {
    // Ensure states are initialized
    initialize_state(state);
    initialize_state(next_state);

    double predict = q_table[state][action];
    double target = reward + gamma_discount * *std::max_element(q_table[next_state].begin(), q_table[next_state].end());
    q_table[state][action] += alpha * (target - predict);
}