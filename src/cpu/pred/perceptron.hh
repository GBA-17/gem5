#include <cstdint>
#include <stdlib.h>
#include <vector>
#include "debug/PerceptronBP.hh"

class Perceptron {
    public:
        Perceptron(int global_hist_size) {
            size = global_hist_size + 1;
            theta = 1.93*size + 14;
            weights = std::vector<int>(size, 0);
        }

        Perceptron(int global_hist_size, int init_weight) {
            size = global_hist_size + 1;
            theta = 1.93*size + 14;
            weights = std::vector<int>(size, init_weight);
        }

        int predict(uint64_t global_hist) {
            last_input = (global_hist << 1) | 0x1;
            int y = weights[0];
            for (int i = 1; i < size; i++) {
                if ((last_input >> i) & 0x1) {
                    y += weights[i];
                }
                else {
                    y -= weights[i];
                }
            }
            last_prediction = y;
            return y;
        }

        int getLastPrediction() {
            return last_prediction;
        }

        void train(bool taken) {
            // Update weights if we made the wrong prediction
            //  or the magnitude of y was less than or eq to theta.
            bool predict_taken = last_prediction >= 0;
            if ((taken != predict_taken) || (abs(last_prediction) <= theta)) {
                for (int i = 0; i < size; i++) {
                    bool x = (last_input >> i) & 0x1;
                    if (x == taken) {
                        weights[i]++;
                    }
                    else {
                        weights[i]--;
                    }
                }
            }
        }

    private:
        std::vector<int> weights;
        int theta;
        int size;   // max size 63
        uint64_t last_input;
        int last_prediction;
};
