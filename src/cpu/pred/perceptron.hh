class Perceptron {
    public:
        Perceptron(uint32_t gh_size, int32_t init_weight) {
            size = gh_size;
            theta = 1.93*size + 14;
            last_prediction = 0;
            weights = std::vector<int32_t>(size, init_weight);
        }

        // check if global_hist is 0 or -1
        uint32_t predict(uint64_t global_hist) {
            uint32_t prediction = weights[0];
        }

    private:
        std::vector<int32_t> weights;
        uint32_t theta;
        uint32_t size;
        bool last_prediction;
}
