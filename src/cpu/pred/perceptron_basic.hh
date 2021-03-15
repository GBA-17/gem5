#ifndef _CPU_PERD_PERCEPTRON_
#define _CPU_PERD_PERCEPTRON_

#include "cpu/pred/perceptron.hh"
#include "base/types.hh"
#include "cpu/pred/bpred_unit.hh"
#include "params/PerceptronBasicBP.hh"


class PerceptronBasicBP {
    public:
        PerceptronBasicBP(const PerceptronBasicBPParams *params);

        bool lookup(Addr br_addr);

        void update(Addr branch_addr, bool taken);

        size_t computeIndex(Addr branch_addr);

        void updateGlobalHist(bool taken);

    private:

        std::vector<Perceptron> allPerceptron;

        size_t globalHistory;

        size_t numPerceptron;

        size_t numPerceptronParam;

        size_t hash;
};

#endif // _CPU_PERD_PERCEPTRON_