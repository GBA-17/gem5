#include "cpu/pred/perceptron_basic.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "base/bitfield.hh"

PerceptronBasicBP::PerceptronBasicBP(const PerceptronBasicBPParams *params) {

        numPerceptron = params -> numPredictors;
        numPerceptronParam = params -> localPredictorSize;
        hash = numPerceptron - 1;

        // allPerceptron = std::vector<Perceptron>(numPerceptron, numPerceptronParam);
        allPerceptron.resize(numPerceptron);
        // for(int i = 0; i < numPerceptron;i++) {
        //      allPerceptron[i] = new Perceptron(numPerceptronParam);
        // }

}

bool PerceptronBasicBP::lookup(Addr branch_addr) {
    size_t index = computeIndex(branch_addr);
    return allPerceptron[index].predict(globalHistory);
}

void PerceptronBasicBP::update(Addr branch_addr, bool taken) {
    size_t index = computeIndex(branch_addr);
    allPerceptron[index].train(taken);
    updateGlobalHist(taken);
}

size_t PerceptronBasicBP::computeIndex(Addr branch_addr) {
    return (branch_addr >> 5) & hash;
}

void PerceptronBasicBP::updateGlobalHist(bool taken) {
    if(taken == true) {
        globalHistory = (globalHistory << 1) | 1;
    } else {
        globalHistory = (globalHistory << 1);
    }
}

PerceptronBasicBP* PerceptronBasicBPParams::create(){
    return new PerceptronBasicBP(this);
}
