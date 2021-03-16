/*
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cpu/pred/perceptron_hashed.hh"
#include <algorithm>
#include <cstdint>
#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Fetch.hh"

PerceptronHashedBP::PerceptronHashedBP(const PerceptronHashedBPParams *params)
:   BPredUnit(params)
{
    numWeights = params->numWeights;
    numPerceptrons = params->numPerceptrons;
    globalHistory = 0;
    theta = (1.93 * numPerceptrons) + (numPerceptrons / 2);
    indexes.resize(numPerceptrons);
    weights.resize(numPerceptrons, std::vector<int>(numWeights, 0));
    DPRINTF(Fetch, "Init hashed perceptron. %d perc, %d weights\n",
        numPerceptrons, numWeights);
}

void
PerceptronHashedBP::btbUpdate(ThreadID tid, Addr branch_addr,
                void * &bp_history)
{
// Place holder for a function that is called to update predictor history when
// a BTB entry is invalid or not found.
}


bool
PerceptronHashedBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    computeIndex(branch_addr);
    int y = 0;
    for (int i = 0; i < numPerceptrons; i++) {
        y += weights[i][indexes[i]];
    }
    lastPrediction = y;
    DPRINTF(Fetch, "Predict %d for %x\n", lastPrediction, branch_addr);
    return y >= 0;
}

void
PerceptronHashedBP::update(ThreadID tid, Addr branch_addr,
                bool taken, void *bp_history,
                bool squashed, const StaticInstPtr & inst,
                Addr corrTarget)
{
    updateGlobalHist(taken);
    bool predict_taken = lastPrediction >= 0;
    DPRINTF(Fetch, "Predicted/actual %d %d for %x\n",
        predict_taken, taken, branch_addr);
    if ((predict_taken != taken) || (abs(lastPrediction) <= theta)) {
        for (int i = 0; i < numPerceptrons; i++) {
            if (taken) {
                weights[i][indexes[i]]++;
            }
            else {
                weights[i][indexes[i]]--;
            }
        }
    }
}

void
PerceptronHashedBP::uncondBranch(ThreadID tid,
                Addr pc, void *&bp_history)
{
}

void PerceptronHashedBP::computeIndex(Addr branch_addr)
{
    indexes[0] = branch_addr % numWeights;
    int stride = std::max(1, int(64 / numPerceptrons));
    for (int i = 1; i < numPerceptrons; i++) {
        uint64_t bitmask = generateBitmask(stride*i);
        uint64_t histSegment = globalHistory & bitmask;
        indexes[i] = (histSegment ^ branch_addr) % numWeights;
    }
}

void PerceptronHashedBP::updateGlobalHist(bool taken) {
    globalHistory <<= 1;
    if (taken) {
        globalHistory |= 1;
    }
}

uint64_t PerceptronHashedBP::generateBitmask(int bits) {
    uint bitmask = 1;
    for (int i = 0; i < bits; i++) {
        bitmask = (bitmask << 1) | 1;
    }
    return bitmask;
}

PerceptronHashedBP*
PerceptronHashedBPParams::create()
{
    return new PerceptronHashedBP(this);
}
