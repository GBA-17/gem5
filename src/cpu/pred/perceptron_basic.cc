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

#include "cpu/pred/perceptron_basic.hh"

#include "base/intmath.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PerceptronBP.hh"

PerceptronBasicBP::PerceptronBasicBP(const PerceptronBasicBPParams *params)
:   BPredUnit(params)
{
    globalHistSize = params->globalHistSize;
    numPerceptrons = params->numPerceptrons;
    globalHistory = 0;
    perceptronTable.reserve(numPerceptrons);
    for (int i = 0; i < numPerceptrons; i++) {
        perceptronTable.push_back(Perceptron(globalHistSize));
    }
}

void
PerceptronBasicBP::btbUpdate(ThreadID tid, Addr branch_addr,
                void * &bp_history)
{
// Place holder for a function that is called to update predictor history when
// a BTB entry is invalid or not found.
}


bool
PerceptronBasicBP::lookup(ThreadID tid, Addr branch_addr, void * &bp_history)
{
    size_t index = computeIndex(branch_addr);
    int y = perceptronTable[index].predict(globalHistory);
    DPRINTF(PerceptronBP, "lookup %x %d %d\n",
                branch_addr, y, y >= 0);
    return y >= 0;
}

void
PerceptronBasicBP::update(ThreadID tid, Addr branch_addr,
                bool taken, void *bp_history,
                bool squashed, const StaticInstPtr & inst,
                Addr corrTarget)
{
    size_t index = computeIndex(branch_addr);
    perceptronTable[index].train(taken);
    updateGlobalHist(taken);

    bool lastPrediction = perceptronTable[index].getLastPrediction();
    DPRINTF(PerceptronBP, "update %x %d %d\n",
        branch_addr, lastPrediction >= 0, taken);
}

void
PerceptronBasicBP::uncondBranch(ThreadID tid,
                Addr pc, void *&bp_history)
{
}

size_t PerceptronBasicBP::computeIndex(Addr branch_addr) {
    // Simple hash function
    return (branch_addr >> 5) % numPerceptrons;
}

void PerceptronBasicBP::updateGlobalHist(bool taken) {
    globalHistory <<= 1;
    if (taken) {
        globalHistory |= 1;
    }
}

PerceptronBasicBP*
PerceptronBasicBPParams::create()
{
    return new PerceptronBasicBP(this);
}
