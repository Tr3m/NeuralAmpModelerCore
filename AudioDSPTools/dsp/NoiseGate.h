//
//  NoiseGate.h
//  NeuralAmpModeler-macOS
//
//  Created by Steven Atkinson on 2/5/23.
//

#pragma once

#include <cmath>
#include <unordered_set>
#include <vector>

#include "dsp.h"

namespace namdsp
{
namespace noise_gate
{
// Disclaimer: No one told me how noise gates work. I'm just going to try
// and have fun with it and see if I like what I get! :D

// "The noise floor." The loudness of anything quieter than this is bumped
// up to as if it were this loud for gating purposes (i.e. computing gain
// reduction).
const double MINIMUM_LOUDNESS_DB = -120.0;
const double MINIMUM_LOUDNESS_POWER = pow(10.0, MINIMUM_LOUDNESS_DB / 10.0);

// Parts 2: The gain module.
// This applies the gain reduction taht was determined by the trigger.
// It's declared first so that the trigger can define listeners without a
// forward declaration.

// The class that applies the gain reductions calculated by a trigger instance.
template <typename SampleType>
class Gain : public DSP<SampleType>
{
public:
  SampleType** Process(SampleType** inputs, const size_t numChannels, const size_t numFrames) override;

  void SetGainReductionDB(std::vector<std::vector<SampleType>>& gainReductionDB)
  {
    this->mGainReductionDB = gainReductionDB;
  }

private:
  std::vector<std::vector<SampleType>> mGainReductionDB;
};

// Part 1 of the noise gate: the trigger.
// This listens to a stream of incoming audio and determines how much gain
// to apply based on the loudness of the signal.

template <typename SampleType>
class TriggerParams
{
public:
  TriggerParams(const SampleType time, const SampleType threshold, const SampleType ratio, const SampleType openTime,
                const SampleType holdTime, const SampleType closeTime)
  : mTime(time)
  , mThreshold(threshold)
  , mRatio(ratio)
  , mOpenTime(openTime)
  , mHoldTime(holdTime)
  , mCloseTime(closeTime){};

  SampleType GetTime() const { return this->mTime; };
  SampleType GetThreshold() const { return this->mThreshold; };
  SampleType GetRatio() const { return this->mRatio; };
  SampleType GetOpenTime() const { return this->mOpenTime; };
  SampleType GetHoldTime() const { return this->mHoldTime; };
  SampleType GetCloseTime() const { return this->mCloseTime; };

private:
  // The time constant for quantifying the loudness of the signal.
  SampleType mTime;
  // The threshold at which expanssion starts
  SampleType mThreshold;
  // The compression ratio.
  SampleType mRatio;
  // How long it takes to go from maximum gain reduction to zero.
  SampleType mOpenTime;
  // How long to stay open before starting to close.
  SampleType mHoldTime;
  // How long it takes to go from open to maximum gain reduction.
  SampleType mCloseTime;
};

template <typename SampleType>
class Trigger : public DSP<SampleType>
{
public:
  Trigger();

  SampleType** Process(SampleType** inputs, const size_t numChannels, const size_t numFrames) override;
  std::vector<std::vector<SampleType>> GetGainReduction() const { return this->mGainReductionDB; };
  void SetParams(const TriggerParams<SampleType>& params) { this->mParams = params; };
  void SetSampleRate(const SampleType sampleRate) { this->mSampleRate = sampleRate; }
  std::vector<std::vector<SampleType>> GetGainReductionDB() const { return this->mGainReductionDB; };

  void AddListener(Gain<SampleType>* gain)
  {
    // This might be risky dropping a raw pointer, but I don't think that the
    // gain would be destructed, so probably ok.
    this->mGainListeners.insert(gain);
  }

private:
  enum class State
  {
    MOVING = 0,
    HOLDING
  };

  SampleType _GetGainReduction(const SampleType levelDB) const
  {
    const SampleType threshold = this->mParams.GetThreshold();
    // Quadratic gain reduction? :)
    return levelDB < threshold ? -(this->mParams.GetRatio()) * (levelDB - threshold) * (levelDB - threshold) : 0.0;
  }
  SampleType _GetMaxGainReduction() const { return this->_GetGainReduction(MINIMUM_LOUDNESS_DB); }
  virtual void _PrepareBuffers(const size_t numChannels, const size_t numFrames) override;

  TriggerParams<SampleType> mParams;
  std::vector<State> mState; // One per channel
  std::vector<SampleType> mLevel;

  // Hold the vectors of gain reduction for the block, in dB.
  // These can be given to the Gain object.
  std::vector<std::vector<SampleType>> mGainReductionDB;
  std::vector<SampleType> mLastGainReductionDB;

  SampleType mSampleRate;
  // How long we've been holding
  std::vector<SampleType> mTimeHeld;

  std::unordered_set<Gain<SampleType>*> mGainListeners;
};

}; // namespace noise_gate
}; // namespace namdsp