//
//  RecursiveLinearFilter.h
//
//
//  Created by Steven Atkinson on 12/28/22.
//
// Recursive linear filters (LPF, HPF, Peaking, Shelving)

#pragma once

#include "coredsp.h"
#include <cmath> // pow, sin
#include <vector>

#define MATH_PI 3.14159265358979323846

// TODO refactor base DSP into a common abstraction.

namespace recursive_linear_filter
{
template <typename SampleType>
class Base : public dsp::DSP<SampleType>
{
public:
  Base(const size_t inputDegree, const size_t outputDegree);
  SampleType** Process(SampleType** inputs, const size_t numChannels, const size_t numFrames) override;

protected:
  // Methods
  size_t _GetInputDegree() const { return this->mInputCoefficients.size(); };
  size_t _GetOutputDegree() const { return this->mOutputCoefficients.size(); };
  // Additionally prepares mInputHistory and mOutputHistory.
  void _PrepareBuffers(const size_t numChannels, const size_t numFrames) override;

  // Coefficients for the DSP filter
  std::vector<SampleType> mInputCoefficients;
  std::vector<SampleType> mOutputCoefficients;

  // Arrays holding the history on which the filter depends recursively.
  // First index is channel
  // Second index, [0] is the current input/output, [1] is the previous, [2] is
  // before that, etc.
  std::vector<std::vector<SampleType>> mInputHistory;
  std::vector<std::vector<SampleType>> mOutputHistory;
  // Indices for history.
  // Designates which index is currently "0". Use modulus to wrap around.
  long mInputStart;
  long mOutputStart;
};

template <typename SampleType>
class LevelParams : public dsp::Params
{
public:
  LevelParams(const SampleType gain)
  : Params()
  , mGain(gain){};
  SampleType GetGain() const { return this->mGain; };

private:
  // The gain (multiplicative, i.e. not dB)
  SampleType mGain;
};

template <typename SampleType>
class Level : public Base<SampleType>
{
public:
  Level()
  : Base<SampleType>(1, 0){};
  // Invalid usage: require a pointer to recursive_linear_filter::Params so
  // that SetCoefficients() is defined.
  void SetParams(const LevelParams<SampleType>& params) { this->mInputCoefficients[0] = params.GetGain(); };
  ;
};

// The same 3 params (frequency, quality, gain) describe a bunch of filters.
// (Low shelf, high shelf, peaking)
template <typename SampleType>
class BiquadParams : public dsp::Params
{
public:
  BiquadParams(const SampleType sampleRate, const SampleType frequency, const SampleType quality, const SampleType gainDB)
  : dsp::Params()
  , mFrequency(frequency)
  , mGainDB(gainDB)
  , mQuality(quality)
  , mSampleRate(sampleRate){};

  // Parameters defined in
  // https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
  SampleType GetA() const { return pow(10.0, this->mGainDB / 40.0); };
  SampleType GetOmega0() const { return 2.0 * MATH_PI * this->mFrequency / this->mSampleRate; };
  SampleType GetAlpha(const SampleType omega_0) const { return sin(omega_0) / (2.0 * this->mQuality); };
  SampleType GetCosW(const SampleType omega_0) const { return cos(omega_0); };

private:
  SampleType mFrequency;
  SampleType mGainDB;
  SampleType mQuality;
  SampleType mSampleRate;
};

template <typename SampleType>
class Biquad : public Base<SampleType>
{
public:
  Biquad()
  : Base<SampleType>(3, 3){};
  virtual void SetParams(const BiquadParams<SampleType>& params) = 0;

protected:
  void _AssignCoefficients(const SampleType a0, const SampleType a1, const SampleType a2, const SampleType b0, const SampleType b1,
                           const SampleType b2);
};

template <typename SampleType>
class LowShelf : public Biquad<SampleType>
{
public:
  void SetParams(const BiquadParams<SampleType>& params) override;
};

template <typename SampleType>
class Peaking : public Biquad<SampleType>
{
public:
  void SetParams(const BiquadParams<SampleType>& params) override;
};

template <typename SampleType>
class HighShelf : public Biquad<SampleType>
{
public:
  void SetParams(const BiquadParams<SampleType>& params) override;
};
}; // namespace recursive_linear_filter