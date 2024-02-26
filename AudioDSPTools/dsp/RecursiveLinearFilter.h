//
//  RecursiveLinearFilter.h
//
//
//  Created by Steven Atkinson on 12/28/22.
//
// Recursive linear filters (LPF, HPF, Peaking, Shelving)

#pragma once

#include "dsp.h"
#include <cmath> // pow, sin
#include <vector>

#define MATH_PI 3.14159265358979323846

// TODO refactor base DSP into a common abstraction.

namespace recursive_linear_filter
{

template<typename SampleType>
class Base : public namdsp::DSP<SampleType>
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
  // [0] is for the current sample
  // [1] is for the previous
  // [2] before that
  // (mOutputCoefficients[0] should always be zero. It'll never be used.)
  std::vector<double> mInputCoefficients;
  std::vector<double> mOutputCoefficients;

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
class LevelParams : public namdsp::Params
{
public:
  LevelParams(const SampleType gain)
  : Params()
  , mGain(gain){};
  SampleType GetGain() const { return this->mGain; };

private:
  // The gain (multiplicative, i.e. not dB)
  double mGain;
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
class BiquadParams : public namdsp::Params
{
public:
  BiquadParams(const SampleType sampleRate, const SampleType frequency, const SampleType quality, const SampleType gainDB)
  : namdsp::Params()
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
  double mSampleRate;
};

template <typename SampleType>
class Biquad : public Base <SampleType>
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

// HPF only has one param: frequency
// TODO LPF (alpha calculation is different though)
template <typename SampleType>
class HighPassParams : public namdsp::Params
{
public:
  HighPassParams(const SampleType sampleRate, const SampleType frequency)
  : namdsp::Params()
  , mFrequency(frequency)
  , mSampleRate(sampleRate){};

  SampleType GetAlpha() const
  {
    const SampleType c = 2.0 * MATH_PI * mFrequency / mSampleRate;
    return 1.0 / (c + 1.0);
  };

private:
  SampleType mFrequency;
  SampleType mSampleRate;
};

template <typename SampleType>
class HighPass : public Base<SampleType>
{
public:
  HighPass()
  : Base<SampleType>(2, 2){};
  void SetParams(const HighPassParams<SampleType>& params)
  {
    const double alpha = params.GetAlpha();
    // y[i] = alpha * y[i-1] + alpha * (x[i]-x[i-1])
    this->mInputCoefficients[0] = alpha;
    this->mInputCoefficients[1] = -alpha;
    this->mOutputCoefficients[0] = 0.0;
    this->mOutputCoefficients[1] = alpha;
  }
};

template <typename SampleType>
class LowPassParams : public namdsp::Params
{
public:
  LowPassParams(const SampleType sampleRate, const SampleType frequency)
  : namdsp::Params()
  , mFrequency(frequency)
  , mSampleRate(sampleRate){};

  SampleType GetAlpha() const
  {
    const SampleType c = 2.0 * MATH_PI * mFrequency / mSampleRate;
    return c / (c + 1.0);
  };

private:
  double mFrequency;
  double mSampleRate;
};

template <typename SampleType>
class LowPass : public Base<SampleType>
{
public:
  LowPass()
  : Base<SampleType>(1, 2){};
  void SetParams(const LowPassParams<SampleType>& params)
  {
    const SampleType alpha = params.GetAlpha();
    // y[i] = alpha * x[i] + (1-alpha) * y[i-1]
    this->mInputCoefficients[0] = alpha;
    this->mOutputCoefficients[0] = 0.0;
    this->mOutputCoefficients[1] = 1.0 - alpha;
  }
};

}; // namespace recursive_linear_filter
