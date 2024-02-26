//
//  RecursiveLinearFilter.cpp
//
//
//  Created by Steven Atkinson on 12/28/22.
//
// See: https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html

#include <algorithm> // std::fill
#include <cmath> // isnan
#include <stdexcept>

#include "RecursiveLinearFilter.h"

template <typename SampleType>
recursive_linear_filter::Base<SampleType>::Base(const size_t inputDegree, const size_t outputDegree)
: namdsp::DSP<SampleType>()
, mInputStart(inputDegree)
, // 1 is subtracted before first use
mOutputStart(outputDegree)
{
  this->mInputCoefficients.resize(inputDegree);
  this->mOutputCoefficients.resize(outputDegree);
}

template <typename SampleType>
SampleType** recursive_linear_filter::Base<SampleType>::Process(SampleType** inputs, const size_t numChannels,
                                                    const size_t numFrames)
{
  this->_PrepareBuffers(numChannels, numFrames);
  long inputStart = 0;
  long outputStart = 0;
  // Degree = longest history
  // E.g. if y[n] explicitly depends on x[n-2], then input degree is 3
  // (n,n-1,n-2). NOTE: output degree is never 1 because y[n] is never used to
  // explicitly calculate...itself!
  //  0,2,3,... are fine.
  const size_t inputDegree = this->_GetInputDegree();
  const size_t outputDegree = this->_GetOutputDegree();
  for (auto c = 0; c < numChannels; c++)
  {
    inputStart = this->mInputStart; // Should be plenty fine
    outputStart = this->mOutputStart;
    for (auto s = 0; s < numFrames; s++)
    {
      SampleType out = 0.0;
      // Compute input terms
      inputStart -= 1;
      if (inputStart < 0)
        inputStart = inputDegree - 1;
      this->mInputHistory[c][inputStart] = inputs[c][s]; // Store current input
      for (auto i = 0; i < inputDegree; i++)
        out += this->mInputCoefficients[i] * this->mInputHistory[c][(inputStart + i) % inputDegree];

      // Output terms
      outputStart -= 1;
      if (outputStart < 0)
        outputStart = outputDegree - 1;
      for (auto i = 1; i < outputDegree; i++)
        out += this->mOutputCoefficients[i] * this->mOutputHistory[c][(outputStart + i) % outputDegree];
      // Prevent a NaN from jamming the filter!
      if (std::isnan(out))
        out = 0.0;
      // Store the output!
      if (outputDegree >= 1)
        this->mOutputHistory[c][outputStart] = out;
      this->mOutputs[c][s] = out;
    }
  }
  this->mInputStart = inputStart;
  this->mOutputStart = outputStart;
  return this->_GetPointers();
}

template <typename SampleType>
void recursive_linear_filter::Base<SampleType>::_PrepareBuffers(const size_t numChannels, const size_t numFrames)
{
  // Check for new channel count *before* parent class ensures they match!
  const bool newChannels = this->_GetNumChannels() != numChannels;
  // Parent implementation takes care of mOutputs and mOutputPointers
  this->namdsp::DSP<SampleType>::_PrepareBuffers(numChannels, numFrames);
  if (newChannels)
  {
    this->mInputHistory.resize(numChannels);
    this->mOutputHistory.resize(numChannels);
    const size_t inputDegree = this->_GetInputDegree();
    const size_t outputDegree = this->_GetOutputDegree();
    for (auto c = 0; c < numChannels; c++)
    {
      this->mInputHistory[c].resize(inputDegree);
      this->mOutputHistory[c].resize(outputDegree);
      std::fill(this->mInputHistory[c].begin(), this->mInputHistory[c].end(), 0.0);
      std::fill(this->mOutputHistory[c].begin(), this->mOutputHistory[c].end(), 0.0);
    }
  }
}

template <typename SampleType>
void recursive_linear_filter::Biquad<SampleType>::_AssignCoefficients(const SampleType a0, const SampleType a1, const SampleType a2,
                                                          const SampleType b0, const SampleType b1, const SampleType b2)
{
  this->mInputCoefficients[0] = b0 / a0;
  this->mInputCoefficients[1] = b1 / a0;
  this->mInputCoefficients[2] = b2 / a0;
  // this->mOutputCoefficients[0] = 0.0;  // Always
  // Sign flip due so we add during main loop (cf Eq. (4))
  this->mOutputCoefficients[1] = -a1 / a0;
  this->mOutputCoefficients[2] = -a2 / a0;
}

template <typename SampleType>
void recursive_linear_filter::LowShelf<SampleType>::SetParams(const recursive_linear_filter::BiquadParams<SampleType>& params)
{
  const SampleType a = params.GetA();
  const SampleType omega_0 = params.GetOmega0();
  const SampleType alpha = params.GetAlpha(omega_0);
  const SampleType cosw = params.GetCosW(omega_0);

  const SampleType ap = a + 1.0;
  const SampleType am = a - 1.0;
  const SampleType roota2alpha = 2.0 * sqrt(a) * alpha;

  const SampleType b0 = a * (ap - am * cosw + roota2alpha);
  const SampleType b1 = 2.0 * a * (am - ap * cosw);
  const SampleType b2 = a * (ap - am * cosw - roota2alpha);
  const SampleType a0 = ap + am * cosw + roota2alpha;
  const SampleType a1 = -2.0 * (am + ap * cosw);
  const SampleType a2 = ap + am * cosw - roota2alpha;

  this->_AssignCoefficients(a0, a1, a2, b0, b1, b2);
}

template <typename SampleType>
void recursive_linear_filter::Peaking<SampleType>::SetParams(const recursive_linear_filter::BiquadParams<SampleType>& params)
{
  const SampleType a = params.GetA();
  const SampleType omega_0 = params.GetOmega0();
  const SampleType alpha = params.GetAlpha(omega_0);
  const SampleType cosw = params.GetCosW(omega_0);

  const SampleType b0 = 1.0 + alpha * a;
  const SampleType b1 = -2.0 * cosw;
  const SampleType b2 = 1.0 - alpha * a;
  const SampleType a0 = 1.0 + alpha / a;
  const SampleType a1 = -2.0 * cosw;
  const SampleType a2 = 1.0 - alpha / a;

  this->_AssignCoefficients(a0, a1, a2, b0, b1, b2);
}

template <typename SampleType>
void recursive_linear_filter::HighShelf<SampleType>::SetParams(const recursive_linear_filter::BiquadParams<SampleType>& params)
{
  const SampleType a = params.GetA();
  const SampleType omega_0 = params.GetOmega0();
  const SampleType alpha = params.GetAlpha(omega_0);
  const SampleType cosw = params.GetCosW(omega_0);

  const SampleType roota2alpha = 2.0 * sqrt(a) * alpha;
  const SampleType ap = a + 1.0;
  const SampleType am = a - 1.0;

  const SampleType b0 = a * (ap + am * cosw + roota2alpha);
  const SampleType b1 = -2.0 * a * (am + ap * cosw);
  const SampleType b2 = a * (ap + am * cosw - roota2alpha);
  const SampleType a0 = ap - am * cosw + roota2alpha;
  const SampleType a1 = 2.0 * (am - ap * cosw);
  const SampleType a2 = ap - am * cosw - roota2alpha;

  this->_AssignCoefficients(a0, a1, a2, b0, b1, b2);
}

template class recursive_linear_filter::Base<float>;
template class recursive_linear_filter::Base<double>;
template class recursive_linear_filter::LevelParams<float>;
template class recursive_linear_filter::LevelParams<double>;
template class recursive_linear_filter::Level<float>;
template class recursive_linear_filter::Level<double>;
template class recursive_linear_filter::BiquadParams<float>;
template class recursive_linear_filter::BiquadParams<double>;
template class recursive_linear_filter::Biquad<float>;
template class recursive_linear_filter::Biquad<double>;
template class recursive_linear_filter::LowShelf<float>;
template class recursive_linear_filter::LowShelf<double>;
template class recursive_linear_filter::Peaking<float>;
template class recursive_linear_filter::Peaking<double>;
template class recursive_linear_filter::HighShelf<float>;
template class recursive_linear_filter::HighShelf<double>;
template class recursive_linear_filter::HighPassParams<float>;
template class recursive_linear_filter::HighPassParams<double>;
template class recursive_linear_filter::HighPass<float>;
template class recursive_linear_filter::HighPass<double>;
template class recursive_linear_filter::LowPassParams<float>;
template class recursive_linear_filter::LowPassParams<double>;
template class recursive_linear_filter::LowPass<float>;
template class recursive_linear_filter::LowPass<double>;