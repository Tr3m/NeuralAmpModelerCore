//
//  ImpulseResponse.h
//  NeuralAmpModeler-macOS
//
//  Created by Steven Atkinson on 12/30/22.
//
// Impulse response processing

#pragma once

#include <filesystem>

#include <Eigen/Dense>

#include "dsp.h"
#include "wav.h"

namespace namdsp
{

template <typename SampleType>
struct IRData
{
  std::vector<SampleType> mRawAudio;
  SampleType mRawAudioSampleRate;
};

template <typename SampleType>
class ImpulseResponse : public History<SampleType>
{
public:
  //struct IRData;
  ImpulseResponse(const char* fileName, const SampleType sampleRate);
  ImpulseResponse(const IRData<SampleType>& irData, const SampleType sampleRate);
  SampleType** Process(SampleType** inputs, const size_t numChannels, const size_t numFrames) override;
  IRData<SampleType> GetData();
  SampleType GetSampleRate() const { return mSampleRate; };
  // TODO states for the IR class
  namdsp::wav::LoadReturnCode GetWavState() const { return this->mWavState; };

private:
  // Set the weights, given that the plugin is running at the provided sample
  // rate.
  void _SetWeights();

  // State of audio
  namdsp::wav::LoadReturnCode mWavState;
  // Keep a copy of the raw audio that was loaded so that it can be resampled
  std::vector<SampleType> mRawAudio;
  SampleType mRawAudioSampleRate;
  // Resampled to the required sample rate.
  std::vector<SampleType> mResampled;
  SampleType mSampleRate;

  const size_t mMaxLength = 8192;
  // The weights
  Eigen::VectorXf mWeight;
};

}; // namespace namdsp
