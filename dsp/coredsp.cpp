#include <algorithm> // std::max_element
#include <algorithm>
#include <cmath> // pow, tanh, expf
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "coredsp.h"

// ============================================================================
// Implementation of Version 2 interface

template <typename SampleType>
dsp::DSP<SampleType>::DSP()
: mOutputPointers(nullptr)
, mOutputPointersSize(0)
{
}

template <typename SampleType>
dsp::DSP<SampleType>::~DSP()
{
  this->_DeallocateOutputPointers();
};

template <typename SampleType>
void dsp::DSP<SampleType>::_AllocateOutputPointers(const size_t numChannels)
{
  if (this->mOutputPointers != nullptr)
    throw std::runtime_error("Tried to re-allocate over non-null mOutputPointers");
  this->mOutputPointers = new SampleType*[numChannels];
  if (this->mOutputPointers == nullptr)
    throw std::runtime_error("Failed to allocate pointer to output buffer!\n");
  this->mOutputPointersSize = numChannels;
}

template <typename SampleType>
void dsp::DSP<SampleType>::_DeallocateOutputPointers()
{
  if (this->mOutputPointers != nullptr)
  {
    delete[] this->mOutputPointers;
    this->mOutputPointers = nullptr;
  }
  if (this->mOutputPointers != nullptr)
    throw std::runtime_error("Failed to deallocate output pointer!");
  this->mOutputPointersSize = 0;
}

template <typename SampleType>
SampleType** dsp::DSP<SampleType>::_GetPointers()
{
  for (auto c = 0; c < this->_GetNumChannels(); c++)
    this->mOutputPointers[c] = this->mOutputs[c].data();
  return this->mOutputPointers;
}

template <typename SampleType>
void dsp::DSP<SampleType>::_PrepareBuffers(const size_t numChannels, const size_t numFrames)
{
  const size_t oldFrames = this->_GetNumFrames();
  const size_t oldChannels = this->_GetNumChannels();

  const bool resizeChannels = oldChannels != numChannels;
  const bool resizeFrames = resizeChannels || (oldFrames != numFrames);
  if (resizeChannels)
  {
    this->mOutputs.resize(numChannels);
    this->_ResizePointers(numChannels);
  }
  if (resizeFrames)
    for (auto c = 0; c < numChannels; c++)
      this->mOutputs[c].resize(numFrames);
}

template <typename SampleType>
void dsp::DSP<SampleType>::_ResizePointers(const size_t numChannels)
{
  if (this->mOutputPointersSize == numChannels)
    return;
  this->_DeallocateOutputPointers();
  this->_AllocateOutputPointers(numChannels);
}

template <typename SampleType>
dsp::History<SampleType>::History()
: DSP<SampleType>()
, mHistoryRequired(0)
, mHistoryIndex(0)
{
}

template <typename SampleType>
void dsp::History<SampleType>::_AdvanceHistoryIndex(const size_t bufferSize)
{
  this->mHistoryIndex += bufferSize;
}

template <typename SampleType>
void dsp::History<SampleType>::_EnsureHistorySize(const size_t bufferSize)
{
  const size_t repeatSize = std::max(bufferSize, this->mHistoryRequired);
  const size_t requiredHistoryArraySize = 10 * repeatSize; // Just so we don't spend too much time copying back.
  if (this->mHistory.size() < requiredHistoryArraySize)
  {
    this->mHistory.resize(requiredHistoryArraySize);
    std::fill(this->mHistory.begin(), this->mHistory.end(), 0.0f);
    this->mHistoryIndex = this->mHistoryRequired; // Guaranteed to be less than
                                                  // requiredHistoryArraySize
  }
}

template <typename SampleType>
void dsp::History<SampleType>::_RewindHistory()
{
  // TODO memcpy?  Should be fine w/ history array being >2x the history length.
  for (size_t i = 0, j = this->mHistoryIndex - this->mHistoryRequired; i < this->mHistoryRequired; i++, j++)
    this->mHistory[i] = this->mHistory[j];
  this->mHistoryIndex = this->mHistoryRequired;
}

template <typename SampleType>
void dsp::History<SampleType>::_UpdateHistory(SampleType** inputs, const size_t numChannels, const size_t numFrames)
{
  this->_EnsureHistorySize(numFrames);
  if (numChannels < 1)
    throw std::runtime_error("Zero channels?");
  if (this->mHistoryIndex + numFrames >= this->mHistory.size())
    this->_RewindHistory();
  // Grabs channel 1, drops hannel 2.
  for (size_t i = 0, j = this->mHistoryIndex; i < numFrames; i++, j++)
    // Convert down to float here.
    this->mHistory[j] = (float)inputs[0][i];
}

template class dsp::DSP<double>;
template class dsp::History<double>;

template class dsp::DSP<float>;
template class dsp::History<float>;