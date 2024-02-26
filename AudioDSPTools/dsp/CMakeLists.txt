target_sources(${PROJECT_NAME}
PRIVATE
    ImpulseResponse.cpp
    ImpulseResponse.h
    NoiseGate.cpp
    NoiseGate.h
    RecursiveLinearFilter.cpp
    RecursiveLinearFilter.h
    Resample.h
    dsp.cpp
    dsp.h
    wav.cpp
    wav.h
)

add_subdirectory(ResamplingContainer)
include_directories(ResamplingContainer)