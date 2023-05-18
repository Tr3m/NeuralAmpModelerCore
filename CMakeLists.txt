target_sources(NAM
PRIVATE
    )

add_subdirectory(dsp)
include_directories(dsp)

add_subdirectory(NAM)
include_directories(NAM)