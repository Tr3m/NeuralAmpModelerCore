target_sources(${PROJECT_NAME}
PRIVATE
    LanczosResampler.h
)

add_subdirectory(WDL)
include_directories(WDL)