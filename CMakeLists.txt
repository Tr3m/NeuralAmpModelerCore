target_sources(${PROJECT_NAME}
PRIVATE)

add_subdirectory(AudioDSPTools)
include_directories(AudioDSPTools)

add_subdirectory(NeuralAmpModelerCore)
include_directories(NeuralAmpModelerCore)