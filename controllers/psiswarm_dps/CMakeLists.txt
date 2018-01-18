add_library(psiswarm_dps SHARED
        psiswarm_dps.h psiswarm_dps.cpp)

target_link_libraries(psiswarm_dps
        argos3core_simulator
        argos3plugin_simulator_psiswarm
        argos3plugin_simulator_genericrobot)
