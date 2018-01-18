/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Display the position where the robot thinks is the robot source.
 * Also shows the neighbourhood where the robots will be exploring.
 * Lastly, it is possible to see the range of vision of the robot
 *
 * This code is meant to be used with the configuration file:
 *    experiments/taskPartitioning.argos
 */

#ifndef PSISWARM_DPS_QT_USER_FUNCTIONS_H
#define PSISWARM_DPS_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/psi-swarm/simulator/psiswarm_entity.h>

using namespace argos;

class CPsiSwarmDPSQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CPsiSwarmDPSQTUserFunctions();

   virtual ~CPsiSwarmDPSQTUserFunctions() {}

   void Draw(CPsiSwarmEntity& c_entity);
   
   void DrawInWorld();

   struct SStateData {

    /* The four possible states in which the controller can be */
    enum EState {
      STATE_EXPLORE,
      STATE_GO_TO_NEST,
      STATE_GO_TO_SOURCE,
      STATE_WAIT_FOR_TRANSFER
    } State;
  };

};

#endif
