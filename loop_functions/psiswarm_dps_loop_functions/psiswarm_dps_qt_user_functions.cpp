/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Draws ID and state for each robot. 
 * Draws item when is being carried by a robot.
 *
 * This code is meant to be used with the configuration file:
 *    experiments/psiswarm_dps.argos
 */

/* Header files */
#include "psiswarm_dps_qt_user_functions.h"
#include "psiswarm_dps_loop_functions.h"
#include <controllers/psiswarm_dps/psiswarm_dps.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>

using namespace argos;

#include "psiswarm_dps_loop_functions.h"
/****************************************/
/****************************************/

CPsiSwarmDPSQTUserFunctions::CPsiSwarmDPSQTUserFunctions() {
   RegisterUserFunction<CPsiSwarmDPSQTUserFunctions,CPsiSwarmEntity>(&CPsiSwarmDPSQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CPsiSwarmDPSQTUserFunctions::Draw(CPsiSwarmEntity& c_entity) {

  /* Get handle to food psi-swarm controller */
  PsiSwarmDPS& cController = dynamic_cast<PsiSwarmDPS&>(c_entity.GetControllableEntity().GetController());
  
  switch(cController.m_sStateData.State) {
    case SStateData::STATE_EXPLORE: {
      DrawText(CVector3(-0.015f,0.0f,0.05f), "E", CColor::BLACK);
      break;
    }
    case SStateData::STATE_GO_TO_NEST: {
      DrawText(CVector3(-0.015f,0.0f,0.05f), "N", CColor::BLACK);
      CQuaternion cCircleOrientation;
      DrawCylinder(CVector3(0.0f,0.0f,0.2f), cCircleOrientation, 0.02f, 0.02f, CColor::BLACK);
      break;
    }
    case SStateData::STATE_GO_TO_SOURCE: {
      DrawText(CVector3(-0.015f,0.0f,0.05f), "S", CColor::BLACK);
      break;
    }
    case SStateData::STATE_WAIT_FOR_TRANSFER: {
      DrawText(CVector3(-0.015f,0.0f,0.05f), "W", CColor::BLACK);
      CQuaternion cCircleOrientation;
      DrawCylinder(CVector3(0.0f,0.0f,0.2f), cCircleOrientation, 0.02f, 0.02f, CColor::BLACK);
      break;
    }
    default: {
       LOGERR << "Controller: State!" << std::endl;
       break;
    }
  }

}

void CPsiSwarmDPSQTUserFunctions::DrawInWorld(){
  
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CPsiSwarmDPSQTUserFunctions, "psiswarm_dps_qt_user_functions")

