/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * This loop function has two main functions:
 * - Oversees items transitions.
 * - Create the files and fill them with data for statistical purposes.
 *
 * This loop function is meant to be used with the configuration file:
 *    experiments/psiswarm_dps.argos
 */

#ifndef DPS_LOOP_FUNCTIONS_H
#define DPS_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <controllers/psiswarm_dps/psiswarm_dps.h>
#include <argos3/core/simulator/entity/floor_entity.h>

using namespace argos;

class CDPSLoopFunctions : public CLoopFunctions {

public:

  CDPSLoopFunctions();
  virtual ~CDPSLoopFunctions() {}

  virtual void Init(TConfigurationNode& t_tree);
  virtual void Reset();
  virtual void Destroy();
  virtual void PreStep();
  virtual CColor GetFloorColor(const CVector2& c_position_on_plane);

private:

  /*
   * Contains all the state information about the controller. 
   */
  struct SStateData {

    /* The four possible states in which the controller can be */
    enum EState {
      STATE_EXPLORE,
      STATE_GO_TO_NEST,
      STATE_GO_TO_SOURCE,
      STATE_WAIT_FOR_TRANSFER
    } State;
  };

  CRandom::CRNG* m_pcRNG;
  CFloorEntity* m_pcFloor;
  UInt32 UITransferType;
  Real rHomeSourceDistance;

  Real rItemsCounter;
  Real rItemsPosition[100][3];

  /* Outputs */
  std::string m_strIndividual;
  std::ofstream m_cIndividual;

  std::string m_strGlobal;
  std::ofstream m_cGlobal;

private:
  
  /*
   * Executes the exploring state.
   */
  bool Explore(CVector2 cActualPosition, bool bObjectFound);

  /*
   * Executes the go to nest.
   */
  bool GoToNest(CVector2 cActualPosition, CDegrees cActualOrientation, CDegrees* cNestAngle);

  /*
   * Executes the go to source state.
   */
  bool GoToSource(CVector2 cActualPosition, bool bObjectFound);

  /*
   * Wait for transfer state.
   */
  bool WaitForTransfer(CVector2 cActualPosition, Real rID);

  Real median(std::vector<Real> &v);
};

#endif
