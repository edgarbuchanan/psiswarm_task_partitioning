/*
 * AUTHOR: Edgar Buchanan <edgar.buchanan@york.ac.uk>
 *
 * Task partitioning controller for the psi-swarm robot platform.
 *
 * This controller used the foraging controller in the
 * ARGoS examples as a template.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/psiswarm_dps.argos
 */

#ifndef PSISWARM_TEST_CONTROLLER_H
#define PSISWARM_TEST_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>

// Sensors
#include <argos3/plugins/robots/psi-swarm/control_interface/ci_psiswarm_proximity_sensor.h>

// Actuators
#include <argos3/plugins/robots/psi-swarm/control_interface/ci_psiswarm_wheels_actuator.h>

// Variables types
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>

#include <argos3/core/utility/math/rng.h>

using namespace argos;

class PsiSwarmDPS : public CCI_Controller
{
public:

  PsiSwarmDPS();
  virtual ~PsiSwarmDPS() {}

  virtual void Init(TConfigurationNode &t_node);
  virtual void ControlStep();
  virtual void Reset() {}
  virtual void Destroy() {}

  struct SWheelTurningParams {

    /*
     * The turning mechanism.
     * The robot can be in three different turning states.
     */
    enum ETurningMechanism {
      NO_TURN = 0,  // go straight
      SOFT_TURN,  // both wheels are turning forwards, but at different speeds
      HARD_TURN,  // wheels are turning with opposite speeds
      DIRECTION_TURN  //change of direction
    } TurningMechanism;

    /*
     * Angular thresholds to change turning state.
     */
    CDegrees HardTurnOnAngleThreshold;
    CDegrees SoftTurnOnAngleThreshold;
    CDegrees NoTurnAngleThreshold;
    /* Maximum wheel speed */
    Real MaxSpeed;

    void Init(TConfigurationNode& t_node);
  };

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

    void Init(TConfigurationNode& t_node);
  };

  /*
   * Contains all the information for patitioning.
    */
  struct SPartitionParams{

    /* Partition list */
    enum EPartition {
      NON_PARTITIONING,
      STATIC_PARTITIONING,
      DYNAMIC_PARTITIONING,
      COST_PARTITIONING
    } Partition;

    Real rInitialPartitionLength;
    Real rGain;
    Real rAlpha;
    Real rMemFacAl;
    Real rGrFacEps;
    UInt32 UIErrorType;
    void Init(TConfigurationNode& t_node);
  };

private:

  /*
   * Executes the exploring state.
   */
  void Explore();

  /*
   * Executes the go to nest state.
   */
  void GoToNest();

  /*
   * Executes the go to source state.
   */
  void GoToSource();

  /*
   * Wait for transfer state.
   */
  void WaitForTransfer();

  /*
   * Partition calculator.
   */
   void PartitionCalculator(bool bObjectFound);

  /*
   * Move Psiswarm.
   */
  void Move();

  /*
   * Executes the obstacle avoidance.
   */
  CVector2 ObstacleAvoidance();

  /*
   * Calculate actual position
    */ 
  void Odometry(Real rLeftMotorSpeed, Real rRightMotorSpeed);

  /*
   * Update target angle according to state.
    */ 
   CDegrees UpdateAngle(CDegrees cPrevAngle);

   /*
    * Add noise to motors 
    */
    CVector2 AddingError(CVector2 cIdealSpeed);

private:

  CRandom::CRNG* m_pcRNG; 

  /* Sensors */
  CCI_PsiSwarmProximitySensor* proximity_sensor;

  /* Actuators */
  CCI_PsiSwarmWheelsActuator* wheels_actuator;

  /* The turning parameters */
  SWheelTurningParams m_sWheelTurningParams;

  /* The partition parameters */
  SPartitionParams m_sPartitionParams;

public:

  /* The controller state information */
  SStateData m_sStateData;

  /* Flag is true if object is found */
  bool bObjectFound;

  /* Relative position of the beacon */
  CDegrees cNestAngle;

  /* Previous position */
  CVector2 cPreviousPosition;

  /* Actual position */
  CVector2 cActualPosition;

  /* Actual orientation */
  CRadians cActualOrientation;

  /* Array to prevent deadlocks when avoiding */
  Real rHistory[3];

  /* Timer to rest robot */
  UInt64 UITimeRested;

  /* Timer for error model */
  UInt64 UIModelErrorTimer;

  /* Timer if position is unreachable */
  UInt64 UIPositionNotReachedTimer;

  /* Timer if object is not handed over. */
  UInt64 UIWaitingTimer;

  /* Robot ID */
  UInt32 UIRobotID;

  /* rPartitionLength */
  Real rPartitionLength;

  /* Go to nest directly flag */
  bool bGoToNest;

  /* Variables used by CPS */
  Real rNumElemN; // Number of partitions
  UInt32 m_UnPartChos;  // Partition chosen
  Real rDisStep;  // Discretization step
  Real rEstDistD; // Estimated distance nest-source 
  Real rsetL[99]; // Set of partition lengths
  Real rsetC[99]; // Set of costs
  Real rLastTripC;// Cost associated with the last trip
  Real rTimeNav;  // Time spent navigating
  Real rTimeGripp;// Time spent gripping
  Real rTimeExp;  // Time spent exploring
  CVector2 cNestSourceDistance; // Distance nest source

  UInt32 m_UnTotalTimeExplore;  // Time exploring
  UInt32 m_UnTotalTimeNest;      // Time nest
  UInt32 m_UnTotalTimeSource;  // Time source
  UInt32 m_UnTotalTimeWaiting; // Time waiting

  UInt32 m_UnItemsFound; // Objects found;
  UInt32 m_UnItemsLost; // Objects lost;  

};

#endif
