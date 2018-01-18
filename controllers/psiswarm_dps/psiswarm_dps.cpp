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

#include "psiswarm_dps.h"

#define STATESWITCHTIME 32.5

PsiSwarmDPS::PsiSwarmDPS():
        proximity_sensor(NULL),
        wheels_actuator(NULL) {}

/*
 * Retrieve values for the wheel turning parameters from xml file
 */
void PsiSwarmDPS::SWheelTurningParams::Init(TConfigurationNode& t_node) {
  try {
    GetNodeAttribute(t_node, "hard_turn_angle_threshold", HardTurnOnAngleThreshold);
    GetNodeAttribute(t_node, "soft_turn_angle_threshold", SoftTurnOnAngleThreshold);
    GetNodeAttribute(t_node, "no_turn_angle_threshold", NoTurnAngleThreshold);
    GetNodeAttribute(t_node, "max_speed", MaxSpeed);
  }
  catch(CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
  }
}

/*
 * Retrieve values for the partitioning parameteres from xml file.
 */
void PsiSwarmDPS::SPartitionParams::Init(TConfigurationNode& t_node){
  try {
    UInt32 UIPartition;
    GetNodeAttribute(t_node, "partition_type", UIPartition);
    GetNodeAttribute(t_node, "initial_partition_length", rInitialPartitionLength);
    GetNodeAttribute(t_node, "gain", rGain);
    GetNodeAttribute(t_node, "alpha", rAlpha);
    GetNodeAttribute(t_node, "epsilon", rGrFacEps);
    GetNodeAttribute(t_node, "memory_factor", rMemFacAl);
    GetNodeAttribute(t_node, "error_type", UIErrorType);
    switch(UIPartition){
      case(1):{
        Partition = NON_PARTITIONING;
        break;
      }
      case(2):{
        Partition = STATIC_PARTITIONING;
        break;
      }
      case(3):{
        Partition = DYNAMIC_PARTITIONING;
        break;
      }
      case(4):{
        Partition = COST_PARTITIONING;
        break;
      }
      default:{
        LOGERR << "Controller: Wrong partition!" << std::endl;
        break;
      }
    }

  }
  catch(CARGoSException& ex) {
    THROW_ARGOSEXCEPTION_NESTED("Error initializing controller partition parameters.", ex);
  }
}

/*
 * Initialise controller
 */
void PsiSwarmDPS::Init(TConfigurationNode &t_node)
{
  try{

    /*
     * Initialise sensors/actuators
     */
    proximity_sensor = GetSensor<CCI_PsiSwarmProximitySensor>("psiswarm_proximity");
    wheels_actuator = GetActuator<CCI_PsiSwarmWheelsActuator>("psiswarm_wheels");

    /*
     * Parse XML parameters
     */
    m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning")); // Wheel turning
    m_sPartitionParams.Init(GetNode(t_node, "partitioning")); // Wheel turning
  }
  catch(CARGoSException& ex){
    THROW_ARGOSEXCEPTION_NESTED("Error initializing the psi-swarm controller for robot \"" << GetId() << "\"", ex);

  }
  m_pcRNG = CRandom::CreateRNG("argos"); 
  /* Initialize variables */
  m_sStateData.State = SStateData::STATE_EXPLORE;
  bObjectFound = false; 
  cActualPosition.Set(0.0, 0.0);
  cActualOrientation.SetValue(0.0);

  rHistory[0] = 0;
  rHistory[1] = 0;
  rHistory[2] = 0;

  UITimeRested = 0;
  UIModelErrorTimer = 0;
  UIPositionNotReachedTimer = 0;
  UIWaitingTimer = 0;

  rPartitionLength = m_sPartitionParams.rInitialPartitionLength;

  UIRobotID = GetId().at(8) - '0'; // Get ID from according to the ID assigned by ARGoS

  /* Variables used for the cost estimation approach */
  rNumElemN = 0.0;  // Number of partitions
  rDisStep = 25; // Discretization step (m)
  rEstDistD = 0.0;  // Estimated distance nest-source
  m_UnPartChos = 0; // Partition chosen 
  for(int i = 0; i < 99; i++){
    rsetL[i] = 400.0;
    CRange<Real> ProbRange(0.0f, 100.0f);
    //ProbRange(0.0f, 100.0f);
    rsetC[i] = m_pcRNG->Uniform(ProbRange);
  }
  rLastTripC = 0.0;// Cost associated with the last trip
  rTimeNav = 0.0; // Time spent navigating
  rTimeGripp = 0.0;// Time spent gripping
  rTimeExp = 0.0; // Time spent exploring

  m_UnTotalTimeExplore = 0;  // Time exploring
  m_UnTotalTimeNest = 0;      // Time nest
  m_UnTotalTimeSource = 0;  // Time source
  m_UnTotalTimeWaiting = 0; // Time waiting

  m_UnItemsFound = 0; // Objects found
  m_UnItemsLost = 0; // Objects lost
  /* Go directly to the nest on the first run for CPS */
  if(m_sPartitionParams.Partition == SPartitionParams::COST_PARTITIONING){
    bGoToNest = true;
  }
  else{
    bGoToNest = false;
  }
}



/*
 * Control step
 */
void PsiSwarmDPS::ControlStep()
{
  switch(m_sStateData.State) {
    case SStateData::STATE_EXPLORE: {
      Explore();
      break;
    }
    case SStateData::STATE_GO_TO_NEST: {
      GoToNest();
      break;
    }
    case SStateData::STATE_GO_TO_SOURCE: {
      GoToSource();
      break;
    }
    case SStateData::STATE_WAIT_FOR_TRANSFER: {
      WaitForTransfer();
      break;
    }
    default: {
       LOGERR << "Controller: State!" << std::endl;
       break;
    }
  }
}

/*
 * Executes the exploring state.
 */
void PsiSwarmDPS::Explore()
{
  m_UnTotalTimeExplore++;
  if(UITimeRested > STATESWITCHTIME){
    Move();
    rTimeExp++;
  }
  else{
    wheels_actuator->SetLinearVelocity(0.0, 0.0);
    UITimeRested++;
    rTimeGripp++;
  }
  
  if(bObjectFound){
    cActualPosition.Set(0.0, 0.0);
    cActualOrientation.SetValue(0.0);
    UITimeRested = 0;
    UIModelErrorTimer = 0;
    if(m_sPartitionParams.Partition == SPartitionParams::COST_PARTITIONING){
      PartitionCalculator(true);
    }
    m_sStateData.State = SStateData::STATE_GO_TO_NEST;
  }
}

/*
 * Executes the go to nest state.
 */
void PsiSwarmDPS::GoToNest(){
  m_UnTotalTimeNest++;
  if(UITimeRested > STATESWITCHTIME){
    Move();
    rTimeNav++;
  }
  else{
    wheels_actuator->SetLinearVelocity(0.0, 0.0);
    UITimeRested++;
    rTimeGripp++;
  }

  if(!bObjectFound){
    UITimeRested = 0;
    UIModelErrorTimer = 0;
    bGoToNest = false;
    cPreviousPosition = cActualPosition;
    cNestSourceDistance = cActualPosition; // To find maximum length for CPS
    m_sStateData.State = SStateData::STATE_GO_TO_SOURCE;
  }
  /* If partitiong length reached change to wait state */
  else if(cActualPosition.Length() > rPartitionLength && !bGoToNest){
    UITimeRested = 0;
    UIModelErrorTimer = 0;
    wheels_actuator->SetLinearVelocity(0.0, 0.0);
    m_sStateData.State = SStateData::STATE_WAIT_FOR_TRANSFER; 
  }
}

/*
 * Executes the go to source state.
 */
void PsiSwarmDPS::GoToSource(){
  m_UnTotalTimeSource++;
  /* Rest */
  if(UITimeRested > STATESWITCHTIME){
    Move();
    rTimeNav++;
    /* If robot gets nearer reset counter */  
    if(cPreviousPosition.Length() > cActualPosition.Length()){
      UIPositionNotReachedTimer = 0;
      cPreviousPosition = cActualPosition;
    }
    else{
      UIPositionNotReachedTimer++;  
    }
  }
  else{
    wheels_actuator->SetLinearVelocity(0.0, 0.0);
    UITimeRested++;
    rTimeGripp++;
  }
  
  /* If object is found */
  if(bObjectFound){
    cActualPosition.Set(0.0, 0.0);
    cActualOrientation.SetValue(0.0);
    UITimeRested = 0;
    UIModelErrorTimer = 0;
    UIPositionNotReachedTimer = 0;
    PartitionCalculator(bObjectFound);
    m_UnItemsFound++;
    m_sStateData.State = SStateData::STATE_GO_TO_NEST;
  }
  /* If position reached or timer is reached */
  else if((cActualPosition.GetX() < 2.5 && cActualPosition.GetY() > -2.5 &&
    cActualPosition.GetY() < 2.5 && cActualPosition.GetY() > -2.5)
    || (UIPositionNotReachedTimer > 200)){

    bObjectFound = false; 
    cActualPosition.Set(0.0, 0.0);
    cActualOrientation.SetValue(0.0); 
    UITimeRested = 0;
    UIModelErrorTimer = 0;
    UIPositionNotReachedTimer = 0;
    m_UnItemsLost++;
    PartitionCalculator(false);
    m_sStateData.State = SStateData::STATE_EXPLORE;
  }

}

/*
 * Wait for transfer state.
 */
void PsiSwarmDPS::WaitForTransfer()
{
  m_UnTotalTimeWaiting;
  UIWaitingTimer++;
  rTimeGripp++;
  /* If the object was handed over to another robot */
  if(!bObjectFound){
    UIWaitingTimer = 0;
    cPreviousPosition = cActualPosition;
    m_sStateData.State = SStateData::STATE_GO_TO_SOURCE;
  }
  else if(UIWaitingTimer > 600){
    UIWaitingTimer = 0;
    bGoToNest = true;
    m_sStateData.State = SStateData::STATE_GO_TO_NEST;
  }
}

/*
 * Partition calculator.
 */
void PsiSwarmDPS::PartitionCalculator(bool bObjectFound){
  switch(m_sPartitionParams.Partition){
    // Nonpartitioning strategy
    case(SPartitionParams::NON_PARTITIONING):{
      rPartitionLength = 100000; 
      break;
    }
    // Static strategy
    case(SPartitionParams::STATIC_PARTITIONING):{
      rPartitionLength = m_sPartitionParams.rInitialPartitionLength;
      break;
    }
    // Dynamic strategy
    case(SPartitionParams::DYNAMIC_PARTITIONING):{
      // Reward
      if(bObjectFound){
        rPartitionLength += m_sPartitionParams.rGain * (100 - m_sPartitionParams.rAlpha) / 100;
      }
      // Penalize
      else{
        rPartitionLength -= m_sPartitionParams.rGain * m_sPartitionParams.rAlpha / 100;
        if(rPartitionLength < 20.0)
          rPartitionLength = 20.0;
      }
      break;
    }
    case(SPartitionParams::COST_PARTITIONING):{
      if(bObjectFound){ 
        Real rTotalDistance;
        rTotalDistance = Distance(cNestSourceDistance, CVector2(0, 0));
        // If understimates update distances
        if(rEstDistD < rTotalDistance){
          rEstDistD = rTotalDistance;
          rNumElemN = floor(rEstDistD / rDisStep);
          for(int i = 0; i < rNumElemN; i++){
            rsetL[i] = rDisStep * (i + 1);
          }  
        }
        // Update costs
        rLastTripC = (rEstDistD / rsetL[m_UnPartChos]) * (rTimeNav +
          rTimeGripp) + rTimeExp;
          rsetC[m_UnPartChos] = (1.0 - (m_sPartitionParams.rMemFacAl / 100)) * rsetC[m_UnPartChos] +
          (m_sPartitionParams.rMemFacAl / 100) * rLastTripC;
        // Find min cost
        CRange<Real> ProbRange(0.0f, 100.0f); // TODO: Is this necessary?
        if(m_pcRNG->Uniform(ProbRange) > m_sPartitionParams.rGrFacEps){
          Real rMinCost = rsetC[0];
          for(int i = 0; i < rNumElemN; i++){
            if(rsetC[i] <= rMinCost){
              rMinCost = rsetC[i];
              m_UnPartChos = i;
            } 
          }
        }
        else{
          m_UnPartChos = floor((m_pcRNG->Uniform(ProbRange) / 100.0) * rNumElemN); 
        }
        // Define lower limit
        rPartitionLength = rsetL[m_UnPartChos];
        
        if(rPartitionLength < 20.0) //TODO: Is this necessary?
          rPartitionLength = 20.0;

        // Reset timers
        rTimeNav = 0.0;
        rTimeGripp = 0.0;
        rTimeExp = 0.0;
      }
      break;
    }
    default:{
      LOGERR << "Controller: Wrong partition!" << std::endl;
      break;
    }
  }
}

/*
 * Executes the exploring state.
 */
void PsiSwarmDPS::Move()
{
  CVector2 cMotorsSpeed(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);
  CDegrees cHeadingAngle(0.0);
  CVector2 cPositionObstacle;
  
  cPositionObstacle = ObstacleAvoidance();

  /* Avoid deadlocks */
  if(rHistory[0] == rHistory[1] && rHistory[1] == rHistory[2] && rHistory[0] == rHistory[2]){
    CRange<Real> cAngleRange(0,20.0);
    Real rRandAngle = m_pcRNG->Uniform(cAngleRange);
    if(cPositionObstacle.Length() > 0.1){ // TODO Parse parameter
      cHeadingAngle.SetValue(ToDegrees(cPositionObstacle.Angle()).GetValue()+180.0);
      cHeadingAngle.SignedNormalize();
    }
    cHeadingAngle = UpdateAngle(cHeadingAngle);
  }
  else{
    if(cPositionObstacle.Length() > 0.1){
      cHeadingAngle.SetValue(ToDegrees(cPositionObstacle.Angle()).GetValue()+180.0);
      cHeadingAngle.SignedNormalize();
    }
  }
  rHistory[2] = rHistory[1];
  rHistory[1] = rHistory[0];
  rHistory[0] = cPositionObstacle.Length();

  /* Select avoidance turn type */
  if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) 
    /* No Turn, heading angle very small */
    m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;

  else if(Abs(cHeadingAngle) < m_sWheelTurningParams.SoftTurnOnAngleThreshold) 
    /* Hard Turn, heading angle very large */
    m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;

  else 
    /* Soft Turn, heading angle in between the two cases */
    m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;

  /* Assign speed to motors */
  switch(m_sWheelTurningParams.TurningMechanism) {

    /* Move straight*/
    case SWheelTurningParams::NO_TURN: {
      break;
    }

    /* Move straight with a slight curvature in the trajectory */
    case SWheelTurningParams::SOFT_TURN: {
      break;
    }

    /* Turn in the same axis */
    case SWheelTurningParams::HARD_TURN: {
      UIModelErrorTimer = 0;
      /* Opposite wheel speeds */
      if(ToRadians(cHeadingAngle) >= CRadians::ZERO)
        cMotorsSpeed.SetX(cMotorsSpeed.GetX() * -1);

      else 
        cMotorsSpeed.SetY(cMotorsSpeed.GetY() * -1);

      break;
    }
    default: {
      LOGERR << "Controller: Movement!" << std::endl;
      break;
    }
  }
  Odometry(cMotorsSpeed.GetX(), cMotorsSpeed.GetY());
  cMotorsSpeed = AddingError(cMotorsSpeed);
  wheels_actuator->SetLinearVelocity(cMotorsSpeed.GetX(), cMotorsSpeed.GetY());
}

/*
 * Executes the obstacle avoidance.
 */
CVector2 PsiSwarmDPS::ObstacleAvoidance()
{
  /* Get readings from proximity sensor */
  const CCI_PsiSwarmProximitySensor::TReadings& tProxReads = proximity_sensor->GetReadings();
  /* Sum them together */
  CVector2 cDiffusionVector;
  /* Use front sensors only */
  for(size_t i = 0; i < 8 && i < tProxReads.size(); ++i) {
    cDiffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
  }

  return cDiffusionVector;
}

/* 
 * Actual position
 */
void PsiSwarmDPS::Odometry(Real rLeftMotorSpeed, Real rRightMotorSpeed){

  if(rLeftMotorSpeed != rRightMotorSpeed){ // Velocities are different  
    CRadians cPreviousOrientation(cActualOrientation);
    Real rPart1 = rRightMotorSpeed - rLeftMotorSpeed;
    Real rPart2 = 10.3 * (rRightMotorSpeed + rLeftMotorSpeed) / 2 * (rPart1);  
    cActualOrientation.SetValue(rPart1 * (0.1)  / 10.3 + cActualOrientation.GetValue());
    cActualOrientation.SignedNormalize();
    cActualPosition.SetX(cActualPosition.GetX() + rPart2 * 0.01 * (Sin(cActualOrientation) - Sin(cPreviousOrientation)));
    cActualPosition.SetY(cActualPosition.GetY() - rPart2 * 0.01 * (Cos(cActualOrientation) - Cos(cPreviousOrientation)));
  }
  else{ // If velocities are the same move in a straight line
    cActualPosition.SetX(cActualPosition.GetX() + rLeftMotorSpeed * Cos(cActualOrientation) * 0.1);
    cActualPosition.SetY(cActualPosition.GetY() + rRightMotorSpeed * Sin(cActualOrientation) * 0.1);
  }
}

/*
 * Update target angle according to state.
  */ 
 CDegrees PsiSwarmDPS::UpdateAngle(CDegrees cPrevAngle){

  CDegrees cHeadingAngle = cPrevAngle;

  switch(m_sStateData.State){
    
    /* Do nothing */
    case SStateData::STATE_EXPLORE:{
      break;
    }

    /* Follow beacon */
    case SStateData::STATE_GO_TO_NEST:{
      cHeadingAngle -= cNestAngle * 0.65;
      break;
    }

    /* Follow odometry */
    case SStateData::STATE_GO_TO_SOURCE:{
      CDegrees cTargetAngle = ToDegrees(ATan2(-cActualPosition.GetY(), -cActualPosition.GetX())) - ToDegrees(cActualOrientation);
      cHeadingAngle += cTargetAngle * 0.75;
      cHeadingAngle.SignedNormalize();
      break;
    }
    default: {
       LOGERR << "Controller angle: State!" << std::endl;
       break;
    }
  }
  return cHeadingAngle;
 }

CVector2 PsiSwarmDPS::AddingError(CVector2 cIdealSpeed){
  Real rError;
  Real rRandomError;
  UInt64 UITime;
  CVector2 cMotorsSpeed;

  /* Calculate time according to ticks */
  UITime = UIModelErrorTimer / 10;
  UIModelErrorTimer++;

  /* Specific error for each robot */
  /* Homogeneous or heterogeneous error? */
  if(m_sPartitionParams.UIErrorType == 0){
    UIRobotID = 99;
  }
  switch(UIRobotID){
    case 0:{ // Robot 0 04_92CB 
        /* Error function */
        rError = sqrt((3.32501*pow(10,31) - 8.61887*pow(10,28)*UITime + 9.94018*pow(10,26)*pow(UITime,2)
        - 1.80579*pow(10,24)*pow(UITime,3))/ 
        pow((146116.0 - 94.6991*UITime + pow(UITime,2)),7));
        /* Generate new random error according to normal distribution */
        rRandomError = m_pcRNG->Gaussian(0.0075, rError);
        /* No distribution */
        //      rRightMotor = rRightMotor - (rError * 23.5/2);
        //      rLeftMotor = rLeftMotor + (rError * 23.5/2);
        /* Distribution */
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 15.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 15.0/2));
        break;
      }
      case 1:{ // Robot 1 12_92C8
        rError = sqrt((3.66662*pow(10,29) - 1.33945*pow(10,27)*UITime + 2.36576*pow(10,25)*pow(UITime,2)
          - 6.09076*pow(10,22)*pow(UITime,3))/ 
          pow((67207.5 - 61.379*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0125, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 15.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 15.0/2));
        break;
      }
      case 2:{ // Robot 2 13_9309 
        rError = sqrt((4.45199*pow(10,32) - 5.08245*pow(10,29)*UITime + 8.4685*pow(10,27)*pow(UITime,2)
          - 7.10592*pow(10,24)*pow(UITime,3))/ 
          pow((215830.0 - 61.5986*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.01, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 20.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 20.0/2));
        break;
      }
      case 3:{ // Robot 3  15_7620
        rError = sqrt((1.87322*pow(10,27) - 1.19143*pow(10,25)*UITime + 3.00896*pow(10,23)*pow(UITime,2)
          - 1.32992*pow(10,21)*pow(UITime,3))/ 
          pow((27498.8 - 43.7256*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.015, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 30.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 30.0/2));
        break;
      }
      case 4:{ // Robot 4 18_7839
        rError = sqrt((5.63348*pow(10,25) - 1.27054*pow(10,23)*UITime + 1.51125*pow(10,22)*pow(UITime,2)
          - 2.54215*pow(10,19)*pow(UITime,3))/ 
          pow((15017.5 - 8.46739*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0175, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX()  - (rRandomError * 40.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY()  + (rRandomError * 40.0/2));
        break;
      } 
      case 5:{ // Robot 5 19_2F54
        rError = sqrt((7.16416*pow(10,35) + 6.222*pow(10,32)*UITime + 4.08277*pow(10,30)*pow(UITime,2)
          + 2.55672*pow(10,27)*pow(UITime,3))/ 
          pow((738549.0 + 160.355*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.005, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 70.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 70.0/2));
        break;
      }
      case 6:{ // Robot 6 01_92C3
        rError = sqrt((7.10237*pow(10,27) - 2.97429*pow(10,25)*UITime + 8.63794*pow(10,23)*pow(UITime,2)
        - 2.59891*pow(10,21)*pow(UITime,3))/ 
        pow((34769.3 - 36.1012*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0125, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 30.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 30.0/2));
        break;
      }
      case 7:{ // Robot 7 03_92D4
        rError = sqrt((5.33055*pow(10,40) + 1.0991*pow(10,37)*UITime + 4.48769*pow(10,34)*pow(UITime,2)
        + 6.83762*pow(10,30)*pow(UITime,3))/
        pow((4.84297*pow(10,6) + 249.642*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.025, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 40.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 40.0/2));
        break;
      }
      case 8:{ // Robot 8 05_92E4
        rError = sqrt((2.23408*pow(10,34) - 1.40936*pow(10,31)*UITime + 2.13276*pow(10,29)*pow(UITime,2)
        - 9.96822*pow(10,25)*pow(UITime,3))/ 
        pow((425654.0 - 67.1309*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.015, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 30.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 30.0/2));
        break;
      }
      case 9:{ // Robot 9 06_92FB
        rError = sqrt((1.90702*pow(10,31) - 1.25031*pow(10,28)*UITime + 6.04638*pow(10,26)*pow(UITime,2)
        - 2.96142*pow(10,23)*pow(UITime,3))/ 
        pow((126804.0 - 20.7844*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0175, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 35.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 35.0/2));
        break;
      }
      case 10:{ // Robot 10 07_9327
        rError = sqrt((9.69612*pow(10,38) + 1.9077*pow(10,36)*UITime + 2.84179*pow(10,33)*pow(UITime,2)
        + 2.5779*pow(10,30)*pow(UITime,3))/ 
        pow((2.70421*pow(10,6) + 1330.12*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.02, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 40.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 40.0/2));
        break;
      }
      case 11:{ // Robot 11 08_931E
        rError = sqrt((9.3383*pow(10,36) + 2.60704*pow(10,33)*UITime + 3.28151*pow(10,31)*pow(UITime,2)
        + 6.82647*pow(10,27)*pow(UITime,3))/ 
        pow((1.14784*pow(10,6) + 80.1127*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0175, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 40.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 40.0/2));
        break;
      }
      case 12:{ // Robot 12 10_A679
        rError = sqrt((3.93326*pow(10,45) - 3.13416*pow(10,41)*UITime + 5.15092*pow(10,38)*pow(UITime,2)
        - 3.03479*pow(10,34)*pow(UITime,3))/ 
        pow((3.11098*pow(10,7) - 619.735*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.0175, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * 35.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * 35.0/2));
        break;
      }
      case 13:{ // Robot 13 20_9238
       rError = sqrt((8.81573*pow(10,41) - 1.65101*pow(10,38)*UITime + 4.5607*pow(10,35)*pow(UITime,2)
        - 6.2794*pow(10,31)*pow(UITime,3))/ 
        pow((7.93346*pow(10,6) - 371.444*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.015, rError);
        cMotorsSpeed.SetX(cIdealSpeed.GetX() - (rRandomError * 40.0/2));
        cMotorsSpeed.SetY(cIdealSpeed.GetY() + (rRandomError * 40.0/2));
        break;
      }
      /* All distributions*/
      case 99:{
        rError = sqrt((3.93368*pow(10,35) - 3.69848*pow(10,32)*UITime + 2.45064*pow(10,30)*pow(UITime,2) 
        -1.65656*pow(10,27)*pow(UITime,2))/
        pow((678152.0 - 159.401*UITime + pow(UITime,2)),7));
        rRandomError = m_pcRNG->Gaussian(0.075, rError); // Was 0.07
        cMotorsSpeed.SetX(cIdealSpeed.GetX() + (rRandomError * m_sWheelTurningParams.MaxSpeed * 20 /2)) ;
        cMotorsSpeed.SetY(cIdealSpeed.GetY() - (rRandomError * m_sWheelTurningParams.MaxSpeed * 20 /2));
        break;
      }

      default: {
        LOGERR << "Controller noise: State!" << std::endl;
        break;
      }
  }
  return  cMotorsSpeed;
}
REGISTER_CONTROLLER(PsiSwarmDPS, "psiswarm_dps")
