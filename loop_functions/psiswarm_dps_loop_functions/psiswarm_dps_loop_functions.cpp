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

#include "psiswarm_dps_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/psi-swarm/simulator/psiswarm_entity.h>
#include <controllers/psiswarm_dps/psiswarm_dps.h>
#include <argos3/core/simulator/entity/floor_entity.h>

CDPSLoopFunctions::CDPSLoopFunctions() :  
 m_pcFloor(NULL),
 m_pcRNG(NULL){}

/****************************************/
/****************************************/

void CDPSLoopFunctions::Init(TConfigurationNode& t_node) {
   try {
  TConfigurationNode& tDPS = GetNode(t_node, "dps");
  /*
   * For 'output' file
   * - Get the output file name from XML
   * - Open the file and erase its contents
   * - Write header row
   */
  GetNodeAttribute(tDPS, "individual", m_strIndividual);
  m_cIndividual.open(m_strIndividual.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_cIndividual << "Clock, Items, ID, State, XPos, YPos, Angle, Partition, Explore, Nest, Source, Waiting, ExploreRate, NestRate, SourceRate, WaitingRate, ItemsFound, ItemsLost, ItemsFoundRate, ItemsLostRate" << std::endl;
  GetNodeAttribute(tDPS, "global", m_strGlobal);
  m_cGlobal.open(m_strGlobal.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_cGlobal << "Clock, Items, mPartitionLength, ExploreTime, NestTime, SourceTime, WaitingTime, ExploreRate, NestRate, SourceRate, WaitingRate, ItemsFound, ItemsLost, ItemsFoundRate, ItemsLostRate" << std::endl;

  GetNodeAttribute(tDPS, "transfer_type", UITransferType);
  GetNodeAttribute(tDPS, "home_source_distance", rHomeSourceDistance);
  /* Create a new RNG */
  m_pcRNG = CRandom::CreateRNG("argos");
  /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
   }
   catch(CARGoSException& ex) {
  THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
   }
   rItemsCounter = 0;
   for(int i=0; i < 100; i++){
      rItemsPosition[i][0] = 1000;
      rItemsPosition[i][1] = 1000;
      rItemsPosition[i][2] = 99;
   }
}

/****************************************/
/****************************************/

void CDPSLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void CDPSLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

CColor CDPSLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) { 
  /* Draw home region */  
  Real rUpperLimit = rHomeSourceDistance / 2.0;
  Real rLowerLimit = rHomeSourceDistance / 2.0 - 0.15;
  if(c_position_on_plane.GetX() > rLowerLimit && c_position_on_plane.GetX() < rUpperLimit 
    && c_position_on_plane.GetY() < 0.25 && -0.25 < c_position_on_plane.GetY()) {
    return CColor::RED;
  }
  if(c_position_on_plane.GetX() < -rLowerLimit && c_position_on_plane.GetX() > -rUpperLimit 
    && c_position_on_plane.GetY() < 0.25 && -0.25 < c_position_on_plane.GetY()) {
    return CColor::BLUE;
  }

   /* Draw positions of the objects */
  for(int i=0; i < 100; i++){
    if((c_position_on_plane - CVector2(rItemsPosition[i][0], rItemsPosition[i][1])).SquareLength() < 0.002
      && rItemsPosition[i][0] != 99){
      return CColor::BLACK;
    }
  }
  return CColor::WHITE;
}

/****************************************/
/****************************************/

void CDPSLoopFunctions::PreStep() {

  /* Analyse all Psiswarms in arena */
  CSpace::TMapPerType& m_cPsiswarms = GetSpace().GetEntitiesByType("psiswarm");
  Real rTempRobotID = 0;

  /* Direct transfer mechanism */
  if(UITransferType == 0){
    for(CSpace::TMapPerType::iterator it_i = m_cPsiswarms.begin();
        it_i != m_cPsiswarms.end();
        ++it_i) {
      for(CSpace::TMapPerType::iterator it_j = m_cPsiswarms.begin();
          it_j != m_cPsiswarms.end();
          ++it_j) {
      
        /* Get handle to foot-bot entity*/
        CPsiSwarmEntity& cPsiSwarm_i = *any_cast<CPsiSwarmEntity*>(it_i->second);
        CPsiSwarmEntity& cPsiSwarm_j = *any_cast<CPsiSwarmEntity*>(it_j->second);
      
        /* Get handle to food foot-bot controller */
        PsiSwarmDPS& cController_i = 
          dynamic_cast<PsiSwarmDPS&>(cPsiSwarm_i.GetControllableEntity().GetController());
        PsiSwarmDPS& cController_j = 
          dynamic_cast<PsiSwarmDPS&>(cPsiSwarm_j.GetControllableEntity().GetController());
      
        /* Get the position of the psi-swarm on the ground as a CVector2 */
        CVector2 cActualPosition_i;
        CVector2 cActualPosition_j;
        cActualPosition_i.Set(cPsiSwarm_i.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          cPsiSwarm_i.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        cActualPosition_j.Set(cPsiSwarm_j.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          cPsiSwarm_j.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      
        /* Calculate distance between vectors. */
        Real rDistance = Distance(cActualPosition_i, cActualPosition_j);
        if(rDistance < 0.15 ){
          if((cController_i.m_sStateData.State == SStateData::STATE_EXPLORE 
            && cController_j.m_sStateData.State == SStateData::STATE_WAIT_FOR_TRANSFER) || 
            (cController_i.m_sStateData.State == SStateData::STATE_GO_TO_SOURCE 
            && cController_j.m_sStateData.State == SStateData::STATE_WAIT_FOR_TRANSFER)){
            
            cController_i.bObjectFound = true;
            cController_j.bObjectFound = false;
          }
        }
      }
    } 
  }
  
  /* Indirect transfer mechanism */
  if(UITransferType == 1){
    for(CSpace::TMapPerType::iterator it = m_cPsiswarms.begin();
          it != m_cPsiswarms.end();
          ++it) {
      
      /* Get handle to foot-bot entity*/
      CPsiSwarmEntity& cPsiSwarm = *any_cast<CPsiSwarmEntity*>(it->second);
      
      /* Get handle to food foot-bot controller */
      PsiSwarmDPS& cController = dynamic_cast<PsiSwarmDPS&>(cPsiSwarm.GetControllableEntity().GetController());
          
      /* Get the position of the psi-swarm on the ground as a CVector2 */
      CVector2 cActualPosition;
      cActualPosition.Set(cPsiSwarm.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
          cPsiSwarm.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
      
      /* Calculate distance between vectors. */
      for(int i=0; i < 100; i++){
        if(rItemsPosition[i][2] != 99 && rItemsPosition[i][2] != rTempRobotID){
          Real rDistance = Distance(cActualPosition, CVector2(rItemsPosition[i][0], rItemsPosition[i][1])); 
          if(rDistance < 0.15){
            if(cController.m_sStateData.State == SStateData::STATE_EXPLORE || 
              cController.m_sStateData.State == SStateData::STATE_GO_TO_SOURCE){ // Prevent collecting the same item
              cController.bObjectFound = true;
              rItemsPosition[i][0] = 1000;
                rItemsPosition[i][1] = 1000;
              rItemsPosition[i][2] = 99;
            }
          }
        }
      }
      rTempRobotID++;
    }
  }
  
  /* Manage states */
  rTempRobotID = 0;
  for(CSpace::TMapPerType::iterator it = m_cPsiswarms.begin();
       it != m_cPsiswarms.end();
       ++it) {

    /* Get handle to foot-bot entity*/
    CPsiSwarmEntity& cPsiSwarm = *any_cast<CPsiSwarmEntity*>(it->second);
    
    /* Get handle to food foot-bot controller */
    PsiSwarmDPS& cController = dynamic_cast<PsiSwarmDPS&>(cPsiSwarm.GetControllableEntity().GetController());
    
    /* Get the position of the psi-swarm on the ground as a CVector2 */
    CVector2 cActualPosition;
    cActualPosition.Set(cPsiSwarm.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
        cPsiSwarm.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    /* Get orientation of the psi-swarm on the ground as a CDegrees */
    CRadians cEulerAngles[3];
    CDegrees cActualOrientation;
    CQuaternion cActualQuaternion;
    cActualQuaternion = cPsiSwarm.GetEmbodiedEntity().GetOriginAnchor().Orientation;
    cActualQuaternion.ToEulerAngles(cEulerAngles[2], cEulerAngles[1], cEulerAngles[0]);
    cActualOrientation =  ToDegrees(cEulerAngles[2]);
    
    /* Send messages according to each state */
    switch(cController.m_sStateData.State){
      case SStateData::STATE_EXPLORE: {
        cController.bObjectFound = Explore(cActualPosition, cController.bObjectFound);
        break;
      }
      case SStateData::STATE_GO_TO_NEST:{
        cController.bObjectFound = GoToNest(cActualPosition, cActualOrientation, &cController.cNestAngle);
        break;
      }
      case SStateData::STATE_GO_TO_SOURCE:{
        cController.bObjectFound = GoToSource(cActualPosition, cController.bObjectFound);
        break;
      }
      case SStateData::STATE_WAIT_FOR_TRANSFER:{
        if(UITransferType == 1){
          cController.bObjectFound =WaitForTransfer(cActualPosition, rTempRobotID);  
        }
        
        break;
      }
      default: {
        LOGERR << "Loop function: State!" << std::endl;
        break;
      }
    }
    /* Sample actual conditions of each individual every time interval */
    if(GetSpace().GetSimulationClock() % 6000 == 0 ){ 
      Real rExploreRate= 0, rNestRate = 0, rSourceRate = 0, rWaitingRate = 0;
      Real rItemsFoundRate = 0, rItemsLostRate = 0;
      Real rTotalTime = 0, rTotalItems = 0;
      rTotalTime = cController.m_UnTotalTimeExplore + cController.m_UnTotalTimeNest + 
       cController.m_UnTotalTimeSource +  cController.m_UnTotalTimeWaiting;
      rTotalItems =  cController.m_UnItemsFound + cController.m_UnItemsLost;

      rExploreRate = cController.m_UnTotalTimeExplore / rTotalTime;
      rNestRate = cController.m_UnTotalTimeNest / rTotalTime;
      rSourceRate = cController.m_UnTotalTimeSource / rTotalTime;
      rWaitingRate = cController.m_UnTotalTimeWaiting / rTotalTime;
      rItemsFoundRate = cController.m_UnItemsFound / rTotalItems;
      rItemsLostRate = cController.m_UnItemsLost / rTotalItems;
      /* Output stuff to file */
      m_cIndividual << GetSpace().GetSimulationClock() << "," 
            << rItemsCounter << "," << rTempRobotID << ","
            << cController.m_sStateData.State << "," << cActualPosition.GetX() << ","
            << cActualPosition.GetY() << "," << cActualOrientation.GetValue() << ","
            << cController.rPartitionLength << "," << cController.m_UnTotalTimeExplore << ","
            << cController.m_UnTotalTimeNest << "," << cController.m_UnTotalTimeSource << ","
            << cController.m_UnTotalTimeWaiting << "," << rExploreRate << ","
            << rNestRate << "," << rSourceRate << ","
            << rWaitingRate << "," << cController.m_UnItemsFound << ","
            << cController.m_UnItemsLost << "," << rItemsFoundRate << ","
            << rItemsLostRate << ","     
            << std::endl;
    }
    
    rTempRobotID++;
    m_pcFloor->SetChanged();
  }
  /* Sample actual conditions of the swarm every time interval */
  if(GetSpace().GetSimulationClock() % 6000 == 0 ){ 
    rTempRobotID = 0;
    std::vector<Real> vPartitionLength;
    Real rTotalExploreTime = 0, rTotalNestTime = 0, rTotalSourceTime = 0, rTotalWaitingTime = 0;
    Real rTotalItemsFound = 0, rTotalItemsLost = 0, rExploreRate= 0, rNestRate = 0;
    Real rSourceRate = 0, rWaitingRate = 0, rItemsFoundRate = 0, rItemsLostRate = 0;

    for(CSpace::TMapPerType::iterator it = m_cPsiswarms.begin();
       it != m_cPsiswarms.end();
       ++it) {
      CPsiSwarmEntity& cPsiSwarm = *any_cast<CPsiSwarmEntity*>(it->second);
      PsiSwarmDPS& cController = dynamic_cast<PsiSwarmDPS&>(cPsiSwarm.GetControllableEntity().GetController());
      vPartitionLength.push_back(cController.rPartitionLength);
      rTotalExploreTime += cController.m_UnTotalTimeExplore;
      rTotalNestTime += cController.m_UnTotalTimeNest;
      rTotalSourceTime += cController.m_UnTotalTimeSource;
      rTotalWaitingTime += cController.m_UnTotalTimeWaiting;
      rTotalItemsFound += cController.m_UnItemsFound;
      rTotalItemsLost += cController.m_UnItemsLost;
      rTempRobotID++;
    }
    rExploreRate = rTotalExploreTime / (rTotalExploreTime + rTotalNestTime + rTotalSourceTime + rTotalWaitingTime);
    rNestRate = rTotalNestTime / (rTotalExploreTime + rTotalNestTime + rTotalSourceTime + rTotalWaitingTime);
    rSourceRate = rTotalSourceTime / (rTotalExploreTime + rTotalNestTime + rTotalSourceTime + rTotalWaitingTime);
    rWaitingRate = rTotalWaitingTime / (rTotalExploreTime + rTotalNestTime + rTotalSourceTime + rTotalWaitingTime);
    if(rTotalItemsFound == rTotalItemsLost){
      rItemsFoundRate = 0;
      rItemsFoundRate = 1;
    }
    else{
      rItemsFoundRate = rTotalItemsFound / (rTotalItemsFound + rTotalItemsLost);
      rItemsLostRate = rTotalItemsLost / (rTotalItemsFound + rTotalItemsLost);  
    }

    m_cGlobal << GetSpace().GetSimulationClock() << "," 
          << rItemsCounter << ","
          << median(vPartitionLength) << ","
          << rTotalExploreTime << "," << rTotalNestTime << ","
          << rTotalSourceTime << "," << rTotalWaitingTime << ","
          << rExploreRate << "," << rNestRate << ","
          << rSourceRate << "," << rWaitingRate << ","
          << rTotalItemsFound << "," << rTotalItemsLost << ","
          << rItemsFoundRate << "," << rItemsLostRate << ","
          << std::endl;
  }
}

/*
 * Executes the exploring state.
 */
bool CDPSLoopFunctions::Explore(CVector2 cActualPosition, bool bObjectFound){

  bool bObjectFoundRet;
  Real rUpperLimit = rHomeSourceDistance / 2.0;
  Real rLowerLimit = rHomeSourceDistance / 2.0 - 0.15;
  /* If robot is in items source */ 
  if((cActualPosition.GetX() < -rLowerLimit && cActualPosition.GetX() > -rUpperLimit &&
    cActualPosition.GetY() < 0.25 && cActualPosition.GetY() > -0.25) || bObjectFound){
    bObjectFoundRet = true;
  }
  else{
    bObjectFoundRet = false;
  }

  return bObjectFoundRet;
}

/*
 * Executes the go to nest.
 */
bool CDPSLoopFunctions::GoToNest(CVector2 cActualPosition, CDegrees cActualOrientation, CDegrees* cNestAngle){

  bool bObjectFoundRet;
  Real rUpperLimit = rHomeSourceDistance / 2.0;
  Real rLowerLimit = rHomeSourceDistance / 2.0 - 0.15;

  /* Calculate relative angle of the psi-swarm to the nest and calculate difference with actual angle */
  CDegrees cNestRelAngle;
  cNestRelAngle = ToDegrees(ATan2(0.0 - cActualPosition.GetY(), rUpperLimit - cActualPosition.GetX()));
  *cNestAngle = cActualOrientation - cNestRelAngle;

  /* If robot is in nest */ 
  if(cActualPosition.GetX() > rLowerLimit && cActualPosition.GetX() < rUpperLimit &&
    cActualPosition.GetY() < 0.25 && cActualPosition.GetY() > -0.25){
    bObjectFoundRet = false;
    rItemsCounter++;
  }
  else{
    bObjectFoundRet = true;
  }

  return bObjectFoundRet;
}

/*
 * Executes the go to source state.
 */
bool CDPSLoopFunctions::GoToSource(CVector2 cActualPosition, bool bObjectFound){
  bool bObjectFoundRet;
  Real rUpperLimit = rHomeSourceDistance / 2.0;
  Real rLowerLimit = rHomeSourceDistance / 2.0 - 0.15;

  /* If robot is in items source */ 
  if((cActualPosition.GetX() < -rLowerLimit && cActualPosition.GetX() > -rUpperLimit &&
    cActualPosition.GetY() < 0.25 && cActualPosition.GetY() > -0.25) || bObjectFound){
    bObjectFoundRet = true;
  }
  else{
    bObjectFoundRet = false;
  }

  return bObjectFoundRet;
}

/*
 * Wait for transfer state.
 */
bool CDPSLoopFunctions::WaitForTransfer(CVector2 cActualPosition, Real rID){
  
  for(int i=0; i < 100; i++){
    if(rItemsPosition[i][2] == 99){
      rItemsPosition[i][0] = cActualPosition.GetX();
      rItemsPosition[i][1] = cActualPosition.GetY();
      rItemsPosition[i][2] = rID;
      break;
    }
  }
  return false;
}

Real CDPSLoopFunctions::median(std::vector<Real> &v)
{
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin()+n, v.end());
  int vn = v[n];
  if(v.size()%2 == 1)
  {
    return vn;
  }else
  {
    std::nth_element(v.begin(), v.begin()+n-1, v.end());
    return 0.5*(vn+v[n-1]);
  }
}

REGISTER_LOOP_FUNCTIONS(CDPSLoopFunctions, "psiswarm_dps_loop_functions")
