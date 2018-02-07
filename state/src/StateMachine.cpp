//

// File: StateMachine.cpp
//
// Code generated for Simulink model 'StateMachine'.
//
// Model version                  : 1.18
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Feb  6 13:44:39 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "state/StateMachine.h"
#include "state/StateMachine_private.h"

// Named constants for Chart: '<Root>/StateMachine'
#define StateMachin_IN_GlobalLocationOK ((uint8_T)1U)
#define StateMachin_IN_GlobalLocationOn ((uint8_T)2U)
#define StateMachine_IN_Initialization ((uint8_T)3U)
#define StateMachine_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define StateMachine_IN_Ready          ((uint8_T)4U)
#define StateMachine_IN_SimuLocation   ((uint8_T)5U)

// Block states (auto storage)
DW_StateMachine_T StateMachine_DW;

// External inputs (root inport signals with auto storage)
ExtU_StateMachine_T StateMachine_U;

// External outputs (root outports fed by signals with auto storage)
ExtY_StateMachine_T StateMachine_Y;

// Real-time model
RT_MODEL_StateMachine_T StateMachine_M_;
RT_MODEL_StateMachine_T *const StateMachine_M = &StateMachine_M_;

// Model step function
void StateMachine_step(void)
{
  // Chart: '<Root>/StateMachine' incorporates:
  //   Inport: '<Root>/detect_node_state'
  //   Inport: '<Root>/lidar_state'
  //   Inport: '<Root>/nav_node_state'
  //   Inport: '<Root>/recv_msg_state'
  //   Inport: '<Root>/tcp_state'

  // Gateway: StateMachine
  // During: StateMachine
  if (StateMachine_DW.is_active_c3_StateMachine == 0U) {
    // Entry: StateMachine
    StateMachine_DW.is_active_c3_StateMachine = 1U;

    // Entry Internal: StateMachine
    // Transition: '<S1>:9'
    StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Initialization;

    // Outport: '<Root>/sys_state'
    // Entry 'Initialization': '<S1>:8'
    StateMachine_Y.sys_state = 0U;
  } else {
    switch (StateMachine_DW.is_c3_StateMachine) {
    case StateMachin_IN_GlobalLocationOK:

      // During 'GlobalLocationOK': '<S1>:12'
      if ((StateMachine_U.lidar_state == 0) || (StateMachine_U.tcp_state == 0))
      {
        // Transition: '<S1>:21'
        StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Initialization;

        // Outport: '<Root>/sys_state'
        // Entry 'Initialization': '<S1>:8'
        StateMachine_Y.sys_state = 0U;
      } else {
        if ((StateMachine_U.nav_node_state == 2) &&
            (StateMachine_U.recv_msg_state == 2)) {
          // Transition: '<S1>:22'
          StateMachine_DW.is_c3_StateMachine = StateMachine_IN_SimuLocation;

          // Outport: '<Root>/sys_state'
          // Entry 'SimuLocation': '<S1>:13'
          StateMachine_Y.sys_state = 4U;
        }
      }
      break;

    case StateMachin_IN_GlobalLocationOn:

      // During 'GlobalLocationOn': '<S1>:11'
      if ((StateMachine_U.lidar_state == 0) || (StateMachine_U.tcp_state == 0))
      {
        // Transition: '<S1>:18'
        StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Initialization;

        // Outport: '<Root>/sys_state'
        // Entry 'Initialization': '<S1>:8'
        StateMachine_Y.sys_state = 0U;
      } else {
        if (StateMachine_U.detect_node_state == 2) {
          // Transition: '<S1>:19'
          StateMachine_DW.is_c3_StateMachine = StateMachin_IN_GlobalLocationOK;

          // Outport: '<Root>/sys_state'
          // Entry 'GlobalLocationOK': '<S1>:12'
          StateMachine_Y.sys_state = 3U;
        }
      }
      break;

    case StateMachine_IN_Initialization:

      // During 'Initialization': '<S1>:8'
      if ((StateMachine_U.lidar_state == 1) && (StateMachine_U.tcp_state == 1) &&
          (StateMachine_U.detect_node_state == 0) &&
          (StateMachine_U.nav_node_state == 0) && (StateMachine_U.recv_msg_state
                                                   == 0)) {
        // Transition: '<S1>:15'
        StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Ready;

        // Outport: '<Root>/sys_state'
        // Entry 'Ready': '<S1>:10'
        StateMachine_Y.sys_state = 1U;
      }
      break;

    case StateMachine_IN_Ready:

      // During 'Ready': '<S1>:10'
      if ((StateMachine_U.lidar_state == 0) || (StateMachine_U.tcp_state == 0))
      {
        // Transition: '<S1>:16'
        StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Initialization;

        // Outport: '<Root>/sys_state'
        // Entry 'Initialization': '<S1>:8'
        StateMachine_Y.sys_state = 0U;
      } else {
        if ((StateMachine_U.detect_node_state == 0) &&
            (StateMachine_U.nav_node_state == 0) &&
            (StateMachine_U.recv_msg_state == 1)) {
          // Transition: '<S1>:17'
          StateMachine_DW.is_c3_StateMachine = StateMachin_IN_GlobalLocationOn;

          // Outport: '<Root>/sys_state'
          // Entry 'GlobalLocationOn': '<S1>:11'
          StateMachine_Y.sys_state = 2U;
        }
      }
      break;

    default:

      // During 'SimuLocation': '<S1>:13'
      if ((StateMachine_U.lidar_state == 0) || (StateMachine_U.tcp_state == 0))
      {
        // Transition: '<S1>:23'
        StateMachine_DW.is_c3_StateMachine = StateMachine_IN_Initialization;

        // Outport: '<Root>/sys_state'
        // Entry 'Initialization': '<S1>:8'
        StateMachine_Y.sys_state = 0U;
      } else {
        if (StateMachine_U.recv_msg_state == 1) {
          // Transition: '<S1>:26'
          StateMachine_DW.is_c3_StateMachine = StateMachin_IN_GlobalLocationOn;

          // Outport: '<Root>/sys_state'
          // Entry 'GlobalLocationOn': '<S1>:11'
          StateMachine_Y.sys_state = 2U;
        }
      }
      break;
    }
  }

  // End of Chart: '<Root>/StateMachine'
}

// Model initialize function
void StateMachine_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(StateMachine_M, (NULL));

  // states (dwork)
  (void)memset((void *)&StateMachine_DW, 0,
               sizeof(DW_StateMachine_T));

  // external inputs
  (void)memset((void *)&StateMachine_U, 0, sizeof(ExtU_StateMachine_T));

  // external outputs
  StateMachine_Y.sys_state = 0U;

  // SystemInitialize for Chart: '<Root>/StateMachine'
  StateMachine_DW.is_active_c3_StateMachine = 0U;
  StateMachine_DW.is_c3_StateMachine        = StateMachine_IN_NO_ACTIVE_CHILD;

  // SystemInitialize for Outport: '<Root>/sys_state' incorporates:
  //   SystemInitialize for Chart: '<Root>/StateMachine'

  StateMachine_Y.sys_state = 0U;
}

// Model terminate function
void StateMachine_terminate(void)
{
  // (no terminate code required)
}

//
// File trailer for generated code.
//
// [EOF]
//
