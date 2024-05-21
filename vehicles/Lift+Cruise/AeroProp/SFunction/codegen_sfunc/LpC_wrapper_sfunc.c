/*
 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 * Copyright 1990-2018 The MathWorks, Inc.
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  LpC_wrapper_sfunc
#define S_FUNCTION_LEVEL 2

/* Define the number of s-function inputs and outputs, size of B  */
#define num_Inputs 6
#define num_Outputs 6
#define num_props 9
#define num_surf 4
#define num_FM 6          /* Fx, Fy, Fz, Mx, My, Mz */
#define num_der_states 6  /* states = (u,v,w,p,q,r) */
#define num_der_inputs 42 /* wingprop = 1x3(fl,ail,tilt), tail = 1x3(ele,rud,tilt),
                             prop = 1x4(om,roll,pitch,yaw) per prop, NP = 9
                             so number of inputs = 3 + 3 + 4*9 = 42 */

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/* Include Files */
#include "LpC_wrapper.h"
#include "LpC_wrapper_terminate.h"
//#include "rt_nonfinite.h"


/* Error handling
 * --------------
 *
 * You should use the following technique to report errors encountered within
 * an S-function:
 *
 *       ssSetErrorStatus(S,"Error encountered due to ...");
 *       return;
 *
 * Note that the 2nd argument to ssSetErrorStatus must be persistent memory.
 * It cannot be a local variable. For example the following will cause
 * unpredictable errors:
 *
 *      mdlOutputs()
 *      {
 *         char msg[256];         {ILLEGAL: to fix use "static char msg[256];"}
 *         sprintf(msg,"Error due to %s", string);
 *         ssSetErrorStatus(S,msg);
 *         return;
 *      }
 *
 */

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, num_Inputs)) return;

    /* rho */
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    /* V_b */
    ssSetInputPortWidth(S, 1, 3);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortRequiredContiguous(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    /* om_b */
    ssSetInputPortWidth(S, 2, 3);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortRequiredContiguous(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    /* om_prop */
    ssSetInputPortWidth(S, 3, num_props);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortRequiredContiguous(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    /* surf */
    ssSetInputPortWidth(S, 4, num_surf);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortRequiredContiguous(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    /* ders */
    ssSetInputPortWidth(S, 5, 1);
    ssSetInputPortDataType(S, 5, SS_BOOLEAN);
    ssSetInputPortRequiredContiguous(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);

    if (!ssSetNumOutputPorts(S, num_Outputs)) return;
    /* FM */
    ssSetOutputPortWidth(S, 0, num_FM);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    /* FM_aero */
    ssSetOutputPortWidth(S, 1, num_FM);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    /* FM_prop */
    ssSetOutputPortWidth(S, 2, num_FM);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    /* Prop */
    DECL_AND_INIT_DIMSINFO(di);
    int_T dims[2];
    di.numDims = 2;
    dims[0] = 2;
    dims[1] = num_props;
    di.dims = dims;
    di.width = 2 * num_props;
    ssSetOutputPortDimensionInfo(S, 3, &di);
 //   ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortDataType(S, 3, SS_DOUBLE);

    /* FM_x */
    di.numDims = 2;
    dims[0] = num_FM;
    dims[1] = num_der_states;
    di.dims = dims;
    di.width = num_FM * num_der_states;
    ssSetOutputPortDimensionInfo(S, 4, &di);
    ssSetOutputPortDataType(S, 4, SS_DOUBLE);

    /* FM_u */
    di.numDims = 2;
    dims[0] = num_FM;
    dims[1] = num_der_inputs;
    di.dims = dims;
    di.width = num_FM * num_der_inputs;
    ssSetOutputPortDimensionInfo(S, 5, &di);
    ssSetOutputPortDataType(S, 5, SS_DOUBLE);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the operating point save/restore compliance to be same as a 
     * built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetOptions(S, 0);
}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}



#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#undef MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
  }
#endif /*  MDL_START */



/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
  /* get pointers to input signals */
  const real_T *rho     = (const real_T*)ssGetInputPortRealSignal(S, 0);
  const real_T *V_b     = (const real_T*)ssGetInputPortRealSignal(S, 1);
  const real_T *om_b    = (const real_T*)ssGetInputPortRealSignal(S, 2);
  const real_T *om_prop = (const real_T*)ssGetInputPortRealSignal(S, 3);
  const real_T *surf    = (const real_T*)ssGetInputPortRealSignal(S, 4);
  const boolean_T *ders = (const boolean_T*)ssGetInputPortSignal(S, 5);

  /* get pointers to output signals */
  real_T *FM      = ssGetOutputPortRealSignal (S, 0);
  real_T *FM_aero = ssGetOutputPortRealSignal (S, 1);
  real_T *FM_prop = ssGetOutputPortRealSignal (S, 2);
  real_T *Prop    = ssGetOutputPortRealSignal (S, 3);
  real_T *FM_x    = ssGetOutputPortRealSignal (S, 4);
  real_T *FM_u    = ssGetOutputPortRealSignal (S, 5);

  LpC_wrapper(*rho, V_b, om_b, om_prop, surf, *ders,
              FM, FM_aero, FM_prop, Prop, FM_x, FM_u);
}

#undef MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
  /* Function: mdlUpdate ======================================================
   * Abstract:
   *    This function is called once for every major integration time step.
   *    Discrete states are typically updated here, but this function is useful
   *    for performing any tasks that should only take place once per
   *    integration step.
   */
  static void mdlUpdate(SimStruct *S, int_T tid)
  {
  }
#endif /* MDL_UPDATE */



#undef MDL_DERIVATIVES  /* Change to #undef to remove function */
#if defined(MDL_DERIVATIVES)
  /* Function: mdlDerivatives =================================================
   * Abstract:
   *    In this function, you compute the S-function block's derivatives.
   *    The derivatives are placed in the derivative vector, ssGetdX(S).
   */
  static void mdlDerivatives(SimStruct *S)
  {
  }
#endif /* MDL_DERIVATIVES */



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
  LpC_wrapper_terminate();
}


/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
