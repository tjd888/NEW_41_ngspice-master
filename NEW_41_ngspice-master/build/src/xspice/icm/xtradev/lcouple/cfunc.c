#include "ngspice/cm.h"
extern void cm_lcouple(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE lcouple/cfunc.mod

Public Domain

Georgia Tech Research Corporation
Atlanta, Georgia 30332
PROJECT A-8503-405
               

AUTHORS                      

     6 Jun 1991     Jeffrey P. Murray


MODIFICATIONS   

    13 Sep 1991    Jeffrey P. Murray
     2 Oct 1991    Jeffrey P. Murray
                                   

SUMMARY

    This file contains the model-specific routines used to
    functionally describe the lcouple code model.


INTERFACES       

    FILE                 ROUTINE CALLED     

    CM.c                 void *cm_analog_alloc()
                         void *cm_analog_get_ptr()
                         int  cm_analog_integrate()


REFERENCED FILES

    Inputs from and outputs to ARGS structure.
                     

NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== INCLUDE FILES ====================*/


                                      

/*=== CONSTANTS ========================*/




/*=== MACROS ===========================*/



  
/*=== LOCAL VARIABLES & TYPEDEFS =======*/                         


    
           
/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/




                   
/*==============================================================================

FUNCTION void cm_lcouple()

AUTHORS                      

     6 Jun 1991     Jeffrey P. Murray

MODIFICATIONS   

    13 Sep 1991    Jeffrey P. Murray
     2 Oct 1991    Jeffrey P. Murray

SUMMARY

    This function implements the lcouple code model.

INTERFACES       

    FILE                 ROUTINE CALLED     

    CM.c                 void *cm_analog_alloc()
                         void *cm_analog_get_ptr()
                         int  cm_analog_integrate()

RETURNED VALUE
    
    Returns inputs and outputs via ARGS structure.

GLOBAL VARIABLES
    
    NONE

NON-STANDARD FEATURES

    NONE

==============================================================================*/

/*=== CM_LCOUPLE ROUTINE ===*/
                                                   
/***********************************************
*  Note that this model incorporates a fake    *
*  integration in order to link in truncation  *
*  error checking...this may be removed at a   *
*  future date.                 JPM 9/13/91    *
***********************************************/

                    

void cm_lcouple(Mif_Private_t *mif_private)  /* structure holding parms, 
                                       inputs, outputs, etc.     */
{
    double input_current,   /* input current from electrical-side windings */
          output_voltage,   /* output voltage reflected to electricaL-side */
    *output_voltage_fake,   /* fake output voltage for use with 
                               truncation error checking. */
              /*input_flux,*/   /* input flux value from core side (represented 
                               as a current. */
              output_mmf,   /* output driving amp-turns to core side.      */

               num_turns,   /* number of turns on inductor                 */
         /*pout_pin_fake,*/ /* fake partial derivative of output
                               w.r.t. input (for use with integration */
                *in_flux,   /* current input flux value from core side
                                  (represented as a current).              */
            *in_flux_old,   /* previous timestep flux value                */
           *in_flux_fake,   /* fake input flux value for use with 
                               truncation error checking. */
                   delta;   /* time delta from previous timepoint to
                                  current value                            */


    Mif_Complex_t ac_gain;  /* AC gain */

    /** Retrieve frequently used parameters... **/

    num_turns = mif_private->param[0]->element[0].rvalue;

    if (mif_private->circuit.anal_type != MIF_AC) {     /**** DC & Transient Analyses ****/

        /** Test for INIT; if so, allocate storage, otherwise, retrieve
                                   previous timepoint input value...     **/
    
        if (mif_private->circuit.init==1) {  /* First pass...allocate storage for previous state.   */
                        /* Also, calculate roughly where the current output    */
                        /* will be and use this value to define current state. */
    
            cm_analog_alloc(1,sizeof(double));   
            cm_analog_alloc(2,sizeof(double));   
            cm_analog_alloc(3,sizeof(double));   
        }
        /* Allocation not necessary...retrieve previous values */
       
        in_flux = (double *) cm_analog_get_ptr(1,0);  /* Set out pointer to current 
                                                                time storage */    
        in_flux_old = (double *) cm_analog_get_ptr(1,1);  /* Set old-output-state pointer 
                                                       to previous time storage */

        /* retrieve fake input and output values for truncation
           error checking   */
        in_flux_fake = (double *) cm_analog_get_ptr(2,0);
        output_voltage_fake = (double *) cm_analog_get_ptr(3,0);

        /** Retrieve inputs... **/

        input_current = mif_private->conn[0]->port[0]->input.rvalue;    /* input from electrical side is a current */
        *in_flux = -mif_private->conn[1]->port[0]->input.rvalue;  /* input from core side is a flux 
                                            represented as a current...note
                                            that a negative is introduced,
                                            because current INTO the positive
                                            node would normally result in 
                                            a NEGATIVE output_voltage...
                                            the minus sign corrects this. */


        /** Calculate output value for mmf... **/
         
        output_mmf = num_turns * input_current;

        mif_private->conn[1]->port[0]->output.rvalue = output_mmf;
        mif_private->conn[1]->port[0]->partial[0].port[0] = num_turns;


        /** Calculate output value for output_voltage... **/

        if  (0.0 == mif_private->circuit.time ) {                     /*** Test to see if this is the first ***/
                                                 /***    timepoint calculation...if    ***/
            *in_flux_old = *in_flux;             /***    so, return a zero d/dt value. ***/
            output_voltage = *output_voltage_fake = 0.0;                

            mif_private->conn[0]->port[0]->output.rvalue = output_voltage;
            mif_private->conn[0]->port[0]->partial[1].port[0] = 0.0;
        }
        else {               /*** Calculate value of d_dt.... ***/
            delta = mif_private->circuit.time - mif_private->circuit.t[1];
            output_voltage = *output_voltage_fake = 
                num_turns * (*in_flux - *in_flux_old) / delta;
            mif_private->conn[0]->port[0]->output.rvalue = output_voltage;
            mif_private->conn[0]->port[0]->partial[1].port[0] = -num_turns / delta;

            /* add fake cm_analog_integrate for truncation error checks */
            /* not initialized, not used */
            /*cm_analog_integrate(*output_voltage_fake,in_flux_fake,&pout_pin_fake); */
        }
    }

    else {                    /**** AC Analysis...****/
        ac_gain.real = 0.0;
        ac_gain.imag= num_turns * mif_private->circuit.frequency;
        mif_private->conn[0]->port[0]->ac_gain[1].port[0] = ac_gain;

        ac_gain.real= num_turns;
        ac_gain.imag= 0.0;
        mif_private->conn[1]->port[0]->ac_gain[0].port[0] = ac_gain;
    }
}




