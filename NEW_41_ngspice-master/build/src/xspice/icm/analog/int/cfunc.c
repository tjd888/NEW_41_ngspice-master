#include "ngspice/cm.h"
extern void cm_int(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE int/cfunc.mod

Public Domain

Georgia Tech Research Corporation
Atlanta, Georgia 30332
PROJECT A-8503-405
               

AUTHORS                      

    6 Nov 1991     Jeffrey P. Murray


MODIFICATIONS   

     2 Oct 1991    Jeffrey P. Murray
                                   

SUMMARY

    This file contains the model-specific routines used to
    functionally describe the int code model.


INTERFACES       

    FILE                 ROUTINE CALLED     

    CMutil.c             void cm_smooth_corner(); 

    CM.c                 void *cm_analog_alloc()
                         void *cm_analog_get_ptr()
                         int  cm_analog_integrate()


REFERENCED FILES

    Inputs from and outputs to ARGS structure.
                     

NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== INCLUDE FILES ====================*/

#include "int.h"   

                                      

/*=== CONSTANTS ========================*/




/*=== MACROS ===========================*/



  
/*=== LOCAL VARIABLES & TYPEDEFS =======*/                         


    
           
/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/




                   
/*==============================================================================

FUNCTION void cm_int()

AUTHORS                      

     2 Oct 1991     Jeffrey P. Murray

MODIFICATIONS   

    NONE

SUMMARY

    This function implements the int code model.

INTERFACES       

    FILE                 ROUTINE CALLED     

    CMutil.c             void cm_smooth_corner(); 

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

/*=== CM_INT ROUTINE ===*/

void cm_int(Mif_Private_t *mif_private)  /* structure holding parms, 
                                       inputs, outputs, etc.     */
{
    double        *out, /* current output   */
                   *in, /* input        */
             in_offset, /* input offset */
                  gain, /* gain parameter   */
       out_lower_limit, /* output lower limit   */
       out_upper_limit, /* output upper limit   */
           limit_range, /* range of output below out_upper_limit
                           and above out_lower_limit within which
                           smoothing will take place    */
                out_ic, /* output initial condition - initial output value  */
              pout_pin, /* partial derivative of output w.r.t. input    */
             pout_gain; /* temporary storage variable for partial
                           value returned by smoothing function
                           (subsequently multiplied by pout_pin)    */

    Mif_Complex_t ac_gain;  /* AC gain  */
                                                   


    /** Retrieve frequently used parameters (used by all analyses)... **/

    gain = mif_private->param[1]->element[0].rvalue;
                                     


    if (mif_private->circuit.anal_type != MIF_AC) {     /**** DC & Transient Analyses ****/

        /** Retrieve frequently used parameters... **/

        in_offset = mif_private->param[0]->element[0].rvalue;
        out_lower_limit = mif_private->param[2]->element[0].rvalue;
        out_upper_limit = mif_private->param[3]->element[0].rvalue;                         
        limit_range = mif_private->param[4]->element[0].rvalue;
        out_ic = mif_private->param[5]->element[0].rvalue;



        /** Test for INIT; if so, allocate storage, otherwise, retrieve
                                   previous timepoint input value...     **/

        if (mif_private->circuit.init==1) {  /* First pass...allocate storage for previous value.   */
    
            cm_analog_alloc(INT1,sizeof(double));   
            cm_analog_alloc(INT2,sizeof(double));   
        }
        /* retrieve previous value */
    
            in = (double *) cm_analog_get_ptr(INT1,0);  /* Set out pointer to input storage location */
            out = (double *) cm_analog_get_ptr(INT2,0);  /* Set out pointer to output storage location */
                                  

        /*** Read input value for current time, and calculate pseudo-input ***/
        /***    which includes input offset and gain....                   ***/

        *in = gain*(mif_private->conn[0]->port[0]->input.rvalue+in_offset);

        /*** Test to see if this is the first timepoint calculation... ***/
        /***   this would imply that TIME equals zero.                 ***/
    
        if  (0.0 == mif_private->circuit.time ) {     /*** Test to see if this is the first ***/
                                 /***    timepoint calculation...if    ***/
            *out = out_ic;       /***    so, return out_ic.            ***/
            pout_pin = 0.0;
        }
        else {               /*** Calculate value of integral.... ***/
            cm_analog_integrate(*in,out,&pout_pin);
        }


        /*** Smooth output if it is within limit_range of 
                 out_lower_limit or out_upper_limit.          ***/
                                                                  
        if (*out < (out_lower_limit - limit_range)) {  /* At lower limit. */ 
            *out = out_lower_limit;
            pout_pin = 0.0;
        }
        else {
            if (*out < (out_lower_limit + limit_range)) {  /* Lower smoothing range */
                cm_smooth_corner(*out,out_lower_limit,out_lower_limit,limit_range,
                            0.0,1.0,out,&pout_gain);
                pout_pin = pout_pin * pout_gain;
            }
            else {
                if (*out > (out_upper_limit + limit_range))  {  /* At upper limit */
                    *out = out_upper_limit;
                    pout_pin = 0.0;
                }
                else { 
                    if (*out > (out_upper_limit - limit_range))  {  /* Upper smoothing region */
                        cm_smooth_corner(*out,out_upper_limit,out_upper_limit,limit_range,
                                    1.0,0.0,out,&pout_gain); 
                        pout_pin = pout_pin * pout_gain;
                    }
                }   
            }
        }




        /** Output values for DC & Transient **/

        mif_private->conn[1]->port[0]->output.rvalue = *out;          
        mif_private->conn[1]->port[0]->partial[0].port[0] = pout_pin; 

    }

    else {                    /**** AC Analysis...output (0.0,gain/s) ****/
        ac_gain.real = 0.0;
        ac_gain.imag = -gain / mif_private->circuit.frequency;
        mif_private->conn[1]->port[0]->ac_gain[0].port[0] = ac_gain;
    }
}





