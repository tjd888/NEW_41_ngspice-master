#include "ngspice/cm.h"
extern void cm_d_xnor(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE d_xnor/cfunc.mod

Public Domain

Georgia Tech Research Corporation
Atlanta, Georgia 30332
PROJECT A-8503-405

AUTHORS

    18 Jun 1991     Jeffrey P. Murray

MODIFICATIONS

     7 Aug 1991    Jeffrey P. Murray
     2 Oct 1991    Jeffrey P. Murray

SUMMARY

    This file contains the model-specific routines used to
    functionally describe the d_xnor code model.

INTERFACES

    FILE                 ROUTINE CALLED

    CMutil.c             void cm_toggle_bit();

    CMevt.c              void *cm_event_alloc()
                         void *cm_event_get_ptr()

REFERENCED FILES

    Inputs from and outputs to ARGS structure.

NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== INCLUDE FILES ====================*/

#include "ngspice/inertial.h"

/*=== CONSTANTS ========================*/


/*=== MACROS ===========================*/


/*=== LOCAL VARIABLES & TYPEDEFS =======*/


/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/


/*==============================================================================

FUNCTION cm_toggle_bit()

AUTHORS

    27 Sept 1991     Jeffrey P. Murray

MODIFICATIONS

    NONE

SUMMARY

    Alters the state of a passed digital variable to its
    complement. Thus, a ONE changes to a ZERO. A ZERO changes
    to a ONE, and an UNKNOWN remains unchanged.

INTERFACES

    FILE                 ROUTINE CALLED

    N/A                  N/A

RETURNED VALUE

    No returned value. Passed pointer to variable is used
    to redefine the variable value.

GLOBAL VARIABLES

    NONE

NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== CM_TOGGLE_BIT ROUTINE ===*/

static void cm_toggle_bit(Digital_State_t *bit)
{
    /* Toggle bit from ONE to ZERO or vice versa, unless the
       bit value is UNKNOWN. In the latter case, return
       without changing the bit value.                      */

    if  (UNKNOWN != *bit ) {
        if  (ONE == *bit ) {
            *bit = ZERO;
        }
        else {
            *bit = ONE;
        }
    }
}


/*==============================================================================

FUNCTION cm_d_xnor()

AUTHORS

    18 Jun 1991     Jeffrey P. Murray

MODIFICATIONS

     7 Aug 1991     Jeffrey P. Murray
     2 Oct 1991     Jeffrey P. Murray

SUMMARY

    This function implements the d_xnor code model.

INTERFACES

    FILE                 ROUTINE CALLED

    CMutil.c             void cm_toggle_bit();

    CMevt.c              void *cm_event_alloc()
                         void *cm_event_get_ptr()

RETURNED VALUE

    Returns inputs and outputs via ARGS structure.

GLOBAL VARIABLES

    NONE

NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== CM_D_XNOR ROUTINE ===*/

/************************************************
*      The following is the model for the       *
*   digital XNOR gate for the                   *
*   ATESSE Version 2.0 system.                  *
*                                               *
*   Created 6/18/91              J.P.Murray     *
************************************************/

void cm_d_xnor(Mif_Private_t *mif_private)
{
    int                    i,   /* generic loop counter index */
                        size;   /* number of input & output ports */



    Digital_State_t      val,
                        *out,   /* temporary output for buffers */
                       input;   /* temp storage for input bits  */


    /** Retrieve size value... **/
    size = mif_private->conn[0]->size;

    /*** Setup required state variables ***/

    if(mif_private->circuit.init) {  /* initial pass */
        /* allocate storage for the outputs */

        cm_event_alloc(0, sizeof (Digital_State_t));

        /* Inertial delay? */

        mif_private->inst_var[0]->element[0].bvalue =
            cm_is_inertial(mif_private->param[4]->is_null ? Not_set :
                         mif_private->param[4]->element[0].bvalue);
        if (mif_private->inst_var[0]->element[0].bvalue) {
            /* Allocate storage for event time. */

            cm_event_alloc(1, sizeof (struct idata));
            ((struct idata *)cm_event_get_ptr(1, 0))->when = -1.0;
        }

        /* Prepare initial output. */

        out = (Digital_State_t *)cm_event_get_ptr(0, 0);
        *out = (Digital_State_t)(UNKNOWN + 1); // Force initial output.

        for (i=0; i<size; i++) mif_private->conn[0]->port[i]->load = mif_private->param[2]->element[0].rvalue;
    } else {      /* Retrieve previous values */
        /* retrieve storage for the outputs */

        out = (Digital_State_t *) cm_event_get_ptr(0,0);
    }

    /*** Calculate new output value based on inputs ***/

    val = ONE;
    for (i=0; i<size; i++) {
        /* if a 1, toggle bit value */
        if  (ONE == (input = ((Digital_t*)(mif_private->conn[0]->port[i]->input.pvalue))->state) ) {
            cm_toggle_bit(&val);
        } else {
            /* if an unknown input, set val to unknown & break */
            if  (UNKNOWN == input ) {
                val = UNKNOWN;
                break;
            }
        }
    }

    /*** Check for change and output appropriate values ***/

    if (val == *out) { /* output value is not changing */
        mif_private->conn[1]->port[0]->changed = FALSE;
    } else {
        switch (val) {

            /* fall to zero value */
        case 0:
            mif_private->conn[1]->port[0]->delay = mif_private->param[1]->element[0].rvalue;
            break;

            /* rise to one value */
        case 1:
            mif_private->conn[1]->port[0]->delay = mif_private->param[0]->element[0].rvalue;
            break;

            /* unknown output */
        default:
            /* based on old value, add rise or fall delay */
            if (0 == *out) {  /* add rising delay */
                mif_private->conn[1]->port[0]->delay = mif_private->param[0]->element[0].rvalue;
            } else {                /* add falling delay */
                mif_private->conn[1]->port[0]->delay = mif_private->param[1]->element[0].rvalue;
            }
            break;
        }

        if (mif_private->inst_var[0]->element[0].bvalue && mif_private->circuit.anal_type == TRANSIENT) {
            struct idata *idp;

            idp = (struct idata *)cm_event_get_ptr(1, 0);
            if (idp->when <= mif_private->circuit.time) {
                /* Normal transition. */

                idp->prev = *out;
                idp->when = mif_private->circuit.time + mif_private->conn[1]->port[0]->delay; // Actual output time
            } else if (val != idp->prev) {
                Digital_t ov = {idp->prev, STRONG};

                /* Third value: cancel earlier change and output as usual. */

                cm_schedule_output(1, 0, (idp->when - mif_private->circuit.time) / 2.0, &ov);
		if (val == UNKNOWN) {
                    /* Delay based in idp->prev, not *out. */

                    if (idp->prev == ZERO)
                        mif_private->conn[1]->port[0]->delay = mif_private->param[0]->element[0].rvalue;
                    else
                        mif_private->conn[1]->port[0]->delay = mif_private->param[1]->element[0].rvalue;
                }
                idp->when = mif_private->circuit.time + mif_private->conn[1]->port[0]->delay; // Actual output time
            } else {
                /* Changing back: override pending change. */

                mif_private->conn[1]->port[0]->delay = (idp->when - mif_private->circuit.time) / 2.0; // Override
		idp->when = -1.0;
            }
        }
        *out = val;
        ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->state = val;
        ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->strength = STRONG;
    }
}
