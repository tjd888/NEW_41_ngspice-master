#include "ngspice/cm.h"
extern void cm_d_tristate(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE d_tristate/cfunc.mod

Public Domain

Georgia Tech Research Corporation
Atlanta, Georgia 30332
PROJECT A-8503-405

AUTHORS

    18 Nov 1991     Jeffrey P. Murray

MODIFICATIONS

    26 Nov 1991    Jeffrey P. Murray

SUMMARY

    This file contains the functional description of the d_tristate
    code model.

INTERFACES

    FILE                 ROUTINE CALLED

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

FUNCTION cm_d_tristate()

AUTHORS

    18 Nov 1991     Jeffrey P. Murray

MODIFICATIONS

    26 Nov 1991     Jeffrey P. Murray

SUMMARY

    This function implements the d_tristate code model.

INTERFACES

    FILE                 ROUTINE CALLED

    CMevt.c              void *cm_event_alloc()
                         void *cm_event_get_ptr()

RETURNED VALUE
    
    Returns inputs and outputs via ARGS structure.

GLOBAL VARIABLES

    NONE

NON-STANDARD FEATURES

    NONE

==============================================================================*/

/*=== CM_D_TRISTATE ROUTINE ===*/

/************************************************
*      The following is a model for a simple    *
*   digital tristate for the ATESSE Version     *
*   2.0 system. Note that this version has      *
*   a single delay for both input and enable... *
*   a more realistic model is anticipated in    *
*   the not-so-distant future.                  *
*                                               *
*   Created 11/18/91              J.P,Murray    *
*   Last Modified 11/26/91                      *
************************************************/

#define DEBUG 0
#if DEBUG
const char * const Image[] = {"Idle", "Normal", "Same", "Revert", "Both"};
#endif

void cm_d_tristate(Mif_Private_t *mif_private) 
{
    Digital_State_t     val;
    Digital_Strength_t  str;
    Digital_t          *out;
    struct idata       *idp;

    if (mif_private->circuit.init) {  /* initial pass */ 
        /* allocate storage for the outputs */

        cm_event_alloc(0, sizeof (Digital_t));

        /* Inertial delay? */

        mif_private->inst_var[0]->element[0].bvalue =
            cm_is_inertial(mif_private->param[3]->is_null ? Not_set :
                         mif_private->param[3]->element[0].bvalue);
        if (mif_private->inst_var[0]->element[0].bvalue) {
            /* Allocate storage for event times. A little rude,
             * as strength values will be stored in idp[1].prev.
             */

            cm_event_alloc(1, 2 * sizeof (struct idata));
            idp = (struct idata *)cm_event_get_ptr(1, 0);
            idp[1].when = idp[0].when = -1.0;
        }

        out = (Digital_t *) cm_event_get_ptr(0,0);
        out->state = (Digital_State_t)(UNKNOWN + 1); // Force initial output.

        /* define input loading... */
        mif_private->conn[0]->port[0]->load = mif_private->param[1]->element[0].rvalue;
        mif_private->conn[1]->port[0]->load = mif_private->param[2]->element[0].rvalue;
	mif_private->conn[2]->port[0]->delay = mif_private->param[0]->element[0].rvalue; // Never changes, unless inertial.
    } else {
        out = (Digital_t *)cm_event_get_ptr(0, 0);
    }

    /* Retrieve input values. */

    val = ((Digital_t*)(mif_private->conn[0]->port[0]->input.pvalue))->state;
    switch (((Digital_t*)(mif_private->conn[1]->port[0]->input.pvalue))->state) {
    case ZERO:
        str = HI_IMPEDANCE;
	break;
    case ONE:
        str = STRONG;
	break;
    default:
        str = UNDETERMINED;
	break;
    }

    /*** Check for change and output appropriate values ***/

    if (val == out->state && str == out->strength) { /* output not changing */
        mif_private->conn[2]->port[0]->changed = FALSE;
    } else {
        if (mif_private->inst_var[0]->element[0].bvalue && mif_private->circuit.anal_type == TRANSIENT) {
            /* Each channel (State, Strength) of the output has three values,
             * that set by the input, the current node value and its value
             * following a pending change. The channel is in one of
             * five states:
             *  Idle - no new value and no pending output;
             *  Normal - there is a new value with no pending output;
             *  Same - no new value, there is pending output;
             *  Revert - new value same as current, conflicting pending output;
             *  Both - new value differs from both current and pending.
             */

            enum {Idle, Normal, Same, Revert, Both}
                          d_ctl, s_ctl, ctl1, ctl2;
            double        first_time, second_time; /* Scheduled changes */
            double        delay_1;                 /* Delay to first output. */
            double        cancel_delay;            /* Delay to canclling. */
            int           d_first;
            Digital_t     reversion, restoration;

            idp = (struct idata *)cm_event_get_ptr(1, 0);

            /* Combine two independent streams (state and strength) into
             * a sequence of output events.  Earlier changes cancel later ones
             * that may need to be restored.
             */

            /* Identify earlier change. */

            if (idp[0].when <= idp[1].when) {
                first_time = idp[0].when;
                second_time = idp[1].when;
                d_first = 1;
            } else {
                first_time = idp[1].when;
                second_time = idp[0].when;
                d_first = 0;
            }

            /* What happens to state? */

            mif_private->conn[2]->port[0]->delay = mif_private->param[0]->element[0].rvalue;
            if (idp[0].when <= mif_private->circuit.time) {
                if (val == out->state) {
                    d_ctl = Idle;      /* Output is stable and no change. */
                    idp[0].prev = val;
                } else {
                    d_ctl = Normal;    /* Output was stable, changing now. */
                    idp[0].prev = out->state;
                    idp[0].when = mif_private->circuit.time + mif_private->conn[2]->port[0]->delay;
                }
            } else {
                if (val == out->state) {
                    d_ctl = Same;      /* Output pending, no change. */
                } else if (val == idp[0].prev) {
                    d_ctl = Revert;    /* Returning to previous state. */
                    idp[0].when = -1.0;
                } else {
                    d_ctl = Both;      /* Output pending, now changing. */
                    idp[0].when = mif_private->circuit.time + mif_private->conn[2]->port[0]->delay;
                }
            }

            /* Strength? */

            if (idp[1].when <= mif_private->circuit.time) {
                if (str == out->strength) {
                    s_ctl = Idle;
                    idp[1].prev = str;
                } else {
                    s_ctl = Normal;
                    idp[1].prev = out->strength;
                    idp[1].when = mif_private->circuit.time + mif_private->conn[2]->port[0]->delay;
                }
            } else {
                if (str == out->strength) {
                    s_ctl = Same;
                } else if (str == (Digital_Strength_t)idp[1].prev) {
                    s_ctl = Revert;
                    idp[1].when = -1.0;
                } else {
                    s_ctl = Both;
                    idp[1].when = mif_private->circuit.time + mif_private->conn[2]->port[0]->delay;
                }
            }

            if (d_first) {
                ctl1 = d_ctl;
                ctl2 = s_ctl;
            } else {
                ctl1 = s_ctl;
                ctl2 = d_ctl;
            }
#if DEBUG
            cm_message_printf("%g: %s first, "
                              "state ctl %s %d->%d->%d @ %g, "
                              "strength ctl %s %d->%d->%d @ %g",
                              mif_private->circuit.time, d_first ? "state" : "strength",
                              Image[d_ctl], idp[0].prev, out->state, val,
                              idp[0].when,
                              Image[s_ctl], idp[1].prev, out->strength, str,
                              idp[1].when);
#endif
            switch (ctl1) {
            case Idle:
                switch (ctl2) {
                default:
                    break;
                case Revert:
                    /* Normal output is used to revert. */

                    delay_1 = (second_time - mif_private->circuit.time) / 2.0;
direct_revert:
                    if (d_first) {
                        str = (Digital_Strength_t)idp[1].prev;
                    } else {
                        val = idp[0].prev;
                    }
                    mif_private->conn[2]->port[0]->delay = delay_1;
                    break;

                case Both:
                    /* Push out reversion before normal output. */

                    cancel_delay = (second_time - mif_private->circuit.time) / 2.0;
push_revert:
                    if (d_first) {
                        reversion.state = out->state;
                        reversion.strength = (Digital_Strength_t)idp[1].prev;
                    } else {
                        reversion.state = idp[0].prev;
                        reversion.strength = out->strength;
                    }
                    cm_schedule_output(2, 0, cancel_delay, &reversion);
                    break;
                }
                break;
            case Normal:
                switch (ctl2) {
                default:
                    break;
                case Revert:
                    /* Push out reversion before normal output. */

                    if (d_first) {
                        reversion.state = out->state;
                        reversion.strength = (Digital_Strength_t)idp[1].prev;
                        str = reversion.strength;
                    } else {
                        reversion.state = idp[0].prev;
                        val = reversion.state;
                        reversion.strength = out->strength;
                    }
                    cancel_delay = (second_time - mif_private->circuit.time) / 2.0;
                    cm_schedule_output(2, 0, cancel_delay, &reversion);
                    break;
                case Both:
                    /* Push out reversion before normal output. */

                    cancel_delay = (second_time - mif_private->circuit.time) / 2.0;
                    goto push_revert;
                    break;
                }
                break;
            case Same:
                switch (ctl2) {
                default:
                    break;
                case Revert:
                    /* Normal output is used to revert. */

                    delay_1 = (first_time + second_time) / 2.0 - mif_private->circuit.time;
                    goto direct_revert;
                    break;
                case Both:
                    /* Push out reversion before normal output. */

                    cancel_delay =  (first_time + second_time) / 2.0 - mif_private->circuit.time;
                    goto push_revert;
                    break;
                }
                break;
            case Revert:
                switch (ctl2) {
                default:
                    /* Ordinary reversion. */

                    delay_1 = (first_time  - mif_private->circuit.time) / 2.0;
                    d_first = !d_first;
                    goto direct_revert;
                    break;

                case Normal:
                    /* Push out state reversion before normal output. */

                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    d_first = !d_first;
                    goto push_revert;
                    break;

                case Same:
                    /* Set normal output time to restore scheduled output
                     * and push out reversion.
                     */

                    reversion.state = idp[0].prev;
                    reversion.strength = (Digital_Strength_t)idp[1].prev;
                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    cm_schedule_output(2, 0, cancel_delay, &reversion);
                    mif_private->conn[2]->port[0]->delay = second_time - mif_private->circuit.time;
                    break;

                case Revert:
                    /* Revert both together. */

                    val = idp[0].prev;
                    str = (Digital_Strength_t)idp[1].prev;
                    mif_private->conn[2]->port[0]->delay = (first_time - mif_private->circuit.time) / 2.0;
                    break;

                case Both:
                    /* Double revert with normal output. */

		    reversion.state = idp[0].prev;
                    reversion.strength = (Digital_Strength_t)idp[1].prev;
                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    cm_schedule_output(2, 0, cancel_delay, &reversion);

                    if (d_first) {
                        val = reversion.state;
                    } else {
                        str = reversion.strength;
                    }
                    break;
                }
                break;

            case Both:
                switch (ctl2) {
                default:
                    /* Push out state reversion before normal output. */

                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    d_first = !d_first;
                    goto push_revert;
                    break;

                case Same:
                    /* Push out reversion, then restore scheduled change,
                     * then normal output.
                     */

                    reversion.state = idp[0].prev;
                    reversion.strength = (Digital_Strength_t)idp[1].prev;
                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    cm_schedule_output(2, 0, cancel_delay, &reversion);

                    if (d_first) {
                        restoration.state = reversion.state;
                        restoration.strength = out->strength;
                    } else {
                        restoration.state = out->state;
                        restoration.strength = reversion.strength;
                    }
                    cm_schedule_output(2, 0, second_time - mif_private->circuit.time, &restoration);
                    break;

                case Revert:
                case Both:
                    /* Push out double reversion, then normal output. */

                    reversion.state = idp[0].prev;
                    reversion.strength = (Digital_Strength_t)idp[1].prev;
                    cancel_delay = (first_time - mif_private->circuit.time) / 2.0;
                    cm_schedule_output(2, 0, cancel_delay, &reversion);
                    break;
                }
            }
        }
	out->state = val;
	out->strength = str;
	*(Digital_t *)mif_private->conn[2]->port[0]->output.pvalue = *out;
    }
}
