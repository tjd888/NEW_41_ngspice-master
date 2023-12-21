#include "ngspice/cm.h"
extern void cm_d_osc(Mif_Private_t *);
/* XSPICE code model for the Controlled Digital Oscillator.
 * This is a complete redesign of the original version by the
 * Georgia Tech team, as a fix for ngspice Bug #629 - "XSPICE d_osc failures".
 */

#include <stdlib.h>

#define FACTOR 0.75     // Controls timing of next scheduled call. */

/* PWL table entry. */

struct pwl {
    double ctl, freq;
};

/* Called at end to free memory. */

static void cm_d_osc_callback(Mif_Private_t *mif_private, Mif_Callback_Reason_t reason)
{
    struct panel_instance *instance;

    if (reason == MIF_CB_DESTROY) {
        struct pwl *table = mif_private->inst_var[0]->element[0].pvalue;

        if (table)
            free(table);
        mif_private->inst_var[0]->element[0].pvalue = NULL;
    }
}

/* Get the current period. */

static double get_period(double ctl, struct pwl *table, int csize)
{
    double f;
    int    i;

    for (i = 0; i < csize; ++i) {
         if (table[i].ctl > ctl)
             break;
    }

    /* Interpolation outside input range continues slope. */

    if (i > 0) {
       if (i == csize)
           i -= 2;
       else
           i--;
    }
    f = table[i].freq +
            (ctl - table[i].ctl) * ((table[i + 1].freq - table[i].freq) /
                                    (table[i + 1].ctl - table[i].ctl));
     return 1.0 / f;
}

/* The state data. */

struct state {
    double          last_time; // Time of last output change.
    Digital_State_t last;      // Last value output.
};

/* The code-model function. */

void cm_d_osc(Mif_Private_t *mif_private)
{
    struct pwl   *table;
    struct state *state;
    double        ctl, delta, when;
    int           csize, i;

    *(mif_private->callback) = cm_d_osc_callback;

    csize = mif_private->param[0]->size;
    if (mif_private->circuit.init) {

        /* Validate PWL table. */

        for (i = 0; i < csize - 1; ++i) {
            if (mif_private->param[0]->element[i].rvalue >=   mif_private->param[0]->element[i+1].rvalue)
                break;
        }

        if (i < csize - 1 || csize != mif_private->param[1]->size) {
            cm_message_send("Badly-formed control table");
            mif_private->inst_var[0]->element[0].pvalue = NULL;
            return;
        }

        /* Allocate PWL table. */

        table = malloc(csize * sizeof (struct pwl));
        mif_private->inst_var[0]->element[0].pvalue = table;
        if (!table)
            return;

        for (i = 0; i < csize; ++i) {
            table[i].ctl = mif_private->param[0]->element[i].rvalue;
            table[i].freq = mif_private->param[1]->element[i].rvalue;
            if (table[i].freq <= 0) {
                cm_message_printf("Error: frequency %g is not positve, "
                                  "value replaced by 1e-16.",
                                  table[i].freq);
                table[i].freq = 1.0e-16;
            }
        }

        /* Allocate state data. */

        cm_event_alloc(0, sizeof (struct state));
        return;
    }

    table = mif_private->inst_var[0]->element[0].pvalue;
    if (!table)
         return;
    state = (struct state *)cm_event_get_ptr(0, 0);

    if (mif_private->circuit.call_type != EVENT) {
        if (mif_private->circuit.time == 0.0) {
            double phase;

            /* Set initial output and state data. */

            ctl = mif_private->conn[0]->port[0]->input.rvalue;
            delta = get_period(ctl, table, csize);

            phase = mif_private->param[3]->element[0].rvalue;
            phase /= 360.0;
            if (phase < 0.0)
                phase += 1.0;

            /* When would a hypothetical previous transition have been? */

            state->last_time = delta * (1.0 - mif_private->param[2]->element[0].rvalue - phase);
            if (state->last_time < 0.0) {
                state->last = ONE;
            } else {
                state->last = ZERO;
                state->last_time = -delta * phase;
            }
        }
        return;
    }

    /* Event call; either one requested previously or just before
     * a time-step is accepted.
     */

     if (mif_private->circuit.time == 0.0) {
         ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->state = state->last;
         ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->strength = STRONG;
         return;
     }

     /* When is the next transition due? */

     ctl = mif_private->conn[0]->port[0]->input.rvalue;
     delta = get_period(ctl, table, csize);
     if (state->last)
         delta *= mif_private->param[2]->element[0].rvalue;
     else
         delta *= (1.0 - mif_private->param[2]->element[0].rvalue);
     when = state->last_time + delta;

     if (mif_private->circuit.time >= when) {
         /* If the frequency rose rapidly, the transition has been missed.
          * Force a shorter time-step and schedule then.
          */

         cm_analog_set_temp_bkpt(state->last_time + FACTOR * delta);
         mif_private->conn[1]->port[0]->changed = FALSE;
         return;
     }

     if (mif_private->circuit.time >= state->last_time + FACTOR * delta) {
         /* TIME is reasonably close to transition time.  Request output. */

         state->last_time = when;
         state->last ^= ONE;
         ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->state = state->last;
         ((Digital_t*)(mif_private->conn[1]->port[0]->output.pvalue))->strength = STRONG;
         mif_private->conn[1]->port[0]->delay = when - mif_private->circuit.time;

         /* Request a call in the next half-cycle. */

         cm_event_queue(when + FACTOR * delta);
     } else {
         mif_private->conn[1]->port[0]->changed = FALSE;

         if (mif_private->circuit.time < state->last_time) {
             /* Output transition pending, nothing to do. */

             return;
         } else {
             /* Request a call nearer to transition time. */

             cm_event_queue(state->last_time + FACTOR * delta);
         }
     }
}
