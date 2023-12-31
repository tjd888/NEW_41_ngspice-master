#include "ngspice/cm.h"
extern void cm_delay(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE delay/cfunc.mod

3-clause BSD

Copyright 2021
The ngspice team

AUTHORS

    12 June 2021     Holger Vogt


MODIFICATIONS



SUMMARY

    This file contains the functional description of the analog
    delay code model.


INTERFACES

    FILE                 ROUTINE CALLED

    CMutil.c             void cm_smooth_corner();

    CM.c                 void *cm_analog_alloc()
                         void *cm_analog_get_ptr()
                         int  cm_analog_integrate()
                              cm_delay_callback()



REFERENCED FILES

    Inputs from and outputs to ARGS structure.


NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== INCLUDE FILES ====================*/

#include <stdlib.h>


/*=== CONSTANTS ========================*/




/*=== MACROS ===========================*/




/*=== LOCAL VARIABLES & TYPEDEFS =======*/

typedef struct {

    double   *buffer;       /* the storage array for input values   */
    int       buffer_size;  /* size of buffer */
    int       buff_write;   /* buffer write index */
    int       buff_del;     /* buffer index, delayed */
    int       step_count;   /* steps processed */
    double    tdelmin;      /* min delay, if controlled */
    double    tdelmax;      /* max delay, if controlled */
    double    tdelay;       /* time delay */
    double    tstep;        /* tran step size */
    double    tstop;        /* tran stop value */
    double    tprev;        /* previous time value */
    double    prev_val;     /* previous data value */
    double    start_val;    /* signal time 0 value */
} mLocal_Data_t;


struct CKTcircuitmin {

/* This is a minimum re-definition of the circuit structure defined in
   cktdefs.h. We are interested in TSTEP and TSTOP */

    GENmodel **CKThead;
    STATistics *CKTstat;        /* The STATistics structure */
    double *CKTstates[8];       /* Used as memory of past steps ??? */
    double CKTtime;             /* Current transient simulation time */
    double CKTdelta;            /* next time step in transient simulation */
    double CKTdeltaOld[7];      /* Memory for the 7 most recent CKTdelta */
    double CKTtemp;             /* Actual temperature of CKT, initialzed to 300.15 K in cktinit.c*/
    double CKTnomTemp;          /* Reference temperature 300.15 K set in cktinit.c */
    double CKTvt;               /* Thernmal voltage at CKTtemp */
    double CKTag[7];            /* the gear variable coefficient matrix */
#ifdef PREDICTOR
    double CKTagp[7];           /* the gear predictor variable coefficient matrix */
#endif /*PREDICTOR*/
    int CKTorder;               /* the integration method order */
    int CKTmaxOrder;            /* maximum integration method order */
    int CKTintegrateMethod;     /* the integration method to be used */
    double CKTxmu;              /* for trapezoidal method */
    int CKTindverbosity;        /* control check of inductive couplings */
    SMPmatrix *CKTmatrix;       /* pointer to sparse matrix */
    int CKTniState;             /* internal state */
    double *CKTrhs;             /* current rhs value - being loaded */
    double *CKTrhsOld;          /* previous rhs value for convergence testing */
    double *CKTrhsSpare;        /* spare rhs value for reordering */
    double *CKTirhs;            /* current rhs value - being loaded imag) */
    double *CKTirhsOld;         /* previous rhs value (imaginary)*/
    double *CKTirhsSpare;       /* spare rhs value (imaginary)*/
#ifdef PREDICTOR
    double *CKTpred;            /* predicted solution vector */
    double *CKTsols[8];         /* previous 8 solutions */
#endif /* PREDICTOR */
    double *CKTrhsOp;           /* opearating point values */
    double *CKTsenRhs;          /* current sensitivity rhs values */
    double *CKTseniRhs;         /* current sensitivity rhs values (imag)*/
    int CKTmaxEqNum;            /* And this ? */
    int CKTcurrentAnalysis;     /* the analysis in progress (if any) */
    CKTnode *CKTnodes;          /* ??? */
    CKTnode *CKTlastNode;       /* ??? */
    CKTnode *prev_CKTlastNode;  /* just before model setup */
    int CKTnumStates;           /* Number of sates effectively valid ??? */
    long CKTmode;               /* Mode of operation of the circuit ??? */
    int CKTbypass;              /* bypass option, how does it work ?  */
    int CKTdcMaxIter;           /* iteration limit for dc op.  (itl1) */
    int CKTdcTrcvMaxIter;       /* iteration limit for dc tran. curv (itl2) */
    int CKTtranMaxIter;         /* iteration limit for each timepoint for tran*/
    int CKTbreakSize;           /* ??? */
    int CKTbreak;               /* ??? */
    double CKTsaveDelta;        /* ??? */
    double CKTminBreak;         /* ??? */
    double *CKTbreaks;          /* List of breakpoints ??? */
    double CKTabstol;           /* --- */
    double CKTpivotAbsTol;      /* --- */
    double CKTpivotRelTol;      /* --- */
    double CKTreltol;           /* --- */
    double CKTchgtol;           /* --- */
    double CKTvoltTol;          /* --- */
    /* What is this define for  ? */
#ifdef NEWTRUNC
    double CKTlteReltol;
    double CKTlteAbstol;
#endif /* NEWTRUNC */
    double CKTgmin;             /* .options GMIN */
    double CKTgshunt;           /* .options RSHUNT */
    double CKTcshunt;           /* .options CSHUNT */
    double CKTdelmin;           /* minimum time step for tran analysis */
    double CKTtrtol;            /* .options TRTOL */
    double CKTfinalTime;        /* TSTOP */
    double CKTstep;             /* TSTEP */
    double CKTmaxStep;          /* TMAX */
    double CKTinitTime;         /* TSTART */
    /* struct is truncated here */
};

typedef struct CKTcircuitmin CKTcircuitmin;

/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/

static void cm_delay_callback(Mif_Private_t *mif_private, Mif_Callback_Reason_t reason);

/*=== CM_DELAY ROUTINE ===*/

void cm_delay(Mif_Private_t *mif_private)
{
    int buffer_size, delay_step;
    double delay, lcntrl;
    double delmin, delmax;

    mLocal_Data_t *loc;        /* Pointer to local static data, not to be included
                                       in the state vector */
    CKTcircuitmin *ckt;


    if (mif_private->circuit.anal_type != MIF_AC) {     /**** only Transient Analysis and dc ****/

        /** INIT: allocate storage **/

        if (mif_private->circuit.init==1) {

            *(mif_private->callback) = cm_delay_callback;

            ckt = (CKTcircuitmin*)cm_get_circuit();

            if (mif_private->param[1]->is_null) {
            /* size depends on TSTOP/TSTEP, if no parameter given */
                buffer_size = (int) (ckt->CKTfinalTime / ckt->CKTstep) + 1;
            }
            else {
                buffer_size = mif_private->param[1]->element[0].ivalue;
                if (buffer_size < 0) {
                    cm_message_send("Negative buffer size is not allowed "
                    "in a delay code model");
                    return;
                }
            }

            delay = mif_private->param[0]->element[0].rvalue;
            if (delay < 0.0) {
                delay = 0.0;
                cm_message_send("Negative delay not allowed, set to 0");
            }

            /*** allocate static storage for *loc ***/
            if ((loc = (mLocal_Data_t *) (mif_private->inst_var[0]->element[0].pvalue = calloc(1,
                    sizeof(mLocal_Data_t)))) == (mLocal_Data_t *) NULL) {
                cm_message_send("Unable to allocate Local_Data_t "
                    "in cm_delay()");
                return;
            }
            /*** allocate static storage for the delay buffer ***/
            loc->buffer = (double *) calloc((size_t)buffer_size, sizeof(double));
            if (loc->buffer == (double *) NULL) {
                cm_message_send("Unable to allocate delay buffer "
                    "in cm_delay()");
                return;
            }
            loc->buffer_size = buffer_size;

            /* The delay is controlled by input delay_cnt */
            if (mif_private->param[2]->element[0].bvalue == MIF_TRUE) {
                if (mif_private->param[3]->is_null)
                    loc->tdelmin = 0.0;
                else {
                    loc->tdelmin = mif_private->param[3]->element[0].rvalue;
                    if (loc->tdelmin < 0) {
                        loc->tdelmin = 0.0;
                        cm_message_send("Negative min delay not allowed, set to 0");
                    }
                    else if (loc->tdelmin > ckt->CKTfinalTime) {
                        loc->tdelmin = ckt->CKTfinalTime;
                        cm_message_send("min delay greater than final sim time not allowed, set to final time");
                    }
                }

                if (mif_private->param[4]->is_null)
                    loc->tdelmax = ckt->CKTfinalTime;
                else {
                    loc->tdelmax = mif_private->param[4]->element[0].rvalue;
                    if (loc->tdelmax < 0) {
                        loc->tdelmin = 0.0;
                        cm_message_send("Negative max delay not allowed, set to 0");
                    }
                    else if (loc->tdelmax > ckt->CKTfinalTime) {
                        loc->tdelmax = ckt->CKTfinalTime;
                        cm_message_send("max delay greater than final sim time not allowed, set to final time");
                    }
                }
                if (loc->tdelmax < loc->tdelmin) {
                    loc->tdelmax = loc->tdelmin;
                    cm_message_send("max delay smaller than min delay, set to min delay");
                }
            }

            loc->buff_write = 0;
            loc->buff_del = 0;
            loc->step_count = 0;
            loc->tdelay = delay;
            loc->tstop = ckt->CKTfinalTime;
            loc->tstep = ckt->CKTstep;
            loc->tprev = 0.0;
            loc->prev_val = 0.0;
        }
        /* retrieve previous values */

        loc =  mif_private->inst_var[0]->element[0].pvalue;

        delay = loc->tdelay;
        if (delay == 0.0 && !mif_private->param[2]->element[0].bvalue) {
            mif_private->conn[1]->port[0]->output.rvalue = mif_private->conn[0]->port[0]->input.rvalue;
            return;
        }

        if (delay < loc->tstep) {
            delay = 0.;
        }

        if (mif_private->circuit.time == 0.0)
            loc->start_val = mif_private->conn[0]->port[0]->input.rvalue;

        /* input, simply interpolated to TSTEP        */
        double tacct = loc->step_count * loc->tstep;
        if (mif_private->circuit.time >= tacct) {
            loc->buffer[loc->buff_write] = mif_private->conn[0]->port[0]->input.rvalue;
            /* next buffer location, circular, for writing */
            loc->buff_write = (loc->buff_write + 1) % loc->buffer_size;
            loc->step_count++;
            loc->tprev = tacct;
            loc->prev_val = mif_private->conn[0]->port[0]->input.rvalue;
        }

        delmin = loc->tdelmin;
        delmax = loc->tdelmax;

        if (mif_private->param[2]->element[0].bvalue == MIF_TRUE) {
            if (!mif_private->conn[2]->is_null) {
                lcntrl =  mif_private->conn[2]->port[0]->input.rvalue;
                if (lcntrl < 0)
                    lcntrl = 0.;
                else if (lcntrl > 1.)
                    lcntrl = 1.;
            }
            else {
                lcntrl = 0;
            }
            delay = (delmax - delmin) * lcntrl + delmin;
        }

        /* time not yet advanced for delay output */
        if (mif_private->circuit.time < delay) {
            mif_private->conn[1]->port[0]->output.rvalue = loc->start_val;
        }
        else if (delay == 0)
            mif_private->conn[1]->port[0]->output.rvalue = mif_private->conn[0]->port[0]->input.rvalue;
        else
            mif_private->conn[1]->port[0]->output.rvalue = loc->buffer[loc->buff_del];

        /* add offset to reduce error */
        /* FIXME: may not be needed if (better) interpolation */
        delay += 0.5 * loc->tstep;

        /* delay in steps */
        delay_step = (int)(delay / loc->tstep);
        /* FIXME: For whatever reason the model is assessed two times per time step */

        /* next readout location, delayed after writing */
        if ((loc->buff_write - delay_step) >= 0)
            loc->buff_del = loc->buff_write - delay_step;
        else
            loc->buff_del = loc->buff_write - delay_step + loc->buffer_size;
    }

    else {                    /**** all others ****/
        mif_private->conn[1]->port[0]->output.rvalue = mif_private->conn[0]->port[0]->input.rvalue;
    }
}

/* free the memory created locally */
static void cm_delay_callback(Mif_Private_t *mif_private, Mif_Callback_Reason_t reason)
{
    switch (reason) {
        case MIF_CB_DESTROY: {
            mLocal_Data_t *loc = (mLocal_Data_t *) mif_private->inst_var[0]->element[0].pvalue;
            if (loc == (mLocal_Data_t *) NULL) {
                break;
            }

            if (loc->buffer != (double *) NULL) {
                free(loc->buffer);
            }

            free(loc);

            mif_private->inst_var[0]->element[0].pvalue = NULL;
            break;
        }
    }
} /* end of function cm_delay_callback */
