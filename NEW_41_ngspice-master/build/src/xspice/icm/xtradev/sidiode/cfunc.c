#include "ngspice/cm.h"
extern void cm_sidiode(Mif_Private_t *);
/*.......1.........2.........3.........4.........5.........6.........7.........8
================================================================================

FILE sidiode/cfunc.mod

-------------------------------------------------------------------------
 Copyright 2012
 The ngspice team
 All Rights Reserved
 3 - Clause BSD license
 (see COPYING or https://opensource.org/licenses/BSD-3-Clause)
-------------------------------------------------------------------------

AUTHORS

     2 October 2018     Holger Vogt


MODIFICATIONS



SUMMARY

    This file contains the model-specific routines used to
    functionally describe a simplified diode code model.


INTERFACES

    FILE                 ROUTINE CALLED



REFERENCED FILES

    Inputs from and outputs to ARGS structure.


NON-STANDARD FEATURES

    NONE

===============================================================================*/

/*=== INCLUDE FILES ====================*/

#include <stdlib.h>
#include <math.h>




/*=== CONSTANTS ========================*/




/*=== MACROS ===========================*/




/*=== LOCAL VARIABLES & TYPEDEFS =======*/

typedef struct {

    double a1;   /* parameter of first quadratic equation */
    double a2;   /* parameter of first tanh equation */
    double b1;   /* parameter of second quadratic equation */
    double b2;   /* parameter of second tanh equation */
    double hRevepsilon; /* half delta between Va and Vb */
    double hEpsilon;    /* half delta between Vc and Vd */
    double Va;   /* voltage limits of the quadratic regions */
    double Vb;
    double Vc;
    double Vd;
    double grev;  /* conductance in all three regions */
    double goff;
    double gon;
    Boolean_t Ili;    /* TRUE, if current limits are given */
    Boolean_t Revili;
    Boolean_t epsi;    /* TRUE, if quadratic overlap */
    Boolean_t revepsi;

} Local_Data_t;



/*=== FUNCTION PROTOTYPE DEFINITIONS ===*/


static void
cm_sidiode_callback(Mif_Private_t *mif_private, Mif_Callback_Reason_t reason)
{
    switch (reason) {
        case MIF_CB_DESTROY: {
            Local_Data_t *loc =  mif_private->inst_var[0]->element[0].pvalue;
	    if (loc) {
                free(loc);
                 mif_private->inst_var[0]->element[0].pvalue = NULL;
            }
            break;
        }
    }
}



/*==============================================================================

FUNCTION void cm_sidiode()

==============================================================================*/

void cm_sidiode(Mif_Private_t *mif_private)  /* structure holding parms,
                                          inputs, outputs, etc.     */
{

    Local_Data_t *loc;   /* Pointer to local static data, not to be included
                            in the state vector */

    Mif_Complex_t ac_gain;  /* AC gain  */

    double Vrev, Vfwd, Vin, Idd, deriv, Ilimit, Revilimit;

    Vrev = -mif_private->param[3]->element[0].rvalue;
    Vfwd = mif_private->param[2]->element[0].rvalue;
    Ilimit = mif_private->param[4]->element[0].rvalue;
    Revilimit = -mif_private->param[5]->element[0].rvalue;

    if (mif_private->circuit.init==1) {  /* First pass...allocate memory and calculate some constants... */

        double grev, goff, gon, Va, Vb, Vc, Vd;
        double lEpsilon = mif_private->param[6]->element[0].rvalue;
        double lRevepsilon = mif_private->param[7]->element[0].rvalue;

        *(mif_private->callback) = cm_sidiode_callback;

        /* allocate static storage for *loc */
        mif_private->inst_var[0]->element[0].pvalue = calloc (1, sizeof(Local_Data_t));
        loc = mif_private->inst_var[0]->element[0].pvalue;

        goff = 1./mif_private->param[1]->element[0].rvalue;
        gon = 1./mif_private->param[0]->element[0].rvalue;
        if (mif_private->param[8]->element[0].rvalue == 0.)
            grev = gon;
        else
            grev = 1./mif_private->param[8]->element[0].rvalue;

        loc->Va = Va = Vrev - lRevepsilon;
        loc->Vb = Vb = Vrev;
        loc->a2 = grev/Revilimit;
        if(lRevepsilon > 0.0) {
            loc->a1 = (goff - grev)/lRevepsilon;
            loc->revepsi = MIF_TRUE;
        }
        else
            loc->revepsi = MIF_FALSE;

        loc->Vc = Vc = Vfwd;
        loc->Vd = Vd = Vfwd + lEpsilon;
        loc->b2 = gon/Ilimit;
        if(lEpsilon > 0.0) {
            loc->b1 = (gon - goff)/lEpsilon;
            loc->epsi = MIF_TRUE;
        }
        else
            loc->epsi = MIF_FALSE;

        if (Ilimit < 1e29)
            loc->Ili = MIF_TRUE;
        else
            loc->Ili = MIF_FALSE;
        if (Revilimit > -1e29)
            loc->Revili = MIF_TRUE;
        else
            loc->Revili = MIF_FALSE;


        loc->grev = grev;
        loc->goff = goff;
        loc->gon = gon;
    }
    else
        loc = mif_private->inst_var[0]->element[0].pvalue;


    /* Calculate diode current Id and its derivative deriv=dId/dVin */

    Vin = mif_private->conn[0]->port[0]->input.rvalue;

    if (Vin < loc->Va) {
        if(loc->Revili) {
            double tmp = tanh(loc->a2 * (Vin - loc->Va));
            double ia = loc->goff * loc->Va
                + 0.5 * (loc->Va - loc->Vb) * (loc->Va - loc->Vb) * loc->a1;
            Idd = (Revilimit - ia) * tmp + ia;
            deriv = loc->grev * (1. - tmp * tmp);
        }
        else {
            Idd = Vrev * loc->goff + (Vin - Vrev) * loc->grev;
            deriv = loc->grev;
        }
    }
    else if (loc->revepsi && Vin >= loc->Va && Vin < loc->Vb) {
        Idd = 0.5 * (Vin -loc->Vb) * (Vin - loc->Vb) * loc->a1 + Vin * loc->goff;
        deriv = (Vin - loc->Vb) * loc->a1 + loc->goff;
    }
    else if (Vin >= loc->Vb && Vin < loc->Vc) {
        Idd = Vin * loc->goff;
        deriv = loc->goff;
    }
    else if (loc->epsi && Vin >= loc->Vc && Vin < loc->Vd) {
        Idd = 0.5 * (Vin -loc->Vc) * (Vin - loc->Vc) * loc->b1 + Vin * loc->goff;
        deriv = (Vin - loc->Vc) * loc->b1 + loc->goff;
    }
    else {
        if(loc->Ili) {
            double tmp = tanh(loc->b2 * (Vin - loc->Vd));
            double id = loc->goff * loc->Vd
                + 0.5 * (loc->Vd - loc->Vc) * (loc->Vd - loc->Vc) * loc->b1;
            Idd = (Ilimit - id) * tmp + id;
            deriv = loc->gon * (1. - tmp * tmp);
        }
        else {
            Idd = Vfwd * loc->goff + (Vin - Vfwd) * loc->gon;
            deriv = loc->gon;
        }
    }

    if(mif_private->circuit.anal_type != MIF_AC) {        /* Output DC & Transient Values */
        mif_private->conn[0]->port[0]->output.rvalue = Idd;
        mif_private->conn[0]->port[0]->partial[0].port[0] = deriv;
    }
    else {                      /* Output AC Gain */
        ac_gain.real = deriv;
        ac_gain.imag= 0.0;
        mif_private->conn[0]->port[0]->ac_gain[0].port[0] = ac_gain;
    }
}
