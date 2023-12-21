
/*
 * Structures for model: d_source
 *
 * Automatically generated by cmpp preprocessor
 *
 * !!! DO NOT EDIT !!!
 *
 */


#include "ngspice/ngspice.h"
#include <stdio.h>
#include "ngspice/devdefs.h"
#include "ngspice/ifsim.h"
#include "ngspice/mifdefs.h"
#include "ngspice/mifproto.h"
#include "ngspice/mifparse.h"


static IFparm MIFmPTable[] = {
    IOP("input_file", 0, IF_STRING, "digital input vector filename"),
    IOP("input_load", 1, IF_REAL, "input loading capacitance (F)"),
};


static IFparm MIFpTable[] = {
    OP("locdata", 2, IF_STRING, "local static data"),
};


static Mif_Port_Type_t MIFportEnum0[] = {
    MIF_DIGITAL,
};


static char *MIFportStr0[] = {
    "d",
};


static Mif_Conn_Info_t MIFconnTable[] = {
  {
    "out",
    "output",
    MIF_OUT,
    MIF_DIGITAL,
    "d",
    1,
    MIFportEnum0,
    MIFportStr0,
    MIF_TRUE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
  },
};


static Mif_Param_Info_t MIFparamTable[] = {
  {
    "input_file",
    "digital input vector filename",
    MIF_STRING,
    MIF_TRUE,
    {MIF_FALSE, 0, 0.0, {0.0, 0.0}, "source.txt"},
    MIF_FALSE,
    {MIF_FALSE, 0, 0.0, {0.0, 0.0}, NULL},
    MIF_FALSE,
    {MIF_FALSE, 0, 0.0, {0.0, 0.0}, NULL},
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
  },
  {
    "input_load",
    "input loading capacitance (F)",
    MIF_REAL,
    MIF_TRUE,
    {MIF_FALSE, 0, 1.000000e-12, {0.0, 0.0}, NULL},
    MIF_FALSE,
    {MIF_FALSE, 0, 0.0, {0.0, 0.0}, NULL},
    MIF_FALSE,
    {MIF_FALSE, 0, 0.0, {0.0, 0.0}, NULL},
    MIF_FALSE,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
    0,
    MIF_FALSE,
  },
};


static Mif_Inst_Var_Info_t MIFinst_varTable[] = {
  {
    "locdata",
    "local static data",
    MIF_STRING,
    MIF_FALSE,
  },
};


extern void cm_d_source(Mif_Private_t *);

static int val_terms             = 0;
static int val_numNames          = 0;
static int val_numInstanceParms  = 1;
static int val_numModelParms     = 2;
static int val_sizeofMIFinstance = sizeof(MIFinstance);
static int val_sizeofMIFmodel    = sizeof(MIFmodel);

SPICEdev cm_d_source_info = {
    .DEVpublic = {
        .name = "d_source",
        .description = "digital signal source",
        .terms = &val_terms,
        .numNames = &val_numNames,
        .termNames = NULL,
        .numInstanceParms = &val_numInstanceParms,
        .instanceParms = MIFpTable,
        .numModelParms = &val_numModelParms,
        .modelParms = MIFmPTable,
        .flags = 0,

        .cm_func = cm_d_source,
        .num_conn = 1,
        .conn = MIFconnTable,
        .num_param = 2,
        .param = MIFparamTable,
        .num_inst_var = 1,
        .inst_var = MIFinst_varTable,
    },

    .DEVparam = NULL,
    .DEVmodParam = MIFmParam,
    .DEVload = MIFload,
    .DEVsetup = MIFsetup,
    .DEVunsetup = MIFunsetup,
    .DEVpzSetup = NULL,
    .DEVtemperature = NULL,
    .DEVtrunc = MIFtrunc,
    .DEVfindBranch = NULL,
    .DEVacLoad = MIFload,
    .DEVaccept = NULL,
    .DEVdestroy = NULL,
    .DEVmodDelete = MIFmDelete,
    .DEVdelete = MIFdelete,
    .DEVsetic = NULL,
    .DEVask = MIFask,
    .DEVmodAsk = MIFmAsk,
    .DEVpzLoad = NULL,
    .DEVconvTest = MIFconvTest,
    .DEVsenSetup = NULL,
    .DEVsenLoad = NULL,
    .DEVsenUpdate = NULL,
    .DEVsenAcLoad = NULL,
    .DEVsenPrint = NULL,
    .DEVsenTrunc = NULL,
    .DEVdisto = NULL,
    .DEVnoise = NULL,
    .DEVsoaCheck = NULL,
    .DEVinstSize = &val_sizeofMIFinstance,
    .DEVmodSize = &val_sizeofMIFmodel,

#ifdef CIDER
    .DEVdump = NULL,
    .DEVacct = NULL,
#endif

#ifdef KLU
    .DEVbindCSC = MIFbindCSC,
    .DEVbindCSCComplex = MIFbindCSCComplex,
    .DEVbindCSCComplexToReal = MIFbindCSCComplexToReal,
#endif
};

