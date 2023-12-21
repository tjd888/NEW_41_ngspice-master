/*
 * forward declaration of important structs
 * and central typedefs which are not allowed to be repeated
 */

#ifndef ngspice_TYPEDEFS_H
#define ngspice_TYPEDEFS_H


typedef struct CKTcircuit CKTcircuit;
typedef struct CKTnode CKTnode;


typedef struct GENinstance GENinstance;
typedef struct GENmodel GENmodel;


/* 结构体用于存储提取的元件和节点信息 */
typedef struct {
    GENinstance *instances;   /* 元件实例列表 */
    CKTnode *nodes;          /* 节点列表 */
} CircuitElements;


typedef struct IFparm IFparm;
typedef union  IFvalue IFvalue;
typedef struct IFparseTree IFparseTree;
typedef struct IFcomplex IFcomplex;
typedef struct IFdevice IFdevice;
typedef struct IFanalysis IFanalysis;
typedef struct IFsimulator IFsimulator;
typedef struct IFfrontEnd IFfrontEnd;
typedef char *IFuid;


typedef struct TFan TFan;


typedef struct graph GRAPH;


struct dbcomm;


typedef struct PZtrial PZtrial;
typedef struct PZAN PZAN;


typedef struct SENstruct SENstruct;

typedef struct TSKtask TSKtask;
typedef struct JOB JOB;

typedef struct SPICEanalysis SPICEanalysis;

typedef struct runDesc runDesc;

#endif
