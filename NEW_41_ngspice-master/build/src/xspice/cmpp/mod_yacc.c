/* A Bison parser, made by GNU Bison 3.0.4.  */

/* Bison implementation for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "3.0.4"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Push parsers.  */
#define YYPUSH 0

/* Pull parsers.  */
#define YYPULL 1




/* Copy the first part of user declarations.  */
#line 1 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:339  */


/*============================================================================
FILE  mod_yacc.y

MEMBER OF process cmpp

Copyright 1991
Georgia Tech Research Corporation
Atlanta, Georgia 30332
All Rights Reserved

PROJECT A-8503

AUTHORS

    9/12/91  Steve Tynor

MODIFICATIONS

    <date> <person name> <nature of modifications>
  20050420 Steven Borley Renamed strcmpi() to local_strcmpi() to avoid
                         clash with strcmpi() in a windows header file.

SUMMARY

    This file contains a BNF specification of the translation of
    cfunc.mod files to cfunc.c files, together with various support
    functions.

INTERFACES

    mod_yyparse() -    Function 'yyparse()' is generated automatically
                       by UNIX 'yacc' utility.  All yy* global names
                       are converted to mod_yy* by #define.

REFERENCED FILES

    mod_lex.l

============================================================================*/


#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "mod_yacc_y.h"

extern int mod_yylex(void);

#define	yymaxdepth mod_yymaxdepth
#define	yyparse	mod_yyparse
#define	yylex	mod_yylex
#define	yyerror	mod_yyerror
#define	yylval	mod_yylval
#define	yychar	mod_yychar
#define	yydebug	mod_yydebug
#define	yypact	mod_yypact
#define	yyr1	mod_yyr1
#define	yyr2	mod_yyr2
#define	yydef	mod_yydef
#define	yychk	mod_yychk
#define	yypgo	mod_yypgo
#define	yyact	mod_yyact
#define	yyexca	mod_yyexca
#define yyerrflag mod_yyerrflag
#define yynerrs	mod_yynerrs
#define	yyps	mod_yyps
#define	yypv	mod_yypv
#define	yys	mod_yys
#define	yy_yys	mod_yyyys
#define	yystate	mod_yystate
#define	yytmp	mod_yytmp
#define	yyv	mod_yyv
#define	yy_yyv	mod_yyyyv
#define	yyval	mod_yyval
#define	yylloc	mod_yylloc
#define yyreds	mod_yyreds
#define yytoks	mod_yytoks
#define yylhs	mod_yyyylhs
#define yylen	mod_yyyylen
#define yydefred mod_yyyydefred
#define yydgoto	mod_yyyydgoto
#define yysindex mod_yyyysindex
#define yyrindex mod_yyyyrindex
#define yygindex mod_yyyygindex
#define yytable	 mod_yyyytable
#define yycheck	 mod_yyyycheck
#define yyname   mod_yyyyname
#define yyrule   mod_yyyyrule

Ifs_Table_t *mod_ifs_table;

extern char *mod_yytext;
extern FILE* mod_yyout;

#include <string.h>
#include <ctype.h>

int mod_num_errors;

#define BUFFER_SIZE 3000
static char buffer [BUFFER_SIZE];
static int buf_len;
   
typedef enum {CONN, PARAM, STATIC_VAR} Id_Kind_t;

/*--------------------------------------------------------------------------*/
static char *subscript (Sub_Id_t sub_id)
{
   if (sub_id.has_subscript) {
      return sub_id.subscript;
   } else {
      return "0";
   }
}

/*--------------------------------------------------------------------------*/
static int
local_strcmpi(char *s, char *t)
     /* string compare -  case insensitive */
{
   for (; *s && t && tolower_c(*s) == tolower_c(*t); s++, t++)
      ;
   if (*s && !*t) {
      return 1;
   }
   if (!*s && *t) {
      return -1;
   }
   if (! (*s || *t)) {
      return 0;
   }
   return tolower((unsigned char) *s) - tolower((unsigned char) *t);
}

/*---------------------------------------------------------------------------*/
static void put_type (FILE *fp, Data_Type_t type)
{
   char ch = ' ';
   
   switch (type) {
   case CMPP_INTEGER:
      ch = 'i';
      break;
   case CMPP_REAL:
      ch = 'r';
      break;
   case CMPP_COMPLEX:
      ch = 'c';
      break;
   case CMPP_BOOLEAN:
      ch = 'b';
      break;
   case CMPP_STRING:
      ch = 's';
      break;
   case CMPP_POINTER:
      ch = 'p';
      break;
   }
   fprintf (fp, ".%cvalue", ch);
}

/*---------------------------------------------------------------------------*/
static void put_conn_type (FILE *fp, Port_Type_t type)
{
   char ch;
   
   switch (type) {
   case USER_DEFINED:
      ch = 'p';
      break;
   case DIGITAL:
      ch = 'p';
      break;
   default:
      ch = 'r';
      break;
   }
   fprintf (fp, ".%cvalue", ch);
}

/*---------------------------------------------------------------------------*/
static void check_dir (int conn_number, Dir_t dir, char *context)
{
   Dir_t conn_dir;
   
   if (conn_number >= 0) {
      /*
       * If negative, this is an invalid port ID and we've already issued
       * an error.
       */
      conn_dir = mod_ifs_table->conn[conn_number].direction;
      if ((conn_dir != dir) && (conn_dir != CMPP_INOUT)) {
	 char error_str[200];
	 
	 sprintf (error_str,
		  "Direction of port `%s' in %s() is not %s or INOUT",
		  mod_ifs_table->conn[conn_number].name, context,
		  (dir == CMPP_IN) ? "IN" : "OUT");
	 yyerror (error_str);
	 mod_num_errors++;
      }
   }
}

/*---------------------------------------------------------------------------*/
static void check_subscript (bool formal, bool actual,
			     bool missing_actual_ok,
			     char *context, char *id)
{
   char error_str[200];

   if ((formal && !actual) && !missing_actual_ok) {
      sprintf (error_str,
	       "%s `%s' is an array - subscript required",
	       context, id);
      yyerror (error_str);
      mod_num_errors++;
      return;
   } else if (!formal && actual) {
      sprintf (error_str,
	       "%s `%s' is not an array - subscript prohibited",
	       context, id);
      yyerror (error_str);
      mod_num_errors++;
      return;
   }
}

/*---------------------------------------------------------------------------*/
static int check_id (Sub_Id_t sub_id, Id_Kind_t kind, bool do_subscript)
{
   int i;
   char error_str[200];
   
   switch (kind) {
   case CONN:
      for (i = 0; i < mod_ifs_table->num_conn; i++) {
	 if (0 == local_strcmpi (sub_id.id, mod_ifs_table->conn[i].name)) {
	    if (do_subscript) {
	       check_subscript (mod_ifs_table->conn[i].is_array,
				sub_id.has_subscript, false, "Port",
				sub_id.id);
	    }
	    return i;
	 }
      }
      break;
   case PARAM:
      for (i = 0; i < mod_ifs_table->num_param; i++) {
      	 if (0 == local_strcmpi (sub_id.id, mod_ifs_table->param[i].name)) {
	    if (do_subscript) {
	       check_subscript (mod_ifs_table->param[i].is_array,
				sub_id.has_subscript, false, "Parameter",
				sub_id.id);
	    }
	    return i;
	 }
      }
      break;
   case STATIC_VAR:
      for (i = 0; i < mod_ifs_table->num_inst_var; i++) {
      	 if (0 == local_strcmpi (sub_id.id, mod_ifs_table->inst_var[i].name)) {
	    if (do_subscript) {
	       check_subscript (mod_ifs_table->inst_var[i].is_array,
				sub_id.has_subscript, true,
				"Static Variable",
				sub_id.id);
	    }
	    return i;
	 }
      }
      break;
   }
   
   sprintf (error_str, "No %s named '%s'",
	    ((kind==CONN)
	     ? "port"
	     : ((kind==PARAM)
		? "parameter"
		:"static variable")),
	    sub_id.id);
   yyerror (error_str);
   mod_num_errors++;
   return -1;
}

/*---------------------------------------------------------------------------*/
static int valid_id (Sub_Id_t sub_id, Id_Kind_t kind)
{
    return check_id (sub_id, kind, false);
}

/*---------------------------------------------------------------------------*/
static int valid_subid (Sub_Id_t sub_id, Id_Kind_t kind)
{
    return check_id (sub_id, kind, true);
}

/*---------------------------------------------------------------------------*/
static void init_buffer (void)
{
   buf_len = 0;
   buffer[0] = '\0';
}

/*---------------------------------------------------------------------------*/
static void append (char *str)
{
   int len = (int) strlen (str);
   if (len + buf_len > BUFFER_SIZE) {
      yyerror ("Buffer overflow - try reducing the complexity of CM-macro array subscripts");
      exit (1);
   }
   (void)strcat (buffer,str);
}


#line 388 "mod_yacc.c" /* yacc.c:339  */

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* In a future release of Bison, this section will be replaced
   by #include "mod_yacc.h".  */
#ifndef YY_YY_MOD_YACC_H_INCLUDED
# define YY_YY_MOD_YACC_H_INCLUDED
/* Debug traces.  */
#ifndef YYDEBUG
# define YYDEBUG 0
#endif
#if YYDEBUG
extern int yydebug;
#endif

/* Token type.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
  enum yytokentype
  {
    TOK_ARGS = 258,
    TOK_INIT = 259,
    TOK_CALLBACK = 260,
    TOK_ANALYSIS = 261,
    TOK_NEW_TIMEPOINT = 262,
    TOK_TIME = 263,
    TOK_RAD_FREQ = 264,
    TOK_TEMPERATURE = 265,
    TOK_T = 266,
    TOK_PARAM = 267,
    TOK_PARAM_SIZE = 268,
    TOK_PARAM_NULL = 269,
    TOK_PORT_SIZE = 270,
    TOK_PORT_NULL = 271,
    TOK_PARTIAL = 272,
    TOK_AC_GAIN = 273,
    TOK_CHANGED = 274,
    TOK_OUTPUT_DELAY = 275,
    TOK_STATIC_VAR = 276,
    TOK_STATIC_VAR_SIZE = 277,
    TOK_STATIC_VAR_INST = 278,
    TOK_INPUT = 279,
    TOK_INPUT_STRENGTH = 280,
    TOK_INPUT_STATE = 281,
    TOK_INPUT_TYPE = 282,
    TOK_OUTPUT = 283,
    TOK_OUTPUT_CHANGED = 284,
    TOK_OUTPUT_STRENGTH = 285,
    TOK_OUTPUT_STATE = 286,
    TOK_OUTPUT_TYPE = 287,
    TOK_COMMA = 288,
    TOK_LPAREN = 289,
    TOK_RPAREN = 290,
    TOK_LBRACKET = 291,
    TOK_RBRACKET = 292,
    TOK_MISC_C = 293,
    TOK_IDENTIFIER = 294,
    TOK_LOAD = 295,
    TOK_TOTAL_LOAD = 296,
    TOK_MESSAGE = 297,
    TOK_CALL_TYPE = 298
  };
#endif
/* Tokens.  */
#define TOK_ARGS 258
#define TOK_INIT 259
#define TOK_CALLBACK 260
#define TOK_ANALYSIS 261
#define TOK_NEW_TIMEPOINT 262
#define TOK_TIME 263
#define TOK_RAD_FREQ 264
#define TOK_TEMPERATURE 265
#define TOK_T 266
#define TOK_PARAM 267
#define TOK_PARAM_SIZE 268
#define TOK_PARAM_NULL 269
#define TOK_PORT_SIZE 270
#define TOK_PORT_NULL 271
#define TOK_PARTIAL 272
#define TOK_AC_GAIN 273
#define TOK_CHANGED 274
#define TOK_OUTPUT_DELAY 275
#define TOK_STATIC_VAR 276
#define TOK_STATIC_VAR_SIZE 277
#define TOK_STATIC_VAR_INST 278
#define TOK_INPUT 279
#define TOK_INPUT_STRENGTH 280
#define TOK_INPUT_STATE 281
#define TOK_INPUT_TYPE 282
#define TOK_OUTPUT 283
#define TOK_OUTPUT_CHANGED 284
#define TOK_OUTPUT_STRENGTH 285
#define TOK_OUTPUT_STATE 286
#define TOK_OUTPUT_TYPE 287
#define TOK_COMMA 288
#define TOK_LPAREN 289
#define TOK_RPAREN 290
#define TOK_LBRACKET 291
#define TOK_RBRACKET 292
#define TOK_MISC_C 293
#define TOK_IDENTIFIER 294
#define TOK_LOAD 295
#define TOK_TOTAL_LOAD 296
#define TOK_MESSAGE 297
#define TOK_CALL_TYPE 298

/* Value type.  */
#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED

union YYSTYPE
{
#line 323 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:355  */

   char *str;
   Sub_Id_t sub_id;

#line 519 "mod_yacc.c" /* yacc.c:355  */
};

typedef union YYSTYPE YYSTYPE;
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED 1
#endif


extern YYSTYPE yylval;

int yyparse (void);

#endif /* !YY_YY_MOD_YACC_H_INCLUDED  */

/* Copy the second part of user declarations.  */

#line 536 "mod_yacc.c" /* yacc.c:358  */

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#else
typedef signed char yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(Msgid) dgettext ("bison-runtime", Msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(Msgid) Msgid
# endif
#endif

#ifndef YY_ATTRIBUTE
# if (defined __GNUC__                                               \
      && (2 < __GNUC__ || (__GNUC__ == 2 && 96 <= __GNUC_MINOR__)))  \
     || defined __SUNPRO_C && 0x5110 <= __SUNPRO_C
#  define YY_ATTRIBUTE(Spec) __attribute__(Spec)
# else
#  define YY_ATTRIBUTE(Spec) /* empty */
# endif
#endif

#ifndef YY_ATTRIBUTE_PURE
# define YY_ATTRIBUTE_PURE   YY_ATTRIBUTE ((__pure__))
#endif

#ifndef YY_ATTRIBUTE_UNUSED
# define YY_ATTRIBUTE_UNUSED YY_ATTRIBUTE ((__unused__))
#endif

#if !defined _Noreturn \
     && (!defined __STDC_VERSION__ || __STDC_VERSION__ < 201112)
# if defined _MSC_VER && 1200 <= _MSC_VER
#  define _Noreturn __declspec (noreturn)
# else
#  define _Noreturn YY_ATTRIBUTE ((__noreturn__))
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(E) ((void) (E))
#else
# define YYUSE(E) /* empty */
#endif

#if defined __GNUC__ && 407 <= __GNUC__ * 100 + __GNUC_MINOR__
/* Suppress an incorrect diagnostic about yylval being uninitialized.  */
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN \
    _Pragma ("GCC diagnostic push") \
    _Pragma ("GCC diagnostic ignored \"-Wuninitialized\"")\
    _Pragma ("GCC diagnostic ignored \"-Wmaybe-uninitialized\"")
# define YY_IGNORE_MAYBE_UNINITIALIZED_END \
    _Pragma ("GCC diagnostic pop")
#else
# define YY_INITIAL_VALUE(Value) Value
#endif
#ifndef YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
# define YY_IGNORE_MAYBE_UNINITIALIZED_END
#endif
#ifndef YY_INITIAL_VALUE
# define YY_INITIAL_VALUE(Value) /* Nothing. */
#endif


#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined EXIT_SUCCESS
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
      /* Use EXIT_SUCCESS as a witness for stdlib.h.  */
#     ifndef EXIT_SUCCESS
#      define EXIT_SUCCESS 0
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's 'empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (0)
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined EXIT_SUCCESS \
       && ! ((defined YYMALLOC || defined malloc) \
             && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef EXIT_SUCCESS
#    define EXIT_SUCCESS 0
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined EXIT_SUCCESS
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined EXIT_SUCCESS
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
         || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss_alloc;
  YYSTYPE yyvs_alloc;
};

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

# define YYCOPY_NEEDED 1

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack_alloc, Stack)                           \
    do                                                                  \
      {                                                                 \
        YYSIZE_T yynewbytes;                                            \
        YYCOPY (&yyptr->Stack_alloc, Stack, yysize);                    \
        Stack = &yyptr->Stack_alloc;                                    \
        yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
        yyptr += yynewbytes / sizeof (*yyptr);                          \
      }                                                                 \
    while (0)

#endif

#if defined YYCOPY_NEEDED && YYCOPY_NEEDED
/* Copy COUNT objects from SRC to DST.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(Dst, Src, Count) \
      __builtin_memcpy (Dst, Src, (Count) * sizeof (*(Src)))
#  else
#   define YYCOPY(Dst, Src, Count)              \
      do                                        \
        {                                       \
          YYSIZE_T yyi;                         \
          for (yyi = 0; yyi < (Count); yyi++)   \
            (Dst)[yyi] = (Src)[yyi];            \
        }                                       \
      while (0)
#  endif
# endif
#endif /* !YYCOPY_NEEDED */

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  2
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   230

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  44
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  15
/* YYNRULES -- Number of rules.  */
#define YYNRULES  65
/* YYNSTATES -- Number of states.  */
#define YYNSTATES  153

/* YYTRANSLATE[YYX] -- Symbol number corresponding to YYX as returned
   by yylex, with out-of-bounds checking.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   298

#define YYTRANSLATE(YYX)                                                \
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[TOKEN-NUM] -- Symbol number corresponding to TOKEN-NUM
   as returned by yylex, without out-of-bounds checking.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43
};

#if YYDEBUG
  /* YYRLINE[YYN] -- Source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   377,   377,   378,   381,   382,   383,   384,   385,   388,
     388,   392,   393,   396,   397,   398,   400,   399,   404,   403,
     409,   410,   411,   412,   414,   413,   417,   419,   418,   424,
     426,   428,   430,   432,   434,   436,   438,   440,   442,   444,
     450,   453,   456,   459,   462,   470,   479,   491,   495,   499,
     505,   511,   519,   525,   531,   540,   549,   557,   566,   575,
     580,   585,   590,   597,   598,   604
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || 0
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "TOK_ARGS", "TOK_INIT", "TOK_CALLBACK",
  "TOK_ANALYSIS", "TOK_NEW_TIMEPOINT", "TOK_TIME", "TOK_RAD_FREQ",
  "TOK_TEMPERATURE", "TOK_T", "TOK_PARAM", "TOK_PARAM_SIZE",
  "TOK_PARAM_NULL", "TOK_PORT_SIZE", "TOK_PORT_NULL", "TOK_PARTIAL",
  "TOK_AC_GAIN", "TOK_CHANGED", "TOK_OUTPUT_DELAY", "TOK_STATIC_VAR",
  "TOK_STATIC_VAR_SIZE", "TOK_STATIC_VAR_INST", "TOK_INPUT",
  "TOK_INPUT_STRENGTH", "TOK_INPUT_STATE", "TOK_INPUT_TYPE", "TOK_OUTPUT",
  "TOK_OUTPUT_CHANGED", "TOK_OUTPUT_STRENGTH", "TOK_OUTPUT_STATE",
  "TOK_OUTPUT_TYPE", "TOK_COMMA", "TOK_LPAREN", "TOK_RPAREN",
  "TOK_LBRACKET", "TOK_RBRACKET", "TOK_MISC_C", "TOK_IDENTIFIER",
  "TOK_LOAD", "TOK_TOTAL_LOAD", "TOK_MESSAGE", "TOK_CALL_TYPE", "$accept",
  "mod_file", "c_code", "buffered_c_code", "$@1", "buffered_c_code2",
  "buffered_c_char", "$@2", "$@3", "c_char", "$@4", "$@5", "macro",
  "subscriptable_id", "id", YY_NULLPTR
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[NUM] -- (External) token number corresponding to the
   (internal) symbol number NUM (which must be that of a token).  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298
};
# endif

#define YYPACT_NINF -122

#define yypact_value_is_default(Yystate) \
  (!!((Yystate) == (-122)))

#define YYTABLE_NINF -1

#define yytable_value_is_error(Yytable_value) \
  0

  /* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
     STATE-NUM.  */
static const yytype_int16 yypact[] =
{
    -122,     5,  -122,  -122,  -122,   142,  -122,  -122,  -122,  -122,
    -122,  -122,  -122,  -122,   -28,    -3,     4,     7,    11,    18,
      20,    23,    24,    25,    26,    63,   102,   143,   145,   152,
     153,   154,   155,   156,   157,   158,  -122,   159,     2,  -122,
    -122,   161,   162,   163,  -122,  -122,  -122,  -122,   160,   160,
     160,   160,   160,   160,   160,   160,   160,   160,   160,   160,
     160,   160,   160,   160,   160,   160,   160,   160,   160,  -122,
      -1,  -122,    -1,   160,   160,   160,   165,  -122,  -122,   166,
     167,   169,   170,   171,   172,   175,   176,   177,   178,   179,
     180,   181,   182,   183,   184,   185,   186,   187,   188,   189,
     190,    60,   101,   191,   192,   193,  -122,    17,  -122,  -122,
    -122,  -122,  -122,  -122,   160,   160,  -122,  -122,  -122,  -122,
    -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,
    -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,  -122,
    -122,   173,   194,   195,  -122,  -122,  -122,  -122,  -122,    -6,
      10,  -122,  -122
};

  /* YYDEFACT[STATE-NUM] -- Default reduction number in state STATE-NUM.
     Performed when YYTABLE does not specify something else to do.  Zero
     means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       2,     4,     1,     7,     8,     3,    31,    29,    30,    32,
      33,    35,    36,    37,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,    22,    27,    24,    21,
      20,     0,     0,     0,    34,     5,     6,     9,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,    26,
       4,    23,     4,     0,     0,     0,     0,    11,    65,     0,
      63,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,     0,     0,     0,     0,
       0,     0,     0,     0,     0,     0,    38,    10,    39,     9,
      40,    41,    42,    43,     0,     0,    50,    49,    46,    47,
      48,    51,    54,    55,    52,    56,    59,    57,    58,    53,
      28,    25,    60,    61,    62,    15,    18,    16,    14,    13,
      12,     0,     0,     0,    11,    11,    64,    44,    45,     0,
       0,    19,    17
};

  /* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
    -122,  -122,   -35,    84,  -122,  -121,  -122,  -122,  -122,  -122,
    -122,  -122,  -122,   -53,   -33
};

  /* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int16 yydefgoto[] =
{
      -1,     1,     5,    76,    77,   107,   140,   145,   144,    45,
      72,    70,    46,    79,    80
};

  /* YYTABLE[YYPACT[STATE-NUM]] -- What to do in state STATE-NUM.  If
     positive, shift that token.  If negative, reduce the rule whose
     number is the opposite.  If YYTABLE_NINF, syntax error.  */
static const yytype_uint8 yytable[] =
{
      85,    86,    87,    88,    89,     2,    47,    92,    93,    94,
      95,    96,    97,    98,    99,   100,    81,    82,    83,    84,
     103,   104,   105,   149,   150,    90,    91,   135,   136,   151,
     137,    48,   138,   139,     3,   101,     4,   102,    49,    71,
       3,    50,     4,   135,   136,    51,   137,   152,   138,   139,
     135,   136,    52,   137,    53,   138,   139,    54,    55,    56,
      57,   142,   143,     6,     7,     8,     9,    10,    11,    12,
      13,    14,    15,    16,    17,    18,    19,    20,    21,    22,
      23,    24,    25,    26,    27,    28,    29,    30,    31,    32,
      33,    34,    35,    36,    37,   130,    38,    58,    39,    40,
      41,    42,    43,    44,     6,     7,     8,     9,    10,    11,
      12,    13,    14,    15,    16,    17,    18,    19,    20,    21,
      22,    23,    24,    25,    26,    27,    28,    29,    30,    31,
      32,    33,    34,    35,    36,    37,    59,    38,   131,    39,
      40,    41,    42,    43,    44,     6,     7,     8,     9,    10,
      11,    12,    13,    14,    15,    16,    17,    18,    19,    20,
      21,    22,    23,    24,    25,    26,    27,    28,    29,    30,
      31,    32,    33,    34,    35,    36,    37,    60,    38,    61,
      39,    40,    41,    42,    43,    44,    62,    63,    64,    65,
      66,    67,    68,   141,    69,    73,    74,    75,     0,    78,
     106,   108,     0,   109,   110,   111,   112,   113,   114,   115,
     146,     0,   116,   117,   118,   119,   120,   121,   122,   123,
     124,   125,   126,   127,   128,   129,   132,   133,   134,   147,
     148
};

static const yytype_int16 yycheck[] =
{
      53,    54,    55,    56,    57,     0,    34,    60,    61,    62,
      63,    64,    65,    66,    67,    68,    49,    50,    51,    52,
      73,    74,    75,   144,   145,    58,    59,    33,    34,    35,
      36,    34,    38,    39,    35,    70,    37,    72,    34,    37,
      35,    34,    37,    33,    34,    34,    36,    37,    38,    39,
      33,    34,    34,    36,    34,    38,    39,    34,    34,    34,
      34,   114,   115,     3,     4,     5,     6,     7,     8,     9,
      10,    11,    12,    13,    14,    15,    16,    17,    18,    19,
      20,    21,    22,    23,    24,    25,    26,    27,    28,    29,
      30,    31,    32,    33,    34,    35,    36,    34,    38,    39,
      40,    41,    42,    43,     3,     4,     5,     6,     7,     8,
       9,    10,    11,    12,    13,    14,    15,    16,    17,    18,
      19,    20,    21,    22,    23,    24,    25,    26,    27,    28,
      29,    30,    31,    32,    33,    34,    34,    36,    37,    38,
      39,    40,    41,    42,    43,     3,     4,     5,     6,     7,
       8,     9,    10,    11,    12,    13,    14,    15,    16,    17,
      18,    19,    20,    21,    22,    23,    24,    25,    26,    27,
      28,    29,    30,    31,    32,    33,    34,    34,    36,    34,
      38,    39,    40,    41,    42,    43,    34,    34,    34,    34,
      34,    34,    34,   109,    35,    34,    34,    34,    -1,    39,
      35,    35,    -1,    36,    35,    35,    35,    35,    33,    33,
      37,    -1,    35,    35,    35,    35,    35,    35,    35,    35,
      35,    35,    35,    35,    35,    35,    35,    35,    35,    35,
      35
};

  /* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
     symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,    45,     0,    35,    37,    46,     3,     4,     5,     6,
       7,     8,     9,    10,    11,    12,    13,    14,    15,    16,
      17,    18,    19,    20,    21,    22,    23,    24,    25,    26,
      27,    28,    29,    30,    31,    32,    33,    34,    36,    38,
      39,    40,    41,    42,    43,    53,    56,    34,    34,    34,
      34,    34,    34,    34,    34,    34,    34,    34,    34,    34,
      34,    34,    34,    34,    34,    34,    34,    34,    34,    35,
      55,    37,    54,    34,    34,    34,    47,    48,    39,    57,
      58,    58,    58,    58,    58,    57,    57,    57,    57,    57,
      58,    58,    57,    57,    57,    57,    57,    57,    57,    57,
      57,    46,    46,    57,    57,    57,    35,    49,    35,    36,
      35,    35,    35,    35,    33,    33,    35,    35,    35,    35,
      35,    35,    35,    35,    35,    35,    35,    35,    35,    35,
      35,    37,    35,    35,    35,    33,    34,    36,    38,    39,
      50,    47,    57,    57,    52,    51,    37,    35,    35,    49,
      49,    35,    37
};

  /* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    44,    45,    45,    46,    46,    46,    46,    46,    48,
      47,    49,    49,    50,    50,    50,    51,    50,    52,    50,
      53,    53,    53,    53,    54,    53,    53,    55,    53,    56,
      56,    56,    56,    56,    56,    56,    56,    56,    56,    56,
      56,    56,    56,    56,    56,    56,    56,    56,    56,    56,
      56,    56,    56,    56,    56,    56,    56,    56,    56,    56,
      56,    56,    56,    57,    57,    58
};

  /* YYR2[YYN] -- Number of symbols on the right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     0,     2,     0,     2,     2,     1,     1,     0,
       2,     0,     2,     1,     1,     1,     0,     4,     0,     4,
       1,     1,     1,     2,     0,     4,     2,     0,     4,     1,
       1,     1,     1,     1,     1,     1,     1,     1,     4,     4,
       4,     4,     4,     4,     6,     6,     4,     4,     4,     4,
       4,     4,     4,     4,     4,     4,     4,     4,     4,     4,
       4,     4,     4,     1,     4,     1
};


#define yyerrok         (yyerrstatus = 0)
#define yyclearin       (yychar = YYEMPTY)
#define YYEMPTY         (-2)
#define YYEOF           0

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab


#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)                                  \
do                                                              \
  if (yychar == YYEMPTY)                                        \
    {                                                           \
      yychar = (Token);                                         \
      yylval = (Value);                                         \
      YYPOPSTACK (yylen);                                       \
      yystate = *yyssp;                                         \
      goto yybackup;                                            \
    }                                                           \
  else                                                          \
    {                                                           \
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;                                                  \
    }                                                           \
while (0)

/* Error token number */
#define YYTERROR        1
#define YYERRCODE       256



/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)                        \
do {                                            \
  if (yydebug)                                  \
    YYFPRINTF Args;                             \
} while (0)

/* This macro is provided for backward compatibility. */
#ifndef YY_LOCATION_PRINT
# define YY_LOCATION_PRINT(File, Loc) ((void) 0)
#endif


# define YY_SYMBOL_PRINT(Title, Type, Value, Location)                    \
do {                                                                      \
  if (yydebug)                                                            \
    {                                                                     \
      YYFPRINTF (stderr, "%s ", Title);                                   \
      yy_symbol_print (stderr,                                            \
                  Type, Value); \
      YYFPRINTF (stderr, "\n");                                           \
    }                                                                     \
} while (0)


/*----------------------------------------.
| Print this symbol's value on YYOUTPUT.  |
`----------------------------------------*/

static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
{
  FILE *yyo = yyoutput;
  YYUSE (yyo);
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# endif
  YYUSE (yytype);
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
{
  YYFPRINTF (yyoutput, "%s %s (",
             yytype < YYNTOKENS ? "token" : "nterm", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

static void
yy_stack_print (yytype_int16 *yybottom, yytype_int16 *yytop)
{
  YYFPRINTF (stderr, "Stack now");
  for (; yybottom <= yytop; yybottom++)
    {
      int yybot = *yybottom;
      YYFPRINTF (stderr, " %d", yybot);
    }
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)                            \
do {                                                            \
  if (yydebug)                                                  \
    yy_stack_print ((Bottom), (Top));                           \
} while (0)


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

static void
yy_reduce_print (yytype_int16 *yyssp, YYSTYPE *yyvsp, int yyrule)
{
  unsigned long int yylno = yyrline[yyrule];
  int yynrhs = yyr2[yyrule];
  int yyi;
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
             yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      YYFPRINTF (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr,
                       yystos[yyssp[yyi + 1 - yynrhs]],
                       &(yyvsp[(yyi + 1) - (yynrhs)])
                                              );
      YYFPRINTF (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)          \
do {                                    \
  if (yydebug)                          \
    yy_reduce_print (yyssp, yyvsp, Rule); \
} while (0)

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif


#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
static YYSIZE_T
yystrlen (const char *yystr)
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
static char *
yystpcpy (char *yydest, const char *yysrc)
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
        switch (*++yyp)
          {
          case '\'':
          case ',':
            goto do_not_strip_quotes;

          case '\\':
            if (*++yyp != '\\')
              goto do_not_strip_quotes;
            /* Fall through.  */
          default:
            if (yyres)
              yyres[yyn] = *yyp;
            yyn++;
            break;

          case '"':
            if (yyres)
              yyres[yyn] = '\0';
            return yyn;
          }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into *YYMSG, which is of size *YYMSG_ALLOC, an error message
   about the unexpected token YYTOKEN for the state stack whose top is
   YYSSP.

   Return 0 if *YYMSG was successfully written.  Return 1 if *YYMSG is
   not large enough to hold the message.  In that case, also set
   *YYMSG_ALLOC to the required number of bytes.  Return 2 if the
   required number of bytes is too large to store.  */
static int
yysyntax_error (YYSIZE_T *yymsg_alloc, char **yymsg,
                yytype_int16 *yyssp, int yytoken)
{
  YYSIZE_T yysize0 = yytnamerr (YY_NULLPTR, yytname[yytoken]);
  YYSIZE_T yysize = yysize0;
  enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
  /* Internationalized format string. */
  const char *yyformat = YY_NULLPTR;
  /* Arguments of yyformat. */
  char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
  /* Number of reported tokens (one for the "unexpected", one per
     "expected"). */
  int yycount = 0;

  /* There are many possibilities here to consider:
     - If this state is a consistent state with a default action, then
       the only way this function was invoked is if the default action
       is an error action.  In that case, don't check for expected
       tokens because there are none.
     - The only way there can be no lookahead present (in yychar) is if
       this state is a consistent state with a default action.  Thus,
       detecting the absence of a lookahead is sufficient to determine
       that there is no unexpected or expected token to report.  In that
       case, just report a simple "syntax error".
     - Don't assume there isn't a lookahead just because this state is a
       consistent state with a default action.  There might have been a
       previous inconsistent state, consistent state with a non-default
       action, or user semantic action that manipulated yychar.
     - Of course, the expected token list depends on states to have
       correct lookahead information, and it depends on the parser not
       to perform extra reductions after fetching a lookahead from the
       scanner and before detecting a syntax error.  Thus, state merging
       (from LALR or IELR) and default reductions corrupt the expected
       token list.  However, the list is correct for canonical LR with
       one exception: it will still contain any token that will not be
       accepted due to an error action in a later state.
  */
  if (yytoken != YYEMPTY)
    {
      int yyn = yypact[*yyssp];
      yyarg[yycount++] = yytname[yytoken];
      if (!yypact_value_is_default (yyn))
        {
          /* Start YYX at -YYN if negative to avoid negative indexes in
             YYCHECK.  In other words, skip the first -YYN actions for
             this state because they are default actions.  */
          int yyxbegin = yyn < 0 ? -yyn : 0;
          /* Stay within bounds of both yycheck and yytname.  */
          int yychecklim = YYLAST - yyn + 1;
          int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
          int yyx;

          for (yyx = yyxbegin; yyx < yyxend; ++yyx)
            if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR
                && !yytable_value_is_error (yytable[yyx + yyn]))
              {
                if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                  {
                    yycount = 1;
                    yysize = yysize0;
                    break;
                  }
                yyarg[yycount++] = yytname[yyx];
                {
                  YYSIZE_T yysize1 = yysize + yytnamerr (YY_NULLPTR, yytname[yyx]);
                  if (! (yysize <= yysize1
                         && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
                    return 2;
                  yysize = yysize1;
                }
              }
        }
    }

  switch (yycount)
    {
# define YYCASE_(N, S)                      \
      case N:                               \
        yyformat = S;                       \
      break
      YYCASE_(0, YY_("syntax error"));
      YYCASE_(1, YY_("syntax error, unexpected %s"));
      YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
      YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
      YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
      YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
# undef YYCASE_
    }

  {
    YYSIZE_T yysize1 = yysize + yystrlen (yyformat);
    if (! (yysize <= yysize1 && yysize1 <= YYSTACK_ALLOC_MAXIMUM))
      return 2;
    yysize = yysize1;
  }

  if (*yymsg_alloc < yysize)
    {
      *yymsg_alloc = 2 * yysize;
      if (! (yysize <= *yymsg_alloc
             && *yymsg_alloc <= YYSTACK_ALLOC_MAXIMUM))
        *yymsg_alloc = YYSTACK_ALLOC_MAXIMUM;
      return 1;
    }

  /* Avoid sprintf, as that infringes on the user's name space.
     Don't have undefined behavior even if the translation
     produced a string with the wrong number of "%s"s.  */
  {
    char *yyp = *yymsg;
    int yyi = 0;
    while ((*yyp = *yyformat) != '\0')
      if (*yyp == '%' && yyformat[1] == 's' && yyi < yycount)
        {
          yyp += yytnamerr (yyp, yyarg[yyi++]);
          yyformat += 2;
        }
      else
        {
          yyp++;
          yyformat++;
        }
  }
  return 0;
}
#endif /* YYERROR_VERBOSE */

/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
{
  YYUSE (yyvaluep);
  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  YYUSE (yytype);
  YY_IGNORE_MAYBE_UNINITIALIZED_END
}




/* The lookahead symbol.  */
int yychar;

/* The semantic value of the lookahead symbol.  */
YYSTYPE yylval;
/* Number of syntax errors so far.  */
int yynerrs;


/*----------.
| yyparse.  |
`----------*/

int
yyparse (void)
{
    int yystate;
    /* Number of tokens to shift before error messages enabled.  */
    int yyerrstatus;

    /* The stacks and their tools:
       'yyss': related to states.
       'yyvs': related to semantic values.

       Refer to the stacks through separate pointers, to allow yyoverflow
       to reallocate them elsewhere.  */

    /* The state stack.  */
    yytype_int16 yyssa[YYINITDEPTH];
    yytype_int16 *yyss;
    yytype_int16 *yyssp;

    /* The semantic value stack.  */
    YYSTYPE yyvsa[YYINITDEPTH];
    YYSTYPE *yyvs;
    YYSTYPE *yyvsp;

    YYSIZE_T yystacksize;

  int yyn;
  int yyresult;
  /* Lookahead token as an internal (translated) token number.  */
  int yytoken = 0;
  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;

#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  yyssp = yyss = yyssa;
  yyvsp = yyvs = yyvsa;
  yystacksize = YYINITDEPTH;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY; /* Cause a token to be read.  */
  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
        /* Give user a chance to reallocate the stack.  Use copies of
           these so that the &'s don't force the real ones into
           memory.  */
        YYSTYPE *yyvs1 = yyvs;
        yytype_int16 *yyss1 = yyss;

        /* Each stack pointer address is followed by the size of the
           data in use in that stack, in bytes.  This used to be a
           conditional around just the two extra args, but that might
           be undefined if yyoverflow is a macro.  */
        yyoverflow (YY_("memory exhausted"),
                    &yyss1, yysize * sizeof (*yyssp),
                    &yyvs1, yysize * sizeof (*yyvsp),
                    &yystacksize);

        yyss = yyss1;
        yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
        goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
        yystacksize = YYMAXDEPTH;

      {
        yytype_int16 *yyss1 = yyss;
        union yyalloc *yyptr =
          (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
        if (! yyptr)
          goto yyexhaustedlab;
        YYSTACK_RELOCATE (yyss_alloc, yyss);
        YYSTACK_RELOCATE (yyvs_alloc, yyvs);
#  undef YYSTACK_RELOCATE
        if (yyss1 != yyssa)
          YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;

      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
                  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
        YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  if (yystate == YYFINAL)
    YYACCEPT;

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     lookahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to lookahead token.  */
  yyn = yypact[yystate];
  if (yypact_value_is_default (yyn))
    goto yydefault;

  /* Not known => get a lookahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid lookahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = yylex ();
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yytable_value_is_error (yyn))
        goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the lookahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token.  */
  yychar = YYEMPTY;

  yystate = yyn;
  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     '$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 7:
#line 384 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {yyerror ("Unmatched )"); YYERROR;}
#line 1732 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 8:
#line 385 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {yyerror ("Unmatched ]"); YYERROR;}
#line 1738 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 9:
#line 388 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {init_buffer();}
#line 1744 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 10:
#line 389 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {(yyval.str) = strdup (buffer);}
#line 1750 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 13:
#line 396 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append (mod_yytext);}
#line 1756 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 14:
#line 397 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append (mod_yytext);}
#line 1762 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 15:
#line 398 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append (mod_yytext);}
#line 1768 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 16:
#line 400 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append("[");}
#line 1774 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 17:
#line 402 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append("]");}
#line 1780 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 18:
#line 404 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append("(");}
#line 1786 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 19:
#line 406 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {append(")");}
#line 1792 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 20:
#line 409 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fputs (mod_yytext, mod_yyout);}
#line 1798 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 21:
#line 410 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fputs (mod_yytext, mod_yyout);}
#line 1804 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 22:
#line 411 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fputs (mod_yytext, mod_yyout);}
#line 1810 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 23:
#line 412 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fputs ("[]", mod_yyout);}
#line 1816 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 24:
#line 414 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {putc ('[', mod_yyout);}
#line 1822 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 25:
#line 416 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {putc (']', mod_yyout);}
#line 1828 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 26:
#line 417 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fputs ("()", mod_yyout);}
#line 1834 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 27:
#line 419 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {putc ('(', mod_yyout);}
#line 1840 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 28:
#line 421 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {putc (')', mod_yyout);}
#line 1846 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 29:
#line 425 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.init");}
#line 1852 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 30:
#line 427 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "*(mif_private->callback)");}
#line 1858 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 31:
#line 429 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "Mif_Private_t *mif_private");}
#line 1864 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 32:
#line 431 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.anal_type");}
#line 1870 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 33:
#line 433 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.anal_init");}
#line 1876 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 34:
#line 435 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.call_type");}
#line 1882 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 35:
#line 437 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.time");}
#line 1888 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 36:
#line 439 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.frequency");}
#line 1894 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 37:
#line 441 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.temperature");}
#line 1900 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 38:
#line 443 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {fprintf (mod_yyout, "mif_private->circuit.t[%s]", (yyvsp[-1].str));}
#line 1906 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 39:
#line 445 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), PARAM);
			    fprintf (mod_yyout, "mif_private->param[%d]->element[%s]",
				     i, subscript ((yyvsp[-1].sub_id)));
			    put_type (mod_yyout, mod_ifs_table->param[i].type);
			   }
#line 1916 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 40:
#line 451 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_id ((yyvsp[-1].sub_id), PARAM);
			    fprintf (mod_yyout, "mif_private->param[%d]->size", i);}
#line 1923 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 41:
#line 454 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_id ((yyvsp[-1].sub_id), PARAM);
			    fprintf (mod_yyout, "mif_private->param[%d]->is_null", i);}
#line 1930 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 42:
#line 457 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_id ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, "mif_private->conn[%d]->size", i);}
#line 1937 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 43:
#line 460 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_id ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, "mif_private->conn[%d]->is_null", i);}
#line 1944 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 44:
#line 464 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-3].sub_id), CONN);
			    int j = valid_subid ((yyvsp[-1].sub_id), CONN);
			    check_dir (i, CMPP_OUT, "PARTIAL");
			    check_dir (j, CMPP_IN, "PARTIAL");
			    fprintf (mod_yyout, "mif_private->conn[%d]->port[%s]->partial[%d].port[%s]",
				     i, subscript((yyvsp[-3].sub_id)), j, subscript((yyvsp[-1].sub_id)));}
#line 1955 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 45:
#line 472 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-3].sub_id), CONN);
			    int j = valid_subid ((yyvsp[-1].sub_id), CONN);
			    check_dir (i, CMPP_OUT, "AC_GAIN");
			    check_dir (j, CMPP_IN, "AC_GAIN");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->ac_gain[%d].port[%s]",
				     i, subscript((yyvsp[-3].sub_id)), j, subscript((yyvsp[-1].sub_id)));}
#line 1967 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 46:
#line 480 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), STATIC_VAR);
			    fprintf (mod_yyout, 
				    "mif_private->inst_var[%d]->element[%s]",
				     i, subscript((yyvsp[-1].sub_id)));
			    if (mod_ifs_table->inst_var[i].is_array
				&& !((yyvsp[-1].sub_id).has_subscript)) {
			       /* null - eg. for malloc lvalue */
			    } else {
			       put_type (mod_yyout, 
					mod_ifs_table->inst_var[i].type);
			    } }
#line 1983 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 47:
#line 492 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), STATIC_VAR);
			    fprintf (mod_yyout, "mif_private->inst_var[%d]->size",
				    i);}
#line 1991 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 48:
#line 496 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), STATIC_VAR);
			    fprintf (mod_yyout, "mif_private->inst_var[%d]",
				    i);}
#line 1999 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 49:
#line 500 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    check_dir (i, CMPP_OUT, "OUTPUT_DELAY");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->delay", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2009 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 50:
#line 506 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    check_dir (i, CMPP_OUT, "CHANGED");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->changed", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2019 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 51:
#line 512 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_IN, "INPUT");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->input",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);}
#line 2031 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 52:
#line 520 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_IN, "INPUT_TYPE");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->type_str",
				     i, subscript((yyvsp[-1].sub_id))); }
#line 2041 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 53:
#line 526 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_OUT, "OUTPUT_TYPE");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->type_str",
				     i, subscript((yyvsp[-1].sub_id))); }
#line 2051 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 54:
#line 532 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_IN, "INPUT_STRENGTH");
			    fprintf (mod_yyout, 
				     "((Digital_t*)(mif_private->conn[%d]->port[%s]->input",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);
			    fprintf (mod_yyout, "))->strength");}
#line 2064 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 55:
#line 541 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_IN, "INPUT_STATE");
			    fprintf (mod_yyout, 
				     "((Digital_t*)(mif_private->conn[%d]->port[%s]->input",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);
			    fprintf (mod_yyout, "))->state");}
#line 2077 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 56:
#line 550 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_OUT, "OUTPUT");
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->output",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);}
#line 2089 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 57:
#line 558 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_OUT, "OUTPUT_STRENGTH");
			    fprintf (mod_yyout, 
				     "((Digital_t*)(mif_private->conn[%d]->port[%s]->output",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);
			    fprintf (mod_yyout, "))->strength");}
#line 2102 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 58:
#line 567 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
 			    check_dir (i, CMPP_OUT, "OUTPUT_STATE");
			    fprintf (mod_yyout, 
				     "((Digital_t*)(mif_private->conn[%d]->port[%s]->output",
				     i, subscript((yyvsp[-1].sub_id)));
			    put_conn_type (mod_yyout, 
			       mod_ifs_table->conn[i].allowed_port_type[0]);
			    fprintf (mod_yyout, "))->state");}
#line 2115 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 59:
#line 576 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->changed", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2124 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 60:
#line 581 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->load", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2133 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 61:
#line 586 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->total_load", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2142 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 62:
#line 591 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {int i = valid_subid ((yyvsp[-1].sub_id), CONN);
			    fprintf (mod_yyout, 
				     "mif_private->conn[%d]->port[%s]->msg", i,
				     subscript((yyvsp[-1].sub_id)));}
#line 2151 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 64:
#line 599 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {(yyval.sub_id) = (yyvsp[-3].sub_id);
			   (yyval.sub_id).has_subscript = true;
			   (yyval.sub_id).subscript = (yyvsp[-1].str);}
#line 2159 "mod_yacc.c" /* yacc.c:1646  */
    break;

  case 65:
#line 605 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1646  */
    {(yyval.sub_id).has_subscript = false;
			      (yyval.sub_id).id = strdup (mod_yytext);}
#line 2166 "mod_yacc.c" /* yacc.c:1646  */
    break;


#line 2170 "mod_yacc.c" /* yacc.c:1646  */
      default: break;
    }
  /* User semantic actions sometimes alter yychar, and that requires
     that yytoken be updated with the new translation.  We take the
     approach of translating immediately before every use of yytoken.
     One alternative is translating here after every semantic action,
     but that translation would be missed if the semantic action invokes
     YYABORT, YYACCEPT, or YYERROR immediately after altering yychar or
     if it invokes YYBACKUP.  In the case of YYABORT or YYACCEPT, an
     incorrect destructor might then be invoked immediately.  In the
     case of YYERROR or YYBACKUP, subsequent parser actions might lead
     to an incorrect destructor call or verbose syntax error message
     before the lookahead is translated.  */
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;

  /* Now 'shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*--------------------------------------.
| yyerrlab -- here on detecting error.  |
`--------------------------------------*/
yyerrlab:
  /* Make sure we have latest lookahead translation.  See comments at
     user semantic actions for why this is necessary.  */
  yytoken = yychar == YYEMPTY ? YYEMPTY : YYTRANSLATE (yychar);

  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
# define YYSYNTAX_ERROR yysyntax_error (&yymsg_alloc, &yymsg, \
                                        yyssp, yytoken)
      {
        char const *yymsgp = YY_("syntax error");
        int yysyntax_error_status;
        yysyntax_error_status = YYSYNTAX_ERROR;
        if (yysyntax_error_status == 0)
          yymsgp = yymsg;
        else if (yysyntax_error_status == 1)
          {
            if (yymsg != yymsgbuf)
              YYSTACK_FREE (yymsg);
            yymsg = (char *) YYSTACK_ALLOC (yymsg_alloc);
            if (!yymsg)
              {
                yymsg = yymsgbuf;
                yymsg_alloc = sizeof yymsgbuf;
                yysyntax_error_status = 2;
              }
            else
              {
                yysyntax_error_status = YYSYNTAX_ERROR;
                yymsgp = yymsg;
              }
          }
        yyerror (yymsgp);
        if (yysyntax_error_status == 2)
          goto yyexhaustedlab;
      }
# undef YYSYNTAX_ERROR
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse lookahead token after an
         error, discard it.  */

      if (yychar <= YYEOF)
        {
          /* Return failure if at end of input.  */
          if (yychar == YYEOF)
            YYABORT;
        }
      else
        {
          yydestruct ("Error: discarding",
                      yytoken, &yylval);
          yychar = YYEMPTY;
        }
    }

  /* Else will try to reuse lookahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule whose action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;      /* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (!yypact_value_is_default (yyn))
        {
          yyn += YYTERROR;
          if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
            {
              yyn = yytable[yyn];
              if (0 < yyn)
                break;
            }
        }

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
        YYABORT;


      yydestruct ("Error: popping",
                  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  YY_IGNORE_MAYBE_UNINITIALIZED_BEGIN
  *++yyvsp = yylval;
  YY_IGNORE_MAYBE_UNINITIALIZED_END


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#if !defined yyoverflow || YYERROR_VERBOSE
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEMPTY)
    {
      /* Make sure we have latest lookahead translation.  See comments at
         user semantic actions for why this is necessary.  */
      yytoken = YYTRANSLATE (yychar);
      yydestruct ("Cleanup: discarding lookahead",
                  yytoken, &yylval);
    }
  /* Do not reclaim the symbols of the rule whose action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
                  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  return yyresult;
}
#line 609 "../../../../src/xspice/cmpp/mod_yacc.y" /* yacc.c:1906  */

