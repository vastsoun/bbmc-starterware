/**
* \file  cmdline.h
*
* \brief Prototypes for command line processing functions.
* 
*   TODO
*/

#ifndef __CMDLINE_H__
#define __CMDLINE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* macro definitions */

#define CMDLINE_BAD_CMD         (-1)
#define CMDLINE_NO_CMD         	(-2)
#define CMDLINE_TOO_MANY_ARGS   (-4)
#define CMDLINE_BAD_RET			(-5)
#define PRINTED_HELP_MENU       (-6)
#define PRINTED_FUNC_HELP       (-7)

/* Defines the maximum number of arguments that can be parsed. */
#ifndef CMDLINE_MAX_ARGS
    #define CMDLINE_MAX_ARGS        16
#endif


/* data structure definitions */

/* Command line function callback type. */
typedef int (*pfnCmdLine)(int argc, char *argv[]);

/* struct for command entries in command table */
typedef struct{
    
    //A pointer to a string containing the name of the command.
    const char *pcCmd;

    //A function pointer to the implementation of the command.
    pfnCmdLine pfnCmd;

    //A pointer to a string of brief help text for the command.
    const char *pcHelp;
}
tCmdLineEntry;

/* function declarations */

extern int CmdLineProcess(char *pcCmdLine, tCmdLineEntry *CmdTable);


#ifdef __cplusplus
}
#endif

#endif // __CMDLINE_H__
