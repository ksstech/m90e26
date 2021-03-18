/*
 * Copyright 2020-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 *
 * m90e26_cmds.h
 */

#pragma		once

#include	"commands.h"

#ifdef __cplusplus
extern "C" {
#endif

// ####################################### Global functions ########################################

int32_t CmndM90C(cli_t * psCLI) ;
int32_t CmndM90D(cli_t * psCLI) ;
int32_t CmndM90L(cli_t * psCLI) ;
int32_t CmndM90N(cli_t * psCLI) ;
int32_t CmndM90O(cli_t * psCLI) ;
int32_t CmndM90P(cli_t * psCLI) ;
int32_t CmndM90S(cli_t * psCLI) ;
int32_t CmndM90Z(cli_t * psCLI) ;

#ifdef __cplusplus
}
#endif
