/*
 * Copyright 2020 AM Maree/KSS Technologies (Pty) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * m90e26_cmds.h
 */

#pragma		once

#include	"x_struct_union.h"

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
