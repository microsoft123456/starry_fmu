/*
 * File      : test.c
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2017-09-07     zoujiachi   	the first version
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "test.h"
#include "global.h"
#include "log.h"

int handle_test_shell_cmd(int argc, char** argv)
{
	Log.console("isfinite case1:%d\n", isfinite(100));
	Log.console("isfinite case2:%d\n", isfinite(sqrt(-1)));
	Log.console("isfinite case3:%d\n", isfinite(100/0));
	
	return 0;
}
