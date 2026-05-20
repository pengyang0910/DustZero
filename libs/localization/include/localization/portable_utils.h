/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-08-21 11:29:04
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2020-08-21 11:50:08
 */
#ifndef PORTABLE_UTILS_H
#define PORTABLE_UTILS_H

#include <stdlib.h>

#define HAVE_DRAND48

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_DRAND48
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
static double drand48(void)
{
    return ((double)rand())/RAND_MAX;
}

static void srand48(long int seedval)
{
    srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif