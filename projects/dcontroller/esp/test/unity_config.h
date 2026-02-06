#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

#include <stdio.h>
#include <stdlib.h>

#define UNITY_EXCLUDE_FLOAT_PRINT
#define UNITY_INCLUDE_FLOAT
#define UNITY_FLOAT_PRECISION 6

// Custom output function
#define UNITY_OUTPUT_CHAR(c) putchar(c)
#define UNITY_OUTPUT_FLUSH() fflush(stdout)

// Custom exit function
#define UNITY_EXIT(code) exit(code)

#endif // UNITY_CONFIG_H
