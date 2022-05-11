/**
 * @file    Standard.h
 * @version 1.4.0
 * @authors Anton Chernov
 * @date    15.05.2017
 * @date    05.04.2022
 */

#ifndef STANDARD_H
#define STANDARD_H
/******************************* Definition ***********************************/
#define BIT(bit)            (1 << (bit))
#define NULLPTR             ((void*)0)
#define UNUSED_PARAM(param) ((void)param)
#define DISCARD_RETURN(func)(UNUSED_PARAM(func))
#define FALSE               (0)
#define TRUE                (1)
/******************************************************************************/
typedef unsigned char       BYTE;   ///<  8-bit unsigned integer.
typedef signed char         SBYTE;  ///<  8-bit signed integer.
typedef unsigned int        WORD;   ///< 16 bit unsigned integer.
typedef signed int          SWORD;  ///< 16 bit signed integer.
typedef unsigned long int   DWORD;  ///< 32 bit unsigned integer.
typedef signed long int     SDWORD; ///< 32 bit signed integer.
typedef unsigned long long  QWORD;  ///< 64 bit unsigned integer.
typedef signed long long    SQWORD; ///< 64 bit signed integer.
/******************************************************************************/
typedef unsigned char       BOOLEAN; ///<  8 bits - use TRUE or FALSE for this
typedef char                CHAR;    ///<  For use in character arrays only.
typedef unsigned char       UINT8;   ///<  8-bit unsigned integer.
typedef signed char         SINT8;   ///<  8-bit signed integer.
typedef unsigned short      UINT16;  ///< 16 bit unsigned integer.
typedef signed short        SINT16;  ///< 16 bit signed integer.
typedef unsigned int        UINT32;  ///< 32 bit unsigned integer.
typedef signed int          SINT32;  ///< 32 bit signed integer.
typedef unsigned long long  UINT64;  ///< 64 bit unsigned integer.
typedef signed long long    SINT64;  ///< 64 bit signed integer.
typedef float               FLOAT;   ///< 32 bit
typedef float               FLOAT32; ///< 32 bit
typedef double              DOUBLE;  ///< 64 bit
typedef double              FLOAT64; ///< 64 bit

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/************************ Something like a <limits.h> ***************************/
#define SBYTE_MIN   -128
#define SWORD_MIN   -32768
#define SDWORD_MIN  -2147483648
#define SQWORD_MIN  -9223372036854775808
#define SBYTE_MAX   127
#define SWORD_MAX   32767
#define SDWORD_MAX  2147483647
#define SQWORD_MAX  9223372036854775807
#define BYTE_MAX    255
#define WORD_MAX    65535
#define DWORD_MAX   4294967295
#define QWORD_MAX   18446744073709551615
/********************************************************************************/
#endif /* STANDARD_H */
