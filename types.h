/**
 * File:
 *  types.h
 *
 * Notes:
 *  This file contains some type definitions to short cut some common C types
 *
 * History:
 *  04/05/2012 Created by Simon Platten
 */
#ifndef TYPES_H
  #define TYPES_H
#ifndef ulong
  typedef unsigned long ulong;
#endif
#ifndef uint
  typedef unsigned int uint;
#endif
#ifndef ushort
  typedef unsigned short ushort;
#endif
#ifndef byte
  typedef unsigned char byte;
#endif
#ifndef boolean
  typedef enum ebool { FALSE, TRUE } boolean;
#endif
#ifndef NULL
  #define NULL          ((void*)0)
#endif

#endif