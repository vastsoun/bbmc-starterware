//*****************************************************************************
//
// fontcmsc40.c - Font definition for the 40 point Cmsc font.
//
// Copyright (c) 2008-2010 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 6288 of the Stellaris Graphics Library.
//
//*****************************************************************************

//*****************************************************************************
//
// This file is generated by ftrasterize; DO NOT EDIT BY HAND!
//
//*****************************************************************************

#include "grlib.h"

//*****************************************************************************
//
// Details of this font:
//     Style: cmsc
//     Size: 40 point
//     Bold: no
//     Italic: no
//     Memory usage: 5440 bytes
//
//*****************************************************************************

//*****************************************************************************
//
// The compressed data for the 40 point Cmsc font.
//
//*****************************************************************************
static const unsigned char g_pucCmsc40Data[5238] =
{
      5,  17,   0,  87,  16,  33,   8, 240,  34,  84,  68,  68,
     68,  68,  68,  68,  82,  98,  98,  98,  98,  98,  98,  98,
     98,  98,  98,  98, 113, 240, 240, 130,  84,  68,  82,   0,
     10,  80,  31,  17,   0,   6,  67,  83,  85,  53,  69,  53,
     84,  68, 129, 113, 129, 113, 129, 113, 113, 113, 129, 113,
    113, 113, 129, 113, 113, 113,   0,  56,  16, 110,  34,   0,
     14,  66, 129, 240, 130, 129, 240, 129, 130, 240, 129, 130,
    240, 114, 129, 240, 130, 129, 240, 129, 130, 240, 114, 130,
    240, 114, 129, 240, 130, 129, 240, 129, 130, 240, 114, 130,
    240, 114, 129, 240, 129, 130, 207,  15, 226, 129, 240, 129,
    130, 240, 129, 130, 240, 114, 129, 240, 130, 129, 240, 129,
    130, 239,  15, 194, 129, 240, 129, 130, 240, 114, 130, 240,
    114, 129, 240, 129, 130, 240, 129, 130, 240, 114, 130, 240,
    114, 129, 240, 129, 130, 240, 129, 130, 240, 114, 129, 240,
    130, 129, 240, 129, 130, 240, 129, 130,   0,  10,  96,  80,
     20, 240, 209, 240,  65, 240,  23, 178,  49,  50, 130,  65,
     81,  98,  81,  97,  66,  97,  67,  66,  97,  67,  66,  97,
     67,  67,  81,  66,  83,  81, 196,  49, 198,  17, 217, 202,
    201, 231, 209,  36, 209,  52, 193,  67,  82,  81,  82,  68,
     65,  82,  68,  65,  82,  67,  81,  82,  65, 113,  82,  81,
     97,  66,  97,  97,  66, 113,  81,  50, 146,  49,  34, 198,
    240,  33, 240,  65,   0,  21,  48, 114,  34,  83, 240,  33,
    178,  49, 242, 162,  82, 195, 147,  98, 163, 163,  97,  18,
    100, 178, 129,  38,  34, 163, 129, 146, 179, 129, 130, 195,
    129, 130, 195, 129, 114, 211, 129,  98, 227, 129,  98, 242,
    129,  82, 240,  19,  97,  82, 240,  50,  97,  82, 240,  50,
     81,  82, 240,  82,  49,  82, 240, 131,  98, 129, 240, 114,
     99,  18, 240,  66,  98,  81, 240,  50,  83,  97, 240,  18,
     98, 113, 242,  99, 129, 226,  99, 129, 210, 115, 129, 194,
    131, 129, 194, 131, 129, 178, 147, 129, 162, 178, 114, 162,
    179,  97, 162, 210,  82, 146, 242,  50, 162, 240,  20,   0,
     35,  89,  33,   0,   9,  52, 240, 195,  49, 240, 163,  66,
    240, 146,  97, 240, 131,  97, 240, 131,  97, 240, 131,  97,
    240, 131,  81, 240, 147,  81, 240, 147,  65, 240, 164,  33,
    240, 195,  33, 240, 195,  17, 168, 164, 211, 227, 210, 229,
    193, 225,  35, 177, 225,  52, 161, 209,  84, 129, 210,  99,
    113, 210, 116,  97, 195, 132,  65, 211, 148,  33, 227, 166,
    227, 180, 161,  83, 179, 161,  84, 150, 113, 116,  83,  67,
     81, 182, 150,   0,  42,  16,  18,   9, 240, 211,  85,  69,
     84, 129, 129, 129, 113, 129, 113, 129, 113,   0,  30,  16,
     44,  14, 145, 194, 178, 178, 178, 178, 194, 178, 194, 179,
    178, 179, 179, 179, 178, 179, 179, 179, 179, 179, 179, 179,
    179, 179, 179, 179, 195, 179, 179, 179, 194, 195, 194, 195,
    194, 195, 194, 210, 210, 210, 209,  64,  44,  14,   1, 225,
    225, 210, 210, 210, 194, 210, 195, 179, 194, 195, 179, 179,
    194, 195, 179, 179, 179, 179, 179, 179, 179, 179, 179, 179,
    178, 194, 179, 179, 178, 194, 178, 194, 178, 194, 178, 193,
    193, 193, 193, 208,  44,  19,  98, 240,  34, 240,  34, 240,
     34, 177,  82,  81,  82,  66,  67,  68,  34,  51, 115,  18,
     19, 179,  18, 242, 243,  18, 164,  18,  35,  99,  50,  52,
     66,  66,  67, 162, 240,  34, 240,  34, 240,  34,   0,  56,
     65,  32,   0,  25,  82, 240, 242, 240, 242, 240, 242, 240,
    242, 240, 242, 240, 242, 240, 242, 240, 242, 240, 242, 240,
    242, 240, 242, 240, 242, 240, 242, 240, 242, 240,  47,  13,
    240,  34, 240, 242, 240, 242, 240, 242, 240, 242, 240, 242,
    240, 242, 240, 242, 240, 242, 240, 242, 240, 242, 240, 242,
    240, 242,   0,  26,  16,  18,   9,   0,  30,  67,  85,  69,
     84, 129, 129, 129, 113, 129, 113, 129, 113, 240, 160,   9,
     15,   0,  39,  59,  75,   0,  34,  32,  11,   8,   0,  27,
     18,  84,  68,  82,   0,  10,  80,  85,  20, 241, 240,  50,
    240,  49, 240,  50, 240,  50, 240,  49, 240,  50, 240,  50,
    240,  49, 240,  50, 240,  50, 240,  34, 240,  50, 240,  49,
    240,  50, 240,  50, 240,  49, 240,  50, 240,  50, 240,  49,
    240,  50, 240,  49, 240,  50, 240,  50, 240,  49, 240,  50,
    240,  50, 240,  49, 240,  50, 240,  50, 240,  34, 240,  50,
    240,  49, 240,  50, 240,  50, 240,  49, 240,  50, 240,  50,
    240,  49, 240,  50, 240,  49, 240,  64,  59,  21,   0,  11,
     37, 226,  82, 178, 114, 146, 146, 130, 146, 114, 178,  98,
    178,  98, 179,  67, 179,  67, 179,  67, 179,  67, 179,  67,
    179,  67, 179,  67, 179,  67, 179,  67, 179,  67, 179,  67,
    179,  67, 179,  82, 178,  99, 147,  99, 147, 114, 146, 146,
    114, 178,  82, 214,   0,  27,  64,  35,  17,   0,   9,  49,
    227, 197, 148,  19, 227, 227, 227, 227, 227, 227, 227, 227,
    227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227,
    227, 227, 157,   0,  21,  96,  60,  21,   0,  11,  22, 210,
     84, 145, 132, 113, 164,  97, 164,  83, 164,  68, 148,  68,
    148,  82, 164, 240,  36, 240,  20, 240,  36, 240,  35, 240,
     35, 240,  35, 240,  35, 240,  50, 240,  50, 240,  50, 240,
     50, 240,  50, 145, 130, 161, 114, 162,  98, 177, 111,  95,
      1,  95,   1,   0,  26, 112,  58,  22,   0,  11,  87, 210,
    100, 145, 148, 115, 148, 100, 132, 100, 132, 114, 148, 240,
     51, 240,  67, 240,  51, 240,  51, 240,  51, 231, 240,  99,
    240,  83, 240,  83, 240,  68, 240,  68, 240,  52,  82, 180,
     68, 164,  68, 164,  68, 163,  83, 164,  98, 148, 131,  99,
    215,   0,  28,  96,  60,  22,   0,  12,  81, 240,  82, 240,
     67, 240,  52, 240,  52, 240,  33,  19, 240,  17,  35, 240,
     17,  35, 241,  51, 225,  67, 225,  67, 209,  83, 193,  99,
    193,  99, 177, 115, 161, 131, 146, 131, 145, 147, 129, 163,
    143,   3, 243, 240,  67, 240,  67, 240,  67, 240,  67, 240,
     67, 251,   0,  28,  58,  21,   0,  10,  99, 130, 140, 155,
    168, 209, 240,  81, 240,  81, 240,  81, 240,  81, 240,  81,
    240,  81,  54, 177,  18,  67, 162, 131, 129, 147, 240,  67,
    240,  51, 240,  52, 240,  36,  67, 164,  68, 148,  67, 164,
     67, 163,  81, 180,  97, 163, 129, 131, 162,  83, 213,   0,
     27,  80,  61,  22,   0,  12,   6, 227,  82, 178, 100, 147,
    100, 131, 116, 115, 131, 131, 240,  52, 240,  51, 240,  67,
    240,  52,  53, 164,  33,  67, 132,  17, 115, 101, 132,  85,
    147,  84, 164,  68, 164,  68, 164,  68, 164,  83, 164,  83,
    164,  83, 163, 114, 163, 115, 131, 147, 114, 179,  67, 230,
      0,  28,  96,  61,  22,   0,  11,  17, 240, 111,   2,  95,
      2,  95,   1,  82, 209,  97, 209, 113, 194, 113, 178, 240,
     81, 240,  81, 240,  82, 240,  66, 240,  82, 240,  66, 240,
     82, 240,  67, 240,  67, 240,  51, 240,  67, 240,  67, 240,
     67, 240,  52, 240,  52, 240,  52, 240,  52, 240,  52, 240,
     66,   0,  29,  32,  56,  22,   0,  11, 102, 226,  98, 177,
    147, 129, 178, 129, 194,  98, 194,  98, 194,  99, 178, 101,
    130, 133,  98, 166,  50, 201, 231, 249, 194,  55, 130, 103,
     98, 149,  98, 180,  66, 211,  66, 226,  66, 226,  66, 226,
     67, 209,  98, 194, 114, 162, 147,  99, 199,   0,  28,  96,
     60,  22,   0,  11, 102, 227,  67, 179,  99, 147, 131, 115,
    147, 115, 163,  84, 163,  84, 163,  84, 164,  68, 164,  68,
    164,  68, 164,  83, 164,  83, 149,  99, 133, 115,  97,  20,
    131,  65,  36, 165,  52, 240,  51, 240,  67, 240,  52, 115,
    131, 116, 130, 132, 115, 131, 115, 162,  98, 230,   0,  29,
     17,   8,   0,  14,  18,  84,  68,  82,   0,   9,  98,  84,
     68,  82,   0,  10,  80,  24,   8,   0,  14,   3,  84,  68,
     67,   0,   9,  83,  84,  68,  68, 113, 113, 113,  97, 113,
    113,  97,  97, 240, 128,  54,  30,   0,  36,  98, 240, 180,
    240, 148, 240, 132, 240, 148, 240, 148, 240, 148, 240, 148,
    240, 147, 240, 148, 240, 148, 240, 148, 240, 180, 240, 212,
    240, 212, 240, 212, 240, 228, 240, 212, 240, 212, 240, 212,
    240, 212, 240, 213, 240, 212, 240, 210,   0,  30,  64,  13,
     32,   0,  68,  15,  13,   0,  24,  79,  13,   0,  64,  64,
     52,  30,   0,  33,  98, 240, 212, 240, 228, 240, 212, 240,
    212, 240, 212, 240, 212, 240, 228, 240, 212, 240, 212, 240,
    212, 240, 212, 240, 180, 240, 132, 240, 148, 240, 148, 240,
    148, 240, 148, 240, 132, 240, 148, 240, 148, 240, 148, 240,
    147,   0,  37,  16,  50,  19,   0,   7,  87, 162,  99, 113,
    162,  83, 147,  68, 131,  68, 131,  82, 147, 243, 243, 243,
    240,  18, 240,  18, 240,  18, 240,  33, 240,  34, 240,  33,
    240,  49, 240,  49, 240,  49, 240,  49,   0,  11,  83, 240,
     20, 244, 243,   0,  25,  16, 103,  32,   0,  13,  39, 240,
    130, 115, 240,  50, 194, 241, 242, 209, 240,  49, 177, 117,
    129, 145,  99,  66, 113, 129,  82, 114,  97, 113,  83, 129,
    113,  97,  67, 163,  65,  81,  83, 163,  66,  65,  67, 179,
     81,  65,  67, 179,  81,  65,  67, 179,  81,  65,  67, 179,
     81,  65,  67, 179,  81,  65,  67, 179,  81,  66,  67, 163,
     81,  81,  67, 163,  81,  81,  83, 132,  65, 113,  82, 117,
     65, 113,  99,  66,  35,  33, 145, 117,  84, 177, 240, 240,
     33, 240, 240,  34, 240,  19, 195, 164, 240,  58,   0,  41,
     64,  82,  33,   0,  10,   1, 240, 240,  19, 240, 243, 240,
    243, 240, 229, 240, 209,  19, 240, 194,  20, 240, 177,  36,
    240, 177,  51, 240, 162,  52, 240, 145,  68, 240, 145,  83,
    240, 129, 100, 240, 113, 115, 240, 113, 115, 240,  97, 132,
    240,  81, 147, 240,  66, 148, 240,  63, 240,  49, 179, 240,
     33, 196, 240,  17, 196, 240,  17, 211, 241, 228, 225, 228,
    225, 244, 194, 244, 180, 229, 121, 155,   0,  41,  96,  63,
     30,   0,  11,  47,   4, 243, 164, 211, 196, 179, 212, 163,
    212, 163, 228, 147, 228, 147, 228, 147, 228, 147, 212, 163,
    211, 179, 195, 195, 163, 238, 240,  19, 180, 195, 211, 179,
    227, 163, 228, 147, 244, 131, 244, 131, 244, 131, 244, 131,
    244, 131, 228, 147, 228, 147, 212, 163, 180, 143,   5,   0,
     38,  96,  77,  29,   0,  12,  39,  97, 196,  83,  50, 163,
    162,  34, 147, 212, 131, 228, 115, 240,  19, 100, 240,  34,
     99, 240,  50,  84, 240,  50,  84, 240,  65,  68, 240,  81,
     68, 240, 164, 240, 164, 240, 164, 240, 164, 240, 164, 240,
    164, 240, 180, 240,  65,  84, 240,  65,  99, 240,  65, 100,
    240,  33, 131, 240,  33, 147, 241, 179, 209, 212, 161, 240,
     20,  98, 240,  87,   0,  37,  80,  80,  32,   0,  12,  15,
      3, 240,  51, 164, 243, 195, 227, 212, 195, 228, 179, 243,
    179, 240,  19, 163, 240,  19, 163, 240,  20, 147, 240,  35,
    147, 240,  36, 131, 240,  36, 131, 240,  36, 131, 240,  36,
    131, 240,  36, 131, 240,  36, 131, 240,  36, 131, 240,  36,
    131, 240,  35, 147, 240,  20, 147, 240,  20, 147, 240,  19,
    163, 243, 179, 243, 179, 227, 195, 195, 227, 164, 191,   3,
      0,  41,  96,  77,  30,   0,  11,  47,   8, 180, 181, 164,
    211, 164, 226, 164, 226, 164, 241, 164, 241, 164, 241, 164,
    129,  97, 164, 129, 240,  36, 129, 240,  36, 129, 240,  36,
    114, 240,  45, 240,  36,  99, 240,  36, 129, 240,  36, 129,
    240,  36, 129, 129, 132, 129, 113, 148, 240,  17, 148, 240,
     17, 148, 240,  17, 148, 240,  17, 148, 242, 148, 226, 164,
    226, 164, 196, 111,   9,   0,  38,  32,  62,  28,   0,  10,
     79,   8, 148, 180, 148, 210, 148, 225, 148, 226, 132, 226,
    132, 241, 132, 241, 132, 241, 132, 129,  97, 132, 129, 244,
    129, 244, 129, 244, 114, 253, 244,  99, 244, 129, 244, 129,
    244, 129, 244, 129, 244, 240, 148, 240, 148, 240, 148, 240,
    148, 240, 148, 240, 148, 240,  93,   0,  36, 112,  79,  33,
      0,  13, 103, 113, 244,  98,  66, 212, 162,  19, 195, 213,
    179, 244, 163, 240,  35, 148, 240,  35, 147, 240,  66, 132,
    240,  66, 132, 240,  66, 131, 240,  97, 116, 240, 228, 240,
    228, 240, 228, 240, 228, 240, 228, 240, 228, 220,  84, 240,
     36, 132, 240,  36, 147, 240,  36, 148, 240,  20, 163, 240,
     20, 179, 244, 195, 213, 212, 177,  19, 244, 114,  50, 240,
     55, 113,   0,  42,  16,  63,  33,   0,  12,  60,  92, 132,
    212, 196, 212, 196, 212, 196, 212, 196, 212, 196, 212, 196,
    212, 196, 212, 196, 212, 196, 212, 196, 212, 196, 212, 207,
      6, 196, 212, 196, 212, 196, 212, 196, 212, 196, 212, 196,
    212, 196, 212, 196, 212, 196, 212, 196, 212, 196, 212, 196,
    212, 196, 212, 140,  92,   0,  41,  96,  35,  16,   0,   6,
     12, 132, 196, 196, 196, 196, 196, 196, 196, 196, 196, 196,
    196, 196, 196, 196, 196, 196, 196, 196, 196, 196, 196, 196,
    196, 196, 196, 140,   0,  20,  64,  60,  21,   0,   8,  76,
    243, 240,  51, 240,  51, 240,  51, 240,  51, 240,  51, 240,
     51, 240,  51, 240,  51, 240,  51, 240,  51, 240,  51, 240,
     51, 240,  51, 240,  51, 240,  51, 240,  51, 240,  51, 240,
     51, 240,  51, 130, 131, 116, 115, 116, 115, 116,  99, 146,
    115, 161,  84, 198,   0,  27,  96,  81,  34,   0,  12, 108,
    137, 148, 228, 196, 226, 228, 225, 244, 209, 240,  20, 178,
    240,  36, 162, 240,  52, 146, 240,  68, 130, 240,  84, 114,
    240, 100,  98, 240, 116,  83, 240, 116,  69, 240, 100,  54,
    240, 100,  33,  52, 240,  84,  17,  84, 240,  69, 100, 240,
     68, 132, 240,  52, 148, 240,  36, 148, 240,  36, 164, 240,
     20, 180, 244, 181, 228, 196, 228, 212, 212, 213, 196, 214,
    124, 123,   0,  43,  63,  27,   0,  10,  29, 240,  52, 240,
    132, 240, 132, 240, 132, 240, 132, 240, 132, 240, 132, 240,
    132, 240, 132, 240, 132, 240, 132, 240, 132, 240, 132, 240,
    132, 240, 132, 240, 132, 240, 132, 225, 132, 225, 132, 225,
    132, 225, 132, 210, 132, 210, 132, 209, 148, 194, 148, 179,
    148, 149,  95,   7,   0,  34,  48, 114,  38,   0,  14,  40,
    240,  56, 132, 240,  52, 193,  19, 240,  17,  19, 193,  19,
    240,  17,  19, 193,  19, 240,  17,  19, 193,  35, 225,  35,
    193,  35, 225,  35, 193,  51, 193,  51, 193,  51, 193,  51,
    193,  51, 193,  51, 193,  67, 161,  67, 193,  67, 161,  67,
    193,  67, 145,  83, 193,  83, 129,  83, 193,  83, 129,  83,
    193,  99,  97,  99, 193,  99,  97,  99, 193,  99,  81, 115,
    193, 115,  65, 115, 193, 115,  65, 115, 193, 131,  33, 131,
    193, 131,  33, 131, 193, 131,  33, 131, 193, 148, 147, 193,
    148, 147, 193, 147, 163, 179, 146, 163, 137,  98, 107,   0,
     48,  83,  32,   0,  12,   8, 185, 133, 211, 181, 225, 193,
     20, 209, 193,  36, 193, 193,  36, 193, 193,  52, 177, 193,
     68, 161, 193,  83, 161, 193,  84, 145, 193, 100, 129, 193,
    116, 113, 193, 116, 113, 193, 132,  97, 193, 148,  81, 193,
    148,  81, 193, 164,  65, 193, 180,  49, 193, 180,  49, 193,
    196,  33, 193, 212,  17, 193, 229, 193, 229, 193, 244, 193,
    240,  19, 193, 240,  19, 179, 240,  18, 137, 225,   0,  41,
     83,  32,   0,  13,  40, 240, 115,  99, 240,  51, 163, 243,
    195, 211, 227, 179, 240,  19, 147, 240,  51, 131, 240,  51,
    116, 240,  52,  99, 240,  83,  84, 240,  84,  68, 240,  84,
     68, 240,  84,  68, 240,  84,  68, 240,  84,  68, 240,  84,
     68, 240,  84,  68, 240,  84,  84, 240,  52, 100, 240,  52,
    115, 240,  51, 132, 240,  20, 147, 240,  19, 179, 227, 211,
    195, 243, 163, 240,  51,  99, 240, 120,   0,  41,  96,  61,
     29,   0,  10, 127,   3, 243, 164, 195, 195, 179, 211, 163,
    227, 147, 228, 131, 228, 131, 228, 131, 228, 131, 228, 131,
    227, 147, 211, 163, 195, 179, 164, 206, 243, 240, 179, 240,
    179, 240, 179, 240, 179, 240, 179, 240, 179, 240, 179, 240,
    179, 240, 179, 240, 179, 240, 179, 240, 124,   0,  38,  48,
    110,  32,   0,  13,  39, 240, 131,  84, 240,  51, 148, 243,
    195, 211, 227, 179, 240,  19, 148, 240,  20, 131, 240,  51,
    116, 240,  52, 100, 240,  52,  99, 240,  84,  68, 240,  84,
     68, 240,  84,  68, 240,  84,  68, 240,  84,  68, 240,  84,
     68, 240,  84,  68, 240,  84,  83, 240,  83, 100, 240,  52,
    100, 240,  51, 131, 240,  51, 147, 100,  99, 163,  81,  50,
     67, 195,  49,  82,  35, 242,  33,  97,  19, 240,  37,  83,
    240, 121, 129, 240,  99, 113, 240, 114, 113, 240, 115,  82,
    240, 116,  51, 240, 136, 240, 152, 240, 166, 240, 196,   0,
      8, 112,  73,  33,   0,  12,  63,   2, 240,  83, 147, 240,
     51, 179, 240,  19, 195, 243, 196, 227, 212, 211, 212, 211,
    212, 211, 212, 211, 212, 211, 196, 227, 195, 243, 179, 240,
     19, 147, 240,  61, 240,  83, 132, 240,  51, 163, 240,  35,
    164, 240,  19, 180, 243, 180, 243, 180, 243, 180, 243, 180,
    243, 181, 227, 181,  81, 131, 196,  81, 131, 212,  49, 105,
    197,   0,  42,  60,  23,   0,   9,  54,  65, 147,  83,  33,
    131, 132, 115, 163, 114, 194,  99, 194,  99, 194,  99, 209,
    100, 193, 101, 177, 117, 240,  56, 240,  28, 205, 204, 249,
    240,  53, 240,  69, 240,  68,  65, 243,  65, 243,  66, 227,
     66, 227,  67, 210,  83, 195,  85, 147,  98,  35,  99, 113,
    102,   0,  30,  72,  32,   0,  12,  31,  11, 111,  11,  99,
    132, 131,  98, 148, 146,  97, 164, 161,  97, 164, 162,  66,
    164, 162,  65, 180, 177,  65, 180, 177,  65, 180, 177, 240,
     20, 240, 212, 240, 212, 240, 212, 240, 212, 240, 212, 240,
    212, 240, 212, 240, 212, 240, 212, 240, 212, 240, 212, 240,
    212, 240, 212, 240, 212, 240, 212, 240, 213, 240, 111,   1,
      0,  41,  32,  66,  32,   0,  12,  12, 121, 132, 227, 180,
    241, 196, 241, 196, 241, 196, 241, 196, 241, 196, 241, 196,
    241, 196, 241, 196, 241, 196, 241, 196, 241, 196, 241, 196,
    241, 196, 241, 196, 241, 196, 241, 196, 241, 196, 241, 196,
    241, 211, 225, 228, 209, 243, 194, 240,  19, 177, 240,  51,
    145, 240,  84,  82, 240, 135,   0,  41,  96,  84,  34,   0,
     12, 107, 184, 132, 240,  20, 164, 240,  18, 211, 240,  18,
    212, 241, 228, 241, 244, 210, 244, 209, 240,  35, 194, 240,
     36, 177, 240,  67, 177, 240,  68, 146, 240,  68, 145, 240,
     99, 130, 240, 100, 113, 240, 131, 113, 240, 132,  82, 240,
    132,  81, 240, 163,  66, 240, 164,  49, 240, 195,  49, 240,
    196,  18, 240, 196,  17, 240, 229, 240, 228, 240, 240,  19,
    240, 240,  19, 240, 240,  18,   0,  44,  96, 120,  46,   0,
     17,  43,  91, 105, 117, 181, 196, 164, 195, 225, 196, 196,
    209, 211, 196, 194, 212, 180, 193, 228, 181, 177, 243, 181,
    161, 240,  20, 145,  35, 161, 240,  20, 145,  36, 145, 240,
     35, 145,  36, 129, 240,  52, 113,  67, 129, 240,  52, 113,
     68, 113, 240,  67, 113,  68,  97, 240,  84,  81,  99,  97,
    240,  84,  81, 100,  81, 240,  99,  81, 100,  65, 240, 116,
     49, 132,  49, 240, 116,  49, 132,  49, 240, 131,  49, 132,
     33, 240, 148,  17, 164,  17, 240, 148,  17, 164,  17, 240,
    163,  17, 180, 240, 180, 196, 240, 180, 196, 240, 195, 210,
    240, 210, 226, 240, 225, 226,   0,  59,  64,  79,  33,   0,
     12,  59, 121, 165, 165, 228, 178, 240,  20, 177, 240,  52,
    146, 240,  68, 114, 240,  85,  97, 240, 116,  82, 240, 132,
     50, 240, 149,  33, 240, 180,  18, 240, 197, 240, 213, 240,
    228, 240, 229, 240, 214, 240, 177,  36, 240, 162,  52, 240,
    145,  84, 240, 113, 100, 240,  98, 116, 240,  66, 148, 240,
     49, 164, 240,  34, 180, 242, 212, 226, 212, 197, 197, 137,
    155,   0,  41,  96,  83,  34,   0,  12, 107, 169, 117, 244,
    180, 242, 228, 225, 244, 210, 240,  20, 193, 240,  37, 162,
    240,  52, 146, 240,  84, 129, 240, 100, 114, 240, 116,  97,
    240, 148,  65, 240, 164,  50, 240, 180,  33, 240, 211,  17,
    240, 229, 240, 243, 240, 240,  19, 240, 240,  19, 240, 240,
     19, 240, 240,  19, 240, 240,  19, 240, 240,  19, 240, 240,
     19, 240, 240,  19, 240, 240,  19, 240, 240,  19, 240, 203,
      0,  44,  32,  64,  25,   0,   9,  79,   5,  85, 165,  83,
    196,  98, 196, 114, 181, 113, 196, 129, 180, 145, 180, 145,
    164, 240,  84, 240, 100, 240,  84, 240,  84, 240, 100, 240,
     84, 240,  85, 240,  84, 240,  84, 161, 164, 161, 148, 177,
    132, 193, 132, 193, 116, 194, 100, 210, 100, 195,  84, 196,
     68, 182,  79,   6,   0,  31,  96,  43,  10, 166,  66, 130,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    134,  64,  31,  17,   0,   6,  97, 113, 113, 114, 113, 113,
    113, 113, 129, 113, 113, 113, 129, 113, 129, 113, 132,  68,
     85,  53,  84,  68,  83,  83,   0,  55, 112,  43,  10, 166,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130, 130,
    130, 130,  70,  64,  17,  15,   0,   6,  33, 211, 178,  18,
    146,  50, 113, 113,  81, 145,   0,  60,  64,  11,   8, 240,
    240,  50,  84,  68,  82,   0,  33,  80,  18,   9, 240, 241,
    114, 113, 113, 129, 113, 129, 129, 132,  85,  84,  83,   0,
     29, 112,  56,  26,   0,  30,  66, 240, 146, 240, 147, 240,
    116, 240, 116, 240,  97,  35, 240,  81,  35, 240,  81,  35,
    240,  65,  67, 240,  49,  67, 240,  49,  68, 240,  17,  99,
    240,  17,  99, 241, 131, 236, 225, 131, 209, 163, 193, 163,
    178, 179, 162, 179, 147, 180, 102, 136,   0,  33,  46,  23,
      0,  28, 110, 195, 115, 163, 131, 147, 147, 131, 147, 131,
    147, 131, 147, 131, 147, 131, 131, 147, 115, 171, 195, 131,
    147, 147, 131, 162, 131, 163, 115, 163, 115, 163, 115, 163,
    115, 147, 131, 131, 111,   0,  29,  96,  51,  24,   0,  31,
      6,  81, 163,  82,  34, 131, 148, 114, 195,  99, 210,  83,
    226,  83, 241,  82, 240,  17,  67, 240,  99, 240,  99, 240,
     99, 240,  99, 240,  99, 240,  17,  83, 241,  83, 241,  99,
    209, 131, 193, 147, 161, 180,  98, 246,   0,  31,  32,  47,
     25,   0,  31,  47, 211, 116, 179, 147, 163, 163, 147, 178,
    147, 179, 131, 194, 131, 195, 115, 195, 115, 195, 115, 195,
    115, 195, 115, 195, 115, 195, 115, 194, 131, 179, 131, 178,
    147, 163, 147, 147, 163, 115, 158,   0,  32,  80,  50,  23,
      0,  28, 111,   3, 131, 147, 131, 162, 131, 177, 131, 177,
    131, 177, 131,  97,  65, 131,  97, 211,  97, 211,  82, 218,
    211,  82, 211,  97, 211,  97,  81, 115, 193, 115, 193, 115,
    193, 115, 178, 115, 178, 115, 148,  79,   3,   0,  29,  48,
     47,  22,   0,  27,  79,   3, 115, 147, 115, 162, 115, 177,
    115, 177, 115, 177, 115,  97,  65, 115,  97, 195,  97, 195,
     82, 202, 195,  82, 195,  97, 195,  97, 195,  97, 195, 240,
     67, 240,  67, 240,  67, 240,  67, 240,  26,   0,  29,  52,
     26,   0,  33,  70,  81, 180,  82,  34, 162, 164, 146, 195,
    131, 210, 115, 226, 115, 241,  99, 240,  17,  99, 240, 131,
    240, 131, 240, 131, 240, 131, 169,  82, 227, 115, 211, 115,
    211, 131, 195, 147, 179, 163, 148, 195,  98,  18, 231,  65,
      0,  33,  32,  48,  25,   0,  31,  41,  57, 115, 147, 163,
    147, 163, 147, 163, 147, 163, 147, 163, 147, 163, 147, 163,
    147, 163, 147, 175, 163, 147, 163, 147, 163, 147, 163, 147,
    163, 147, 163, 147, 163, 147, 163, 147, 163, 147, 121,  57,
      0,  31,  96,  28,  13,   0,  16,  41, 115, 163, 163, 163,
    163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163, 163,
    163, 163, 163, 121,   0,  16,  96,  32,  17,   0,  21, 105,
    195, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227,
    227, 227, 227,  99,  83,  99,  83,  99,  82, 129,  67, 165,
      0,  22,  64,  54,  26,   0,  32,  73, 118, 115, 164, 147,
    162, 179, 146, 195, 130, 211, 114, 227,  98, 243,  81, 240,
     35,  66, 240,  35,  52, 240,  19,  33,  19, 240,  19,  17,
     51, 244,  83, 227, 100, 211, 115, 211, 131, 195, 147, 179,
    147, 179, 163, 163, 164, 105, 103,   0,  33,  48,  21,   0,
     26,  42, 227, 240,  51, 240,  51, 240,  51, 240,  51, 240,
     51, 240,  51, 240,  51, 240,  51, 240,  51, 240,  51, 240,
     51, 240,  51, 161, 115, 161, 115, 161, 115, 161, 115, 146,
    115, 146, 115, 116,  79,   2,   0,  26,  96,  83,  31,   0,
     38, 102, 246, 116, 212, 161,  18, 193,  19, 161,  18, 193,
     19, 161,  19, 177,  19, 161,  34, 161,  35, 161,  35, 145,
     35, 161,  50, 129,  51, 161,  50, 129,  51, 161,  51, 113,
     51, 161,  66,  97,  67, 161,  67,  81,  67, 161,  82,  65,
     83, 161,  82,  65,  83, 161,  83,  49,  83, 161,  98,  33,
     99, 161,  99,  17,  99, 161, 115, 115, 161, 115, 115, 147,
     99, 115, 119,  81,  89,   0,  39,  32,  62,  25,   0,  31,
     38, 135, 116, 147, 148, 161, 161,  19, 145, 161,  35, 129,
    161,  35, 129, 161,  51, 113, 161,  67,  97, 161,  68,  81,
    161,  83,  81, 161,  99,  65, 161, 100,  49, 161, 115,  49,
    161, 131,  33, 161, 147,  17, 161, 147,  17, 161, 164, 161,
    179, 161, 179, 147, 178, 119, 161,   0,  32,  16,  49,  25,
      0,  32,  23, 240,  19,  83, 210, 146, 178, 178, 146, 210,
    115, 211,  99, 226,  83, 243,  67, 243,  67, 243,  67, 243,
     67, 243,  67, 243,  67, 243,  83, 211,  99, 211, 115, 194,
    146, 178, 178, 146, 211,  83, 240,  23,   0,  32,  80,  46,
     22,   0,  27,  78, 179, 115, 147, 131, 131, 147, 115, 147,
    115, 147, 115, 147, 115, 147, 115, 131, 131, 115, 155, 179,
    240,  67, 240,  67, 240,  67, 240,  67, 240,  67, 240,  67,
    240,  67, 240,  67, 240,  25,   0,  29,  16,  69,  25,   0,
     32,  38, 240,  19,  83, 210, 146, 178, 178, 147, 194, 115,
    211,  99, 211,  83, 243,  67, 243,  67, 243,  67, 243,  67,
    243,  67, 243,  67, 243,  83, 211,  99, 211, 115,  67,  82,
    146,  49,  49,  51, 162,  33,  65,  19, 196,  67, 240,  24,
     81, 240,  50,  65, 240,  51,  34, 240,  55, 240,  54, 240,
     85, 240,  99,   0,  13,  32,  51,  26,   0,  32,  77, 240,
     19,  99, 227, 115, 211, 131, 195, 131, 195, 131, 195, 131,
    195, 130, 211, 115, 211,  99, 234, 240,  19,  83, 243,  99,
    227, 115, 211, 115, 211, 115, 211, 115, 211, 115, 211, 116,
     65, 115, 131,  50,  73, 101,   0,  33,  32,  43,  18,   0,
     23,   5,  49, 114,  84,  98, 115,  97, 146,  82, 146,  82,
    161,  83, 145,  84, 245, 232, 185, 199, 243, 240,  19,  65,
    178,  65, 178,  65, 178,  66, 161,  83, 130,  85,  82,  97,
     54,   0,  23,  64,  54,  25,   0,  31,  63,   4,  83,  99,
     99,  66, 115, 114,  66, 115, 114,  65, 131, 129,  65, 131,
    129,  65, 131, 129, 211, 240, 115, 240, 115, 240, 115, 240,
    115, 240, 115, 240, 115, 240, 115, 240, 115, 240, 115, 240,
    115, 240, 115, 240, 115, 240,  59,   0,  32,  48,  49,  25,
      0,  31,  41,  87, 115, 163, 147, 177, 163, 177, 163, 177,
    163, 177, 163, 177, 163, 177, 163, 177, 163, 177, 163, 177,
    163, 177, 163, 177, 163, 177, 163, 177, 163, 177, 178, 161,
    195, 145, 211, 113, 243,  66, 240,  54,   0,  32,  80,  57,
     27,   0,  33, 104, 150, 100, 195, 147, 194, 163, 193, 195,
    177, 195, 161, 227, 145, 227, 129, 244, 113, 240,  19, 113,
    240,  19,  97, 240,  51,  81, 240,  51,  65, 240,  83,  49,
    240,  83,  33, 240,  99,  33, 240, 115,  17, 240, 116, 240,
    147, 240, 146, 240, 177,   0,  35,  80,  85,  36,   0,  45,
      8,  72,  87, 100, 132, 147, 147, 147, 146, 163, 147, 145,
    179, 147, 145, 195, 132, 129, 195, 113,  19, 113, 211, 113,
     19, 113, 227,  97,  35,  97, 227,  81,  51,  81, 243,  81,
     51,  81, 240,  19,  65,  67,  65, 240,  19,  49,  83,  49,
    240,  50,  49,  99,  33, 240,  51,  18,  99,  33, 240,  51,
     17, 115,  17, 240,  84, 132, 240,  83, 147, 240,  99, 147,
    240, 114, 162, 240, 113, 177,   0,  46,  96,  56,  26,   0,
     32,  88,  87, 133, 116, 180, 114, 227, 113, 240,  19,  82,
    240,  20,  65, 240,  51,  49, 240,  83,  17, 240, 101, 240,
    115, 240, 147, 240, 117, 240,  97,  20, 240,  65,  51, 240,
     49,  83, 240,  18,  84, 241, 115, 225, 147, 194, 148, 163,
    149, 103, 120,   0,  33,  53,  27,   0,  33, 104, 150, 101,
    179, 148, 178, 179, 177, 211, 145, 228, 114, 243, 113, 240,
     35,  81, 240,  52,  50, 240,  68,  33, 240,  99,  17, 240,
    117, 240, 131, 240, 147, 240, 147, 240, 147, 240, 147, 240,
    147, 240, 147, 240, 147, 240, 105,   0,  35,  16,  50,  20,
      0,  25,  15,   1,  68, 132,  67, 147,  82, 147,  97, 148,
     97, 147, 113, 131, 240,  19, 240,  35, 240,  19, 240,  19,
    240,  35, 240,  19, 240,  19, 129, 131, 129, 115, 145,  99,
    161,  99, 161,  83, 162,  67, 148,  79,   1,   0,  25,  64,
      8,  25,   0,  62,  79,   6,   0,  63,  10,  46,   0, 115,
     15,  15,  12,   0, 115,  64,  21,  16,   0,   6,  50,  82,
     99,  67,  99,  67,  83,  67,  98,  82, 113,  97, 114,  82,
      0,  62, 112,  15,  18,   0,   9,  51, 113, 102,  65,  97,
     70,  97, 115,   0,  75,  16,
};

//*****************************************************************************
//
// The font definition for the 40 point Cmsc font.
//
//*****************************************************************************
const tFont g_sFontCmsc40 =
{
    //
    // The format of the font.
    //
    FONT_FMT_PIXEL_RLE,

    //
    // The maximum width of the font.
    //
    41,

    //
    // The height of the font.
    //
    41,

    //
    // The baseline of the font.
    //
    31,

    //
    // The offset to each character in the font.
    //
    {
           0,    5,   38,   69,  179,  259,  373,  462,
         480,  524,  568,  612,  677,  695,  704,  715,
         800,  859,  894,  954, 1012, 1072, 1130, 1191,
        1252, 1308, 1368, 1385, 1409, 1463, 1476, 1528,
        1578, 1681, 1763, 1826, 1903, 1983, 2060, 2122,
        2201, 2264, 2299, 2359, 2440, 2503, 2617, 2700,
        2783, 2844, 2954, 3027, 3087, 3159, 3225, 3309,
        3429, 3508, 3591, 3655, 3698, 3729, 3772, 3789,
        3800, 3818, 3874, 3920, 3971, 4018, 4068, 4115,
        4167, 4215, 4243, 4275, 4329, 4377, 4460, 4522,
        4571, 4617, 4686, 4737, 4780, 4834, 4883, 4940,
        5025, 5081, 5134, 5184, 5192, 5202, 5223,
    },

    //
    // A pointer to the actual font data
    //
    g_pucCmsc40Data
};
