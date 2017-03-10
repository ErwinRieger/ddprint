
/*
* This file is part of ddprint - a direct drive 3D printer firmware.
* 
* Copyright 2017 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>

#define OVERSAMPLENR 8

// PT100 with INA826 amp on Ultimaker v2.0 electronics
#if (THERMISTORHEATER_0 == 20) || (THERMISTORHEATER_1 == 20) || (THERMISTORBED == 20)

//
// Table to convert ADC values into temperatures.
// This is for the ultimaker2 board with pt100 and INA826 amplifier.
// The table begins with ADC value 220.
// The last table entry is for ADC value 614.
//
#define ThermistorTableLowADC 220
#define ThermistorTableHighADC 614
#define MaxThermistorTableIndex (ThermistorTableHighADC - ThermistorTableLowADC)
const float thermistorTable[] PROGMEM = {
    -8.378692, // Index 0, ADC value 220
    -7.231602, // Index 1, ADC value 221
    -6.083894, // Index 2, ADC value 222
    -4.935567, // Index 3, ADC value 223
    -3.786622, // Index 4, ADC value 224
    -2.637057, // Index 5, ADC value 225
    -1.486872, // Index 6, ADC value 226
    -0.336066, // Index 7, ADC value 227
    0.815362, // Index 8, ADC value 228
    1.967412, // Index 9, ADC value 229
    3.120086, // Index 10, ADC value 230
    4.273383, // Index 11, ADC value 231
    5.427305, // Index 12, ADC value 232
    6.581853, // Index 13, ADC value 233
    7.737026, // Index 14, ADC value 234
    8.892826, // Index 15, ADC value 235
    10.049253, // Index 16, ADC value 236
    11.206309, // Index 17, ADC value 237
    12.363993, // Index 18, ADC value 238
    13.522307, // Index 19, ADC value 239
    14.681252, // Index 20, ADC value 240
    15.840827, // Index 21, ADC value 241
    17.001034, // Index 22, ADC value 242
    18.161873, // Index 23, ADC value 243
    19.323346, // Index 24, ADC value 244
    20.485452, // Index 25, ADC value 245
    21.648194, // Index 26, ADC value 246
    22.811570, // Index 27, ADC value 247
    23.975583, // Index 28, ADC value 248
    25.140232, // Index 29, ADC value 249
    26.305519, // Index 30, ADC value 250
    27.471445, // Index 31, ADC value 251
    28.638009, // Index 32, ADC value 252
    29.805213, // Index 33, ADC value 253
    30.973058, // Index 34, ADC value 254
    32.141543, // Index 35, ADC value 255
    33.310671, // Index 36, ADC value 256
    34.480442, // Index 37, ADC value 257
    35.650856, // Index 38, ADC value 258
    36.821914, // Index 39, ADC value 259
    37.993617, // Index 40, ADC value 260
    39.165966, // Index 41, ADC value 261
    40.338961, // Index 42, ADC value 262
    41.512604, // Index 43, ADC value 263
    42.686894, // Index 44, ADC value 264
    43.861834, // Index 45, ADC value 265
    45.037422, // Index 46, ADC value 266
    46.213661, // Index 47, ADC value 267
    47.390551, // Index 48, ADC value 268
    48.568093, // Index 49, ADC value 269
    49.746287, // Index 50, ADC value 270
    50.925134, // Index 51, ADC value 271
    52.104636, // Index 52, ADC value 272
    53.284792, // Index 53, ADC value 273
    54.465604, // Index 54, ADC value 274
    55.647072, // Index 55, ADC value 275
    56.829198, // Index 56, ADC value 276
    58.011981, // Index 57, ADC value 277
    59.195423, // Index 58, ADC value 278
    60.379524, // Index 59, ADC value 279
    61.564286, // Index 60, ADC value 280
    62.749709, // Index 61, ADC value 281
    63.935793, // Index 62, ADC value 282
    65.122540, // Index 63, ADC value 283
    66.309950, // Index 64, ADC value 284
    67.498024, // Index 65, ADC value 285
    68.686764, // Index 66, ADC value 286
    69.876169, // Index 67, ADC value 287
    71.066240, // Index 68, ADC value 288
    72.256979, // Index 69, ADC value 289
    73.448385, // Index 70, ADC value 290
    74.640461, // Index 71, ADC value 291
    75.833206, // Index 72, ADC value 292
    77.026622, // Index 73, ADC value 293
    78.220709, // Index 74, ADC value 294
    79.415468, // Index 75, ADC value 295
    80.610900, // Index 76, ADC value 296
    81.807006, // Index 77, ADC value 297
    83.003786, // Index 78, ADC value 298
    84.201241, // Index 79, ADC value 299
    85.399373, // Index 80, ADC value 300
    86.598181, // Index 81, ADC value 301
    87.797667, // Index 82, ADC value 302
    88.997831, // Index 83, ADC value 303
    90.198675, // Index 84, ADC value 304
    91.400199, // Index 85, ADC value 305
    92.602404, // Index 86, ADC value 306
    93.805291, // Index 87, ADC value 307
    95.008860, // Index 88, ADC value 308
    96.213113, // Index 89, ADC value 309
    97.418050, // Index 90, ADC value 310
    98.623672, // Index 91, ADC value 311
    99.829980, // Index 92, ADC value 312
    101.036975, // Index 93, ADC value 313
    102.244657, // Index 94, ADC value 314
    103.453028, // Index 95, ADC value 315
    104.662088, // Index 96, ADC value 316
    105.871838, // Index 97, ADC value 317
    107.082279, // Index 98, ADC value 318
    108.293412, // Index 99, ADC value 319
    109.505237, // Index 100, ADC value 320
    110.717756, // Index 101, ADC value 321
    111.930969, // Index 102, ADC value 322
    113.144877, // Index 103, ADC value 323
    114.359481, // Index 104, ADC value 324
    115.574782, // Index 105, ADC value 325
    116.790781, // Index 106, ADC value 326
    118.007478, // Index 107, ADC value 327
    119.224874, // Index 108, ADC value 328
    120.442971, // Index 109, ADC value 329
    121.661769, // Index 110, ADC value 330
    122.881269, // Index 111, ADC value 331
    124.101472, // Index 112, ADC value 332
    125.322378, // Index 113, ADC value 333
    126.543989, // Index 114, ADC value 334
    127.766305, // Index 115, ADC value 335
    128.989328, // Index 116, ADC value 336
    130.213058, // Index 117, ADC value 337
    131.437496, // Index 118, ADC value 338
    132.662643, // Index 119, ADC value 339
    133.888499, // Index 120, ADC value 340
    135.115067, // Index 121, ADC value 341
    136.342345, // Index 122, ADC value 342
    137.570337, // Index 123, ADC value 343
    138.799041, // Index 124, ADC value 344
    140.028460, // Index 125, ADC value 345
    141.258594, // Index 126, ADC value 346
    142.489444, // Index 127, ADC value 347
    143.721010, // Index 128, ADC value 348
    144.953295, // Index 129, ADC value 349
    146.186298, // Index 130, ADC value 350
    147.420021, // Index 131, ADC value 351
    148.654464, // Index 132, ADC value 352
    149.889629, // Index 133, ADC value 353
    151.125516, // Index 134, ADC value 354
    152.362126, // Index 135, ADC value 355
    153.599460, // Index 136, ADC value 356
    154.837519, // Index 137, ADC value 357
    156.076304, // Index 138, ADC value 358
    157.315815, // Index 139, ADC value 359
    158.556055, // Index 140, ADC value 360
    159.797023, // Index 141, ADC value 361
    161.038721, // Index 142, ADC value 362
    162.281149, // Index 143, ADC value 363
    163.524308, // Index 144, ADC value 364
    164.768200, // Index 145, ADC value 365
    166.012825, // Index 146, ADC value 366
    167.258185, // Index 147, ADC value 367
    168.504279, // Index 148, ADC value 368
    169.751110, // Index 149, ADC value 369
    170.998677, // Index 150, ADC value 370
    172.246982, // Index 151, ADC value 371
    173.496026, // Index 152, ADC value 372
    174.745810, // Index 153, ADC value 373
    175.996335, // Index 154, ADC value 374
    177.247602, // Index 155, ADC value 375
    178.499611, // Index 156, ADC value 376
    179.752363, // Index 157, ADC value 377
    181.005860, // Index 158, ADC value 378
    182.260103, // Index 159, ADC value 379
    183.515092, // Index 160, ADC value 380
    184.770828, // Index 161, ADC value 381
    186.027313, // Index 162, ADC value 382
    187.284547, // Index 163, ADC value 383
    188.542531, // Index 164, ADC value 384
    189.801266, // Index 165, ADC value 385
    191.060754, // Index 166, ADC value 386
    192.320995, // Index 167, ADC value 387
    193.581990, // Index 168, ADC value 388
    194.843740, // Index 169, ADC value 389
    196.106246, // Index 170, ADC value 390
    197.369509, // Index 171, ADC value 391
    198.633530, // Index 172, ADC value 392
    199.898310, // Index 173, ADC value 393
    201.163851, // Index 174, ADC value 394
    202.430152, // Index 175, ADC value 395
    203.697215, // Index 176, ADC value 396
    204.965041, // Index 177, ADC value 397
    206.233631, // Index 178, ADC value 398
    207.502986, // Index 179, ADC value 399
    208.773107, // Index 180, ADC value 400
    210.043995, // Index 181, ADC value 401
    211.315651, // Index 182, ADC value 402
    212.588076, // Index 183, ADC value 403
    213.861271, // Index 184, ADC value 404
    215.135236, // Index 185, ADC value 405
    216.409974, // Index 186, ADC value 406
    217.685485, // Index 187, ADC value 407
    218.961769, // Index 188, ADC value 408
    220.238829, // Index 189, ADC value 409
    221.516665, // Index 190, ADC value 410
    222.795277, // Index 191, ADC value 411
    224.074668, // Index 192, ADC value 412
    225.354838, // Index 193, ADC value 413
    226.635788, // Index 194, ADC value 414
    227.917519, // Index 195, ADC value 415
    229.200033, // Index 196, ADC value 416
    230.483329, // Index 197, ADC value 417
    231.767410, // Index 198, ADC value 418
    233.052276, // Index 199, ADC value 419
    234.337929, // Index 200, ADC value 420
    235.624369, // Index 201, ADC value 421
    236.911597, // Index 202, ADC value 422
    238.199615, // Index 203, ADC value 423
    239.488423, // Index 204, ADC value 424
    240.778023, // Index 205, ADC value 425
    242.068415, // Index 206, ADC value 426
    243.359602, // Index 207, ADC value 427
    244.651583, // Index 208, ADC value 428
    245.944359, // Index 209, ADC value 429
    247.237933, // Index 210, ADC value 430
    248.532305, // Index 211, ADC value 431
    249.827475, // Index 212, ADC value 432
    251.123446, // Index 213, ADC value 433
    252.420218, // Index 214, ADC value 434
    253.717792, // Index 215, ADC value 435
    255.016169, // Index 216, ADC value 436
    256.315351, // Index 217, ADC value 437
    257.615338, // Index 218, ADC value 438
    258.916132, // Index 219, ADC value 439
    260.217734, // Index 220, ADC value 440
    261.520144, // Index 221, ADC value 441
    262.823364, // Index 222, ADC value 442
    264.127395, // Index 223, ADC value 443
    265.432238, // Index 224, ADC value 444
    266.737895, // Index 225, ADC value 445
    268.044365, // Index 226, ADC value 446
    269.351651, // Index 227, ADC value 447
    270.659753, // Index 228, ADC value 448
    271.968673, // Index 229, ADC value 449
    273.278412, // Index 230, ADC value 450
    274.588970, // Index 231, ADC value 451
    275.900350, // Index 232, ADC value 452
    277.212551, // Index 233, ADC value 453
    278.525576, // Index 234, ADC value 454
    279.839424, // Index 235, ADC value 455
    281.154099, // Index 236, ADC value 456
    282.469599, // Index 237, ADC value 457
    283.785928, // Index 238, ADC value 458
    285.103085, // Index 239, ADC value 459
    286.421073, // Index 240, ADC value 460
    287.739891, // Index 241, ADC value 461
    289.059542, // Index 242, ADC value 462
    290.380026, // Index 243, ADC value 463
    291.701345, // Index 244, ADC value 464
    293.023499, // Index 245, ADC value 465
    294.346490, // Index 246, ADC value 466
    295.670320, // Index 247, ADC value 467
    296.994988, // Index 248, ADC value 468
    298.320497, // Index 249, ADC value 469
    299.646848, // Index 250, ADC value 470
    300.974041, // Index 251, ADC value 471
    302.302078, // Index 252, ADC value 472
    303.630960, // Index 253, ADC value 473
    304.960688, // Index 254, ADC value 474
    306.291263, // Index 255, ADC value 475
    307.622688, // Index 256, ADC value 476
    308.954962, // Index 257, ADC value 477
    310.288086, // Index 258, ADC value 478
    311.622064, // Index 259, ADC value 479
    312.956894, // Index 260, ADC value 480
    314.292579, // Index 261, ADC value 481
    315.629120, // Index 262, ADC value 482
    316.966517, // Index 263, ADC value 483
    318.304773, // Index 264, ADC value 484
    319.643888, // Index 265, ADC value 485
    320.983864, // Index 266, ADC value 486
    322.324702, // Index 267, ADC value 487
    323.666402, // Index 268, ADC value 488
    325.008967, // Index 269, ADC value 489
    326.352397, // Index 270, ADC value 490
    327.696694, // Index 271, ADC value 491
    329.041859, // Index 272, ADC value 492
    330.387893, // Index 273, ADC value 493
    331.734797, // Index 274, ADC value 494
    333.082573, // Index 275, ADC value 495
    334.431222, // Index 276, ADC value 496
    335.780745, // Index 277, ADC value 497
    337.131143, // Index 278, ADC value 498
    338.482418, // Index 279, ADC value 499
    339.834570, // Index 280, ADC value 500
    341.187602, // Index 281, ADC value 501
    342.541514, // Index 282, ADC value 502
    343.896308, // Index 283, ADC value 503
    345.251985, // Index 284, ADC value 504
    346.608545, // Index 285, ADC value 505
    347.965992, // Index 286, ADC value 506
    349.324325, // Index 287, ADC value 507
    350.683545, // Index 288, ADC value 508
    352.043656, // Index 289, ADC value 509
    353.404656, // Index 290, ADC value 510
    354.766549, // Index 291, ADC value 511
    356.129335, // Index 292, ADC value 512
    357.493015, // Index 293, ADC value 513
    358.857591, // Index 294, ADC value 514
    360.223063, // Index 295, ADC value 515
    361.589435, // Index 296, ADC value 516
    362.956705, // Index 297, ADC value 517
    364.324877, // Index 298, ADC value 518
    365.693951, // Index 299, ADC value 519
    367.063928, // Index 300, ADC value 520
    368.434811, // Index 301, ADC value 521
    369.806599, // Index 302, ADC value 522
    371.179295, // Index 303, ADC value 523
    372.552900, // Index 304, ADC value 524
    373.927416, // Index 305, ADC value 525
    375.302842, // Index 306, ADC value 526
    376.679182, // Index 307, ADC value 527
    378.056435, // Index 308, ADC value 528
    379.434605, // Index 309, ADC value 529
    380.813691, // Index 310, ADC value 530
    382.193695, // Index 311, ADC value 531
    383.574619, // Index 312, ADC value 532
    384.956464, // Index 313, ADC value 533
    386.339232, // Index 314, ADC value 534
    387.722922, // Index 315, ADC value 535
    389.107538, // Index 316, ADC value 536
    390.493081, // Index 317, ADC value 537
    391.879551, // Index 318, ADC value 538
    393.266951, // Index 319, ADC value 539
    394.655281, // Index 320, ADC value 540
    396.044543, // Index 321, ADC value 541
    397.434738, // Index 322, ADC value 542
    398.825868, // Index 323, ADC value 543
    400.217934, // Index 324, ADC value 544
    401.610938, // Index 325, ADC value 545
    403.004880, // Index 326, ADC value 546
    404.399763, // Index 327, ADC value 547
    405.795588, // Index 328, ADC value 548
    407.192356, // Index 329, ADC value 549
    408.590068, // Index 330, ADC value 550
    409.988727, // Index 331, ADC value 551
    411.388333, // Index 332, ADC value 552
    412.788888, // Index 333, ADC value 553
    414.190393, // Index 334, ADC value 554
    415.592849, // Index 335, ADC value 555
    416.996259, // Index 336, ADC value 556
    418.400624, // Index 337, ADC value 557
    419.805945, // Index 338, ADC value 558
    421.212223, // Index 339, ADC value 559
    422.619460, // Index 340, ADC value 560
    424.027657, // Index 341, ADC value 561
    425.436817, // Index 342, ADC value 562
    426.846939, // Index 343, ADC value 563
    428.258027, // Index 344, ADC value 564
    429.670081, // Index 345, ADC value 565
    431.083102, // Index 346, ADC value 566
    432.497093, // Index 347, ADC value 567
    433.912055, // Index 348, ADC value 568
    435.327989, // Index 349, ADC value 569
    436.744896, // Index 350, ADC value 570
    438.162779, // Index 351, ADC value 571
    439.581638, // Index 352, ADC value 572
    441.001476, // Index 353, ADC value 573
    442.422294, // Index 354, ADC value 574
    443.844092, // Index 355, ADC value 575
    445.266874, // Index 356, ADC value 576
    446.690639, // Index 357, ADC value 577
    448.115391, // Index 358, ADC value 578
    449.541129, // Index 359, ADC value 579
    450.967857, // Index 360, ADC value 580
    452.395575, // Index 361, ADC value 581
    453.824285, // Index 362, ADC value 582
    455.253989, // Index 363, ADC value 583
    456.684687, // Index 364, ADC value 584
    458.116382, // Index 365, ADC value 585
    459.549075, // Index 366, ADC value 586
    460.982768, // Index 367, ADC value 587
    462.417462, // Index 368, ADC value 588
    463.853159, // Index 369, ADC value 589
    465.289860, // Index 370, ADC value 590
    466.727568, // Index 371, ADC value 591
    468.166283, // Index 372, ADC value 592
    469.606007, // Index 373, ADC value 593
    471.046741, // Index 374, ADC value 594
    472.488488, // Index 375, ADC value 595
    473.931249, // Index 376, ADC value 596
    475.375026, // Index 377, ADC value 597
    476.819819, // Index 378, ADC value 598
    478.265632, // Index 379, ADC value 599
    479.712464, // Index 380, ADC value 600
    481.160319, // Index 381, ADC value 601
    482.609197, // Index 382, ADC value 602
    484.059101, // Index 383, ADC value 603
    485.510031, // Index 384, ADC value 604
    486.961990, // Index 385, ADC value 605
    488.414979, // Index 386, ADC value 606
    489.869000, // Index 387, ADC value 607
    491.324054, // Index 388, ADC value 608
    492.780144, // Index 389, ADC value 609
    494.237270, // Index 390, ADC value 610
    495.695434, // Index 391, ADC value 611
    497.154639, // Index 392, ADC value 612
    498.614885, // Index 393, ADC value 613
    500.076175, // Index 394, ADC value 614
};

inline float tempFromRawADC(float rawADC) {

    int16_t index = rawADC/OVERSAMPLENR + 0.5 - ThermistorTableLowADC;

    // Check lower end of table
    if (index < 0)
        return pgm_read_float(thermistorTable);

    // Check high end of table
    if (index > MaxThermistorTableIndex)
        return pgm_read_float(thermistorTable + MaxThermistorTableIndex);

    return pgm_read_float(thermistorTable + index);
}

#endif

