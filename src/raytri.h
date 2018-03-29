//
//  raytri.h
//  VoxelHeatDiffuse
//
//  Created by MINGFENWANG on 2017/10/10.
//  Copyright © 2017年 MINGFENWANG. All rights reserved.
//

#ifndef raytri_h
#define raytri_h

int intersect_triangle(float orig[3], float dir[3],
                       float vert0[3], float vert1[3], float vert2[3],
                       float *t, float *u, float *v, float *det);

int intersect_triangle1(float orig[3], float dir[3],
                       float vert0[3], float vert1[3], float vert2[3],
                       float *t, float *u, float *v, float *det);

int intersect_triangle2(float orig[3], float dir[3],
                       float vert0[3], float vert1[3], float vert2[3],
                       float *t, float *u, float *v, float *det);

int intersect_triangle3(float orig[3], float dir[3],
                       float vert0[3], float vert1[3], float vert2[3],
                       float *t, float *u, float *v, float *det);

#endif /* raytri_h */
