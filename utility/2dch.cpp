/*
 * Ken Clarkson wrote this.  Copyright (c) 1996 by AT&T..
 * Permission to use, copy, modify, and distribute this software for any
 * purpose without fee is hereby granted, provided that this entire notice
 * is included in all copies of any software which is or includes a copy
 * or modification of this software and in all copies of the supporting
 * documentation for such software.
 * THIS SOFTWARE IS BEING PROVIDED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 * WARRANTY.  IN PARTICULAR, NEITHER THE AUTHORS NOR AT&T MAKE ANY
 * REPRESENTATION OR WARRANTY OF ANY KIND CONCERNING THE MERCHANTABILITY
 * OF THIS SOFTWARE OR ITS FITNESS FOR ANY PARTICULAR PURPOSE.
 */


/*
 * two-dimensional convex hull
 * read points from stdin,
 *      one point per line, as two numbers separated by whitespace
 * on stdout, points on convex hull in order around hull, given
 *      by their numbers in input order
 * the results should be "robust", and not return a wildly wrong hull,
 *	despite using floating point
 * works in O(n log n); I think a bit faster than Graham scan;
 * 	somewhat like Procedure 8.2 in Edelsbrunner's "Algorithms in Combinatorial
 *	Geometry", and very close to:
 *	    A.M. Andrew, "Another Efficient Algorithm for Convex Hulls in Two Dimensions",
 *		Info. Proc. Letters 9, 216-219 (1979)
 *	(See also http://geometryalgorithms.com/Archive/algorithm_0110/algorithm_0110.htm)
 */


#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "2dch.h"



int ccw(double **P, int i, int j, int k) {
	double	a = P[i][0] - P[j][0],
		b = P[i][1] - P[j][1],
		c = P[k][0] - P[j][0],
		d = P[k][1] - P[j][1];
	return a*d - b*c <= 0;	   /* true if points i, j, k counterclockwise */
}


#define CMPM(c,A,B) \
	v = (*(double**)A)[c] - (*(double**)B)[c];\
	if (v>0) return 1;\
	if (v<0) return -1;

int cmpl(const void *a, const void *b) {
	double v; 
	CMPM(0,a,b);
	CMPM(1,b,a);
	return 0;
}

int cmph(const void *a, const void *b) {return cmpl(b,a);}


int make_chain(double** V, int n, int (*cmp)(const void*, const void*)) {
	int i, j, s = 1;
	double* t;

	qsort(V, n, sizeof(double*), cmp);
	for (i=2; i<n; i++) {
		for (j=s; j>=1 && ccw(V, i, j, j-1); j--){}
		s = j+1;
		t = V[s]; V[s] = V[i]; V[i] = t;
	}
	return s;
}

int ch2d(double **P, int n)  
{
	if (!n) return 0;

	int u = make_chain(P, n, cmpl);		/* make lower hull */
	P[n] = P[0];
	u += make_chain(P+u, n-u+1, cmph);	/* make upper hull */
	return u;
}


