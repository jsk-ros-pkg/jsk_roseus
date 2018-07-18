#if Solaris2 && GCC
#pragma init (register_roseus_c_util)
#endif
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include "eus.h"

extern pointer ___roseus_c_util();
//static void register_roseus_c_util()
//  { add_module_initializer("___roseus_c_util", ___roseus_c_util);}

#define colsize(p) (intval(p->c.ary.dim[1]))
#define rowsize(p) (intval(p->c.ary.dim[0]))
#define ismatrix(p) ((isarray(p) && \
                      p->c.ary.rank==makeint(2) && \
                      elmtypeof(p->c.ary.entity)==ELM_FLOAT))
#define ckvsize(a,b) ((a->c.vec.size==b->c.vec.size)?vecsize(a):(int)error(E_VECINDEX))

pointer CONV_MSG2_PC (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  int i, nanflag;
  int step, size;
  int pos_x, pos_y, pos_z;
  int pos_nx, pos_ny, pos_nz;
  int pos_rgb;
  eusfloat_t *mat, *nmat, *cmat;
  byte *data;

  /* data step size mat x y z nmat nx ny nz cmat rgb nan-remove-flag*/
  /*    0    1    2   3 4 5 6    7  8  9 10   11  12 13 */
  ckarg(14);

  if ((!ismatrix(argv[3])) && NIL != argv[3]) {
    error(E_TYPEMISMATCH);
  }
  if ((!ismatrix(argv[7])) && NIL != argv[7]) {
    error(E_TYPEMISMATCH);
  }
  if ((!ismatrix(argv[11])) && NIL != argv[11]) {
    error(E_TYPEMISMATCH);
  }

  if (!isstring(argv[0])) error(E_TYPEMISMATCH);

  /*
  if ((colsize(argv[0]) != 3) ||
      (colsize(argv[1]) != 3) ||
      (vecsize(argv[2]) != 3) ||
      (colsize(argv[3]) != 3) ||
      (rowsize(argv[3]) != 3))
    error(E_VECINDEX);
  */

  step = ckintval(argv[1]);
  size = ckintval(argv[2]);

  nanflag = ckintval(argv[13]);

  data = argv[0]->c.str.chars;
  if ( NIL != argv[3] ) {
    mat = argv[3]->c.ary.entity->c.fvec.fv;
    pos_x = ckintval(argv[4]);
    pos_y = ckintval(argv[5]);
    pos_z = ckintval(argv[6]);
  } else {
    mat = NULL;
    pos_x = pos_y = pos_z = 0;
  }
  if ( NIL != argv[7] ) {
    nmat = argv[7]->c.ary.entity->c.fvec.fv;
    pos_nx = ckintval(argv[8]);
    pos_ny = ckintval(argv[9]);
    pos_nz = ckintval(argv[10]);
  } else {
    nmat = NULL;
    pos_nx = pos_ny = pos_nz = 0;
  }
  if ( NIL != argv[11] ) {
    cmat = argv[11]->c.ary.entity->c.fvec.fv;
    pos_rgb = ckintval(argv[12]);
  } else {
    cmat = NULL;
    pos_rgb = 0;
  }

  for( i = 0; i < size; i++, data += step ) {
    if (mat != NULL) {
      *mat++ = ( *((float *)(data + pos_x)) ) * 1000.0;
      *mat++ = ( *((float *)(data + pos_y)) ) * 1000.0;
      *mat++ = ( *((float *)(data + pos_z)) ) * 1000.0;
    }
    if (nmat != NULL) {
      *nmat++ = *((float *)(data + pos_nx));
      *nmat++ = *((float *)(data + pos_ny));
      *nmat++ = *((float *)(data + pos_nz));
    }
    if (cmat != NULL) {
      int rgb = *((int *) (data + pos_rgb));
      *cmat++ = ( ((rgb >> 16) & 0x000000FF) / 255.0 );
      *cmat++ = ( ((rgb >> 8) & 0x000000FF) / 255.0 );
      *cmat++ = ( (rgb & 0x000000FF) / 255.0 );
    }
  }

  if (nanflag > 0) {
    // replace nan
    if ( mat != NULL ) { mat--; }
    if ( cmat != NULL ) { cmat--; }
    for( i = 0; i < size; i++ ) {
      if ( mat != NULL ) {
        if ( isnan(*mat) ) {
          *mat-- = 0.0;
          *mat-- = 0.0;
          *mat-- = 0.0;
        } else {
          mat -= 3;
        }
      }
      if ( cmat != NULL ) {
        if ( isnan(*cmat) ) {
          *cmat-- = 0.0;
          *cmat-- = 0.0;
          *cmat-- = 0.0;
        } else {
          cmat -= 3;
        }
      }
    }
  }

  return NIL;
}

pointer CONV_PC_MSG2 (ctx,n,argv)
     register context *ctx;
     int n;
     register pointer argv[];
{
  int i, step, size;
  eusfloat_t *mat, *nmat, *cmat;
  byte *data;
  /* raw_data  size psize parray carray narray */
  /*        0     1     2      3      4      5 */
  ckarg(6);

  if (!isstring(argv[0])) error(E_TYPEMISMATCH);

  if ((!ismatrix(argv[3])) && NIL != argv[3]) {
    error(E_TYPEMISMATCH);
  }
  if ((!ismatrix(argv[4])) && NIL != argv[4]) {
    error(E_TYPEMISMATCH);
  }
  if ((!ismatrix(argv[5])) && NIL != argv[5]) {
    error(E_TYPEMISMATCH);
  }

  size = ckintval(argv[1]);
  step = ckintval(argv[2]);

  data = argv[0]->c.str.chars;

  if ( NIL != argv[3] ) {
    mat = argv[3]->c.ary.entity->c.fvec.fv;
  } else {
    mat = NULL;
  }
  if ( NIL != argv[4] ) {
    cmat = argv[4]->c.ary.entity->c.fvec.fv;
  } else {
    cmat = NULL;
  }
  if ( NIL != argv[5] ) {
    nmat = argv[5]->c.ary.entity->c.fvec.fv;
  } else {
    nmat = NULL;
  }

  for( i = 0; i < size; i++, data += step ) {
    float *tmp = (float *)data;
    if (mat != NULL) {
      *tmp++ = (*mat++) * 0.001;
      *tmp++ = (*mat++) * 0.001;
      *tmp++ = (*mat++) * 0.001;
    }
    if (cmat != NULL) {
      int r = (floor ((*cmat++) * 255));
      int g = (floor ((*cmat++) * 255));
      int b = (floor ((*cmat++) * 255));

      *((int *)tmp) = ((r & 0x000000FF) << 16) | ((g & 0x000000FF) << 8) | (b & 0x000000FF);
      tmp++;
    }
    if (nmat != NULL) {
      *tmp++ = *nmat++;
      *tmp++ = *nmat++;
      *tmp++ = *nmat++;
      *tmp++ = 0.0; // curvature
    }
  }

  return NIL;
}

#include "defun.h"
pointer ___roseus_c_util(ctx,n, argv, env)
register context *ctx;int n;pointer *argv;pointer env;
{
  defun(ctx,"CONVERT-MSG2-POINTCLOUD", argv[0], CONV_MSG2_PC,NULL);
  defun(ctx,"CONVERT-POINTCLOUD-MSG2", argv[0], CONV_PC_MSG2,NULL);
  return NULL;
}

