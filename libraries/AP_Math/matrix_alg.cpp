#include<AP_Math.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

float det6x6(const float C[36])
{
    float f;
    float A[36];
    int8_t ipiv[6];
    int32_t i0;
    int32_t j;
    int32_t c;
    int32_t iy;
    int32_t ix;
    float smax;
    int32_t jy;
    float s;
    int32_t b_j;
    int32_t ijA;
    bool isodd;
    memcpy(&A[0], &C[0], 36U * sizeof(float));
    for (i0 = 0; i0 < 6; i0++) {
        ipiv[i0] = (int8_t)(1 + i0);
    }

    for (j = 0; j < 5; j++) {
        c = j * 7;
        iy = 0;
        ix = c;
        smax = fabsf(A[c]);
        for (jy = 2; jy <= 6 - j; jy++) {
            ix++;
            s = fabsf(A[ix]);
            if (s > smax) {
                iy = jy - 1;
                smax = s;
            }
        }

        if (A[c + iy] != 0.0f) {
            if (iy != 0) {
                ipiv[j] = (int8_t)((j + iy) + 1);
                ix = j;
                iy += j;
                for (jy = 0; jy < 6; jy++) {
                    smax = A[ix];
                    A[ix] = A[iy];
                    A[iy] = smax;
                    ix += 6;
                    iy += 6;
                }
            }

            i0 = (c - j) + 6;
            for (iy = c + 1; iy + 1 <= i0; iy++) {
                A[iy] /= A[c];
            }
        }

        iy = c;
        jy = c + 6;
        for (b_j = 1; b_j <= 5 - j; b_j++) {
            smax = A[jy];
            if (A[jy] != 0.0f) {
                ix = c + 1;
                i0 = (iy - j) + 12;
                for (ijA = 7 + iy; ijA + 1 <= i0; ijA++) {
                    A[ijA] += A[ix] * -smax;
                    ix++;
                }
            }

            jy += 6;
            iy += 6;
        }
    }

    f = A[0];
    isodd = false;
    for (jy = 0; jy < 5; jy++) {
        f *= A[(jy + 6 * (1 + jy)) + 1];
        if (ipiv[jy] > 1 + jy) {
            isodd = !isodd;
        }
    }

    if (isodd) {
        f = -f;
    }

    return f;
}

float det9x9(const float C[81])
{
  float f;
  float A[81];
  int8_t ipiv[9];
  int32_t i0;
  int32_t j;
  int32_t c;
  int32_t iy;
  int32_t ix;
  float smax;
  int32_t jy;
  float s;
  int32_t b_j;
  int32_t ijA;
  bool isodd;
  memcpy(&A[0], &C[0], 81U * sizeof(float));
  for (i0 = 0; i0 < 9; i0++) {
    ipiv[i0] = (int8_t)(1 + i0);
  }

  for (j = 0; j < 8; j++) {
    c = j * 10;
    iy = 0;
    ix = c;
    smax = fabs(A[c]);
    for (jy = 2; jy <= 9 - j; jy++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        iy = jy - 1;
        smax = s;
      }
    }

    if (A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (int8_t)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 9; jy++) {
          smax = A[ix];
          A[ix] = A[iy];
          A[iy] = smax;
          ix += 9;
          iy += 9;
        }
      }

      i0 = (c - j) + 9;
      for (iy = c + 1; iy + 1 <= i0; iy++) {
        A[iy] /= A[c];
      }
    }

    iy = c;
    jy = c + 9;
    for (b_j = 1; b_j <= 8 - j; b_j++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i0 = (iy - j) + 18;
        for (ijA = 10 + iy; ijA + 1 <= i0; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 9;
      iy += 9;
    }
  }

  f = A[0];
  isodd = false;
  for (jy = 0; jy < 8; jy++) {
    f *= A[(jy + 9 * (1 + jy)) + 1];
    if (ipiv[jy] > 1 + jy) {
      isodd = !isodd;
    }
  }

  if (isodd) {
    f = -f;
  }

  return f;
}

bool inverse9x9(const float x[81], float y[81])
{
  if(fabsf(det9x9(x)) < 1.0e-20f) {
     return false;
  }
  float A[81];
  int32_t i0;
  int8_t ipiv[9];
  int32_t j;
  int32_t c;
  int32_t pipk;
  int32_t ix;
  float smax;
  int32_t k;
  float s;
  int32_t jy;
  int32_t ijA;
  int8_t p[9];
  for (i0 = 0; i0 < 81; i0++) {
    A[i0] = x[i0];
    y[i0] = 0.0;
  }

  for (i0 = 0; i0 < 9; i0++) {
    ipiv[i0] = (int8_t)(1 + i0);
  }

  for (j = 0; j < 8; j++) {
    c = j * 10;
    pipk = 0;
    ix = c;
    smax = fabs(A[c]);
    for (k = 2; k <= 9 - j; k++) {
      ix++;
      s = fabs(A[ix]);
      if (s > smax) {
        pipk = k - 1;
        smax = s;
      }
    }

    if (A[c + pipk] != 0.0) {
      if (pipk != 0) {
        ipiv[j] = (int8_t)((j + pipk) + 1);
        ix = j;
        pipk += j;
        for (k = 0; k < 9; k++) {
          smax = A[ix];
          A[ix] = A[pipk];
          A[pipk] = smax;
          ix += 9;
          pipk += 9;
        }
      }

      i0 = (c - j) + 9;
      for (jy = c + 1; jy + 1 <= i0; jy++) {
        A[jy] /= A[c];
      }
    }

    pipk = c;
    jy = c + 9;
    for (k = 1; k <= 8 - j; k++) {
      smax = A[jy];
      if (A[jy] != 0.0) {
        ix = c + 1;
        i0 = (pipk - j) + 18;
        for (ijA = 10 + pipk; ijA + 1 <= i0; ijA++) {
          A[ijA] += A[ix] * -smax;
          ix++;
        }
      }

      jy += 9;
      pipk += 9;
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    p[i0] = (int8_t)(1 + i0);
  }

  for (k = 0; k < 8; k++) {
    if (ipiv[k] > 1 + k) {
      pipk = p[ipiv[k] - 1];
      p[ipiv[k] - 1] = p[k];
      p[k] = (int8_t)pipk;
    }
  }

  for (k = 0; k < 9; k++) {
    y[k + 9 * (p[k] - 1)] = 1.0;
    for (j = k; j + 1 < 10; j++) {
      if (y[j + 9 * (p[k] - 1)] != 0.0) {
        for (jy = j + 1; jy + 1 < 10; jy++) {
          y[jy + 9 * (p[k] - 1)] -= y[j + 9 * (p[k] - 1)] * A[jy + 9 * j];
        }
      }
    }
  }

  for (j = 0; j < 9; j++) {
    c = 9 * j;
    for (k = 8; k > -1; k += -1) {
      pipk = 9 * k;
      if (y[k + c] != 0.0) {
        y[k + c] /= A[k + pipk];
        for (jy = 0; jy + 1 <= k; jy++) {
          y[jy + c] -= y[k + c] * A[jy + pipk];
        }
      }
    }
  }
  return true;
}


bool inverse6x6(const float x[], float y[])
{
    if(fabsf(det6x6(x)) < 1.0e-20f) {
        return false;
    }

    float A[36];
    int32_t i0;
    int32_t ipiv[6];
    int32_t j;
    int32_t c;
    int32_t pipk;
    int32_t ix;
    float smax;
    int32_t k;
    float s;
    int32_t jy;
    int32_t ijA;
    int32_t p[6];
    for (i0 = 0; i0 < 36; i0++) {
        A[i0] = x[i0];
        y[i0] = 0.0f;
    }

    for (i0 = 0; i0 < 6; i0++) {
        ipiv[i0] = (int32_t)(1 + i0);
    }

    for (j = 0; j < 5; j++) {
        c = j * 7;
        pipk = 0;
        ix = c;
        smax = fabsf(A[c]);
        for (k = 2; k <= 6 - j; k++) {
            ix++;
            s = fabsf(A[ix]);
            if (s > smax) {
                pipk = k - 1;
                smax = s;
            }
        }

        if (A[c + pipk] != 0.0f) {
            if (pipk != 0) {
                ipiv[j] = (int32_t)((j + pipk) + 1);
                ix = j;
                pipk += j;
                for (k = 0; k < 6; k++) {
                    smax = A[ix];
                    A[ix] = A[pipk];
                    A[pipk] = smax;
                    ix += 6;
                    pipk += 6;
                }
            }

            i0 = (c - j) + 6;
            for (jy = c + 1; jy + 1 <= i0; jy++) {
                A[jy] /= A[c];
            }
        }

        pipk = c;
        jy = c + 6;
        for (k = 1; k <= 5 - j; k++) {
            smax = A[jy];
            if (A[jy] != 0.0f) {
                ix = c + 1;
                i0 = (pipk - j) + 12;
                for (ijA = 7 + pipk; ijA + 1 <= i0; ijA++) {
                    A[ijA] += A[ix] * -smax;
                    ix++;
                }
            }

            jy += 6;
            pipk += 6;
        }
    }

    for (i0 = 0; i0 < 6; i0++) {
        p[i0] = (int32_t)(1 + i0);
    }

    for (k = 0; k < 5; k++) {
        if (ipiv[k] > 1 + k) {
            pipk = p[ipiv[k] - 1];
            p[ipiv[k] - 1] = p[k];
            p[k] = (int32_t)pipk;
        }
    }

    for (k = 0; k < 6; k++) {
        y[k + 6 * (p[k] - 1)] = 1.0;
        for (j = k; j + 1 < 7; j++) {
            if (y[j + 6 * (p[k] - 1)] != 0.0f) {
                for (jy = j + 1; jy + 1 < 7; jy++) {
                    y[jy + 6 * (p[k] - 1)] -= y[j + 6 * (p[k] - 1)] * A[jy + 6 * j];
                }
            }
        }
    }

    for (j = 0; j < 6; j++) {
        c = 6 * j;
        for (k = 5; k > -1; k += -1) {
            pipk = 6 * k;
            if (y[k + c] != 0.0f) {
                y[k + c] /= A[k + pipk];
                for (jy = 0; jy + 1 <= k; jy++) {
                    y[jy + c] -= y[k + c] * A[jy + pipk];
                }
            }
        }
    }
    return true;
}

bool inverse3x3(float m[], float invOut[])
{
    float inv[9];
    // computes the inverse of a matrix m
    float  det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
    if(fabsf(det) < 1.0e-20f){
        return false;
    }

    float invdet = 1 / det;

    inv[0] = (m[4] * m[8] - m[7] * m[5]) * invdet;
    inv[1] = (m[2] * m[7] - m[1] * m[8]) * invdet;
    inv[2] = (m[1] * m[5] - m[2] * m[4]) * invdet;
    inv[3] = (m[5] * m[6] - m[5] * m[8]) * invdet;
    inv[4] = (m[0] * m[8] - m[2] * m[6]) * invdet;
    inv[5] = (m[3] * m[2] - m[0] * m[5]) * invdet;
    inv[6] = (m[3] * m[7] - m[6] * m[4]) * invdet;
    inv[7] = (m[6] * m[1] - m[0] * m[7]) * invdet;
    inv[8] = (m[0] * m[4] - m[3] * m[1]) * invdet;

    for(uint8_t i = 0; i < 9; i++){
        invOut[i] = inv[i];
    }

    return true;
}

/*
 *    matrix inverse code only for 4x4 square matrix copied from
 *    gluInvertMatrix implementation in
 *    opengl for 4x4 matrices.
 *
 *    @param     m,           input 4x4 matrix
 *    @param     invOut,      Output inverted 4x4 matrix
 *    @returns                false = matrix is Singular, true = matrix inversion successful
 *    Known Issues/ Possible Enhancements:
 *                -Will need a different implementation for more number
 *                 of parameters like in the case of addition of soft
 *                 iron calibration
 */
bool inverse4x4(float m[],float invOut[])
{
    float inv[16], det;
    uint8_t i;

    inv[0] = m[5]  * m[10] * m[15] -
    m[5]  * m[11] * m[14] -
    m[9]  * m[6]  * m[15] +
    m[9]  * m[7]  * m[14] +
    m[13] * m[6]  * m[11] -
    m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
    m[4]  * m[11] * m[14] +
    m[8]  * m[6]  * m[15] -
    m[8]  * m[7]  * m[14] -
    m[12] * m[6]  * m[11] +
    m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
    m[4]  * m[11] * m[13] -
    m[8]  * m[5] * m[15] +
    m[8]  * m[7] * m[13] +
    m[12] * m[5] * m[11] -
    m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
    m[4]  * m[10] * m[13] +
    m[8]  * m[5] * m[14] -
    m[8]  * m[6] * m[13] -
    m[12] * m[5] * m[10] +
    m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
    m[1]  * m[11] * m[14] +
    m[9]  * m[2] * m[15] -
    m[9]  * m[3] * m[14] -
    m[13] * m[2] * m[11] +
    m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
    m[0]  * m[11] * m[14] -
    m[8]  * m[2] * m[15] +
    m[8]  * m[3] * m[14] +
    m[12] * m[2] * m[11] -
    m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
    m[0]  * m[11] * m[13] +
    m[8]  * m[1] * m[15] -
    m[8]  * m[3] * m[13] -
    m[12] * m[1] * m[11] +
    m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
    m[0]  * m[10] * m[13] -
    m[8]  * m[1] * m[14] +
    m[8]  * m[2] * m[13] +
    m[12] * m[1] * m[10] -
    m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
    m[1]  * m[7] * m[14] -
    m[5]  * m[2] * m[15] +
    m[5]  * m[3] * m[14] +
    m[13] * m[2] * m[7] -
    m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
    m[0]  * m[7] * m[14] +
    m[4]  * m[2] * m[15] -
    m[4]  * m[3] * m[14] -
    m[12] * m[2] * m[7] +
    m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
    m[0]  * m[7] * m[13] -
    m[4]  * m[1] * m[15] +
    m[4]  * m[3] * m[13] +
    m[12] * m[1] * m[7] -
    m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
    m[0]  * m[6] * m[13] +
    m[4]  * m[1] * m[14] -
    m[4]  * m[2] * m[13] -
    m[12] * m[1] * m[6] +
    m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
    m[1] * m[7] * m[10] +
    m[5] * m[2] * m[11] -
    m[5] * m[3] * m[10] -
    m[9] * m[2] * m[7] +
    m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
    m[0] * m[7] * m[10] -
    m[4] * m[2] * m[11] +
    m[4] * m[3] * m[10] +
    m[8] * m[2] * m[7] -
    m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
    m[0] * m[7] * m[9] +
    m[4] * m[1] * m[11] -
    m[4] * m[3] * m[9] -
    m[8] * m[1] * m[7] +
    m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
    m[0] * m[6] * m[9] -
    m[4] * m[1] * m[10] +
    m[4] * m[2] * m[9] +
    m[8] * m[1] * m[6] -
    m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if(fabsf(det) < 1.0e-20f){
        return false;
    }

    det = 1.0f / det;

    for (i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;
    return true;
}


bool inverse(float x[], float y[], uint16_t dim)
{
    switch(dim){
        case 3: return inverse3x3(x,y);
        case 4: return inverse4x4(x,y);
        case 6: return inverse6x6(x,y);
        case 9: return inverse9x9(x,y);
        default: return false;
    }
}