#if !defined(__APPLE__) && !defined(__MACH__)

#include <fenv.h>

#else
#include <xmmintrin.h>

#define  FE_INEXACT		_MM_MASK_INEXACT
#define  FE_DIVBYZERO	_MM_MASK_DIV_ZERO
#define  FE_UNDERFLOW	_MM_MASK_UNDERFLOW
#define  FE_OVERFLOW	_MM_MASK_OVERFLOW
#define  FE_INVALID		_MM_MASK_INVALID
#define  FE_ALL_EXCEPT	_MM_MASK_MASK

/* Enable individual exceptions.  Will not enable more exceptions than
   EXCEPTS specifies.  Returns the previous enabled exceptions if all
   exceptions are successfully set, otherwise returns -1.  */
int feenableexcept(int _except)
{
	int prev_mask = _MM_GET_EXCEPTION_MASK();
	_MM_SET_EXCEPTION_MASK(prev_mask | _except);
	if(_MM_GET_EXCEPTION_MASK() & _except) {
		return -1;
	} else {
		return prev_mask;
	}
}

/* Disable individual exceptions.  Will not disable more exceptions than
   EXCEPTS specifies.  Returns the previous enabled exceptions if all
   exceptions are successfully disabled, otherwise returns -1.  */
int fedisableexcept(int _except)
{
	int prev_mask = _MM_GET_EXCEPTION_MASK();
	_MM_SET_EXCEPTION_MASK(prev_mask & ~_except);
	if(!(_MM_GET_EXCEPTION_MASK() & _except)) {
		return -1;
	} else {
		return prev_mask;
	}
}
#endif