/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */
#pragma once
#include <time.h>
#include <stdio.h>
#include <AP_Math/AP_Math.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __FILE_defined
typedef int FILE;
#define __FILE_defined
#endif

const char* soap_dateTime2s(struct soap*, time_t);
const char* soap_wchar2s(struct soap *, const wchar_t *);
time_t soap_timegm(struct tm *T);
void soap_print_fault(struct soap *, FILE*);
int soap_outdateTime(struct soap *, const char *, int, const time_t *, const char *, int);
time_t *soap_indateTime(struct soap *, const char *, time_t *, const char *, int);
int soap_s2dateTime(struct soap *, const char *, time_t *);

#ifdef __cplusplus
}
#endif
