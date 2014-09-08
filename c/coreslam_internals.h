/*
ziggurat.h Ziggurat random-number generator

Downloaded from 

  http://people.sc.fsu.edu/~jburkardt/c_src/ziggurat/ziggurat.c

on 20 July 2014.

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
*/

#include <stdint.h>

uint32_t cong_seeded ( uint32_t *jcong );
double cpu_time ( void );
uint32_t kiss_seeded ( uint32_t *jcong, uint32_t *jsr, uint32_t *w, uint32_t *z );
uint32_t mwc_seeded ( uint32_t *w, uint32_t *z );
float r4_exp ( uint32_t *jsr, uint32_t ke[256], float fe[256], float we[256] );
void r4_exp_setup ( uint32_t ke[256], float fe[256], float we[256] );
float r4_nor ( uint32_t *jsr, uint32_t kn[128], float fn[128], float wn[128] );
void r4_nor_setup ( uint32_t kn[128], float fn[128], float wn[128] );
float r4_uni ( uint32_t *jsr );
uint32_t shr3_seeded ( uint32_t *jsr );
void timestamp ( void );
