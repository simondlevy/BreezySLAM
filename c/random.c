/*

random.c - Glue code for BreezySLAM pseudorandom number generator

Copyright (C) 2014 Simon D. Levy

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

#include "ziggurat.h"
#include "random.h"

#include <stdlib.h>
#include <string.h>

typedef struct random_t 
{
    float    fn[128];
    uint32_t kn[128];
    float    wn[128];  
    uint32_t seed;
        
} random_t;

size_t random_size(void)
{
    return sizeof(random_t);
}


void * random_new(int seed)
{
    random_t * r = (random_t *)malloc(sizeof(random_t));
    
    random_init(r, seed);
    
    return r;
    
}


void random_init(void * v, int seed)
{
    random_t * r = (random_t *)v;
    
    r->seed = seed;
        
    r4_nor_setup (r->kn, r->fn, r->wn );    
}


double random_normal(void * v, double mu, double sigma)
{
    random_t * r = (random_t *)v;
    
    return mu + sigma * r4_nor ( &r->seed, r->kn, r->fn, r->wn );
}

void random_free(void * v)
{
    free(v);
}

void * random_copy(void * v)
{
    random_t * r = (random_t *)malloc(sizeof(random_t));
    
    memcpy(r, v, sizeof(random_t));

    return r;    
}

