#ifndef __ANOTC_H
#define __ANOTC_H

void anotc_sendangle(uint16_t A, uint16_t B, uint16_t C);
void anotc_sendquat(uint16_t A, uint16_t B, uint16_t C, uint16_t D);
void anotc_senddata(uint16_t A, uint16_t B, uint16_t C, uint16_t D, uint16_t E, uint16_t F);

#endif