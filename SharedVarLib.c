#include "SharedVarLib.h"

// Fonctions pour les unsigned char

void WriteSharedVarU8_APP(u8_shared_var *pt_u8ShVar, unsigned char val)
{
	while(pt_u8ShVar->mutex == LOCK);
	pt_u8ShVar->mutex=LOCK;
    pt_u8ShVar->u8_data[0] = val;
    pt_u8ShVar->mutex=UNLOCK;
    pt_u8ShVar->u8_data[1] = val;
}

unsigned char ReadSharedVarU8_APP(u8_shared_var *pt_u8ShVar)
{
    unsigned char val;
	while(pt_u8ShVar->mutex == LOCK);
	pt_u8ShVar->mutex=LOCK;
    val = pt_u8ShVar->u8_data[0];
    pt_u8ShVar->mutex=UNLOCK;
	return val;
}

void WriteSharedVarU8_SPI(u8_shared_var *pt_u8ShVar, unsigned char val)
{
    pt_u8ShVar->u8_data[pt_u8ShVar->mutex] = val;
}

unsigned char ReadSharedVarU8_SPI(u8_shared_var *pt_u8ShVar)
{
    return pt_u8ShVar->u8_data[pt_u8ShVar->mutex];
}

// Fonctions pour les unsigned int

void WriteSharedVarU16_APP(u16_shared_var *pt_u16ShVar, unsigned int val)
{
	while(pt_u16ShVar->mutex == LOCK);
	pt_u16ShVar->mutex=LOCK;
    pt_u16ShVar->u16_data[0] = val;
    pt_u16ShVar->mutex=UNLOCK;
    pt_u16ShVar->u16_data[1] = val;
}

unsigned int ReadSharedVarU16_APP(u16_shared_var *pt_u16ShVar)
{
    unsigned int val;
	while(pt_u16ShVar->mutex == LOCK);
	pt_u16ShVar->mutex=LOCK;
    val = pt_u16ShVar->u16_data[0];
    pt_u16ShVar->mutex=UNLOCK;
	return val;
}

void WriteSharedVarU16_SPI(u16_shared_var *pt_u16ShVar, unsigned int val)
{
    pt_u16ShVar->u16_data[pt_u16ShVar->mutex] = val;
}

unsigned int ReadSharedVarU16_SPI(u16_shared_var *pt_u16ShVar)
{
    return pt_u16ShVar->u16_data[pt_u16ShVar->mutex];
}

// Fonctions pour les unsigned long

void WriteSharedVarU32_APP(u32_shared_var *pt_u32ShVar, unsigned long int val)
{
	while(pt_u32ShVar->mutex == LOCK);
	pt_u32ShVar->mutex=LOCK;
    pt_u32ShVar->u32_data[0] = val;
    pt_u32ShVar->mutex=UNLOCK;
    pt_u32ShVar->u32_data[1] = val;
}

unsigned long int ReadSharedVarU32_APP(u32_shared_var *pt_u32ShVar)
{
    unsigned long int val;
	while(pt_u32ShVar->mutex == LOCK);
	pt_u32ShVar->mutex=LOCK;
    val = pt_u32ShVar->u32_data[0];
    pt_u32ShVar->mutex=UNLOCK;
	return val;
}

void WriteSharedVarU32_SPI(u32_shared_var *pt_u32ShVar, unsigned long int val)
{
    pt_u32ShVar->u32_data[pt_u32ShVar->mutex] = val;
}

unsigned long int ReadSharedVarU32_SPI(u32_shared_var *pt_u32ShVar)
{
    return pt_u32ShVar->u32_data[pt_u32ShVar->mutex];
}

