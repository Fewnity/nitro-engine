/****************************************************************************
*****************************************************************************
****       _   _ _ _               ______             _                  ****
****      | \ | (_) |             |  ____|           (_)                 ****
****      |  \| |_| |_ _ __ ___   | |__   _ __   __ _ _ _ __   ___       ****
****      | . ` | | __| '__/ _ \  |  __| | '_ \ / _` | | '_ \ / _ \      ****
****      | |\  | | |_| | | (_) | | |____| | | | (_| | | | | |  __/      ****
****      |_| \_|_|\__|_|  \___/  |______|_| |_|\__, |_|_| |_|\___|      ****
****                                             __/ |                   ****
****                                            |___/      V 0.6.1       ****
****                                                                     ****
****                     Copyright (C) 2008 - 2011 Antonio Ni�o D�az     ****
****                                   All rights reserved.              ****
****                                                                     ****
*****************************************************************************
****************************************************************************/

/****************************************************************************
*                                                                           *
* Nitro Engine V 0.6.1 is licensed under the terms of <readme_license.txt>. *
* If you have any question, email me at <antonio_nd@hotmail.com>.           *
*                                                                           *
****************************************************************************/

#include <nds/arm9/postest.h>
#include "NEMain.h"


/*! \file   NEModel.c */

static NE_Model ** NE_ModelPointers; 
static int NE_MAX_MODELS; 
static bool ne_model_system_inited = false;


NE_Model * NE_ModelCreate(NE_ModelType type)
{
	if(!ne_model_system_inited) return NULL;

	NE_Model * model = malloc(sizeof(NE_Model));
	NE_AssertPointer(model,"NE_ModelCreate: Couldn't allocate model.");

	int a = 0;
	while(1)
	{
		if(NE_ModelPointers[a] == NULL)
		{
			NE_ModelPointers[a] = model;
			break;
		}
		a++;
		if(a == NE_MAX_MODELS) 
		{
			NE_DebugPrint("NE_ModelCreate: No free slots...");
			free(model);
			return NULL;
		}
	}

	model->x = model->y = model->z = model->rx = model->ry = model->rz = 0;
	model->sx = model->sy = model->sz = inttof32(1);
	if(type == NE_Animated)
	{
		model->meshdata = malloc(sizeof(NE_AnimData));
		NE_AssertPointer(model->meshdata,"NE_ModelCreate: Couldn't allocate animation data.");
		NE_AnimData * anim = (NE_AnimData*)model->meshdata;
		for(a = 0; a < NE_MAX_FRAMES; a++) anim->speed[a] = 0;
		anim->animtype = 0;
		anim->currframe = 0;
		anim->startframe = 0;
		anim->endframe = 0;
		anim->direction = 1;
		anim->nextframetime = 0;
		model->anim_interpolate = true;
	}
	else /*if(type == NE_Static)*/ model->meshdata = NULL;
	model->modeltype = type;
	model->texture = NULL;
	model->meshfromfat = false;
	model->iscloned = false;
	return model;
}


void NE_ModelDelete(NE_Model * model)
{
	if(!ne_model_system_inited) return;

	NE_AssertPointer(model,"NE_ModelDelete: NULL pointer.");

	int a = 0;
	while(1)
	{
		if(a == NE_MAX_MODELS) 
		{
			NE_DebugPrint("NE_ModelDelete: Model not found in array.");
			return;
		}
		if(NE_ModelPointers[a] == model) 
		{
			NE_ModelPointers[a] = NULL;
			break;
		}
		a++;
	}

	if(!model->iscloned && model->meshfromfat)
	{
		if(model->modeltype == NE_Animated) free(((NE_AnimData*)model->meshdata)->fileptrtr);
		else /*if(model->modeltype == NE_Static)*/ free(model->meshdata);
	}

	if(model->modeltype == NE_Animated) free(model->meshdata); //Free animation data

	free(model);
}


int NE_ModelLoadStaticMeshFAT(NE_Model * model, char * path)
{
	if(!ne_model_system_inited) return 0;

	NE_AssertPointer(model,"NE_ModelLoadStaticMeshFAT: NULL model pointer.");
	NE_AssertPointer(path,"NE_ModelLoadStaticMeshFAT: NULL path pointer.");
	NE_Assert(model->modeltype == NE_Static,"NE_ModelLoadStaticMeshFAT: Not a static model.");
	NE_Assert(!model->iscloned,"NE_ModelLoadStaticMeshFAT: Can't load a mesh to a cloned model.");

	//Free previous data...
	if(model->meshfromfat && model->meshdata != NULL) 
	{
		free(model->meshdata);
		model->meshdata = NULL;
	}
	model->meshdata = (u32*)NE_FATLoadData(path);
	NE_AssertPointer(model->meshdata,"NE_ModelLoadStaticMeshFAT: Couldn't allocate data from FAT.");
	if(model->meshdata == NULL) return 0;
	model->meshfromfat = true;
	return 1;
}


int NE_ModelLoadStaticMesh(NE_Model * model, u32 * pointer)
{
	if(!ne_model_system_inited) return 0;

	NE_AssertPointer(model,"NE_ModelLoadStaticMesh: NULL model pointer.");
	NE_AssertPointer(pointer,"NE_ModelLoadStaticMesh: NULL data pointer.");
	NE_Assert(model->modeltype == NE_Static,"NE_ModelLoadStaticMesh: Not a static model.");
	NE_Assert(!model->iscloned,"NE_ModelLoadStaticMesh: Can't load a mesh to a cloned model.");

	//Free previous data...
	if(model->meshfromfat && model->meshdata != NULL) 
	{
		free(model->meshdata);
		model->meshdata = NULL;
	}
	model->meshdata = pointer;
	model->meshfromfat = false;
	return 1;
}



inline void NE_ModelSetMaterial(NE_Model * model, NE_Material * material)
{
	NE_AssertPointer(model,"NE_ModelSetMaterial: NULL model pointer.");
	NE_AssertPointer(material,"NE_ModelSetMaterial: NULL material pointer.");
	model->texture = material;
}


extern bool NE_TestTouch; // Internal use... see below
static void __ne_drawanimatedmodel_interpolate(NE_AnimData * data);
static void __ne_drawanimatedmodel_nointerpolate(NE_AnimData * data);

void NE_ModelDraw(NE_Model * model)
{
	NE_AssertPointer(model,"NE_ModelDraw: NULL pointer.");
	if(model->meshdata == NULL) return;
	if(model->modeltype == NE_Animated)
		if(((NE_AnimData*)model->meshdata)->fileptrtr == NULL)
			return;

	MATRIX_PUSH = 0;

	MATRIX_TRANSLATE = model->x;
	MATRIX_TRANSLATE = model->y;
	MATRIX_TRANSLATE = model->z;

	if(model->rx != 0) glRotateXi(model->rx<<6);
	if(model->ry != 0) glRotateYi(model->ry<<6);
	if(model->rz != 0) glRotateZi(model->rz<<6);

	MATRIX_SCALE = model->sx;
	MATRIX_SCALE = model->sy;
	MATRIX_SCALE = model->sz;

	if(NE_TestTouch) 
		PosTest_Asynch(0,0,0);
	else
		NE_MaterialUse(model->texture); 
		//If NULL, this will set GFX_TEX_FORMAT to 0 and GFX_COLOR to white.
		
	if(model->modeltype == NE_Static)
		glCallList(model->meshdata);
	else //if(model->modeltype == NE_Animated)
		{
		if(model->anim_interpolate)
			__ne_drawanimatedmodel_interpolate((NE_AnimData*)model->meshdata);
		else
			__ne_drawanimatedmodel_nointerpolate((NE_AnimData*)model->meshdata);
		}
		
	MATRIX_POP = 1;
}

void NE_ModelClone(NE_Model * dest, NE_Model * source)
{
	NE_AssertPointer(dest,"NE_ModelClone: NULL dest pointer.");
	NE_AssertPointer(source,"NE_ModelClone: NULL source pointer.");
	NE_Assert(dest->modeltype == source->modeltype,"NE_ModelClone: Different model types.");

	if(dest->modeltype == NE_Animated)
	{
		swiCopy(source->meshdata, dest->meshdata, (sizeof(NE_AnimData) >> 2)  | COPY_MODE_WORD);
		dest->iscloned = true;
		dest->texture = source->texture;
	}
	else
	{
		dest->iscloned = true;
		dest->meshdata = source->meshdata;
		dest->texture = source->texture;
	}
}

inline void NE_ModelScaleI(NE_Model * model, int x, int y, int z)
{
	NE_AssertPointer(model,"NE_ModelScaleI: NULL pointer.");
	model->sx = x; model->sy = y; model->sz = z;
}

inline void NE_ModelTranslateI(NE_Model * model, int x, int y, int z)
{
	NE_AssertPointer(model,"NE_ModelTranslateI: NULL pointer.");
	model->x += x; model->y += y; model->z += z;
}

inline void NE_ModelSetCoordI(NE_Model * model, int x, int y, int z)
{
	NE_AssertPointer(model,"NE_ModelSetCoordI: NULL pointer.");
	model->x = x; model->y = y; model->z = z;
}

inline void NE_ModelRotate(NE_Model * model, int rx, int ry, int rz)
{
	NE_AssertPointer(model,"NE_ModelRotate: NULL pointer.");
	model->rx = (model->rx + rx + 512) & 0x1FF;
	model->ry = (model->ry + ry + 512) & 0x1FF;
	model->rz = (model->rz + rz + 512) & 0x1FF;
}

inline void NE_ModelSetRot(NE_Model * model, int rx, int ry, int rz)
{
	NE_AssertPointer(model,"NE_ModelSetRot: NULL pointer.");
	model->rx = rx;
	model->ry = ry;
	model->rz = rz;
}

//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------


//---------------------------------------------------------
//                    INTERNAL
//---------------------------------------------------------
static void __ne_drawanimatedmodel_interpolate(NE_AnimData * data)
{
	int frame_one = data->currframe;
	int frame_two = data->currframe + data->direction;
	u32 time = data->nextframetime;
	
	if(data->direction < 0 && frame_two < data->startframe)
	{
		if(data->animtype == NE_ANIM_LOOP)
			frame_two = data->endframe;
		else if(data->animtype == NE_ANIM_UPDOWN)
			frame_two = data->currframe + 1;
		else if(data->animtype == NE_ANIM_ONESHOT)
			frame_two = frame_one;
	}
	else if(data->direction > 0 && frame_two > data->endframe)
	{
		if(data->animtype == NE_ANIM_LOOP)
			frame_two = data->startframe;
		else if(data->animtype == NE_ANIM_UPDOWN)
			frame_two = data->currframe - 1;
		else if(data->animtype == NE_ANIM_ONESHOT)
			frame_two = frame_one;
	}
	
	u32 * fileptr = data->fileptrtr + 2;
	NE_Assert(frame_one < *fileptr && frame_two < *fileptr,"__ne_drawanimatedmodel_interpolate: Trying to draw unnexistent frame.");
	fileptr ++;
	u32 vtxcount = *fileptr++;
	u16 * framearrayptr = (u16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * vtxarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * normarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * texcoordsarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr);
	
	u16 * frame_one_ptr = (u16*)((int)framearrayptr + ( vtxcount * 6 * frame_one ));
	u16 * frame_two_ptr = (u16*)((int)framearrayptr + ( vtxcount * 6 * frame_two ));
	
	GFX_BEGIN = GL_TRIANGLES;
	
	u32 i;
	int j, vector[3];
	s16 * frame_one_data, * frame_two_data;
	for(i = 0; i < vtxcount; i ++)
	{
		//Texture coordinates
		//-------------------
		frame_one_data = &texcoordsarrayptr[*frame_one_ptr * 2];
		GFX_TEX_COORD = ((s32)*frame_one_data) | (((s32)*(frame_one_data+1))<<16);
		frame_one_ptr++; frame_two_ptr++; 
		
		//Normal
		//------
		frame_one_data = &normarrayptr[*frame_one_ptr * 3];
		frame_two_data = &normarrayptr[*frame_two_ptr * 3];
		for(j = 0; j < 3; j++)
		{
			vector[j] = ((s32)*frame_one_data + ((((s32)*frame_two_data-(s32)*frame_one_data) * time) >> 6))  & 0x3FF;
			frame_one_data++; frame_two_data++;
		}
		GFX_NORMAL = (vector[0]<<20)|(vector[1]<<10)|vector[2];
		frame_one_ptr++; frame_two_ptr++; 
			
		//Vertex
		//------
		frame_one_data = &vtxarrayptr[*frame_one_ptr * 3];
		frame_two_data = &vtxarrayptr[*frame_two_ptr * 3];
		for(j = 0; j < 3; j++)
		{
			vector[j] = ((s32)*frame_one_data + ((((s32)*frame_two_data-(s32)*frame_one_data) * time) >> 6))  & 0xFFFF;
			frame_one_data++; frame_two_data++;
		}
		GFX_VERTEX16 = vector[0] | ( vector[1] << 16 );
		GFX_VERTEX16 = vector[2];
		frame_one_ptr++; frame_two_ptr++; 
	}
	
	//GFX_END = 0;
}


static void __ne_drawanimatedmodel_nointerpolate(NE_AnimData * data)
{
	int frame = data->currframe;
	
	u32 * fileptr = data->fileptrtr + 2;
	NE_Assert(frame < *fileptr,"__ne_drawanimatedmodel_nointerpolate: Trying to draw unnexistent frame.");
	fileptr ++;
	u32 vtxcount = *fileptr++;
	u16 * framearrayptr = (u16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * vtxarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * normarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr++);
	s16 * texcoordsarrayptr = (s16*)((int)data->fileptrtr + (int)*fileptr);
	
	u16 * frame_ptr = (u16*)((int)framearrayptr + ( vtxcount * 6 * frame ));
	
	GFX_BEGIN = GL_TRIANGLES;
	
	u32 i;
	int j, vector[3];
	s16 * frame_data;
	for(i = 0; i < vtxcount; i ++)
	{
		//Texture coordinates
		//-------------------
		frame_data = &texcoordsarrayptr[*frame_ptr * 2];
		GFX_TEX_COORD = ((s32)*frame_data) | (((s32)*(frame_data+1))<<16);
		frame_ptr++;
		
		//Normal
		//------
		frame_data = &normarrayptr[*frame_ptr * 3];
		for(j = 0; j < 3; j++)
		{
			vector[j] = ((s32)*frame_data)  & 0x3FF;
			frame_data++;
		}
		GFX_NORMAL = (vector[0]<<20)|(vector[1]<<10)|vector[2];
		frame_ptr++; 
		
		//Vertex
		//------
		frame_data = &vtxarrayptr[*frame_ptr * 3];
		for(j = 0; j < 3; j++)
		{
			vector[j] = ((s32)*frame_data)  & 0xFFFF;
			frame_data++;
		}
		GFX_VERTEX16 = vector[0] | ( vector[1] << 16 );
		GFX_VERTEX16 = vector[2];
		frame_ptr++;
	}
	
	//GFX_END = 0;
}
//---------------------------------------------------------

void NE_ModelAnimateAll(void)
{
	if(!ne_model_system_inited) return;

	int a;
	for(a = 0; a < NE_MAX_MODELS; a++) if(NE_ModelPointers[a] != NULL)
	if(NE_ModelPointers[a]->modeltype == NE_Animated)
	{
		NE_AnimData * model = (NE_AnimData*)(NE_ModelPointers[a]->meshdata);
		
		model->nextframetime += model->speed[model->currframe];
		
		if(abs(model->nextframetime) > 64)
		{
			model->nextframetime = 0;
			
			switch(model->animtype)
			{
				case NE_ANIM_LOOP:
				{
					if(model->currframe == model->startframe && model->direction < 0)
						model->currframe = model->endframe;
					else if(model->currframe == model->endframe && model->direction > 0)
						model->currframe = model->startframe;
					else
					{
						if(model->direction > 0) model->currframe++;
						else model->currframe--;				
					}
					break;
				}
				case NE_ANIM_ONESHOT:
				{
					if(model->currframe == model->startframe && model->direction < 0)
						NE_ModelAnimSetSpeed(NE_ModelPointers[a], 0);
					else if(model->currframe == model->endframe && model->direction > 0)
						NE_ModelAnimSetSpeed(NE_ModelPointers[a], 0);
					else
					{
						if(model->direction > 0) model->currframe++;
						else model->currframe--;				
					}
					break;
				}
				case NE_ANIM_UPDOWN:
				{
					if(model->currframe == model->startframe && model->direction < 0)
					{
						model->direction *= -1;
						model->currframe++;
					}
					else if(model->currframe == model->endframe && model->direction > 0)
					{
						model->direction *= -1; 
						model->currframe--;
					}
					else
					{
						if(model->direction > 0) model->currframe++;
						else model->currframe--;				
					}				
					break;
				}
			}
		}
	}
}

void NE_ModelAnimStart(NE_Model * model, int min, int start, int max, NE_AnimationTypes type, int speed)
{
	NE_AssertPointer(model,"NE_ModelAnimStart: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimStart: Not an animated model.");

	((NE_AnimData*)model->meshdata)->animtype = type;
	((NE_AnimData*)model->meshdata)->currframe = start;
	((NE_AnimData*)model->meshdata)->startframe = min;
	((NE_AnimData*)model->meshdata)->endframe = max;
	((NE_AnimData*)model->meshdata)->nextframetime = 0;
	((NE_AnimData*)model->meshdata)->direction = ((speed >= 0) ? 1 : -1);

	int a; for(a = 0; a < NE_MAX_FRAMES; a++) 
		((NE_AnimData*)model->meshdata)->speed[a] = abs(speed); 
}


void NE_ModelAnimSetSpeed(NE_Model * model, int speed)
{
	NE_AssertPointer(model,"NE_ModelAnimSetSpeed: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimSetSpeed: Not an animated model.");
	int a;
	for(a = 0; a < NE_MAX_FRAMES; a++) ((NE_AnimData*)model->meshdata)->speed[a] = abs(speed);
	((NE_AnimData*)model->meshdata)->direction = ((speed >= 0) ? 1 : -1);
}


inline void NE_ModelAnimSetFrameSpeed(NE_Model * model, int frame, int speed)
{
	NE_AssertPointer(model,"NE_ModelAnimSetFrameSpeed: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimSetFrameSpeed: Not an animated model.");
	((NE_AnimData*)model->meshdata)->speed[frame] = speed; 
}


inline int NE_ModelAnimGetFrame(NE_Model * model)
{
	NE_AssertPointer(model,"NE_ModelAnimGetFrame: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimGetFrame: Not an animated model.");
	return ((NE_AnimData*)model->meshdata)->currframe;
}


inline void NE_ModelAnimSetFrame(NE_Model * model, int frame)
{
	NE_AssertPointer(model,"NE_ModelAnimSetFrame: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimSetFrame: Not an animated model.");
	((NE_AnimData*)model->meshdata)->currframe = frame;
	((NE_AnimData*)model->meshdata)->nextframetime = 0;
}

inline void NE_ModelAnimInterpolate(NE_Model * model, bool interpolate)
{
	NE_AssertPointer(model,"NE_ModelAnimInterpolate: NULL pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelAnimInterpolate: Not an animated model.");
	model->anim_interpolate = interpolate;
}


int NE_ModelLoadNEAFAT(NE_Model * model, char * path)
{
	if(!ne_model_system_inited) return 0;

	NE_AssertPointer(model,"NE_ModelLoadNEAFAT: NULL model pointer.");
	NE_AssertPointer(path,"NE_ModelLoadNEAFAT: NULL path pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelLoadNEAFAT: Not an animated model.");

	if(model->meshfromfat) free(((NE_AnimData*)model->meshdata)->fileptrtr);

	model->iscloned = 0;
	model->meshfromfat = true;

	u32 * pointer = (u32*)NE_FATLoadData(path);
	NE_AssertPointer(pointer,"NE_ModelLoadNEAFAT: Couldn't allocate data from FAT.");

	if(*pointer != 1296123214) 
	{
		NE_DebugPrint("NE_ModelLoadNEAFAT: Not a NEA file");
		free(pointer);
		return 0; // 'NEAM' - Not a nea file
	}
	if(*(pointer + 1) != 2) 
	{
		NE_DebugPrint("NE_ModelLoadNEAFAT: NEA file version is %d, should be 2.",*(pointer + 1));
		free(pointer);
		return 0; // version 2
	}

	((NE_AnimData*)model->meshdata)->fileptrtr = (void*)pointer;

	return 1;
}

int NE_ModelLoadNEA(NE_Model * model, u32 * pointer)
{
	if(!ne_model_system_inited) return 0;

	NE_AssertPointer(model,"NE_ModelLoadNEA: NULL model pointer.");
	NE_AssertPointer(pointer,"NE_ModelLoadNEA: NULL data pointer.");
	NE_Assert(model->modeltype == NE_Animated,"NE_ModelLoadNEA: Not an animated model.");

	if(model->meshfromfat) free(((NE_AnimData*)model->meshdata)->fileptrtr);

	model->iscloned = 0;
	model->meshfromfat = false;

	if(*pointer != 1296123214) // 'NEAM' - Not a nea file
	{
		NE_DebugPrint("NE_ModelLoadNEA: Not a NEA file");
		return 0; 
	}
	if(*(pointer + 1) != 2) // version 2
	{
		NE_DebugPrint("NE_ModelLoadNEA: NEA file version is %d, should be 2.",*(pointer + 1));
		return 0; 
	}
	((NE_AnimData*)model->meshdata)->fileptrtr = pointer;

	return 1;
}


void NE_ModelDeleteAll(void)
{
	if(!ne_model_system_inited) return;

	int a;
	for(a = 0; a < NE_MAX_MODELS; a++)
		if(NE_ModelPointers[a] != NULL) 
			NE_ModelDelete(NE_ModelPointers[a]);
}


void NE_ModelSystemReset(int number_of_models)
{
	if(ne_model_system_inited) NE_ModelSystemEnd();

	if(number_of_models < 1) NE_MAX_MODELS = NE_DEFAULT_MODELS;
	else NE_MAX_MODELS = number_of_models;

	NE_ModelPointers = malloc(NE_MAX_MODELS * sizeof(NE_ModelPointers));
	NE_AssertPointer(NE_ModelPointers,"NE_ModelSystemReset: Not enough memory to allocate array.");

	int a;
	for(a = 0; a < NE_MAX_MODELS; a++) NE_ModelPointers[a] = NULL;

	ne_model_system_inited = true;
}

void NE_ModelSystemEnd(void)
{
	if(!ne_model_system_inited) return;

	NE_ModelDeleteAll();

	free(NE_ModelPointers);

	ne_model_system_inited = false;
}


