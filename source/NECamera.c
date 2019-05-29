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

#include "NEMain.h"

/*! \file   NECamera.c */

static NE_Camera ** NE_UserCamera;
static int NE_MAX_CAMERAS;
static bool ne_camera_system_inited = false;

//----------------------------------------------------------------
//              INTERNAL
static void __NE_CameraUpdateMatrix(NE_Camera * cam)
{
	//From libnds, modified a bit
	int32 side[3], forward[3], up[3], a;

	for(a = 0; a < 3; a++) forward[a] = cam->from[a] - cam->to[a]; 

	normalizef32(forward); 

	crossf32(cam->up, forward, side); 

	normalizef32(side); 

	// Recompute local up
	crossf32(forward, side, up);

	cam->matrix.m[0] = side[0]; 
	cam->matrix.m[1] = up[0]; 
	cam->matrix.m[2] = forward[0]; 
	cam->matrix.m[3] = 0;

	cam->matrix.m[4] = side[1]; 
	cam->matrix.m[5] = up[1]; 
	cam->matrix.m[6] = forward[1]; 
	cam->matrix.m[7] = 0;

	cam->matrix.m[8] = side[2]; 
	cam->matrix.m[9] = up[2]; 
	cam->matrix.m[10] = forward[2]; 
	cam->matrix.m[11] = 0;

	cam->matrix.m[12] = -dotf32(cam->from,side); 
	cam->matrix.m[13] = -dotf32(cam->from,up); 
	cam->matrix.m[14] = -dotf32(cam->from,forward); 
	cam->matrix.m[15] = inttof32(1);	
}



//----------------------------------------------------------------

NE_Camera * NE_CameraCreate(void)
{
	if(!ne_camera_system_inited) return NULL;

	int a;
	for(a = 0; a < NE_MAX_CAMERAS; a++) if(NE_UserCamera[a] == NULL) 
	{
		NE_Camera * cam = malloc(sizeof(NE_Camera));
		NE_AssertPointer(cam,"NE_CameraCreate: Couldn't allocate camera.");
		
		cam->from[0] = cam->from[1] = cam->from[2] = 0;
		cam->to[0] = cam->to[1] = 0; cam->to[2] = inttof32(1);
		cam->up[0] = 0; cam->up[1] = inttof32(1); cam->up[2] = 0;
		
		cam->matrix_is_updated = false;
		NE_UserCamera[a] = cam;
		
		return cam;
	}

	NE_DebugPrint("NE_CameraCreate: No free slots...");

	return NULL;
}



inline void NE_CameraSetI(NE_Camera * cam, int xfrom, int yfrom, int zfrom,
				   int xto, int yto, int zto, int xup, int yup, int zup)
{
	NE_AssertPointer(cam,"NE_CameraSetI: NULL pointer.");

	cam->from[0] = xfrom; cam->from[1] = yfrom; cam->from[2] = zfrom;
	cam->to[0] = xto; cam->to[1] = yto; cam->to[2] = zto;
	cam->up[0] = xup; cam->up[1] = yup; cam->up[2] = zup;

	cam->matrix_is_updated = false;
}


inline void NE_CameraUse(NE_Camera * cam)
{
	NE_AssertPointer(cam,"NE_CameraUse: NULL pointer.");

	if(!cam->matrix_is_updated) 
	{
		__NE_CameraUpdateMatrix(cam);
		cam->matrix_is_updated = true;
	}

	glLoadMatrix4x4(&cam->matrix);
}


void NE_CameraMoveFreeI(NE_Camera * cam, int front, int right, int up)
{
	NE_AssertPointer(cam,"NE_CameraMoveFreeI: NULL pointer.");

	cam->matrix_is_updated = false;

	int vec_front[3], vec_right[3], vec_up[3];
	int temp[3]; //Used to normalize other vectors 
	int result[3] = {0,0,0};

	int a;
	for(a = 0; a < 3; a++) vec_front[a] = cam->to[a] - cam->from[a];

	if(front != 0)
	{
		for(a = 0; a < 3; a++)	temp[a] = vec_front[a];
		normalizef32((int32*)&temp);
		for(a = 0; a < 3; a++)	result[a] += mulf32(temp[a], front);
	}

	if(right != 0 || up != 0) //Right vector is needed to get the up vector
	{
		crossf32(vec_front,cam->up,vec_right);
		
		if(right != 0)
		{
			for(a = 0; a < 3; a++)	temp[a] = vec_right[a];
			normalizef32((int32*)&temp);
			for(a = 0; a < 3; a++)	result[a] += mulf32(temp[a], right);		
		}
		
		if(up != 0)
		{
			crossf32(vec_right,vec_front,vec_up);
			normalizef32((int32*)&vec_up); //Last vector, not needed to use "temp"
			for(a = 0; a < 3; a++)	result[a] += mulf32(vec_up[a], up);
		}
	}

	for(a = 0; a < 3; a++)
	{
		cam->from[a] += result[a];
		cam->to[a] += result[a];
	}

	/*
	cam->matrix.m[12] -= right;
	cam->matrix.m[13] -= up;
	cam->matrix.m[14] += front;
	*/
}


inline void NE_CameraMoveI(NE_Camera * cam, int x, int y, int z)
{
	NE_AssertPointer(cam,"NE_CameraMoveI: NULL pointer.");

	cam->matrix_is_updated = false;

	cam->from[0] += x;
	cam->from[1] += y;
	cam->from[2] += z;

	cam->to[0] += x;
	cam->to[1] += y;
	cam->to[2] += z;
}


void NE_CameraRotate(NE_Camera * cam, int rx, int ry, int rz)
{
	NE_AssertPointer(cam,"NE_CameraRotate: NULL pointer.");

	int cam_vector[3], result_vector[3];

	int a;
	for(a = 0; a < 3; a++) cam_vector[a] = cam->to[a] - cam->from[a];

	int sine, cosine;

	if(rx != 0)
	{
		cam->matrix_is_updated = false;
		
		sine = sinLerp((rx & LUT_MASK) << 6); 
		cosine = cosLerp((rx & LUT_MASK) << 6); 
		
		result_vector[1] = mulf32(cam_vector[1],cosine) - mulf32(cam_vector[2],sine);
		result_vector[2] = mulf32(cam_vector[1],sine) + mulf32(cam_vector[2],cosine);
		
		cam->to[1] = cam->from[1] + result_vector[1];
		cam->to[2] = cam->from[2] + result_vector[2];
	}
	if(ry != 0)
	{
		cam->matrix_is_updated = false;
		
		sine = sinLerp((ry & LUT_MASK) << 6); 
		cosine = cosLerp((ry & LUT_MASK) << 6); 
		
		result_vector[0] = mulf32(cam_vector[0],cosine) - mulf32(cam_vector[2],sine);
		result_vector[2] = mulf32(cam_vector[0],sine) + mulf32(cam_vector[2],cosine);
		
		cam->to[0] = cam->from[0] + result_vector[0];
		cam->to[2] = cam->from[2] + result_vector[2];
	}
	if(rz != 0)
	{
		cam->matrix_is_updated = false;
		
		sine = sinLerp((rz & LUT_MASK) << 6); 
		cosine = cosLerp((rz & LUT_MASK) << 6); 
		
		result_vector[0] = mulf32(cam_vector[0],cosine) - mulf32(cam_vector[1],sine);
		result_vector[1] = mulf32(cam_vector[0],sine) + mulf32(cam_vector[1],cosine);
		
		cam->to[0] = cam->from[0] + result_vector[0];
		cam->to[1] = cam->from[1] + result_vector[1];
	}

	/*
	glMatrixMode(GL_PROJECTION); 
	glPushMatrix(); 
	glLoadMatrix4x4(&cam->matrix);

	//modifications - rotations, translations, etc...

	glGetFixed(GL_GET_MATRIX_PROJECTION, (int32*)&cam->matrix);
	glMatrixMode(GL_PROJECTION); 
	glPopMatrix(1);
	glMatrixMode(GL_MODELVIEW); 
	*/
}


//----------------------------------------------------------------
//              INTERNAL
static void __NE_RotateVectorAxis(int * vector, int angle, int x, int y, int z)
{
	//vector must be an array of 3 ints with your vector in f32 format!
	int sin = sinLerp((angle & LUT_MASK) << 6); 
	int cos = cosLerp((angle & LUT_MASK) << 6); 

	int one_minus_cos = inttof32(1) - cos;

	int x_by_one_minus_cos = mulf32(one_minus_cos,x);
	int y_by_one_minus_cos = mulf32(one_minus_cos,y);
	int z_by_one_minus_cos = mulf32(one_minus_cos,z);

	int x_by_sin = mulf32(sin,x);
	int y_by_sin = mulf32(sin,y);
	int z_by_sin = mulf32(sin,z);

	int matrix[9];
	matrix[0] = mulf32(x,x_by_one_minus_cos) + cos;
	matrix[1] = mulf32(y,x_by_one_minus_cos) - z_by_sin;
	matrix[2] = mulf32(z,x_by_one_minus_cos) + y_by_sin;

	matrix[3] = mulf32(x,y_by_one_minus_cos) + z_by_sin;
	matrix[4] = mulf32(y,y_by_one_minus_cos) + cos;
	matrix[5] = mulf32(z,y_by_one_minus_cos) - x_by_sin;

	matrix[6] = mulf32(x,z_by_one_minus_cos) - y_by_sin;
	matrix[7] = mulf32(y,z_by_one_minus_cos) + x_by_sin;
	matrix[8] = mulf32(z,z_by_one_minus_cos) + cos;

	int result_vector[3];

	result_vector[0] = mulf32(vector[0],matrix[0]) + mulf32(vector[1],matrix[3]) + 
						mulf32(vector[2],matrix[6]);
	result_vector[1] = mulf32(vector[0],matrix[1]) + mulf32(vector[1],matrix[4]) + 
						mulf32(vector[2],matrix[7]);
	result_vector[2] = mulf32(vector[0],matrix[2]) + mulf32(vector[1],matrix[5]) + 
						mulf32(vector[2],matrix[8]);

	int a;
	for(a = 0; a < 3; a++) vector[a] = result_vector[a];
}

//----------------------------------------------------------------

void NE_CameraRotateAxisI(NE_Camera * cam, int angle, int x, int y, int z)
{
	NE_AssertPointer(cam,"NE_CameraRotateAxisI: NULL pointer.");
	if(angle == 0) return;

	cam->matrix_is_updated = false;

	int cam_vector[3];

	int a;
	for(a = 0; a < 3; a++) cam_vector[a] = cam->to[a] - cam->from[a];

	//__NE_RotateVectorAxis(cam->up,angle, x, y, z);
	__NE_RotateVectorAxis(cam_vector,angle, x, y, z);

	for(a = 0; a < 3; a++) cam->to[a] = cam->from[a] + cam_vector[a];
}


void NE_CameraRotateFree(NE_Camera * cam, int rx, int ry, int rz)
{
	NE_AssertPointer(cam,"NE_CameraRotateFree: NULL pointer.");

	if(rx || ry  || rz)
	{
		cam->matrix_is_updated = false;
		
		int vec_front[3], vec_right[3], vec_up[3];
		
		int a;
		for(a = 0; a < 3; a++) vec_front[a] = cam->to[a] - cam->from[a];
		
		if(ry || rx)
		{
			crossf32(vec_front,cam->up,vec_right);
			
			if(ry) 
			{
				crossf32(vec_right,vec_front,vec_up);
				normalizef32(vec_up);
				NE_CameraRotateAxisI(cam,ry,vec_up[0],vec_up[1],vec_up[2]);
//				__NE_RotateVectorAxis(cam->up,ry, vec_up[0],vec_up[1],vec_up[2]);
			}
			
			if(rx)
			{
				normalizef32(vec_right);
				NE_CameraRotateAxisI(cam,rx,vec_right[0],vec_right[1],vec_right[2]);
//				__NE_RotateVectorAxis(cam->up,rx, vec_right[0],vec_right[1],vec_right[2]);
			}
		}
		
		if(rz)
		{
			normalizef32(vec_front);
			__NE_RotateVectorAxis(cam->up,rz,vec_front[0],vec_front[1],vec_front[2]);
//			NE_CameraRotateAxisI(cam,rz,vec_front[0],vec_front[1],vec_front[2]);
		}
	}
}



inline void NE_CameraDelete(NE_Camera * cam)
{
	NE_AssertPointer(cam,"NE_CameraDelete: NULL pointer.");

	int a;
	for(a = 0; a < NE_MAX_CAMERAS; a++) if(NE_UserCamera[a] == cam) 
	{
		NE_UserCamera[a] = NULL;
		free(cam);
		return;
	}
	
	NE_DebugPrint("NE_CameraDelete: Camera not found in array.");
}

void NE_CameraSystemReset(int numcameras)
{
	if(ne_camera_system_inited) NE_CameraSystemEnd();

	if(numcameras < 1) NE_MAX_CAMERAS = NE_DEFAULT_CAMERAS;
	else NE_MAX_CAMERAS = numcameras;

	NE_UserCamera = malloc(NE_MAX_CAMERAS * sizeof(NE_UserCamera));
	NE_AssertPointer(NE_UserCamera,"NE_CameraSystemReset: Not enough memory to allocate array.");

	int a;
	for(a = 0; a < NE_MAX_CAMERAS; a++) NE_UserCamera[a] = NULL;

	ne_camera_system_inited = true;
}

void NE_CameraSystemEnd(void)
{
	if(!ne_camera_system_inited) return;

	int a;
	for(a = 0; a < NE_MAX_CAMERAS; a++) if(NE_UserCamera[a]) 
		NE_CameraDelete(NE_UserCamera[a]);	

	free(NE_UserCamera);

	ne_camera_system_inited = false;
}


inline void NE_ViewPush(void)
{
	MATRIX_PUSH = 0;
}


inline void NE_ViewPop(void)
{
	MATRIX_POP = 1;
}


inline void NE_ViewMoveI(int x, int y, int z)
{
	MATRIX_TRANSLATE = x; MATRIX_TRANSLATE = y; MATRIX_TRANSLATE = z;
}

inline void NE_ViewRotate(int rx, int ry, int rz)
{
	if(rx != 0) glRotateXi(rx<<6);
	if(ry != 0) glRotateYi(ry<<6);
	if(rz != 0) glRotateZi(rz<<6);
}

inline void NE_ViewScaleI(int x, int y, int z)
{
	MATRIX_SCALE = x; MATRIX_SCALE = y; MATRIX_SCALE = z;
}

