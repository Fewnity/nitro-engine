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

/*! \file   NEGeneral.c */

const char NE_VERSION[] = 
	"  Nitro Engine - Version " NITRO_ENGINE_VERSION_STRING "\n\n\n" 
	"                  by AntonioND\n\n\n\n\n\n"
	"(C) 2008-2009 Antonio Ni�o D�az";



bool NE_UsingConsole;
bool NE_TestTouch;
static int NE_screenratio;
static u8 NE_viewport[4];
static u8 NE_Screen;
bool NE_Dual;

_NE_input_ NE_Input;

static bool ne_inited = false;

static SpriteEntry * NE_Sprites; //For Dual 3D mode

static int ne_znear, ne_zfar;

void NE_End(void)
{
	if(!ne_inited) return;

	//Hide BG0
	REG_DISPCNT &= ~(DISPLAY_BG0_ACTIVE | ENABLE_3D);

	vramSetBankA(VRAM_A_LCD);
	vramSetBankB(VRAM_B_LCD);
	if(NE_Dual)
	{
		vramSetBankC(VRAM_C_LCD);
		vramSetBankD(VRAM_D_LCD);
		
		free(NE_Sprites);
	}
	else if (GFX_CONTROL & GL_CLEAR_BMP) 
	{
		NE_ClearBMPEnable(false);
	}
	vramSetBankE(VRAM_E_LCD);
	if(NE_UsingConsole)
	{
		vramSetBankF(VRAM_F_LCD);
		NE_UsingConsole = false;
	}

	NE_APISystemEnd();
	NE_SpriteSystemEnd();
	NE_PhysicsSystemEnd();
	NE_ModelSystemEnd();
	NE_TextResetSystem();
	NE_TextureSystemEnd();
	NE_CameraSystemEnd();
	NE_SpecialEffectSet(0);

	//Power off 3D hardware
	powerOff(POWER_3D_CORE|POWER_MATRIX);

	NE_DebugPrint("NE_End: Nitro Engine disactivated.");

	ne_inited = false;
}



void NE_Viewport(int x1, int y1, int x2, int y2)
{
	glViewport(x1, y1, x2, y2);

	NE_viewport[0] = x1;
	NE_viewport[1] = y1;
	NE_viewport[2] = x2;
	NE_viewport[3] = y2;

	NE_screenratio = divf32((x2-x1+1)<<12, (y2-y1+1)<<12);

	MATRIX_CONTROL = GL_PROJECTION; //New projection matix for this viewport
	MATRIX_IDENTITY = 0;
	gluPerspectivef32(70 * DEGREES_IN_CIRCLE / 360, NE_screenratio, ne_znear, ne_zfar);    

	MATRIX_CONTROL = GL_MODELVIEW;
}


static void NE_Init__(void)
{
	//Power all 3D and 2D. Hide 3D screen during init
	powerOn(POWER_ALL);

	videoSetMode(0);

	vramSetBankE(VRAM_E_TEX_PALETTE);

	while (GFX_STATUS & (1<<27));

	// Clear the FIFO
	GFX_STATUS |= (1<<29);

	glFlush(0);
	glFlush(0);

	lcdMainOnTop();

	glResetMatrixStack();

	GFX_CONTROL = GL_TEXTURE_2D | GL_ANTIALIAS | GL_BLEND;

	glAlphaFunc(BLEND_ALPHA);

	NE_ClearColorSet(NE_Black,31,63);
	NE_FogEnableBackground(false);
	glClearDepth(GL_MAX_DEPTH);

	//Default number of objects for everyting - Textures are inited in NE_Init3D and NE_InitDual3D
	NE_CameraSystemReset(0);
	NE_PhysicsSystemReset(0);
	NE_SpriteSystemReset(0);
	NE_APISystemReset(0);
	NE_ModelSystemReset(0);
	NE_TextPriorityReset();

	glMatrixMode(GL_TEXTURE);
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//ds uses a table for shinyness..this generates a half-ass one
	glMaterialShinyness();

	// setup default material properties
	NE_MaterialSetDefaultPropierties(RGB15(20,20,20), RGB15(16,16,16), RGB15(8,8,8), RGB15(5,5,5), false, true);

	//Turn off some things...
	int a = 4;
	while(a--) NE_LightOff(a);

	GFX_COLOR = 0;
	GFX_POLY_FORMAT = 0;

	a = 8;
	while(a--) NE_OutliningSetColor(a,0);

	ne_znear = floattof32(0.1);
	ne_zfar = floattof32(40.0);
	NE_Viewport(0,0,255,191);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Ready!!

	videoSetMode(MODE_0_3D);
}


static inline void NE_SetRegCapture(bool enable, uint8 srcBlend, uint8 destBlend, uint8 bank, uint8 offset, uint8 size, uint8 source, uint8 srcOffset)
{
	uint32 value = 0;
	if (enable) value |= 1 << 31; // 31 is enable

	value |= 3 << 29; // 29-30 seems to have something to do with the blending
	value |= (srcOffset & 0x3) << 26; // capture source offset is 26-27
	value |= (source & 0x3) << 24; // capture source is 24-25
	value |= (size & 0x3) << 20; // capture data write size is 20-21
	value |= (offset & 0x3) << 18; // write offset is 18-19 
	value |= (bank & 0x3) << 16; // vram bank select is 16-17 
	value |= (srcBlend & 0xFF) << 8; // graphics blend evb is 8..12
	value |= (destBlend & 0xFF) << 0; // ram blend EVA is bits 0..4 

	REG_DISPCAPCNT = value;
}

inline void NE_SwapScreens(void)
{
	REG_POWERCNT ^= POWER_SWAP_LCDS;
}


void NE_UpdateInput(void)
{
	NE_Input.kdown = keysDown();
	NE_Input.kheld = keysHeld();
	NE_Input.kup = keysUp();

	if(NE_Input.kheld & KEY_TOUCH)
	{
		touchRead(&NE_Input.touch);
	}
}


void NE_Init3D(void)
{
	if(ne_inited) NE_End();

	NE_Init__();

	NE_TextureSystemReset(0,0,NE_VRAM_ABCD);

	NE_UpdateInput();

	ne_inited = true;
	NE_Dual = false;

	NE_DebugPrint("NE_Init3D: Nitro Engine has been inited in normal 3D mode.");
}


void NE_InitDual3D(void)
{
	if(ne_inited) NE_End();

	NE_Init__();

	NE_TextureSystemReset(0,0,NE_VRAM_AB);

	NE_UpdateInput();

	videoSetModeSub(0);

	REG_BG2CNT_SUB = BG_BMP16_256x256;
	REG_BG2PA_SUB = 1 << 8;
	REG_BG2PB_SUB = 0;
	REG_BG2PC_SUB = 0;
	REG_BG2PD_SUB = 1 << 8;
	REG_BG2X_SUB = 0;
	REG_BG2Y_SUB = 0;

	vramSetBankC(VRAM_C_SUB_BG);
	vramSetBankD(VRAM_D_SUB_SPRITE);

	NE_Sprites = malloc(sizeof(SpriteEntry) * 128);

	int y = 0, x = 0;

	int i; //Reset sprites
	for(i = 0; i < 128; i++)
	{
		NE_Sprites[i].attribute[0] = ATTR0_DISABLED;
		NE_Sprites[i].attribute[1] = 0;
		NE_Sprites[i].attribute[2] = 0;
	}
	
	i = 0;
	for (y = 0; y < 3; y++) for (x = 0; x < 4; x++)
	{
		NE_Sprites[i].attribute[0] = ATTR0_BMP | ATTR0_SQUARE | (64 * y);
		NE_Sprites[i].attribute[1] = ATTR1_SIZE_64 | (64 * x);
		NE_Sprites[i].attribute[2] = ATTR2_ALPHA(1) | (8 * 32 * y) | (8 * x);
		i++;
	}

	videoSetModeSub(MODE_5_2D | DISPLAY_BG2_ACTIVE | DISPLAY_SPR_ACTIVE | DISPLAY_SPR_2D_BMP_256);

	ne_inited = true;
	NE_Dual = true;

	NE_DebugPrint("NE_InitDual3D: Nitro Engine has been inited in dual 3D mode.");
}

void NE_InitConsole(void)
{
	if(!ne_inited) return;

	NE_UsingConsole = true;

	videoSetMode(MODE_0_3D | DISPLAY_BG1_ACTIVE);
		
	vramSetBankF(VRAM_F_MAIN_BG);
	 
	BG_PALETTE[255] = 0xFFFF;

	REG_BG1CNT = BG_MAP_BASE(4) | BG_PRIORITY(0); // use bg 1 for text, set to highest priority
	REG_BG0CNT = BG_PRIORITY(1); //set bg 0 (3d background) to be a lower priority than bg 1

	consoleInit(0,1, BgType_Text4bpp, BgSize_T_256x256, 4,0, true, true);
}



inline void NE_SetConsoleColor(u32 color)
{
	BG_PALETTE[255] = color;
}


void NE_Process(NE_Voidfunc drawscene)
{
	NE_UpdateInput();

	glViewport(NE_viewport[0],NE_viewport[1],NE_viewport[2],NE_viewport[3]);

	NE_PolyFormat(31,0,NE_LIGHT_ALL, NE_CULL_BACK,0);

	MATRIX_CONTROL = GL_PROJECTION;
	MATRIX_IDENTITY = 0;
	gluPerspectivef32(70 * DEGREES_IN_CIRCLE / 360, NE_screenratio, ne_znear, ne_zfar); 

	MATRIX_CONTROL = GL_MODELVIEW;
	MATRIX_IDENTITY = 0;

	NE_AssertPointer(drawscene,"NE_Process: NULL function pointer.");
	drawscene();

	GFX_FLUSH = (1<<0)/* | (1<<1)*/; //GL_TRANS_MANUALSORT | GL_WBUFFERING
}


void NE_ProcessDual(NE_Voidfunc topscreen, NE_Voidfunc downscreen)
{
	NE_UpdateInput();

	REG_POWERCNT ^= POWER_SWAP_LCDS;
	NE_Screen ^= 1;

	if(NE_Screen == 1)	
	{
		if(NE_UsingConsole)
		{
			REG_BG1CNT = BG_MAP_BASE(4) | BG_PRIORITY(0);
			REG_BG0CNT = BG_PRIORITY(1); 
		}
		
		vramSetBankC(VRAM_C_SUB_BG); vramSetBankD(VRAM_D_LCD);
		NE_SetRegCapture(true, 0, 31, 3, 0, 3, 0, 0);
	}	   
	else
	{
		if(NE_UsingConsole)
		{
			REG_BG1CNT = BG_PRIORITY(1); 
			REG_BG0CNT = BG_PRIORITY(0);
		}
		
		vramSetBankC(VRAM_C_LCD); vramSetBankD(VRAM_D_SUB_SPRITE);
		NE_SetRegCapture(true, 0, 31, 2, 0, 3, 1, 0);
	}	

	NE_PolyFormat(31,0,NE_LIGHT_ALL, NE_CULL_BACK,0);

	glViewport(0,0,255,191);

	NE_viewport[0] = 0;   NE_viewport[1] = 0;
	NE_viewport[2] = 255; NE_viewport[3] = 191;

	MATRIX_CONTROL = GL_PROJECTION;
	MATRIX_IDENTITY = 0;
	gluPerspectivef32(70 * DEGREES_IN_CIRCLE / 360, floattof32(256.0/192.0), ne_znear, ne_zfar); 

	MATRIX_CONTROL = GL_MODELVIEW;
	MATRIX_IDENTITY = 0;

	NE_AssertPointer(topscreen,"NE_Process: NULL function pointer (top screen).");
	NE_AssertPointer(downscreen,"NE_Process: NULL function pointer (lower screen).");

	if(NE_Screen == 1) { topscreen(); }
	else { downscreen(); }

	GFX_FLUSH = (1<<0) /*| (1<<1)*/; //GL_TRANS_MANUALSORT | GL_WBUFFERING

	dmaCopy(NE_Sprites, OAM_SUB, 128 * sizeof(SpriteEntry));
}


inline void NE_ClippingPlanesSetI(int znear, int zfar)
{
	NE_Assert(znear < zfar,"NE_ClippingPlanesSet: znear should be smaller than zfar.");
	ne_znear = znear;
	ne_zfar = zfar; 
}


inline void NE_AntialiasEnable(bool value)
{
	if(value) GFX_CONTROL |= GL_ANTIALIAS;
	else GFX_CONTROL &= ~GL_ANTIALIAS;
}


inline int NE_GetPolygonCount(void)
{
	while (GFX_STATUS & (1<<27));
	return GFX_POLYGON_RAM_USAGE;
}

inline int NE_GetVertexCount(void)
{
	while (GFX_STATUS & (1<<27));
	return GFX_VERTEX_RAM_USAGE;
}


static int NE_Effect = 0;
static int NE_lastvbladd = 0;
static bool NE_effectpause;
static int * ne_noisepause;
static int ne_cpucount;
static int ne_noise_value = 0xF;
static int ne_sine_mult = 10, ne_sine_shift = 9;

void NE_VBLFunc(void) 
{ 
	if(!ne_inited) return;

	if(NE_Effect == 1 || NE_Effect == 2)
	{
		if(!NE_effectpause) NE_lastvbladd = (NE_lastvbladd + 1) & LUT_MASK;
	}
}

void NE_SpecialEffectPause(bool pause)
{
	if(NE_Effect == 0) return;

	NE_effectpause = pause;
	if(pause)
	{
		ne_noisepause = malloc(sizeof(int) * 512);
		int a;
		for(a = 0; a < 512; a++) ne_noisepause[a] = (rand() & ne_noise_value) - (ne_noise_value>>1);
	}
	else
	{
		if(ne_noisepause != NULL) 
		{
			free(ne_noisepause);
			ne_noisepause = NULL;
		}
	}
}

void NE_HBLFunc(void)
{
	if(!ne_inited) return;

	int a;
	ne_cpucount ++;
	int vcount = REG_VCOUNT;
	if(vcount == 262) vcount = 0; //Fixes a problem with first line

	switch(NE_Effect)
	{
		case NE_NOISE:
		{
			if(NE_effectpause && ne_noisepause)
				a = ne_noisepause[vcount & LUT_MASK];
			else a = (rand() & ne_noise_value) - (ne_noise_value>>1);
			REG_BGOFFSETS = a;
			break;
		}
		case NE_SINE:
		{
			REG_BGOFFSETS = (sinLerp(  ( ((vcount + NE_lastvbladd)  * ne_sine_mult) & LUT_MASK ) << 6) >> ne_sine_shift); 
			break;
		}
		default:
		{
			break;
		}
	}
}

inline void NE_SpecialEffectNoiseConfig(int value)
{
	ne_noise_value = value;
}


inline void NE_SpecialEffectSineConfig(int mult, int shift)
{
	ne_sine_mult = mult;
	ne_sine_shift = shift;
}

void NE_SpecialEffectSet(NE_SPECIAL_EFFECTS effect)
{
	switch(effect)
	{
		case 1:
			NE_Effect = 1;
			break;
		case 2:
			NE_Effect = 2;
			break;
		case 0:
		default:
			NE_Effect = 0;
			REG_BGOFFSETS = 0;
			break;
	}
}


int NE_CPUPercent;

void NE_WaitForVBL(NE_UPDATE_FLAGS flags)
{
	if(flags & NE_UPDATE_API) NE_APIUpdate();
	if(flags & NE_UPDATE_ANIMATIONS) NE_ModelAnimateAll();
	if(flags & NE_UPDATE_PHYSICS) NE_PhysicsUpdateAll();

	NE_CPUPercent = div32(ne_cpucount * 100, 263);
	if(flags & NE_CAN_SKIP_VBL) if(NE_CPUPercent > 100)
	{
		ne_cpucount = 0;
		return;
		
		//REG_DISPSTAT & DISP_IN_VBLANK
	}

	swiWaitForVBlank();
	ne_cpucount = 0;
}


inline int NE_GetCPUPercent(void)
{
	return NE_CPUPercent;
}


inline bool NE_GPUIsRendering(void)
{
	if(REG_VCOUNT > 190 && REG_VCOUNT < 214) return false;
	return true;
}



#ifdef NE_DEBUG
//-------------------------------------------------------------------------
//                               DEBUG
void (*ne_userdebugfn)(const char *) = NULL;

inline void __ne_debugoutputtoconsole(const char * text)
{
	printf(text);
	printf("\n");
}

inline void __NE_debugprint(const char * text)
{
	if(!ne_inited) return;
	if(ne_userdebugfn) ne_userdebugfn(text);
}

inline void NE_DebugSetHandler(void (*fn)(const char*))
{
	ne_userdebugfn = fn;
}

inline void NE_DebugSetHandlerConsole(void)
{
	NE_InitConsole();
	ne_userdebugfn = __ne_debugoutputtoconsole;
}
//-------------------------------------------------------------------------
#endif



#include <nds/arm9/postest.h>

static int ne_vertexcount;

void NE_TouchTestStart(void)
{
	glViewport(255,255,255,255); //Hide what we are going to draw

	glMatrixMode(GL_MODELVIEW);  //Save current state
	glPushMatrix();  

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	int temp[4] = {NE_viewport[0],NE_viewport[1],NE_viewport[2],NE_viewport[3]};
	gluPickMatrix(NE_Input.touch.px,191-NE_Input.touch.py,3,3,temp); // render only what is below the cursor
	gluPerspectivef32(70 * DEGREES_IN_CIRCLE / 360, NE_screenratio, ne_znear, ne_zfar);  

	glMatrixMode(GL_MODELVIEW); 

	NE_Assert(!NE_TestTouch,"NE_TouchTestStart: This has been called previously.");

	NE_TestTouch = true;
}

void NE_TouchTestObject(void)
{
	NE_Assert(NE_TestTouch,"NE_TouchTestObject: NE_TouchTestStart hasn't been called previously.");

	while(PosTestBusy()); // wait for the position test to finish
	while(GFX_BUSY); // wait for all the polygons from the last object to be drawn

	ne_vertexcount = NE_GetVertexCount(); // save the vertex ram count
}

int NE_TouchTestResult(void)
{
	NE_Assert(NE_TestTouch,"NE_TouchTestResult: NE_TouchTestStart hasn't been called previously.");

	while(GFX_BUSY); // wait for all the polygons to get drawn
	while(PosTestBusy()); // wait for the position test to finish
	if(NE_GetVertexCount() > ne_vertexcount) // if a polygon was drawn
	{
		return PosTestWresult();
	}
	
	return -1;
}

void NE_TouchTestEnd(void)
{
	NE_Assert(NE_TestTouch,"NE_TouchTestEnd: NE_TouchTestStart hasn't been called previously.");

	NE_TestTouch = false;

	glViewport(NE_viewport[0],NE_viewport[1],NE_viewport[2],NE_viewport[3]); // reset the viewport

	glMatrixMode(GL_PROJECTION); //Get previous state
	glPopMatrix(1);
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix(1);
}

