// SPDX-License-Identifier: CC0-1.0
//
// SPDX-FileContributor: Antonio Niño Díaz, 2008-2011, 2019, 2022
//
// This file is part of Nitro Engine

#include <NEMain.h>

// Files autogenerated from bin files inside of the folder data
#include "teapot_bin.h"
#include "teapot_tex_bin.h"

NE_Camera *Camera;
NE_Model *Model;
NE_Material *Material;

bool wireframe;

void Draw3DScene(void)
{
    NE_CameraUse(Camera);

    if (wireframe)
        NE_PolyFormat(0, 0, NE_LIGHT_ALL, NE_CULL_BACK, 0);
    else
        NE_PolyFormat(31, 0, NE_LIGHT_ALL, NE_CULL_BACK, 0);

    NE_ModelDraw(Model);

    printf("\x1b[10;0H"
           "Polygon count: %d   \n"
           "Vertex count: %d   ",
           NE_GetPolygonCount(),
           NE_GetVertexCount());
}

int main(void)
{
    irqEnable(IRQ_HBLANK);
    irqSet(IRQ_VBLANK, NE_VBLFunc);
    irqSet(IRQ_HBLANK, NE_HBLFunc);

    NE_Init3D();
    // libnds uses VRAM_C for the text console, reserve A and B only
    NE_TextureSystemReset(0, 0, NE_VRAM_AB);
    // Init console in non-3D screen
    consoleDemoInit();

    // Allocate objects
    Model = NE_ModelCreate(NE_Static);
    Camera = NE_CameraCreate();
    Material = NE_MaterialCreate();

    // Setup camera
    NE_CameraSet(Camera,
                 0, 0, 3,
                 0, 0, 0,
                 0, 1, 0);

    // Load model. The texture coordinates are outside of [0.0, 1.0], so it is
    // needed to enable wrapping.
    NE_ModelLoadStaticMesh(Model, (u32*)teapot_bin);
    NE_MaterialTexLoad(Material, NE_A1RGB5, 256, 256,
                       NE_TEXGEN_TEXCOORD | NE_TEXTURE_WRAP_S | NE_TEXTURE_WRAP_T,
                       (u8 *)teapot_tex_bin);
    NE_ModelSetMaterial(Model, Material);

    // Set light color and direction
    NE_LightSet(0, NE_White, 0, 1, 0);
    NE_LightSet(1, NE_Blue, 0, -1, 0);
    NE_LightSet(2, NE_Red, 1, 0, 0);
    NE_LightSet(3, NE_Green, -1, 0, 0);

    bool use_specular = true;
    NE_ShininessFunction shininess = NE_SHININESS_QUADRATIC;

    while (1)
    {
        // Get keys information
        scanKeys();
        uint32 keys = keysHeld();
        uint32 keys_down = keysDown();

        printf("\x1b[0;0H"
               "PAD: Rotate\n"
               "A: Set wireframe mode\n"
               "X/Y: Specular or diffuse\n"
               "L/R: Change shininess\n");

        if (keys & KEY_A)
            wireframe = true;
        else
            wireframe = false;

        // Rotate model
        if (keys & KEY_UP)
            NE_ModelRotate(Model, -2, 0, 0);
        if (keys & KEY_DOWN)
            NE_ModelRotate(Model, 2, 0, 0);
        if (keys & KEY_RIGHT)
            NE_ModelRotate(Model, 0, 2, 0);
        if (keys & KEY_LEFT)
            NE_ModelRotate(Model, 0, -2, 0);

        // Shininess table
        // ---------------

        const char *names[] = {
            [NE_SHININESS_NONE]      = "None     ",
            [NE_SHININESS_LINEAR]    = "Linear   ",
            [NE_SHININESS_QUADRATIC] = "Quadratic",
            [NE_SHININESS_CUBIC]     = "Cubic    ",
            [NE_SHININESS_QUARTIC]   = "Quartic  "
        };
        printf("\nShininess: %s", names[shininess]);

        if (keys_down & KEY_L)
        {
            if (shininess > NE_SHININESS_LINEAR)
                shininess--;
        }
        if (keys_down & KEY_R)
        {
            if (shininess < NE_SHININESS_QUARTIC)
                shininess++;
        }

        NE_ShininessTableGenerate(shininess);

        // Diffuse or specular
        // -------------------

        if (keys & KEY_X)
            use_specular = true;
        if (keys & KEY_Y)
            use_specular = false;

        if (use_specular)
        {
            // Set some propierties to Material
            NE_MaterialSetPropierties(Material,
                        RGB15(0, 0, 0),    // Diffuse
                        RGB15(0, 0, 0),    // Ambient
                        RGB15(31, 31, 31), // Specular
                        RGB15(0, 0, 0),    // Emission
                        false, true);      // Vtx color, use shininess table
        }
        else
        {
            // Set some propierties to Material
            NE_MaterialSetPropierties(Material,
                        RGB15(31, 31, 31), // Diffuse
                        RGB15(0, 0, 0),    // Ambient
                        RGB15(0, 0, 0),    // Specular
                        RGB15(0, 0, 0),    // Emission
                        false, false);     // Vtx color, use shininess table
        }

        printf("\nMaterial type: %s", use_specular ? "Specular" : "Diffuse ");

        NE_Process(Draw3DScene);
        NE_WaitForVBL(0);
    }

    return 0;
}
