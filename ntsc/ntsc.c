#include <stdlib.h>
#include "frei0r.h"
#include "crt_core.h"


typedef struct ntsc_instance
{
    // image dimensions
    int width;
    int height;

    // parameters
    struct CRT crt;

    int noise;
    char progressive;
    
    unsigned int frame_count;

} ntsc_instance_t;


// these functions are for frei0r
// mostly copy/paste/slightly modified from the other frei0r effects
int f0r_init()
{
    return 1;
}

void f0r_deinit()
{}

void f0r_get_plugin_info(f0r_plugin_info_t* info)
{
    info->name = "NTSC-CRT";
    info->author = "EMMIR";
    info->explanation = "Simulates NTSC analog video.";
    info->plugin_type = F0R_PLUGIN_TYPE_FILTER;
    info->color_model = F0R_COLOR_MODEL_RGBA8888;
    info->frei0r_version = FREI0R_MAJOR_VERSION;
    info->major_version = 0;
    info->minor_version = 1;
    info->num_params = 3;
}

void f0r_get_param_info(f0r_param_info_t* info, int param_index)
{
    switch(param_index)
    {
    case 0:
        info->name = "Noise";
        info->explanation = "The amount of signal noise.";
        info->type = F0R_PARAM_DOUBLE;
        break;
        
    case 1:
        info->name = "VHS";
        info->explanation = "Toggles VHS emulation.";
        info->type = F0R_PARAM_BOOL;
        break;
        
    case 2:
        info->name = "Progressive";
        info->explanation = "Toggles progressive scan.";
        info->type = F0R_PARAM_BOOL;
        break;
    }
}

f0r_instance_t f0r_construct(unsigned int width, unsigned int height)
{
    ntsc_instance_t* inst = (ntsc_instance_t*)calloc(1, sizeof(*inst));

    inst->width = width;
    inst->height = height;
    
    crt_init(&(inst->crt), width, height);

    inst->crt.do_vhs = 0;
    
    inst->noise = 0;
    inst->progressive = 1;
    
    inst->frame_count = 0;

    return (f0r_instance_t)inst;
}

void f0r_destruct(f0r_instance_t instance)
{
    ntsc_instance_t* inst = (ntsc_instance_t*)instance;
    free(instance);
}


void f0r_set_param_value(f0r_instance_t instance, f0r_param_t param, int param_index)
{
    ntsc_instance_t* inst = (ntsc_instance_t*)instance;
    switch(param_index)
    {
    case 0:
        inst->noise = *((double*)param) * 100;
        break;
    case 1:
        inst->crt.do_vhs = (*((double*)param) >= 0.5);
        break;
    case 2:
        inst->progressive = (*((double*)param) >= 0.5);
        break;
    }
}

void f0r_get_param_value(f0r_instance_t instance, f0r_param_t param, int param_index)
{
    ntsc_instance_t* inst = (ntsc_instance_t*)instance;
    switch(param_index)
    {
    case 0:
        *((double*)param) = (inst->noise / 100);
        break;
    case 1:
        *((double*)param) = (inst->crt.do_vhs ? 1.0 : 0.0);
        break;
    case 2:
        *((double*)param) = (inst->progressive ? 1.0 : 0.0);
        break;
    }
}


void f0r_update(f0r_instance_t instance, double time, const uint32_t* inframe, uint32_t* outframe)
{
    ntsc_instance_t* inst = (ntsc_instance_t*)instance;
    
    inst->crt.out = (char*)outframe;    
    inst->crt.data = (const char*)inframe;
    
    crt_modulate(&(inst->crt));
    crt_demodulate(&(inst->crt), inst->noise);
    
    if(!inst->progressive)
    {
        inst->crt.field ^= 1;
        if((inst->frame_count & 1) == 0)
        {
            inst->crt.frame ^= 1;
        }
    }
    inst->frame_count++;
}