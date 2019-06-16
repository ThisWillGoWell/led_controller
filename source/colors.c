/*
 * colors.c
 *
 *  Created on: Jun 15, 2019
 *      Author: wggowell
 */

#ifndef COLORS_C_
#define COLORS_C_

#include "colors.h"


RgbColor HsvToRgb(HsvColor hsv)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (hsv.s == 0)
    {
        rgb.r = hsv.v;
        rgb.g = hsv.v;
        rgb.b = hsv.v;
        return rgb;
    }

    region = hsv.h / 43;
    remainder = (hsv.h - (region * 43)) * 6;

    p = (hsv.v * (255 - hsv.s)) >> 8;
    q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = hsv.v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = hsv.v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = hsv.v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = hsv.v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = hsv.v;
            break;
        default:
            rgb.r = hsv.v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * (rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}



#endif /* COLORS_C_ */
//
//
//
//uint8_t Black[3] = {0,0,0};
//uint8_t White[3] = {255,255,255};
//uint8_t Red[3] = {255,0,0};
//uint8_t Lime[3] = {0,255,0};
//uint8_t Blue[3] = {0,0,255};
//uint8_t Yellow[3] = {255,255,0};
//uint8_t Cyan_Aqua[3] = {0,255,255};
//uint8_t Magenta_Fuchsia[3] = {255,0,255};
//uint8_t Silver[3] = {192,192,192};
//uint8_t Gray[3] = {128,128,128};
//uint8_t Maroon[3] = {128,0,0};
//uint8_t Olive[3] = {128,128,0};
//uint8_t Green[3] = {0,128,0};
//uint8_t Purple[3] = {128,0,128};
//uint8_t Teal[3] = {0,128,128};
//uint8_t Navy[3] = {0,0,128};
//uint8_t maroon[3] = {128,0,0};
//uint8_t dark_red[3] = {139,0,0};
//uint8_t brown[3] = {165,42,42};
//uint8_t firebrick[3] = {178,34,34};
//uint8_t crimson[3] = {220,20,60};
//uint8_t red[3] = {255,0,0};
//uint8_t tomato[3] = {255,99,71};
//uint8_t coral[3] = {255,127,80};
//uint8_t indian_red[3] = {205,92,92};
//uint8_t light_coral[3] = {240,128,128};
//uint8_t dark_salmon[3] = {233,150,122};
//uint8_t salmon[3] = {250,128,114};
//uint8_t light_salmon[3] = {255,160,122};
//uint8_t orange_red[3] = {255,69,0};
//uint8_t dark_orange[3] = {255,140,0};
//uint8_t orange[3] = {255,165,0};
//uint8_t gold[3] = {255,215,0};
//uint8_t dark_golden_rod[3] = {184,134,11};
//uint8_t golden_rod[3] = {218,165,32};
//uint8_t pale_golden_rod[3] = {238,232,170};
//uint8_t dark_khaki[3] = {189,183,107};
//uint8_t khaki[3] = {240,230,140};
//uint8_t olive[3] = {128,128,0};
//uint8_t yellow[3] = {255,255,0};
//uint8_t yellow_green[3] = {154,205,50};
//uint8_t dark_olive_green[3] = {85,107,47};
//uint8_t olive_drab[3] = {107,142,35};
//uint8_t lawn_green[3] = {124,252,0};
//uint8_t chart_reuse[3] = {127,255,0};
//uint8_t green_yellow[3] = {173,255,47};
//uint8_t dark_green[3] = {0,100,0};
//uint8_t green[3] = {0,128,0};
//uint8_t forest_green[3] = {34,139,34};
//uint8_t lime[3] = {0,255,0};
//uint8_t lime_green[3] = {50,205,50};
//uint8_t light_green[3] = {144,238,144};
//uint8_t pale_green[3] = {152,251,152};
//uint8_t dark_sea_green[3] = {143,188,143};
//uint8_t medium_spring_green[3] = {0,250,154};
//uint8_t spring_green[3] = {0,255,127};
//uint8_t sea_green[3] = {46,139,87};
//uint8_t medium_aqua_marine[3] = {102,205,170};
//uint8_t medium_sea_green[3] = {60,179,113};
//uint8_t light_sea_green[3] = {32,178,170};
//uint8_t dark_slate_gray[3] = {47,79,79};
//uint8_t teal[3] = {0,128,128};
//uint8_t dark_cyan[3] = {0,139,139};
//uint8_t aqua[3] = {0,255,255};
//uint8_t cyan[3] = {0,255,255};
//uint8_t light_cyan[3] = {224,255,255};
//uint8_t dark_turquoise[3] = {0,206,209};
//uint8_t turquoise[3] = {64,224,208};
//uint8_t medium_turquoise[3] = {72,209,204};
//uint8_t pale_turquoise[3] = {175,238,238};
//uint8_t aqua_marine[3] = {127,255,212};
//uint8_t powder_blue[3] = {176,224,230};
//uint8_t cadet_blue[3] = {95,158,160};
//uint8_t steel_blue[3] = {70,130,180};
//uint8_t corn_flower_blue[3] = {100,149,237};
//uint8_t deep_sky_blue[3] = {0,191,255};
//uint8_t dodger_blue[3] = {30,144,255};
//uint8_t light_blue[3] = {173,216,230};
//uint8_t sky_blue[3] = {135,206,235};
//uint8_t light_sky_blue[3] = {135,206,250};
//uint8_t midnight_blue[3] = {25,25,112};
//uint8_t navy[3] = {0,0,128};
//uint8_t dark_blue[3] = {0,0,139};
//uint8_t medium_blue[3] = {0,0,205};
//uint8_t blue[3] = {0,0,255};
//uint8_t royal_blue[3] = {65,105,225};
//uint8_t blue_violet[3] = {138,43,226};
//uint8_t indigo[3] = {75,0,130};
//uint8_t dark_slate_blue[3] = {72,61,139};
//uint8_t slate_blue[3] = {106,90,205};
//uint8_t medium_slate_blue[3] = {123,104,238};
//uint8_t medium_purple[3] = {147,112,219};
//uint8_t dark_magenta[3] = {139,0,139};
//uint8_t dark_violet[3] = {148,0,211};
//uint8_t dark_orchid[3] = {153,50,204};
//uint8_t medium_orchid[3] = {186,85,211};
//uint8_t purple[3] = {128,0,128};
//uint8_t thistle[3] = {216,191,216};
//uint8_t plum[3] = {221,160,221};
//uint8_t violet[3] = {238,130,238};
//uint8_t magenta_fuchsia[3] = {255,0,255};
//uint8_t orchid[3] = {218,112,214};
//uint8_t medium_violet_red[3] = {199,21,133};
//uint8_t pale_violet_red[3] = {219,112,147};
//uint8_t deep_pink[3] = {255,20,147};
//uint8_t hot_pink[3] = {255,105,180};
//uint8_t light_pink[3] = {255,182,193};
//uint8_t pink[3] = {255,192,203};
//uint8_t antique_white[3] = {250,235,215};
//uint8_t beige[3] = {245,245,220};
//uint8_t bisque[3] = {255,228,196};
//uint8_t blanched_almond[3] = {255,235,205};
//uint8_t wheat[3] = {245,222,179};
//uint8_t corn_silk[3] = {255,248,220};
//uint8_t lemon_chiffon[3] = {255,250,205};
//uint8_t light_golden_rod_yellow[3] = {250,250,210};
//uint8_t light_yellow[3] = {255,255,224};
//uint8_t saddle_brown[3] = {139,69,19};
//uint8_t sienna[3] = {160,82,45};
//uint8_t chocolate[3] = {210,105,30};
//uint8_t peru[3] = {205,133,63};
//uint8_t sandy_brown[3] = {244,164,96};
//uint8_t burly_wood[3] = {222,184,135};
//uint8_t tan[3] = {210,180,140};
//uint8_t rosy_brown[3] = {188,143,143};
//uint8_t moccasin[3] = {255,228,181};
//uint8_t navajo_white[3] = {255,222,173};
//uint8_t peach_puff[3] = {255,218,185};
//uint8_t misty_rose[3] = {255,228,225};
//uint8_t lavender_blush[3] = {255,240,245};
//uint8_t linen[3] = {250,240,230};
//uint8_t old_lace[3] = {253,245,230};
//uint8_t papaya_whip[3] = {255,239,213};
//uint8_t sea_shell[3] = {255,245,238};
//uint8_t mint_cream[3] = {245,255,250};
//uint8_t slate_gray[3] = {112,128,144};
//uint8_t light_slate_gray[3] = {119,136,153};
//uint8_t light_steel_blue[3] = {176,196,222};
//uint8_t lavender[3] = {230,230,250};
//uint8_t floral_white[3] = {255,250,240};
//uint8_t alice_blue[3] = {240,248,255};
//uint8_t ghost_white[3] = {248,248,255};
//uint8_t honeydew[3] = {240,255,240};
//uint8_t ivory[3] = {255,255,240};
//uint8_t azure[3] = {240,255,255};
//uint8_t snow[3] = {255,250,250};
//uint8_t black[3] = {0,0,0};
//uint8_t dim_gray_dim_grey[3] = {105,105,105};
//uint8_t gray_grey[3] = {128,128,128};
//uint8_t dark_gray_dark_grey[3] = {169,169,169};
//uint8_t silver[3] = {192,192,192};
//uint8_t light_gray_light_grey[3] = {211,211,211};
//uint8_t gainsboro[3] = {220,220,220};
//uint8_t white_smoke[3] = {245,245,245};
//uint8_t white[3] = {255,255,255};
//

