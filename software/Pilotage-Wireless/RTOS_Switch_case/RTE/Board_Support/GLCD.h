/******************************************************************************/
/* GLCD.h: Graphic LCD function prototypes and defines                        */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2011 Keil - An ARM Company. All rights reserved.        */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#ifndef _GLCD_H
#define _GLCD_H

/*------------------------------------------------------------------------------
  Color coding
  GLCD is coded:   15..11 red, 10..5 green, 4..0 blue  (unsigned short)  GLCD_R5, GLCD_G6, GLCD_B5   
  original coding: 17..12 red, 11..6 green, 5..0 blue                    ORG_R6,  ORG_G6,  ORG_B6

  ORG_R1..5 = GLCD_R0..4,  ORG_R0 = GLCD_R4
  ORG_G0..5 = GLCD_G0..5,
  ORG_B1..5 = GLCD_B0..4,  ORG_B0 = GLCD_B4
 *----------------------------------------------------------------------------*/
                            
/* GLCD RGB color definitions                                                 */
#define Black           0x0000      /*   0,   0,   0 */
#define Navy            0x000F      /*   0,   0, 128 */
#define DarkGreen       0x03E0      /*   0, 128,   0 */
#define DarkCyan        0x03EF      /*   0, 128, 128 */
#define Maroon          0x7800      /* 128,   0,   0 */
#define Purple          0x780F      /* 128,   0, 128 */
#define Olive           0x7BE0      /* 128, 128,   0 */
#define LightGrey       0xC618      /* 192, 192, 192 */
#define DarkGrey        0x7BEF      /* 128, 128, 128 */
#define Blue            0x001F      /*   0,   0, 255 */
#define Green           0x07E0      /*   0, 255,   0 */
#define Cyan            0x07FF      /*   0, 255, 255 */
#define Red             0xF800      /* 255,   0,   0 */
#define Magenta         0xF81F      /* 255,   0, 255 */
#define Yellow          0xFFE0      /* 255, 255, 0   */
#define White           0xFFFF      /* 255, 255, 255 */

extern void GLCD_Init           (void);
extern void GLCD_WindowMax      (void);
extern void GLCD_PutPixel       (unsigned int x, unsigned int y);
extern void GLCD_SetTextColor   (unsigned short color);
extern void GLCD_SetBackColor   (unsigned short color);
extern void GLCD_Clear          (unsigned short color);
extern void GLCD_DrawChar       (unsigned int x,  unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c);
extern void GLCD_DisplayChar    (unsigned int ln, unsigned int col, unsigned char fi, unsigned char  c);
extern void GLCD_DisplayString  (unsigned int ln, unsigned int col, unsigned char fi, unsigned char *s);
extern void GLCD_ClearLn        (unsigned int ln, unsigned char fi);
extern void GLCD_Bargraph       (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned int val);
extern void GLCD_BargraphBvH    (unsigned int x, unsigned int w, unsigned int h);
extern void GLCD_BargraphHvB 	(unsigned int x, unsigned int w, unsigned int h);
extern void GLCD_Bitmap         (unsigned int x,  unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap);
extern void GLCD_ScrollVertical (unsigned int dy);

extern void GLCD_WrCmd          (unsigned char cmd);
extern void GLCD_WrReg          (unsigned char reg, unsigned short val); 

// Fonctions de dessin sur ecran LCD
extern void Pixel_c				(int x0, int y0, short couleur);
extern void Ligne				(int x0, int y0, int x1, int y1);
extern void Ligne_c				(int x0, int y0, int x1, int y1, short couleur);
extern void Ligne_horiz			(int x0, int x1, int y);
extern void Ligne_verti			(int x, int y0, int y1);
extern void Cercle_vide			(int x0, int y0, int rayon);
extern void Cercle_vide_c		(int x0, int y0, int rayon, short couleur);
extern void Cercle_plein		(int x0, int y0, int rayon);
extern void Cercle_plein_c		(int x0, int y0, int rayon, short couleur);
extern void Ellipse_vide		(int x0, int y0, int rayon_x, int rayon_y);
extern void Ellipse_vide_c		(int x0, int y0, int rayon_x, int rayon_y, short couleur);
extern void Ellipse_plein		(int x0, int y0, int rayon_x, int rayon_y);
extern void Ellipse_plein_c		(int x0, int y0, int rayon_x, int rayon_y, short couleur);
extern void Rectangle_vide		(int x0,int y0, int cote_x, int cote_y);
extern void Rectangle_vide_c	(int x0,int y0, int cote_x, int cote_y, short couleur);
extern void Rectangle_plein		(int x0,int y0, int cote_x, int cote_y);
extern void Rectangle_plein_c	(int x0,int y0, int cote_x, int cote_y, short couleur);

// Fonctions Affichage code Poste INEA S3
extern void AfficheLCD_code		(short sortie);

#endif /* _GLCD_H */
