/******************************************************************************
 * Copyright (c) 2012, 2013 All Rights Reserved, http://www.optris.de         *                                                                          *
 *  Optris GmbH                                                               *
 *  Ferdinand-Buisson-Str. 14                                                 *
 *  13127 Berlin                                                              *
 *  Germany                                                                   *
 *                                                                            *
 * Contributors:                                                              *
 * - Linux platform development in cooperation with Nuremberg Institute of    *
 *   Technology Georg Simon Ohm, http//www.th-nuernberg.de                    *
 * - Linux 64-Bit platform supported by Fraunhofer IPA,                       *
 *   http://www.ipa.fraunhofer.de                                             *
 *****************************************************************************/

#ifndef OPTRISIMAGEBUILDER_H
#define OPTRISIMAGEBUILDER_H

namespace optris
{

struct ExtremalRegion
{
  float t;
  int u1;
  int v1;
  int u2;
  int v2;
};

enum EnumOptrisColoringPalette{eAlarmBlue   = 1,
							   eAlarmBlueHi = 2,
							   eGrayBW      = 3,
							   eGrayWB      = 4,
							   eAlarmGreen  = 5,
							   eIron        = 6,
							   eIronHi      = 7,
							   eMedical     = 8,
							   eRainbow     = 9,
							   eRainbowHi   = 10,
							   eAlarmRed    = 11};

enum EnumOptrisPaletteScalingMethod{eManual = 1,
									eMinMax = 2,
									eSigma1 = 3,
									eSigma3 = 4};

typedef unsigned char (*paletteTable)[3];

/**
 * Image creation module for displaying purposes
 * @author Stefan May (Nuremberg Institute of Technology Georg Simon Ohm), Matthias Wiedemann (Optris GmbH)
 */
class ImageBuilder
{
public:
  /**
   * Standard constructor
   */
	ImageBuilder(bool alignStride=true);

  /**
   * Standard destructor
   */
  ~ImageBuilder();

  /**
   * Set new data
   * @param[in] width image width
   * @param[in] height image height
   * @param[in] data image data
   */
  void setData(unsigned int width, unsigned int height, unsigned short* data);

  /**
   * Get image width
   * @return image width
   */
  unsigned int getWidth();

  /**
   * Get image height
   * @return image height
   */
  unsigned int getHeight();

  /**
   * Get temperature from last acquired image at specified image index
   * @param[in] index Image index (must be within [0; getWidth()*getHeight()])
   * return temperature in degree Celsius
   */
  float getTemperatureAt(int index);

  /**
   * Get temperature from last acquired image at specified image coordinates
   * @param[in] u Image column (must be within [0; getWidth()])
   * @param[in] v Image row (must be within [0; getHeight()])
   * return temperature in degree Celsius
   */
  float getTemperatureAt(int u, int v);

  /**
   * Get mean temperature of rectangular measuring field
   * @param[in] u1 u-component of image coordinate, i. e. column of 1st point
   * @param[in] v1 v-component of image coordinate, i. e. row of 1st point
   * @param[in] u2 u-component of image coordinate, i. e. column of 2nd point
   * @param[in] v2 v-component of image coordinate, i. e. row of 2nd point
   * @return mean temperature
   */
  float getMeanTemperature(int u1, int v1, int u2, int v2);

  /**
   * Get region of minimum/maximum temperature with given radius
   * @param[in] radius Radius of region
   * @param[out] minRegion Region of minimum mean temperature
   * @param[out] maxRegion Region of maximum mean temperature
   */
  void getMinMaxRegion(int radius, ExtremalRegion* minRegion, ExtremalRegion* maxRegion);

  /**
   * Set temperature range for manual scaling method
   * @param[in] min Lower limit in °C
   * @param[in] max Upper limit in °C
   */
  void setManualTemperatureRange(float min, float max);

  /**
   * Get minimum temperature used to scale image
   * @return Temperature in °C
   */
  float getIsothermalMin();

  /**
   * Get maximum temperature used to scale image
   * @return Temperature in °C
   */
  float getIsothermalMax();

  /**
   * Scaling method of color conversion
   * @param[in] method Scaling method
   */
  void setPaletteScalingMethod(EnumOptrisPaletteScalingMethod method);

  /**
   * Accessor for activated color conversion mode
   * @return Scaling method
   */
  EnumOptrisPaletteScalingMethod getPaletteScalingMethod();

  /**
   * If memory alignment is needed, this class provides a stride parameter configured with setSize(...).
   * Memory will be aligned such that the image width is a multiple of 4
   */
  unsigned int getStride(void);

  /**
   * Set palette for color conversion
   * @param[in] palette coloring palette
   */
  void setPalette(EnumOptrisColoringPalette palette);

  /**
   * Get palette for color conversion
   * return coloring palette
   */
  EnumOptrisColoringPalette getPalette();

  /**
   * Get palette table for color conversion
   * @return coloring palette table as rgb triple
   */
  void getPaletteTable(paletteTable& table);

  /**
   * Fill lookup table for false color conversion.
   * @param[out] lut lookup table
   */
  void fillPaletteLookup(unsigned int lut[65536]);

  /**
   * Image conversion to rgb
   * @param[out] dst Destination image
   * @param[in] by default columns are multiple of 4, i.e., stride is added
   */
  void convertTemperatureToPaletteImage(unsigned char* dst, bool ignoreStride=false);

  /**
   * Image conversion to rgb with lookup table. This method is efficient, but works only with fixed temperature ranges (manual mode).
   * @param[in] lut lookup table
   * @param[out] dst Destination image
   */
  void convertTemperatureToPaletteImage(unsigned int lut[65536], unsigned char* dst);

  /**
   * calculate histogram
   * @param[out] hist histogram
   * @param[in] histsize number of quantization steps
   * @param[in] tMin minimum temperature
   * @param[in] tMax maximum temperature
   */
  void calcHistogram(unsigned int* hist, unsigned int histsize, int tMin, int tMax);

  /**
   * Draw crosshair to the center of image
   * @param[in/out] img image in RGB or RGBA format
   * @param[in] x x-position in image
   * @param[out] y y-position in image
   */
  void drawCrosshair(unsigned char* img, unsigned int x, unsigned int y, unsigned channels=3);

  /**
   * Convert YUV422 image to RGB format (8-Bit per channel)
   * @param[in] src source image
   * @param[out] dst destination image
   * @param w image width
   * @param h image height
   */
  void yuv422torgb24(const unsigned char* src, unsigned char* dst, unsigned int w, unsigned int h);

private:

  void calculateIntegralImage();

  /**
   * Calculate scaling boundaries based on minimal and maximal temperature
   */
  void calcMinMaxScalingFactor();

  /**
   * Calculate scaling boundaries based on standard deviation
   * @param[in] sigma Multiplication factor for sigma range
   */
  void calcSigmaScalingFactor(float sigma);

  /**
   * Image data
   */
  unsigned short* _data;

  /**
   * Integral image
   */
  unsigned long* _integral;

  /**
   * Variable indicating the temperature range scaling method
   */
  EnumOptrisPaletteScalingMethod _scalingMethod;

  /**
   * Minimum temperature of display range
   */
  unsigned short _min;

  /**
   * Maximum temperature of display range
   */
  unsigned short _max;

  /**
   * Real width of Imager-Matrix
   */
  unsigned int _width;

  /**
   * Real height of Imager-Matrix
   */
  unsigned int _height;

  /**
   * Width including additional pixels in every row of Imager-Matrix to fit the modulo-4-criteria
   */
  unsigned int  _stride;

  /**
   * Real size if Imager-Matrix
   */
  unsigned int _size;

  /**
   * Coloring palette for conversion method
   */
  EnumOptrisColoringPalette _palette;

  /**
   * Memory alignment
   */
  bool _alignStride;

  /**
   * Flag for recomputation of integral image
   */
  bool _integralIsDirty;
};

}

#endif //OPTRISIMAGEBUILDER_H
