#ifndef OPTRISIMAGEBUILDER_H
#define OPTRISIMAGEBUILDER_H

namespace optris
{

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
 * @author Stefan May
 */
class ImageBuilder
{
public:
  /**
   * Standard constructor
   */
	ImageBuilder();

  /**
   * Standard destructor
   */
  ~ImageBuilder();

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
   * Set the size of Imager-Matrix
   * @param[in] width Image width
   * @param[in] height Image height
   * @param[in] alginStride activate memory alignment (multiple of 4)
   */
  void setSize(unsigned int width, unsigned int height, bool alignStride=true);

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
   * Image conversion to rgb
   * @param[in] src Source image (short-Format)
   * @param[out] dst Destination image
   */
  void convertTemperatureToPaletteImage(unsigned short* src, unsigned char* dst);

  /**
   * calculate histogram
   * @param[in] src source data, i.e. temperature data
   * @param[out] hist histogram
   * @param[in] histsize number of quantization steps
   * @param[in] tMin minimum temperature
   * @param[in] tMax maximum temperature
   */
  void calcHistogram(unsigned short* src, unsigned int* hist, unsigned int histsize, int tMin, int tMax);

  /**
   * Draw crosshair to the center of image
   * @param[in/out] img Image in RGB or RGBA format
   * @param[in] x x-position in image
   * @param[out] y y-position in image
   */
  void drawCrosshair(unsigned char* img, unsigned int x, unsigned int y, unsigned channels=3);

private:

  /**
   * Calculate scaling boundaries based on minimal and maximal temperature
   * @param[in] src Thermal image
   */
  void calcMinMaxScalingFactor(unsigned short* src);

  /**
   * Calculate scaling boundaries based on standard deviation
   * @param[in] src Thermal image
   * @param[in] sigma Multiplication factor for sigma range
   */
  void calcSigmaScalingFactor(unsigned short* src, float sigma);

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

};

}

#endif //OPTRISIMAGEBUILDER_H
