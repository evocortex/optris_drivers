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

#ifndef PIIMAGER_H
#define PIIMAGER_H

enum EnumControlInterface {HIDController=1, UVCController=2};
enum EnumOutputMode {Energy=1, Temperature=2};

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

typedef void (*fptrOptrisFrame)(unsigned short* data, unsigned int w, unsigned int h);

/**
 * @class PIImager
 * @brief Wrapper for optris driver and image processing library
 * @author Stefan May (Nuremberg Institute of Technology Georg Simon Ohm), Matthias Wiedemann (Optris GmbH)
 */
class PIImager
{
public:

  /**
   * Standard constructor
   * @param[in] path Path to XML-configuration file
   */
  PIImager(const char* xmlConfig);

  /**
   * Standard constructor
   * @param[in] v4lPath video4Linux device path
   * @param[in] serial Serial number (if set to 0, the device is tried to be automatically detected
   * @param[in] controller BaseControlInterface, i.e., HID or UVC
   * @param[in] fov Field of view of optics
   * @param[in] tMin Minimum temperature (cf. valid temperature ranges)
   * @param[in] tMax Maximum temperature (cf. valid temperature ranges)
   * @param[in] framerate Desired framerate (must be less or equal than the camera's framerate)
   * @param[in] mode Streaming output mode, i.e., energy data or temperature data
   */
  PIImager(const char* v4lPath, unsigned long serial, EnumControlInterface controller, int fov, int tMin, int tMax, float framerate, EnumOutputMode mode = Temperature);

  /**
   * Destructor
   */
  ~PIImager();

  /**
   * Get serial number of device
   * @return serial number
   */
  unsigned long getSerial();

  /**
   * Start UVC data streaming
   */
  void startStreaming();

  /**
   * Get image width
   * @return Image width, i.e. number of columns
   */
  unsigned int getWidth();

  /**
   * Get image height
   * @return Image height, i.e. number of rows
   */
  unsigned int getHeight();

  /**
   * Get configured frame rate
   * return framerate (in frames/second)
   */
  float getFramerate();

  /**
   * Get maximum frame rate of device
   * return framerate (in frames/second)
   */
  float getMaxFramerate();
  /**
   * Get raw image size (includes meta data)
   * @return Number of bytes
   */
  unsigned int getRawBufferSize();

  /**
   * Get lower limit of temperature range
   * @return lower limit
   */
  int getTemperatureRangeMin();

  /**
   * Get upper limit of temperature range
   * @return upper limit
   */
  int getTemperatureRangeMax();

  /**
   * Get raw image (needs to be processed to get thermal data)
   * @param[out] buffer Output buffer (needs to be allocated outside having the size of getRawBufferSize())
   * @return success flag (==0)
   */
  int getFrame(unsigned char *buffer);

  /**
   * Get thermal image (Temperature can be calculated with ((float)val-1000.f)/10.f)
   * @param[out] Output buffer (needs to be allocated outside having the size of getWidth()*getHeight())
   * @return success flag (==0)
   */
  int acquire(unsigned short* buffer);

  /**
   * Get energy buffer of previously acquired frame
   * @param[out] Output buffer (needs to be allocated outside having the size of getWidth()*getHeight())
   * @return success flag (==0)
   */
  int getEnergyBuffer(unsigned short* buffer);

  /**
   * Get meta data container of previously acquired frame
   * @param[out] Output buffer
   * @param[in] size Size of buffer in bytes
   * @return number of copied bytes
   */
  int getMetaData(unsigned char* buffer, int size);

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
   * Get mean temperature of rectangluar measuring field
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
   * Set callback function to be called for new frames
   * @param[in] callback Pointer to callback function
   */
  void setFrameCallback(fptrOptrisFrame callback);

  /**
   * Release frame bound with getFrame of acquire method (needs to be called within the grabbing loop)
   */
  void releaseFrame();

  /**
   * Get framrate calculated over the last 10 frames
   * @return Framerate in frames/second
   */
  float getAverageFramerate();

  /**
   * Process raw data
   * @param[in] buffer Raw data acquired with getFrame()
   */
  void process(unsigned char* buffer);

  /**
   * Set automatic flag activation state. Disabling will prevent camera from getting freezed frequently for several frames.
   * But temperature data might deviate too much.
   * @param[in] flag Autmatic flag activation state
   */
  void setAutoFlag(bool flag);

  /**
   * Access automatic flag activation state
   * @return automatic flag
   */
  bool getAutoFlag();

  /**
   * Force shutter flag event manually (close/open cycle)
   */
  void forceFlagEvent();

  /**
   * Check if shutter flag is open
   * @return flag open
   */
  bool isFlagOpen();

  /**
   * Get temperature of shutter flag
   * @return temperature
   */
  float getTempFlag();

  /**
   * Get temperature of housing
   * @return temperature
   */
  float getTempBox();

  /**
   * Get temperature of chip
   * @return temperature
   */
  float getTempChip();

  /**
   * Internal method not to be used from any application!
   */
  void onFrameInit(unsigned int width, unsigned int height);

  /**
   * Internal method not to be used from any application!
   */
  void onFrame(unsigned short* buffer);

  int readControl(unsigned int id, int* value);

  int writeControl(unsigned int id, int value);

private:

  void init();

  void calculateIntegralImage();

  unsigned int _widthIn;

  unsigned int _heightIn;

  unsigned int _widthOut;

  unsigned int _heightOut;

  unsigned short* _buffer;

  unsigned long* _integral;

  bool _integralIsDirty;

  fptrOptrisFrame _cbFrame;

  int _fov;

  int _tMin;

  int _tMax;

  float _framerate;

  float _maxFramerate;

  int _outputmode;

  unsigned long _serial;

  const char* _v4lPath;

  EnumControlInterface _controller;

  bool _autoFlag;

  bool _manualFlag;

  float _tBox;

  float _tChip;

  float _tFlag;
};

}

#endif
