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

class ImagerUVC;
class BaseControlDevice;
class NewFrameBuffer;
class ImageProcessing;

namespace optris
{

typedef void (*fptrOptrisFrame)(unsigned short* data, unsigned int w, unsigned int h);
typedef void (*fptrOptrisVisibleFrame)(unsigned char* data, unsigned int w, unsigned int h);

class Timer;

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
   * Standard constructor (Empty -> see comment of init routine)
   */
  PIImager();

  /**
   * Destructor
   */
  ~PIImager();

  /**
   * Initializing routine, to be called if empty constructor was chosen to instantiate device
   * @param[in] v4lPath video4Linux device path
   * @param[in] serial Serial number (if set to 0, the device is tried to be automatically detected
   * @param[in] controller BaseControlInterface, i.e., HID or UVC
   * @param[in] fov Field of view of optics
   * @param[in] tMin Minimum temperature (cf. valid temperature ranges)
   * @param[in] tMax Maximum temperature (cf. valid temperature ranges)
   * @param[in] framerate Desired framerate (must be less or equal than the camera's framerate)
   * @param[in] mode Streaming output mode, i.e., energy data or temperature data
   * @param[in] bispectral 1, if bispectral technology is available (only PI200/PI230) and should be used, else 0
   */
  void init(const char* v4lPath, unsigned long serial, EnumControlInterface controller, int fov, int tMin, int tMax, float framerate, EnumOutputMode mode, int bispectral);

  /**
   * Check for opened device
   * @return device opened
   */
  bool isOpen();

  /**
   * Check existence of calibration file set
   * @param[in] serial Serial number to be checked
   * @return missing files as comma separated list
   */
  char* checkCalibration(unsigned long serial);

  /**
   * Get serial number of device
   * @return serial number
   */
  unsigned long getSerial();

  /**
   * Start UVC data streaming
   */
  bool startStreaming();

  /**
   * Get image width of thermal channel
   * @return Image width, i.e. number of columns
   */
  unsigned int getWidth();

  /**
   * Get image height of thermal channel
   * @return Image height, i.e. number of rows
   */
  unsigned int getHeight();

  /**
   * Get image width of visible channel (if available)
   * @return Image width, i.e. number of columns
   */
  unsigned int getVisibleWidth();

  /**
   * Get image height of visible channel (if available)
   * @return Image height, i.e. number of rows
   */
  unsigned int getVisibleHeight();

  /**
   * Get configured frame rate
   * return frame rate (in frames/second)
   */
  float getFramerate();

  /**
   * Get maximum frame rate of device
   * return frame rate (in frames/second)
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
   * Check if bispectral technology is available
   * @return bispectral technology flag
   */
  bool hasBispectralTechnology();

  /**
   * Get raw image (needs to be processed to obtain thermal data)
   * @param[out] buffer Output buffer (needs to be allocated outside having the size of getRawBufferSize())
   * @return success flag (==0)
   */
  int getFrame(unsigned char *buffer);

  /**
   * Get thermal image (Temperature can be calculated with ((float)val-1000.f)/10.f)
   * @param[out] Output buffer (needs to be allocated outside having the size of getWidth()*getHeight())
   * @return success flag (==1), else (==-1)
   */
  int acquire(unsigned short* buffer);

  //void yield();

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
   * Set callback function to be called for new frames
   * @param[in] callback Pointer to callback function for thermal channel
   */
  void setFrameCallback(fptrOptrisFrame callback);

  /**
   * Set callback function to be called for new frames
   * @param[in] callback Pointer to callback function for visible channel
   */
  void setVisibleFrameCallback(fptrOptrisVisibleFrame callback);

  /**
   * Release frame bound with getFrame of acquire method (needs to be called within the grabbing loop)
   */
  void releaseFrame();

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
   * Internal method not to be used from any application
   */
  void onFlagState(unsigned int flagstate);

  /**
   * Internal method not to be used from any application!
   */
  void onThermalFrameInit(unsigned int width, unsigned int height);

  /**
   * Internal method not to be used from any application!
   */
  void onThermalFrame(unsigned short* buffer);

  /**
   * Internal method not to be used from any application!
   */
  void onVisibleFrameInit(unsigned int width, unsigned int height);

  /**
   * Internal method not to be used from any application!
   */
  void onVisibleFrame(unsigned char* buffer);

  /**
   * Internal method to communicate with uvc device
   */
  int readControl(unsigned int id, int* value);

  /**
   * Internal method to communicate with uvc device
   */
  int writeControl(unsigned int id, int value);

private:

  bool _init;

  unsigned int _widthIn;

  unsigned int _heightIn;

  unsigned int _widthOut;

  unsigned int _heightOut;

  unsigned short* _buffer;

  unsigned int _widthOutVisible;

  unsigned int _heightOutVisible;

  unsigned char* _bufferVisible;

  fptrOptrisFrame _cbFrame;

  fptrOptrisVisibleFrame _cbVisibleFrame;

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

  int _bispectral;

  Timer* _t;

  ImagerUVC* _uvc;

  BaseControlDevice* _udev;

  unsigned int _flagstate;

  NewFrameBuffer* _SGBuffer;

  ImageProcessing* _ip;
};

}

#endif
