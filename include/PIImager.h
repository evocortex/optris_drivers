#ifndef PIIMAGER_H
#define PIIMAGER_H

enum EnumControlInterface {HIDController=1, UVCController=2};

namespace optris
{

typedef void (*fptrOptrisFrame)(unsigned short* data, unsigned int w, unsigned int h);

/**
 * @class PIImager
 * @brief Wrapper for optris driver and image processing library
 * @author Stefan May
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
   * @param[in] sim Simulation mode (if true, v4lPath is interpreted as the path to a previously recorded uvc trace file)
   */
  PIImager(const char* v4lPath, unsigned long serial, EnumControlInterface controller, int fov, int tMin, int tMax, float framerate, bool sim = false);

  /**
   * Destructor
   */
  ~PIImager();

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
   * @return success flag (==0)
   */
  int getMetaData(unsigned short buffer[64]);

  /**
   * Get temperature from last acquired image at specified image index
   * @param[in] index Image index (must be within [0; getWidth()*getHeight()])
   * return temperature in degree Celsius
   */
  float getTemperatureAt(int index);

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
   * Start recording to specified file (Warning: Files are growing rapidly in size)
   * @param path File path
   */
  void startRecording(const char* path);

  /**
   * Stop recording to specified file
   */
  void stopRecording();

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
   * Internal method not to be used from any application!
   */
  void onFrameInit(unsigned int width, unsigned int height);

  /**
   * Internal method not to be used from any application!
   */
  void onFrame(unsigned short* buffer);

private:

  void init();

  unsigned int _widthIn;

  unsigned int _heightIn;

  unsigned int _widthOut;

  unsigned int _heightOut;

  unsigned short* _buffer;

  fptrOptrisFrame _cbFrame;

  int _fov;

  int _tMin;

  int _tMax;

  float _framerate;

  float _maxFramerate;

  unsigned long _serial;

  const char* _v4lPath;

  EnumControlInterface _controller;

  bool _autoFlag;

  int _sim;
};

}

#endif
