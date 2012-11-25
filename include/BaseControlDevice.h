#ifndef BASECONTROLDEVICE_H
#define BASECONTROLDEVICE_H

enum TFlagState {fsFlagOpen, fsFlagClose, fsFlagOpening, fsFlagClosing, fsError};
enum TPifMode	{ pifmOff, pifmFlag, pifmRecording, pifmSnapshot, pifmLinescanner, pifmUser=0xffff };

/**
 * @class BaseControlDevice
 * @brief Generic control interface to PI imagers
 */
class BaseControlDevice
{
public:

  BaseControlDevice();

  virtual ~BaseControlDevice();

  virtual int Init(unsigned long vid, unsigned long pid, bool ForceInitHID) = 0;

  virtual int Init(unsigned long vid, unsigned long pid, unsigned long serno, bool ForceInitHID, bool *SameDevice) = 0;

  virtual unsigned long InitPif(void) = 0;

  virtual int GetBuffer(unsigned char *buffer, int len) = 0;

  virtual unsigned long GetSerialNumber(void) = 0;

  virtual unsigned short GetFirmwareRev(void) = 0;

  virtual unsigned short GetHardwareRev(void) = 0;

  virtual void GetPifIn(unsigned short *pVoltage) = 0;

  virtual void SetPifOut(unsigned short Voltage) = 0;

  virtual void SetTecEnable(bool on) = 0;

  virtual void GetTecEnable(bool *pVal) = 0;

  virtual void SetTempTec(float Temp) = 0;

  virtual void GetTempTec(float *pVal) = 0;

  virtual void GetFlag(TFlagState *pVal) = 0;

  virtual void SetFlag(TFlagState Val) = 0;

  virtual void SetFlagCycle(unsigned short CycleCount) = 0;

  virtual void SetTecA(unsigned short Val) = 0;

  virtual void GetTecA(unsigned short *pVal) = 0;

  virtual void SetTecB(unsigned short Val) = 0;

  virtual void GetTecB(unsigned short *pVal) = 0;

  virtual void SetTecC(unsigned short Val) = 0;

  virtual void GetTecC(unsigned short *pVal) = 0;

  virtual void SetTecD(unsigned short Val) = 0;

  virtual void GetTecD(unsigned short *pVal) = 0;

  virtual void SetSkim(unsigned short voltage) = 0;

  virtual void SetSkim_WaitForFlag(unsigned short voltage) = 0;

  virtual void GetSkim(unsigned short *pVal) = 0;

  virtual void SetSkim_Adjust(unsigned short voltage) = 0;

  virtual void GetSkim_Adjust(unsigned short *pVal) = 0;

  virtual void SetFid(unsigned short voltage) = 0;

  virtual void SetFid_WaitForFlag(unsigned short voltage) = 0;

  virtual void GetFid(unsigned short *pVal) = 0;

  virtual void SetFid_Adjust(unsigned short voltage) = 0;

  virtual void GetFid_Adjust(unsigned short *pVal) = 0;

  virtual void SetBiasEnable(bool on) = 0;

  virtual void GetBiasEnable(bool *pVal) = 0;

  virtual void SetPowerEnable(bool on) = 0;

  virtual void GetPowerEnable(bool *on) = 0;

  virtual void GetAntiFlicker(bool *on) = 0;

  virtual void SetAntiFlicker(bool on) = 0;

  virtual bool GetShortImagerCaps(void) = 0;

  virtual TPifMode GetPIFInMode(void) = 0;

  virtual void SetPIFInMode(TPifMode Val) = 0;

  virtual TPifMode GetPIFInDigitalMode(void) = 0;

  virtual void SetPIFInDigitalMode(TPifMode Val) = 0;

  virtual void GetPIFOutMode(TPifMode *pVal) = 0;

  virtual void SetPIFOutMode(TPifMode Val) = 0;

  virtual unsigned short GetPIFInFlagThreshold(void) = 0;

  virtual void SetPIFInFlagThreshold(unsigned short val) = 0;

  virtual bool GetPIFInFlagOpenIfLower(void) = 0;

  virtual void SetPIFInFlagOpenIfLower(bool val) = 0;

  virtual void SetPIFOutFlagOpen(unsigned short value) = 0;

  virtual void GetPIFOutFlagOpen(unsigned short *pVal) = 0;

  virtual void SetPIFOutFlagClosed(unsigned short value) = 0;

  virtual void GetPIFOutFlagClosed(unsigned short *pVal) = 0;

  virtual void SetPIFOutFlagMoving(unsigned short value) = 0;

  virtual void GetPIFOutFlagMoving(unsigned short *pVal) = 0;

  virtual bool GetPIFInDigitalFlagLowActive(void) = 0;

  virtual void SetPIFInDigitalFlagLowActive(bool val) = 0;

  virtual unsigned short GetPIFInThreshold(void) = 0;

  virtual void SetPIFInThreshold(unsigned short val) = 0;

  virtual bool GetPIFInOpenIfLower(void) = 0;

  virtual void SetPIFInOpenIfLower(bool val) = 0;

  virtual bool GetPIFInDigitalLowActive(void) = 0;

  virtual void SetPIFInDigitalLowActive(bool val) = 0;

  virtual void GetTempChip(float *pVal) = 0;

  virtual void GetTempBox(float *pVal) = 0;

  virtual void GetTempFlag(float *pVal) = 0;

  void SetTempTecMax(float val);

  float GetTempTecMax();

  void SetTempTecMin(float val);

  float GetTempTecMin();

  void SetTempBoxOffset(float val);

  float GetTempBoxOffset();

  void SetTempChipFactor(float val);

  float GetTempChipFactor();

  void SetTempChipOffset(float val);

  float GetTempChipOffset();

  void SetTempFlagOffset(float val);

  float GetTempFlagOffset();

  void SetTempTecOffset(float val);

  float GetTempTecOffset();

  void SetTempTecGain(float val);

  float GetTempTecGain();

  void SetPifInOffset(float val);

  float GetPifInOffset();

  void SetPifInGain(float val);

  float GetPifInGain();

  void SetPifOutOffset(float val);

  float GetPifOutOffset();

  void SetPifOutGain(float val);

  float GetPifOutGain();

protected:

  float                   TempTecMax;

  float                   TempTecMin;

  float                   TempBoxOffset;

  float                   TempChipFactor;

  float                   TempChipOffset;

  float                   TempFlagOffset;

  float                   TempTecOffset;

  float                   TempTecGain;

  float                   PifInOffset;

  float                   PifInGain;

  float                   PifOutOffset;

  float                   PifOutGain;
};

#endif
