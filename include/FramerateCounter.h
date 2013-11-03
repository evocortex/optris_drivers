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

#ifndef FRAMERATECOUNTER_H_
#define FRAMERATECOUNTER_H_

#include <iostream>
#include <list>

namespace optris
{

class Timer;

/**
 * Framerate calculation helper
 * @author Stefan May (Nuremberg Institute of Technology Georg Simon Ohm)
 */
class FramerateCounter
{
public:
  /**
   * Standard constructor
   * @param[in] queue Queue length to average
   */
  FramerateCounter(unsigned int queue=10);

  /**
   * Standard destructor
   */
  virtual ~FramerateCounter();

  /**
   * Trigger, i.e., increase counter
   */
  void trigger();

  /**
   * Calculate mean value of queue
   * @return average frame rate of last trigger events
   */
  double getMeanValue();

  /**
   * Print frame rate at reduced time interval
   * @param[in] interval time interval
   * @param[in] output stream
   */
  void print(double interval, std::ostream& stream);

private:

  Timer* _tElapsed;

  Timer* _tPrint;

  unsigned int _queue;

  std::list<double> _fpsList;

  bool _isDirty;
};

}

#endif /* FRAMERATECOUNTER_H_ */
