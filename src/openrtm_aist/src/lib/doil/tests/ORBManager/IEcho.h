#ifndef IECHO_H
#define IECHO_H
#include <doil/ImplBase.h>
#include <string>
#include <iostream>
//------------------------------------------------------------
// Implementation class
//------------------------------------------------------------
class IEcho
{
public:
  virtual ~IEcho(){};
  virtual void echo(std::string msg) = 0;
};

#endif // IECHO_H
