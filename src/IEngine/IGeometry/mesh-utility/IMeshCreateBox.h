#ifndef IMESHCREATEBOX_H
#define IMESHCREATEBOX_H

#include "IMeshModel.h"

namespace IGeometry
{

class IMeshCreateBox : public IMeshModel
{
  public:

     IMeshCreateBox(const IVector3& halfSize);

  protected:

     void init(const IVector3& halfSize);


};


}

#endif // IMESHCREATEBOX_H
