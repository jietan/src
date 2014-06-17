#ifndef _MULTI_LAYER_MASK_IMAGE_H
#define _MULTI_LAYER_MASK_IMAGE_H

#include "MultilayerImage.h"

class MultilayerMaskImage : public MultilayerImage < int >
{
public:
	virtual ~MultilayerMaskImage();
	virtual void Read(const string& filename);
	virtual void Save(const string& filename);
};

#endif