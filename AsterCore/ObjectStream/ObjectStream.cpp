// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/ObjectStream/ObjectStream.h>

#ifdef ACPH_OBJECT_STREAM

ACPH_NAMESPACE_BEGIN

// Define macro to declare functions for a specific primitive type
#define ACPH_DECLARE_PRIMITIVE(name)																\
	bool	OSIsType(name *, int inArrayDepth, EOSDataType inDataType, const char *inClassName) \
	{																							\
		return inArrayDepth == 0 && inDataType == EOSDataType::T_##name;						\
	}																							\
	bool	OSReadData(IObjectStreamIn &ioStream, name &outPrimitive)							\
	{																							\
		return ioStream.ReadPrimitiveData(outPrimitive);										\
	}																							\
	void	OSWriteDataType(IObjectStreamOut &ioStream, name *)									\
	{																							\
		ioStream.WriteDataType(EOSDataType::T_##name);											\
	}																							\
	void	OSWriteData(IObjectStreamOut &ioStream, const name &inPrimitive)					\
	{																							\
		ioStream.HintNextItem();																\
		ioStream.WritePrimitiveData(inPrimitive);												\
	}

// This file uses the ACPH_DECLARE_PRIMITIVE macro to define all types
#include <AsterCore/ObjectStream/ObjectStreamTypes.h>

ACPH_NAMESPACE_END

#endif // ACPH_OBJECT_STREAM
