// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#ifdef ACPH_OBJECT_STREAM

#include <AsterCore/ObjectStream/SerializableAttribute.h>
#include <AsterCore/ObjectStream/GetPrimitiveTypeOfType.h>
#include <AsterCore/ObjectStream/ObjectStream.h>

ACPH_NAMESPACE_BEGIN

//////////////////////////////////////////////////////////////////////////////////////////
// Macros to add properties to be serialized
//////////////////////////////////////////////////////////////////////////////////////////

template <class MemberType>
inline void AddSerializableAttributeTyped(RTTI &inRTTI, uint inOffset, const char *inName)
{
	inRTTI.AddAttribute(SerializableAttribute(inName, inOffset,
		[]()
		{
			return GetPrimitiveTypeOfType((MemberType *)nullptr);
		},
		[](int inArrayDepth, EOSDataType inDataType, const char *inClassName)
		{
			return OSIsType((MemberType *)nullptr, inArrayDepth, inDataType, inClassName);
		},
		[](IObjectStreamIn &ioStream, void *inObject)
		{
			return OSReadData(ioStream, *reinterpret_cast<MemberType *>(inObject));
		},
		[](IObjectStreamOut &ioStream, const void *inObject)
		{
			OSWriteData(ioStream, *reinterpret_cast<const MemberType *>(inObject));
		},
		[](IObjectStreamOut &ioStream)
		{
			OSWriteDataType(ioStream, (MemberType *)nullptr);
		}));
}

// ACPH_ADD_ATTRIBUTE
#define ACPH_ADD_ATTRIBUTE_WITH_ALIAS(class_name, member_name, alias_name) \
	AddSerializableAttributeTyped<decltype(class_name::member_name)>(inRTTI, offsetof(class_name, member_name), alias_name);

// ACPH_ADD_ATTRIBUTE
#define ACPH_ADD_ATTRIBUTE(class_name, member_name) \
	ACPH_ADD_ATTRIBUTE_WITH_ALIAS(class_name, member_name, #member_name)

ACPH_NAMESPACE_END

#else

#define ACPH_ADD_ATTRIBUTE_WITH_ALIAS(...)
#define ACPH_ADD_ATTRIBUTE(...)

#endif // ACPH_OBJECT_STREAM
