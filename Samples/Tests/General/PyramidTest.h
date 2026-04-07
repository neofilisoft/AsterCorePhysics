// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class PyramidTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, PyramidTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return	"Tests a large pyramid of boxes to check stacking and performance behavior.\n"
				"The large island splitter should ensure that contacts are solved on multiple CPUs in parallel.";
	}

	// See: Test
	virtual void		Initialize() override;
};
