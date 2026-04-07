// AsterCore Physics Library (https://github.com/neofilisoft/AsterCorePhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>

class FunnelTest : public Test
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, FunnelTest)

	// Description of the test
	virtual const char *GetDescription() const override
	{
		return "Spawns a lot of objects and drops them into a funnel to check for performance / stability issues.";
	}

	// See: Test
	virtual void		Initialize() override;
	virtual void		GetInitialCamera(CameraState &ioState) const override;
};
