// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <UI/UIElement.h>

/// Layout class that will automatically layout child elements vertically, stacking them
class UIVerticalStack : public UIElement
{
public:
	ACPH_DECLARE_RTTI_VIRTUAL(ACPH_NO_EXPORT, UIVerticalStack)

	/// Calculate auto layout
	virtual void		AutoLayout() override;

private:
	int					mDeltaY = 0;
	bool				mPlaceInvisibleChildren = false;
};
