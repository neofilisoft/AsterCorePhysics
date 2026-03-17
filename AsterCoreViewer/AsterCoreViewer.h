// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Application/Application.h>
#ifdef ACPH_DEBUG_RENDERER
	#include <AsterCore/Renderer/DebugRendererPlayback.h>
#else
	// Hack to still compile DebugRenderer inside the test framework when AsterCore is compiled without
	#define ACPH_DEBUG_RENDERER
	// Make sure the debug renderer symbols don't get imported or exported
	#define ACPH_DEBUG_RENDERER_EXPORT
	#include <AsterCore/Renderer/DebugRendererPlayback.h>
	#undef ACPH_DEBUG_RENDERER
	#undef ACPH_DEBUG_RENDERER_EXPORT
#endif

using namespace std;

// Application that views recordings produced by DebugRendererRecorder
class AsterCoreViewer : public Application
{
public:
	// Constructor / destructor
							AsterCoreViewer(const String &inCommandLine);

	// Update the application
	virtual bool			UpdateFrame(float inDeltaTime) override;

private:
	enum class EPlaybackMode
	{
		Rewind,
		StepBack,
		Stop,
		StepForward,
		Play
	};

	DebugRendererPlayback	mRendererPlayback { *mDebugRenderer };

	EPlaybackMode			mPlaybackMode = EPlaybackMode::Play;						// Current playback state. Indicates if we're playing or scrubbing back/forward.
	uint					mCurrentFrame = 0;
};
