// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Core/FPException.h>

#if defined(ACPH_PLATFORM_WINDOWS)

#define ENTRY_POINT(AppName, RegisterAllocator)																\
																											\
int WINAPI wWinMain(_In_ HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)		\
{																											\
	RegisterAllocator();																					\
																											\
	ACPH_PROFILE_START("Main");																				\
																											\
	FPExceptionsEnable enable_exceptions;																	\
	ACPH_UNUSED(enable_exceptions);																			\
																											\
	{																										\
		AppName app(GetCommandLineA());																		\
		app.Run();																							\
	}																										\
																											\
	ACPH_PROFILE_END();																						\
																											\
	return 0;																								\
}																											\
																											\
int __cdecl main(int inArgC, char **inArgV)																	\
{																											\
	RegisterAllocator();																					\
																											\
	ACPH_PROFILE_START("Main");																				\
																											\
	FPExceptionsEnable enable_exceptions;																	\
	ACPH_UNUSED(enable_exceptions);																			\
																											\
	{																										\
		AppName app(Application::sCreateCommandLine(inArgC, inArgV));											\
		app.Run();																							\
	}																										\
																											\
	ACPH_PROFILE_END();																						\
																											\
	return 0;																								\
}

#else

#define ENTRY_POINT(AppName, RegisterAllocator)																\
																											\
int main(int inArgC, char **inArgV)																			\
{																											\
	RegisterAllocator();																					\
																											\
	ACPH_PROFILE_START("Main");																				\
																											\
	FPExceptionsEnable enable_exceptions;																	\
	ACPH_UNUSED(enable_exceptions);																			\
																											\
	{																										\
		AppName app(Application::sCreateCommandLine(inArgC, inArgV));											\
		app.Run();																							\
	}																										\
																											\
	ACPH_PROFILE_END();																						\
																											\
	return 0;																								\
}

#endif
