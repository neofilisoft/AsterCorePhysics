// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <AsterCore/AsterCore.h>

#include <AsterCore/Core/Semaphore.h>

#ifdef ACPH_PLATFORM_WINDOWS
	ACPH_SUPPRESS_WARNING_PUSH
	ACPH_MSVC_SUPPRESS_WARNING(5039) // winbase.h(13179): warning C5039: 'TpSetCallbackCleanupGroup': pointer or reference to potentially throwing function passed to 'extern "C"' function under -EHc. Undefined behavior may occur if this function throws an exception.
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN
	#endif
#ifndef ACPH_COMPILER_MINGW
	#include <Windows.h>
#else
	#include <windows.h>
#endif
	ACPH_SUPPRESS_WARNING_POP
#endif

ACPH_NAMESPACE_BEGIN

Semaphore::Semaphore()
{
#ifdef ACPH_PLATFORM_WINDOWS
	mSemaphore = CreateSemaphore(nullptr, 0, INT_MAX, nullptr);
	if (mSemaphore == nullptr)
	{
		Trace("Failed to create semaphore");
		std::abort();
	}
#elif defined(ACPH_USE_PTHREADS)
	int ret = sem_init(&mSemaphore, 0, 0);
	if (ret == -1)
	{
		Trace("Failed to create semaphore");
		std::abort();
	}
#elif defined(ACPH_USE_GRAND_CENTRAL_DISPATCH)
	mSemaphore = dispatch_semaphore_create(0);
	if (mSemaphore == nullptr)
	{
		Trace("Failed to create semaphore");
		std::abort();
	}
#elif defined(ACPH_PLATFORM_BLUE)
	if (!ACPH_PLATFORM_BLUE_SEMAPHORE_INIT(mSemaphore))
	{
		Trace("Failed to create semaphore");
		std::abort();
	}
#endif
}

Semaphore::~Semaphore()
{
#ifdef ACPH_PLATFORM_WINDOWS
	CloseHandle(mSemaphore);
#elif defined(ACPH_USE_PTHREADS)
	sem_destroy(&mSemaphore);
#elif defined(ACPH_USE_GRAND_CENTRAL_DISPATCH)
	dispatch_release(mSemaphore);
#elif defined(ACPH_PLATFORM_BLUE)
	ACPH_PLATFORM_BLUE_SEMAPHORE_DESTROY(mSemaphore);
#endif
}

void Semaphore::Release(uint inNumber)
{
	ACPH_ASSERT(inNumber > 0);

#if defined(ACPH_PLATFORM_WINDOWS) || defined(ACPH_USE_PTHREADS) || defined(ACPH_USE_GRAND_CENTRAL_DISPATCH) || defined(ACPH_PLATFORM_BLUE)
	int old_value = mCount.fetch_add(inNumber, std::memory_order_release);
	if (old_value < 0)
	{
		int new_value = old_value + (int)inNumber;
		int num_to_release = min(new_value, 0) - old_value;
	#ifdef ACPH_PLATFORM_WINDOWS
		::ReleaseSemaphore(mSemaphore, num_to_release, nullptr);
	#elif defined(ACPH_USE_PTHREADS)
		for (int i = 0; i < num_to_release; ++i)
			sem_post(&mSemaphore);
	#elif defined(ACPH_USE_GRAND_CENTRAL_DISPATCH)
		for (int i = 0; i < num_to_release; ++i)
			dispatch_semaphore_signal(mSemaphore);
	#elif defined(ACPH_PLATFORM_BLUE)
		ACPH_PLATFORM_BLUE_SEMAPHORE_SIGNAL(mSemaphore, num_to_release);
	#endif
	}
#else
	std::lock_guard lock(mLock);
	mCount.fetch_add(inNumber, std::memory_order_relaxed);
	if (inNumber > 1)
		mWaitVariable.notify_all();
	else
		mWaitVariable.notify_one();
#endif
}

void Semaphore::Acquire(uint inNumber)
{
	ACPH_ASSERT(inNumber > 0);

#if defined(ACPH_PLATFORM_WINDOWS) || defined(ACPH_USE_PTHREADS) || defined(ACPH_USE_GRAND_CENTRAL_DISPATCH) || defined(ACPH_PLATFORM_BLUE)
	int old_value = mCount.fetch_sub(inNumber, std::memory_order_acquire);
	int new_value = old_value - (int)inNumber;
	if (new_value < 0)
	{
		int num_to_acquire = min(old_value, 0) - new_value;
	#ifdef ACPH_PLATFORM_WINDOWS
		for (int i = 0; i < num_to_acquire; ++i)
			WaitForSingleObject(mSemaphore, INFINITE);
	#elif defined(ACPH_USE_PTHREADS)
		for (int i = 0; i < num_to_acquire; ++i)
			sem_wait(&mSemaphore);
	#elif defined(ACPH_USE_GRAND_CENTRAL_DISPATCH)
		for (int i = 0; i < num_to_acquire; ++i)
			dispatch_semaphore_wait(mSemaphore, DISPATCH_TIME_FOREVER);
	#elif defined(ACPH_PLATFORM_BLUE)
		ACPH_PLATFORM_BLUE_SEMAPHORE_WAIT(mSemaphore, num_to_acquire);
	#endif
	}
#else
	std::unique_lock lock(mLock);
	mWaitVariable.wait(lock, [this, inNumber]() {
		return mCount.load(std::memory_order_relaxed) >= int(inNumber);
	});
	mCount.fetch_sub(inNumber, std::memory_order_relaxed);
#endif
}

ACPH_NAMESPACE_END
