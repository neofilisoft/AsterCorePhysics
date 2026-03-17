// AsterCore Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <AsterCore/Core/Profiler.h>
#include <AsterCore/Core/NonCopyable.h>

ACPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <mutex>
#include <shared_mutex>
#include <thread>
ACPH_SUPPRESS_WARNINGS_STD_END

ACPH_NAMESPACE_BEGIN

// Things we're using from STL
using std::mutex;
using std::shared_mutex;
using std::thread;
using std::lock_guard;
using std::shared_lock;
using std::unique_lock;

#ifdef ACPH_PLATFORM_BLUE

// On Platform Blue the mutex class is not very fast so we implement it using the official APIs
class MutexBase : public NonCopyable
{
public:
					MutexBase()
	{
		ACPH_PLATFORM_BLUE_MUTEX_INIT(mMutex);
	}

					~MutexBase()
	{
		ACPH_PLATFORM_BLUE_MUTEX_DESTROY(mMutex);
	}

	inline bool		try_lock()
	{
		return ACPH_PLATFORM_BLUE_MUTEX_TRYLOCK(mMutex);
	}

	inline void		lock()
	{
		ACPH_PLATFORM_BLUE_MUTEX_LOCK(mMutex);
	}

	inline void		unlock()
	{
		ACPH_PLATFORM_BLUE_MUTEX_UNLOCK(mMutex);
	}

private:
	ACPH_PLATFORM_BLUE_MUTEX		mMutex;
};

// On Platform Blue the shared_mutex class is not very fast so we implement it using the official APIs
class SharedMutexBase : public NonCopyable
{
public:
					SharedMutexBase()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_INIT(mRWLock);
	}

					~SharedMutexBase()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_DESTROY(mRWLock);
	}

	inline bool		try_lock()
	{
		return ACPH_PLATFORM_BLUE_RWLOCK_TRYWLOCK(mRWLock);
	}

	inline bool		try_lock_shared()
	{
		return ACPH_PLATFORM_BLUE_RWLOCK_TRYRLOCK(mRWLock);
	}

	inline void		lock()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_WLOCK(mRWLock);
	}

	inline void		unlock()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_WUNLOCK(mRWLock);
	}

	inline void		lock_shared()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_RLOCK(mRWLock);
	}

	inline void		unlock_shared()
	{
		ACPH_PLATFORM_BLUE_RWLOCK_RUNLOCK(mRWLock);
	}

private:
	ACPH_PLATFORM_BLUE_RWLOCK	mRWLock;
};

#else

// On other platforms just use the STL implementation
using MutexBase = mutex;
using SharedMutexBase = shared_mutex;

#endif // ACPH_PLATFORM_BLUE

#if defined(ACPH_ENABLE_ASSERTS) || defined(ACPH_PROFILE_ENABLED) || defined(ACPH_EXTERNAL_PROFILE)

/// Very simple wrapper around MutexBase which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread
class Mutex : public MutexBase
{
public:
	inline bool		try_lock()
	{
		ACPH_ASSERT(mLockedThreadID != std::this_thread::get_id());
		if (MutexBase::try_lock())
		{
			ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
			return true;
		}
		return false;
	}

	inline void		lock()
	{
		if (!try_lock())
		{
			ACPH_PROFILE("Lock", 0xff00ffff);
			MutexBase::lock();
			ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
		}
	}

	inline void		unlock()
	{
		ACPH_ASSERT(mLockedThreadID == std::this_thread::get_id());
		ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = thread::id();)
		MutexBase::unlock();
	}

#ifdef ACPH_ENABLE_ASSERTS
	inline bool		is_locked()
	{
		return mLockedThreadID != thread::id();
	}
#endif // ACPH_ENABLE_ASSERTS

private:
	ACPH_IF_ENABLE_ASSERTS(thread::id mLockedThreadID;)
};

/// Very simple wrapper around SharedMutexBase which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread
class SharedMutex : public SharedMutexBase
{
public:
	inline bool		try_lock()
	{
		ACPH_ASSERT(mLockedThreadID != std::this_thread::get_id());
		if (SharedMutexBase::try_lock())
		{
			ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
			return true;
		}
		return false;
	}

	inline void		lock()
	{
		if (!try_lock())
		{
			ACPH_PROFILE("WLock", 0xff00ffff);
			SharedMutexBase::lock();
			ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
		}
	}

	inline void		unlock()
	{
		ACPH_ASSERT(mLockedThreadID == std::this_thread::get_id());
		ACPH_IF_ENABLE_ASSERTS(mLockedThreadID = thread::id();)
		SharedMutexBase::unlock();
	}

#ifdef ACPH_ENABLE_ASSERTS
	inline bool		is_locked()
	{
		return mLockedThreadID != thread::id();
	}
#endif // ACPH_ENABLE_ASSERTS

	inline void		lock_shared()
	{
		if (!try_lock_shared())
		{
			ACPH_PROFILE("RLock", 0xff00ffff);
			SharedMutexBase::lock_shared();
		}
	}

private:
	ACPH_IF_ENABLE_ASSERTS(thread::id mLockedThreadID;)
};

#else

using Mutex = MutexBase;
using SharedMutex = SharedMutexBase;

#endif

ACPH_NAMESPACE_END
