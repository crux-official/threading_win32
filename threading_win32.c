#define WIN32_LEAN_AND_MEAN
#include "crux_threading.h"
#include "extension.h"
#include "kernel_types.h"
#include "logging.h"
#include "memory.h"
#include "threading_api.h"
#include <stdbool.h>
#include <stdint.h>
#include <windows.h>

// The Win32 threading PRIMITIVE: CreateThread is the platform thread primitive
// (the counterpart to the posix driver's pthread_create). Like the posix driver,
// it is the PRIMITIVE, not a pool: Thread_Create spawns ONE real thread the
// caller tracks by the returned opaque handle and releases with Join/Detach;
// the hardware facts (logical/physical core counts, the concurrency ceiling)
// are reported. The pool that composes a worker set + queue + batch barrier on
// top is the thread_pool standalone (backlog 0037).
//
// Ownership (race-free, the same shape as the posix driver): the WORK ITEM
// (fn + arg) is owned by the thread after Create and freed by the thread on
// exit (only the thread touches it after Create); the HANDLE (the Windows
// thread HANDLE the caller tracks) is owned by the caller and freed by the
// caller — Join after WaitForSingleObject + CloseHandle, Detach after
// CloseHandle. The thread never touches the CruxThread, so there is no
// use-after-free and no leak on either path.
//
// The physical core count is EXACT: GetLogicalProcessorInformationEx with
// RelationProcessorCore counts the processor cores (each SMT thread is a
// logical processor; the cores are the physical fact). This is the "no
// confound" the design calls for on native — no logical/2 heuristic.

// The opaque handle: the Windows thread HANDLE, wrapped so the caller tracks an
// opaque pointer it passes back (never dereferenced).
struct CruxThread {
  HANDLE handle;
};

// The unit of work the thread consumes (fn + arg), owned by the thread after
// Create — kept separate from the handle so the caller can free the handle
// without a (possibly still-running) thread's work item going stale.
typedef struct TwbWork {
  void (*fn)(void *arg);
  void *arg;
} TwbWork;

// The Windows thread entry (LPTHREAD_START_ROUTINE: DWORD WINAPI (LPVOID)).
static DWORD WINAPI twb_thread_main(LPVOID arg) {
  TwbWork *work = (TwbWork *)arg;
  work->fn(work->arg);
  Memory_Free(work); // the thread reclaims its work item on exit (the handle is the caller's)
  return 0;
}

CruxThread *Thread_Create(void (*func)(void *arg), void *arg) {
  if (func == NULL) {
    return NULL;
  }
  CruxThread *thread = (CruxThread *)Memory_Calloc(1, sizeof *thread);
  TwbWork *work = (TwbWork *)Memory_Calloc(1, sizeof *work);
  if (thread == NULL || work == NULL) {
    if (thread != NULL) {
      Memory_Free(thread);
    }
    if (work != NULL) {
      Memory_Free(work);
    }
    LogError("[threading_win32] Create: allocation failed (handle or work item)");
    return NULL;
  }
  work->fn = func;
  work->arg = arg;
  thread->handle = CreateThread(NULL, 0, twb_thread_main, work, 0, NULL);
  if (thread->handle == NULL) {
    // A refused thread is a failure the caller (a pool) refuses to boot named
    // on — do NOT degrade to inline (a pool worker's loop is infinite; running
    // it inline would block the creating thread forever).
    Memory_Free(thread);
    Memory_Free(work);
    LogError("[threading_win32] Create: CreateThread refused (error %lu)", (unsigned long)GetLastError());
    return NULL;
  }
  return thread;
}

void Thread_Join(CruxThread *thread) {
  if (thread == NULL) {
    return;
  }
  WaitForSingleObject(thread->handle, INFINITE); // the thread reclaimed its work item on exit
  CloseHandle(thread->handle);
  Memory_Free(thread); // the caller owns the handle (the thread never frees it)
}

void Thread_Detach(CruxThread *thread) {
  if (thread == NULL) {
    return;
  }
  CloseHandle(thread->handle); // the thread's work item reclaims itself on exit
  Memory_Free(thread);         // the caller owns the handle; the thread keeps running
}

// The logical core count (SMT threads on Windows).
int Thread_Count_Logical(void) {
  SYSTEM_INFO info;
  GetSystemInfo(&info);
  return ((int)info.dwNumberOfProcessors > 0) ? (int)info.dwNumberOfProcessors : 1;
}

// The PHYSICAL core count — EXACT (no SMT /2 heuristic): each
// RelationProcessorCore entry is one physical core.
static int tpx_physical_cores_win32(void) {
  DWORD size = 0;
  // First call: query the required buffer size (returns FALSE + sets size).
  GetLogicalProcessorInformationEx(RelationProcessorCore, NULL, &size);
  if (size == 0) {
    return Thread_Count_Logical(); // the honest fallback (an unusual failure)
  }
  SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *info =
      (SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *)Memory_Calloc(1, size);
  if (info == NULL) {
    return Thread_Count_Logical();
  }
  DWORD returned = size;
  if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &returned)) {
    Memory_Free(info);
    return Thread_Count_Logical();
  }
  int count = 0;
  for (DWORD offset = 0; offset < returned;) {
    SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *entry =
        (SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *)((BYTE *)info + offset);
    if (entry->Relationship == RelationProcessorCore) {
      count++;
    }
    offset += entry->Size;
  }
  Memory_Free(info);
  return (count > 0) ? count : Thread_Count_Logical();
}

int Thread_Count_Physical(void) {
  // A one-time hardware fact (the pool asks for it AND for the concurrency at
  // Create); derive it once. Create runs single-threaded at boot, so a plain
  // static is race-free.
  static int cached = 0;
  if (cached == 0) {
    cached = tpx_physical_cores_win32();
  }
  return cached;
}

// The concurrency ceiling: one real thread per physical core (the "probably
// physical" one-per-core ceiling the pool keys off; capping further by the
// process's affinity mask and allowing a deliberate oversubscription are both
// later refinements).
int Thread_Concurrency(void) {
  return Thread_Count_Physical();
}

// The served struct — valid from load, before Construct. The package's
// identity is the manifest's (the implementation targets carry no baked
// identity defines — the org's convention, the time_posix shape).
static ThreadingAPI g_threading_api = {
    .version = {1, 0, 0},
    .Thread_Create = Thread_Create,
    .Thread_Join = Thread_Join,
    .Thread_Detach = Thread_Detach,
    .Thread_Count_Logical = Thread_Count_Logical,
    .Thread_Count_Physical = Thread_Count_Physical,
    .Thread_Concurrency = Thread_Concurrency,
};

static bool threading_win32_construct(void) {
  Engine_RegisterConcern(CRUX_CONCERN_THREADING);
  return true;
}

static void *threading_win32_get_api(void) {
  return &g_threading_api;
}

ExtensionInterface g_extension_threading_win32 = {
    "threading_win32",
    {1, 0, 0},
    threading_win32_construct,
    NULL,
    NULL,
    NULL,
    NULL,
    threading_win32_get_api,
};
