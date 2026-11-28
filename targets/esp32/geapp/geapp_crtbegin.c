// SPDX-License-Identifier: Apache-2.0
// First object of every .geapp program (the toolchain ships no crtbeginS).
//
// __dso_handle names the program to __cxa_atexit: Home's loader records the
// destructors a program registers against its own handle and runs them when
// the program unloads. Hidden, so each program has its own.
__attribute__((visibility("hidden"))) void *__dso_handle = &__dso_handle;
