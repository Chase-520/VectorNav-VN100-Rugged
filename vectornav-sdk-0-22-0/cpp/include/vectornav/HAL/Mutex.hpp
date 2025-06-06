// The MIT License (MIT)
// 
// VectorNav SDK (v0.22.0)
// Copyright (c) 2024 VectorNav Technologies, LLC
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef VN_MUTEX_HPP_
#define VN_MUTEX_HPP_

#include "vectornav/Config.hpp"

#if (THREADING_ENABLE)

#ifdef __CLI__
#include "vectornav/HAL/Mutex_CLI.hpp"
#elif (_WIN32 | __linux__)
#include "vectornav/HAL/Mutex_PC.hpp"
#elif __MBED__
#include "vectornav/HAL/Mutex_MBED.hpp"
#else
static_assert(false);
#endif

#else  // THREADING_ENABLE

#include "vectornav/HAL/Mutex_Disabled.hpp"

#endif
#endif  // VN_MUTEX_HPP_
