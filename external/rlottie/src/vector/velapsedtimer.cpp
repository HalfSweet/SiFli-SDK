/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd. All rights reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "velapsedtimer.h"

#ifndef MT_CPPLIB
#include "rtconfig.h"
extern "C" unsigned long rt_tick_get(void);
#endif
void VElapsedTimer::start()
{
#ifdef MT_CPPLIB
    clock = std::chrono::high_resolution_clock::now();
#else
    clock = rt_tick_get();
#endif
    m_valid = true;
}

double VElapsedTimer::restart()
{
    double elapsedTime = elapsed();
    start();
    return elapsedTime;
}

double VElapsedTimer::elapsed() const
{
    if (!isValid()) return 0;
#ifdef MT_CPPLIB    
    return std::chrono::duration<double, std::milli>(
               std::chrono::high_resolution_clock::now() - clock)
        .count();
#else
    unsigned long cur=rt_tick_get();
    double delta;
    if (cur>clock)
        delta=double(cur-clock);
    else 
        delta=double(0xFFFFFFFF-cur+clock);
    delta = delta/RT_TICK_PER_SECOND;
    return delta;
#endif
}

bool VElapsedTimer::hasExpired(double time)
{
    double elapsedTime = elapsed();
    if (elapsedTime > time) return true;
    return false;
}
