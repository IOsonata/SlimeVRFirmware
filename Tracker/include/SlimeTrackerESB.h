/**-------------------------------------------------------------------------
@file	SlimeTrackerESB.h

@brief	ESB definition file.

Implementation based on SlimenRF ESB protocol

@author	Nguyen Hoan Hoang
@date	Dec. 24, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __SLIMETRACKERESB_H__
#define __SLIMETRACKERESB_H__

#include "nrf_esb.h"

#define RECEIVER_ID_LENGTH		8

#ifdef __cplusplus
extern "C" {
#endif

bool IsPaired(void);
void SetReceiverMacAddr(uint64_t Id);
uint64_t GetReceiverMacAddr();
void SetTrackerId(uint8_t Id);
bool EsbInit(void);
bool EsbSendPairing(void);
bool EsbSend(uint8_t *pData, size_t Len);

#ifdef __cplusplus
}
#endif

#endif // __SLIMETRACKERESB_H__
