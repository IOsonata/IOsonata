/**-------------------------------------------------------------------------
@file	fatfs_format.h

@brief	Initialize a blank DiskIO medium with a simple FAT12 filesystem.

The filesystem formatter has no dependency on USB or the storage medium.
The caller supplies one writable sector buffer for the entire operation.

@license
MIT License
Copyright (c) 2026, I-SYST inc., all rights reserved
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
----------------------------------------------------------------------------*/
#ifndef __FATFS_FORMAT_H__
#define __FATFS_FORMAT_H__

#include <stdint.h>

#ifdef __cplusplus
class DiskIO;

/**
 * @brief	Initialize a blank, whole-disk FAT12 volume.
 *
 * A valid sector-0 55AA signature is preserved (including an MBR or a FAT
 * layout different from this formatter). Only a fully erased (0xFF) or zero
 * filled sector 0 is formatted. A recognizable interrupted IOsonata boot
 * sector may be repaired, without rewriting the FATs or root directory.
 * Other nonblank, unrecognized media are never overwritten.
 *
 * Newly formatted media use 512/1024/2048/4096-byte sectors, two FATs,
 * one reserved sector, one root-directory sector and one sector per cluster.
 * A blank medium that cannot hold a FAT12 volume is rejected.
 *
 * @param Disk		Initialized DiskIO medium
 * @param pSector	Caller-owned work buffer of at least one disk sector
 * @param BufferSize	Capacity of pSector in bytes
 * @return	true if an existing volume was preserved or initialization finished
 */
bool FatFsFormatIfBlank(DiskIO &Disk, uint8_t *pSector, uint32_t BufferSize);

#endif
#endif // __FATFS_FORMAT_H__
