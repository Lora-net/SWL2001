/*!
 * \file      circularfs.h
 *
 * \brief     Circular file system fifo on flash
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CIRCULARFS_H__
#define __CIRCULARFS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * Flash memory+partition descriptor.
 */
struct circularfs_flash_partition
{
    int32_t sector_size;   /**< Sector size, in bytes. */
    int32_t sector_offset; /**< Partition offset, in sectors. */
    int32_t sector_count;  /**< Partition size, in sectors. */

    /**
     * Erase a sector.
     * @param address Any address inside the sector.
     * @returns Zero on success, -1 on failure.
     */
    int32_t ( *sector_erase )( struct circularfs_flash_partition* flash, uint32_t address );
    /**
     * Program flash memory bits by toggling them from 1 to 0.
     * @param address Start address, in bytes.
     * @param data Data to program.
     * @param size Size of data.
     * @returns size on success, -1 on failure.
     */
    int32_t ( *program )( struct circularfs_flash_partition* flash, uint32_t address, const void* data, uint32_t size );
    /**
     * Read flash memory.
     * @param address Start address, in bytes.
     * @param data Buffer to store read data.
     * @param size Size of data.
     * @returns size on success, -1 on failure.
     */
    int32_t ( *read )( struct circularfs_flash_partition* flash, uint32_t address, void* data, uint32_t size );
};

/** @private */
struct circularfs_loc
{
    int32_t sector;
    int32_t slot;
};

/**
 * RingFS instance. Should be initialized with circularfs_init() before use.
 * Structure fields should not be accessed directly.
 * */
struct circularfs
{
    /* Constant values, set once at circularfs_init(). */
    struct circularfs_flash_partition* flash;
    uint32_t                           version;
    int32_t                            object_size;
    /* Cached values. */
    int32_t slots_per_sector;

    /* Read/write pointers. Modified as needed. */
    struct circularfs_loc read;
    struct circularfs_loc write;
    struct circularfs_loc cursor;
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Initialize a RingFS instance. Must be called before the instance can be used
 * with the other circularfs_* functions.
 *
 * @param fs RingFS instance to be initialized.
 * @param flash Flash memory interface. Must be implemented externally.
 * @param version Object version. Should be incremented whenever the object's
 *                semantics or size change in a backwards-incompatible way.
 * @param object_size Size of one stored object, in bytes.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_init( struct circularfs* fs, struct circularfs_flash_partition* flash, uint32_t version,
                         int32_t object_size );

/**
 * Format the flash memory.
 *
 * @param fs Initialized RingFS instance.
 * @param guard set status before formatting to prevent half-erased filesystems
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_format( struct circularfs* fs, bool guard );

/**
 * Scan the flash memory for a valid filesystem.
 *
 * @param fs Initialized RingFS instance.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_scan( struct circularfs* fs );

/**
 * Calculate maximum RingFS capacity.
 *
 * @param fs Initialized RingFS instance.
 * @returns Maximum capacity on success, -1 on failure.
 */
int32_t circularfs_capacity( struct circularfs* fs );

int32_t circularfs_free_slot_estimate( struct circularfs* fs );

/**
 * Calculate approximate object count.
 * Runs in O(1).
 *
 * @param fs Initialized RingFS instance.
 * @returns Estimated object count on success, -1 on failure.
 */
int32_t circularfs_count_estimate( struct circularfs* fs );

int32_t circularfs_count_estimate_from_last_fetch( struct circularfs* fs );

/**
 * Calculate exact object count.
 * Runs in O(n).
 *
 * @param fs Initialized RingFS instance.
 * @returns Exact object count on success, -1 on failure.
 */
int32_t circularfs_count_exact( struct circularfs* fs );

int32_t circularfs_count_exact_from_last_fetch( struct circularfs* fs );

/**
 * Append an object at the end of the ring. Deletes oldest objects as needed.
 *
 * @param fs Initialized RingFS instance.
 * @param object Object to be stored.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_append( struct circularfs* fs, const void* object );

/**
 * Fetch next object from the ring, oldest-first. Advances read cursor.
 *
 * @param fs Initialized RingFS instance.
 * @param object Buffer to store retrieved object.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_fetch( struct circularfs* fs, void* object );

/**
 * Discard all fetched objects up to the read cursor.
 *
 * @param fs Initialized RingFS instance.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_discard( struct circularfs* fs );

int32_t circularfs_discard_item_x_from_head_position( struct circularfs* fs, uint32_t item_x );

/**
 * Rewind the read cursor back to the oldest object.
 *
 * @param fs Initialized RingFS instance.
 * @returns Zero on success, -1 on failure.
 */
int32_t circularfs_rewind( struct circularfs* fs );

/**
 * Dump filesystem metadata. For debugging purposes.
 * @param stream File stream to write to.
 * @param fs Initialized RingFS instance.
 */
void circularfs_dump( struct circularfs* fs );

#ifdef __cplusplus
}
#endif

#endif  // __CIRCULARFS_H__

/* --- EOF ------------------------------------------------------------------ */
