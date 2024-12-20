/*!
 * \file      circularfs.c CircularFS implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <inttypes.h>
#include <stdbool.h>

#include "smtc_modem_hal_dbg_trace.h"
#include "circularfs.h"

#ifndef MIN
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )
#endif

/**
 * @}
 * @defgroup sector status
 * @{
 */
#define SECTOR_ERASED 0xFFFFFFFF     /**< Default state after NOR flash erase. */
#define SECTOR_FREE 0xFFFFFF00       /**< Sector erased. */
#define SECTOR_IN_USE 0xFFFF0000     /**< Sector contains valid data. */
#define SECTOR_FORMATTING 0xFF000000 /**< The entire partition is being formatted. */
#define SECTOR_ERASING 0x00000000    /**< Sector should be erased. */

// status
// [F][F][F][F] erased
// [0][F][F][F] free
// [0][0][F][F] in used
// [0][0][0][F] formatting
// [0][0][0][0] erasing

typedef struct sector_header
{
    uint64_t version;
    uint64_t status_free;
    uint64_t status_in_used;
    uint64_t status_formating;
    uint64_t status_erasing;
} sector_header_t;

static int32_t _sector_address( struct circularfs* fs, int32_t sector_offset )
{
    return ( fs->flash->sector_offset + sector_offset ) * fs->flash->sector_size;
}

static int32_t _sector_get_version( struct circularfs* fs, int32_t sector, uint32_t* version )
{
    sector_header_t status_tmp = { 0 };
    int32_t ret = fs->flash->read( fs->flash, _sector_address( fs, sector ) + offsetof( struct sector_header, version ),
                                   &status_tmp.version, sizeof( status_tmp.version ) );
    *version    = ( uint32_t ) status_tmp.version;
    return ret;
}
static int32_t _sector_get_status( struct circularfs* fs, int32_t sector, uint32_t* status )
{
    sector_header_t status_tmp = { 0 };

    int32_t ret = fs->flash->read( fs->flash, _sector_address( fs, sector ), &status_tmp, sizeof( sector_header_t ) );

    if( ( status_tmp.status_free == ~0ULL ) && ( status_tmp.status_in_used == ~0ULL ) &&
        ( status_tmp.status_formating == ~0ULL ) && ( status_tmp.status_erasing == ~0ULL ) )
    {
        *status = SECTOR_ERASED;
    }
    else if( ( status_tmp.status_free == 0ULL ) && ( status_tmp.status_in_used == ~0ULL ) &&
             ( status_tmp.status_formating == ~0ULL ) && ( status_tmp.status_erasing == ~0ULL ) )
    {
        *status = SECTOR_FREE;
    }
    else if( ( status_tmp.status_free == 0ULL ) && ( status_tmp.status_in_used == 0ULL ) &&
             ( status_tmp.status_formating == ~0ULL ) && ( status_tmp.status_erasing == ~0ULL ) )
    {
        *status = SECTOR_IN_USE;
    }
    else if( ( status_tmp.status_free == 0ULL ) && ( status_tmp.status_in_used == 0ULL ) &&
             ( status_tmp.status_formating == 0ULL ) && ( status_tmp.status_erasing == ~0ULL ) )
    {
        *status = SECTOR_FORMATTING;
    }
    else if( ( status_tmp.status_free == 0ULL ) && ( status_tmp.status_in_used == 0ULL ) &&
             ( status_tmp.status_formating == 0ULL ) && ( status_tmp.status_erasing == 0ULL ) )
    {
        *status = SECTOR_ERASING;
    }
    else
    {
        ret = -1;
    }
    return ret;
}

static int32_t _sector_set_status( struct circularfs* fs, int32_t sector, uint32_t status )
{
    uint64_t status_tmp = 0ULL;

    if( status == SECTOR_FREE )
    {
        return fs->flash->program( fs->flash,
                                   _sector_address( fs, sector ) + offsetof( struct sector_header, status_free ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else if( status == SECTOR_IN_USE )
    {
        return fs->flash->program( fs->flash,
                                   _sector_address( fs, sector ) + offsetof( struct sector_header, status_in_used ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else if( status == SECTOR_FORMATTING )
    {
        return fs->flash->program( fs->flash,
                                   _sector_address( fs, sector ) + offsetof( struct sector_header, status_formating ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else if( status == SECTOR_ERASING )
    {
        return fs->flash->program( fs->flash,
                                   _sector_address( fs, sector ) + offsetof( struct sector_header, status_erasing ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else
    {
        return -1;
    }
}

static int32_t _sector_free( struct circularfs* fs, int32_t sector, bool guard )
{
    int32_t sector_addr = _sector_address( fs, sector );
    if( guard == true )
    {
        _sector_set_status( fs, sector, SECTOR_ERASING );
    }

    fs->flash->sector_erase( fs->flash, sector_addr );
    fs->flash->program( fs->flash, sector_addr + offsetof( struct sector_header, version ), &fs->version,
                        sizeof( fs->version ) );
    _sector_set_status( fs, sector, SECTOR_FREE );
    return 0;
}

/**
 * @}
 * @defgroup slot status
 * @{
 */

#define SLOT_ERASED 0xFFFFFFFF   /**< Default state after NOR flash erase. */
#define SLOT_RESERVED 0xFFFFFF00 /**< Write started but not yet committed. */
#define SLOT_VALID 0xFFFF0000    /**< Write committed, slot contains valid data. */
#define SLOT_GARBAGE 0xFF000000  /**< Slot contents discarded and no longer valid. */
// [F][F][F] erased
// [0][F][F] reserved
// [0][0][F] SLOT_VALID
// [0][0][0] GARBAGE

typedef struct slot_header
{
    // uint64_t status[3];
    uint64_t status_reserved;
    uint64_t status_valid;
    uint64_t status_garbage;
} slot_header_t;

static int32_t _slot_address( struct circularfs* fs, struct circularfs_loc* loc )
{
    return _sector_address( fs, loc->sector ) + sizeof( struct sector_header ) +
           ( sizeof( struct slot_header ) + fs->object_size ) * loc->slot;
}

static int32_t _slot_get_status( struct circularfs* fs, struct circularfs_loc* loc, uint32_t* status )
{
    slot_header_t status_tmp;

    int32_t ret =
        fs->flash->read( fs->flash, _slot_address( fs, loc ) + offsetof( struct slot_header, status_reserved ),
                         &status_tmp, sizeof( status_tmp ) );

    if( ( status_tmp.status_reserved == ~0ULL ) && ( status_tmp.status_valid == ~0ULL ) &&
        ( status_tmp.status_garbage == ~0ULL ) )
    {
        *status = SLOT_ERASED;
    }
    else if( ( status_tmp.status_reserved == 0ULL ) && ( status_tmp.status_valid == ~0ULL ) &&
             ( status_tmp.status_garbage == ~0ULL ) )
    {
        *status = SLOT_RESERVED;
    }
    else if( ( status_tmp.status_reserved == 0ULL ) && ( status_tmp.status_valid == 0ULL ) &&
             ( status_tmp.status_garbage == ~0ULL ) )
    {
        *status = SLOT_VALID;
    }
    else if( ( status_tmp.status_reserved == 0ULL ) && ( status_tmp.status_valid == 0ULL ) &&
             ( status_tmp.status_garbage == 0ULL ) )
    {
        *status = SLOT_GARBAGE;
    }
    else
    {
        ret = -1;
    }
    return ret;
}

static int32_t _slot_set_status( struct circularfs* fs, struct circularfs_loc* loc, uint32_t status )
{
    uint64_t status_tmp = 0ULL;
    if( status == SLOT_RESERVED )
    {
        return fs->flash->program( fs->flash,
                                   _slot_address( fs, loc ) + offsetof( struct slot_header, status_reserved ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else if( status == SLOT_VALID )
    {
        return fs->flash->program( fs->flash, _slot_address( fs, loc ) + offsetof( struct slot_header, status_valid ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    else if( status == SLOT_GARBAGE )
    {
        return fs->flash->program( fs->flash, _slot_address( fs, loc ) + offsetof( struct slot_header, status_garbage ),
                                   &status_tmp, sizeof( status_tmp ) );
    }
    return -1;
}

/**
 * @}
 * @defgroup loc
 * @{
 */

static bool _loc_equal( struct circularfs_loc* a, struct circularfs_loc* b )
{
    return ( a->sector == b->sector ) && ( a->slot == b->slot );
}

/** Advance a location to the beginning of the next sector. */
static void _loc_advance_sector( struct circularfs* fs, struct circularfs_loc* loc )
{
    loc->slot = 0;
    loc->sector++;
    if( loc->sector >= fs->flash->sector_count )
    {
        loc->sector = 0;
    }
}

/** Advance a location to the next slot, advancing the sector too if needed. */
static void _loc_advance_slot( struct circularfs* fs, struct circularfs_loc* loc )
{
    loc->slot++;
    if( loc->slot >= fs->slots_per_sector )
    {
        _loc_advance_sector( fs, loc );
    }
}

/**
 * @}
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

int32_t circularfs_init( struct circularfs* fs, struct circularfs_flash_partition* flash, uint32_t version,
                         int32_t object_size )
{
    /* Copy arguments to instance. */
    fs->flash       = flash;
    fs->version     = version;
    fs->object_size = object_size;

    /* Precalculate commonly used values. */
    fs->slots_per_sector = ( fs->flash->sector_size - sizeof( struct sector_header ) ) /
                           ( sizeof( struct slot_header ) + fs->object_size );

    return 0;
}

int32_t circularfs_format( struct circularfs* fs, bool guard )
{
    if( guard == true )
    {
        /* Mark all sectors to prevent half-erased filesystems. */
        for( int32_t sector = 0; sector < fs->flash->sector_count; sector++ )
        {
            _sector_set_status( fs, sector, SECTOR_FORMATTING );
        }
    }

    /* Erase, update version, mark as free. */
    for( int32_t sector = 0; sector < fs->flash->sector_count; sector++ )
    {
        _sector_free( fs, sector, guard );
    }

    /* Start reading & writing at the first sector. */
    fs->read.sector   = 0;
    fs->read.slot     = 0;
    fs->write.sector  = 0;
    fs->write.slot    = 0;
    fs->cursor.sector = 0;
    fs->cursor.slot   = 0;

    return 0;
}

int32_t circularfs_scan( struct circularfs* fs )
{
    uint32_t previous_sector_status = SECTOR_FREE;
    /* The read sector is the first IN_USE sector *after* a FREE sector
     * (or the first one). */
    int32_t read_sector = 0;
    /* The write sector is the last IN_USE sector *before* a FREE sector
     * (or the last one). */
    int32_t write_sector = fs->flash->sector_count - 1;
    /* There must be at least one FREE sector available at all times. */
    bool free_seen = false;
    /* If there's no IN_USE sector, we start at the first one. */
    bool used_seen = false;

    /* Iterate over sectors. */
    for( int32_t sector = 0; sector < fs->flash->sector_count; sector++ )
    {
        // int32_t addr = _sector_address( fs, sector );

        // /* Read sector header. */
        // struct sector_header header;
        // fs->flash->read( fs->flash, addr, &header, sizeof( header ) );

        uint32_t header_version;
        uint32_t header_status;
        _sector_get_version( fs, sector, &header_version );
        _sector_get_status( fs, sector, &header_status );

        /* Detect partially-formatted partitions. */
        if( ( header_status == SECTOR_FORMATTING ) ||
            ( ( header_status != SECTOR_ERASED ) && ( header_status != SECTOR_FREE ) &&
              ( header_status != SECTOR_IN_USE ) && ( header_status != SECTOR_ERASING ) ) )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "circularfs_scan: partially formatted partition\r\n" );
            return -1;
        }

        /* Detect and fix partially erased sectors. */
        if( header_status == SECTOR_ERASING || header_status == SECTOR_ERASED )
        {
            _sector_free( fs, sector, true );
            header_status = SECTOR_FREE;
        }

        /* Detect corrupted sectors. */
        if( header_status != SECTOR_FREE && header_status != SECTOR_IN_USE )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "circularfs_scan: corrupted sector %d\r\n", sector );
            return -1;
        }

        /* Detect obsolete versions. We can't do this earlier because the version
         * could have been invalid due to a partial erase. */
        if( header_version != fs->version )
        {
            SMTC_MODEM_HAL_TRACE_PRINTF( "circularfs_scan: incompatible version 0x%08" PRIx32 "\r\n", header_version );
            return -1;
        }

        /* Record the presence of a FREE sector. */
        if( header_status == SECTOR_FREE )
        {
            free_seen = true;
        }

        /* Record the presence of a IN_USE sector. */
        if( header_status == SECTOR_IN_USE )
        {
            used_seen = true;
        }

        /* Update read & write sectors according to the above rules. */
        if( header_status == SECTOR_IN_USE && previous_sector_status == SECTOR_FREE )
        {
            read_sector = sector;
        }
        if( header_status == SECTOR_FREE && previous_sector_status == SECTOR_IN_USE )
        {
            write_sector = sector - 1;
        }

        previous_sector_status = header_status;
    }

    ///////////////////////////////////////
    ////////////
    /* Detect the lack of a FREE sector. */
    if( !free_seen )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "circularfs_scan: invariant violated: no FREE sector found\r\n" );
        return -1;
    }
    ////////////
    ///////////////////////////////////////

    /* Start writing at the first sector if the filesystem is empty. */
    if( !used_seen )
    {
        write_sector = 0;
    }

    /* Scan the write sector and skip all occupied slots at the beginning. */
    fs->write.sector = write_sector;
    fs->write.slot   = 0;
    while( fs->write.sector == write_sector )
    {
        uint32_t status;
        _slot_get_status( fs, &fs->write, &status );
        if( status == SLOT_ERASED )
        {
            break;
        }

        _loc_advance_slot( fs, &fs->write );
    }
    /* If the sector was full, we're at the beginning of a FREE sector now. */

    /* Position the read head at the start of the first IN_USE sector, then skip
     * over garbage/invalid slots until something of value is found or we reach
     * the write head which means there's no data. */
    fs->read.sector = read_sector;
    fs->read.slot   = 0;
    while( !_loc_equal( &fs->read, &fs->write ) )
    {
        uint32_t status;
        _slot_get_status( fs, &fs->read, &status );
        if( status == SLOT_VALID )
        {
            break;
        }

        _loc_advance_slot( fs, &fs->read );
    }

    /* Move the read cursor to the read head position. */
    fs->cursor = fs->read;

    return 0;
}

int32_t circularfs_capacity( struct circularfs* fs )
{
    return fs->slots_per_sector * ( fs->flash->sector_count - 1 );
}

int32_t circularfs_free_slot_estimate( struct circularfs* fs )
{
    uint32_t capacity       = circularfs_capacity( fs );
    uint32_t count_estimate = circularfs_count_estimate( fs );

    uint32_t free_slot = capacity - count_estimate;

    if( fs->read.slot != 0 )
    {
        free_slot -= fs->read.slot;
    }
    return free_slot;
}

int32_t circularfs_count_estimate( struct circularfs* fs )
{
    int32_t sector_diff = ( fs->write.sector - fs->read.sector + fs->flash->sector_count ) % fs->flash->sector_count;

    return sector_diff * fs->slots_per_sector + fs->write.slot - fs->read.slot;
}

int32_t circularfs_count_estimate_from_last_fetch( struct circularfs* fs )
{
    int32_t sector_diff = ( fs->write.sector - fs->cursor.sector + fs->flash->sector_count ) % fs->flash->sector_count;

    return sector_diff * fs->slots_per_sector + fs->write.slot - fs->cursor.slot;
}

int32_t circularfs_count_exact( struct circularfs* fs )
{
    int32_t count = 0;

    /* Use a temporary loc for iteration. */
    struct circularfs_loc loc = fs->read;
    while( !_loc_equal( &loc, &fs->write ) )
    {
        uint32_t status = 0;
        _slot_get_status( fs, &loc, &status );

        if( status == SLOT_VALID )
        {
            count++;
        }

        _loc_advance_slot( fs, &loc );
    }

    return count;
}

int32_t circularfs_count_exact_from_last_fetch( struct circularfs* fs )
{
    int32_t count = 0;

    /* Use a temporary loc for iteration. */
    struct circularfs_loc loc = fs->cursor;
    while( !_loc_equal( &loc, &fs->write ) )
    {
        uint32_t status = 0;
        _slot_get_status( fs, &loc, &status );

        if( status == SLOT_VALID )
        {
            count++;
        }

        _loc_advance_slot( fs, &loc );
    }

    return count;
}

int32_t circularfs_append( struct circularfs* fs, const void* object )
{
    uint32_t status;

    /*
     * There are three sectors involved in appending a value:
     * - the sector where the append happens: it has to be writable
     * - the next sector: it must be free (invariant)
     * - the next-next sector: read & cursor heads are moved there if needed
     */
    ///////////////////////////////////////
    ////////////
    /* Make sure the next sector is free. */
    int32_t next_sector = ( fs->write.sector + 1 ) % fs->flash->sector_count;
    _sector_get_status( fs, next_sector, &status );
    if( status != SECTOR_FREE )
    {
        /* Next sector must be freed. But first... */

        /* Move the read & cursor heads out of the way. */
        if( fs->read.sector == next_sector )
        {
            _loc_advance_sector( fs, &fs->read );
        }
        if( fs->cursor.sector == next_sector )
        {
            _loc_advance_sector( fs, &fs->cursor );
        }

        /* Free the next sector. */
        _sector_free( fs, next_sector, true );
    }
    ////////////
    ///////////////////////////////////////

    /* Now we can make sure the current write sector is writable. */
    _sector_get_status( fs, fs->write.sector, &status );
    if( status == SECTOR_FREE )
    {
        /* Free sector. Mark as used. */
        _sector_set_status( fs, fs->write.sector, SECTOR_IN_USE );
    }
    else if( status != SECTOR_IN_USE )
    {
        SMTC_MODEM_HAL_TRACE_PRINTF( "circularfs_append: corrupted filesystem\r\n" );
        return -1;
    }

    /* Preallocate slot. */
    _slot_set_status( fs, &fs->write, SLOT_RESERVED );

    /* Write object. */
    fs->flash->program( fs->flash, _slot_address( fs, &fs->write ) + sizeof( struct slot_header ), object,
                        fs->object_size );

    /* Commit write. */
    _slot_set_status( fs, &fs->write, SLOT_VALID );

    /* Advance the write head. */
    _loc_advance_slot( fs, &fs->write );

    return 0;
}

int32_t circularfs_fetch( struct circularfs* fs, void* object )
{
    /* Advance forward in search of a valid slot. */
    while( !_loc_equal( &fs->cursor, &fs->write ) )
    {
        uint32_t status = 0;

        _slot_get_status( fs, &fs->cursor, &status );

        if( status == SLOT_VALID )
        {
            fs->flash->read( fs->flash, _slot_address( fs, &fs->cursor ) + sizeof( struct slot_header ), object,
                             fs->object_size );
            _loc_advance_slot( fs, &fs->cursor );
            return 0;
        }

        _loc_advance_slot( fs, &fs->cursor );
    }

    return -1;
}

int32_t circularfs_discard( struct circularfs* fs )
{
    while( !_loc_equal( &fs->read, &fs->cursor ) )
    {
        _slot_set_status( fs, &fs->read, SLOT_GARBAGE );
        _loc_advance_slot( fs, &fs->read );
    }

    return 0;
}

int32_t circularfs_discard_item_x_from_head_position( struct circularfs* fs, uint32_t item_x_bitfield )
{
    uint32_t count = 0;
    /* Use a temporary loc for iteration. */
    struct circularfs_loc loc = fs->read;

    while( ( !_loc_equal( &loc, &fs->cursor ) ) && ( count < ( sizeof( item_x_bitfield ) * 8 ) ) )
    {
        /* garbage requested data */
        if( ( ( item_x_bitfield >> count ) & 0x01 ) == 1 )
        {
            uint32_t status;
            _slot_get_status( fs, &loc, &status );

            /* Search the next valid data */
            while( ( !_loc_equal( &loc, &fs->cursor ) ) && ( status != SLOT_VALID ) )
            {
                _loc_advance_slot( fs, &loc );
                _slot_get_status( fs, &loc, &status );
            }
            if( status == SLOT_VALID )
            {
                _slot_set_status( fs, &loc, SLOT_GARBAGE );
            }
        }

        _loc_advance_slot( fs, &loc );
        count++;
    }

    /* Move the read cursor to the next valid data */
    while( !_loc_equal( &fs->read, &fs->write ) )
    {
        uint32_t status;
        _slot_get_status( fs, &fs->read, &status );
        if( status == SLOT_VALID )
        {
            break;
        }

        _loc_advance_slot( fs, &fs->read );
    }

    return 0;
}

int32_t circularfs_rewind( struct circularfs* fs )
{
    fs->cursor = fs->read;
    return 0;
}

void circularfs_dump( struct circularfs* fs )
{
#if( MODEM_HAL_DBG_TRACE == MODEM_HAL_FEATURE_ON )
    const char* description;
    char        slots_description[170];

    SMTC_MODEM_HAL_TRACE_PRINTF( "CIRCULARFS read: {%d,%d} cursor: {%d,%d} write: {%d,%d}\n", fs->read.sector,
                                 fs->read.slot, fs->cursor.sector, fs->cursor.slot, fs->write.sector, fs->write.slot );

    for( int32_t sector = 0; sector < fs->flash->sector_count; sector++ )
    {
        // int32_t addr = _sector_address( fs, sector );

        /* Read sector header. */
        // struct sector_header header;
        // fs->flash->read( fs->flash, addr, &header, sizeof( header ) );

        uint32_t header_version = 0;
        uint32_t header_status  = 0xFF;
        _sector_get_version( fs, sector, &header_version );
        _sector_get_status( fs, sector, &header_status );

        switch( header_status )
        {
        case SECTOR_ERASED:
            description = "ERASED";
            break;
        case SECTOR_FREE:
            description = "FREE";
            break;
        case SECTOR_IN_USE:
            description = "IN_USE";
            break;
        case SECTOR_ERASING:
            description = "ERASING";
            break;
        case SECTOR_FORMATTING:
            description = "FORMATTING";
            break;
        default:
            description = "UNKNOWN";
            break;
        }

        for( int32_t slot = 0; slot < fs->slots_per_sector; slot++ )
        {
            if( slot < ( int32_t ) ( sizeof( slots_description ) - 1 ) )
            {
                struct circularfs_loc loc    = { sector, slot };
                uint32_t              status = 0;
                _slot_get_status( fs, &loc, &status );

                switch( status )
                {
                case SLOT_ERASED:
                    slots_description[slot] = 'E';
                    break;
                case SLOT_RESERVED:
                    slots_description[slot] = 'R';
                    break;
                case SLOT_VALID:
                    slots_description[slot] = 'V';
                    break;
                case SLOT_GARBAGE:
                    slots_description[slot] = 'G';
                    break;
                default:
                    slots_description[slot] = '?';
                    break;
                }
            }
            else
            {
                SMTC_MODEM_HAL_TRACE_WARNING( "%s: buffer trace too short\n", __func__ );
                break;
            }
        }
        slots_description[MIN( fs->slots_per_sector, ( int32_t ) ( sizeof( slots_description ) - 1 ) )] = '\0';

        SMTC_MODEM_HAL_TRACE_PRINTF( "[%04d] [v=0x%08" PRIx32 "] [%-10s] %s\n", sector, header_version, description,
                                     slots_description );
    }
#endif  // MODEM_HAL_DBG_TRACE
}
