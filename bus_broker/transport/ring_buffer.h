// bus_broker/transport/ring_buffer.h

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

// ── Constants ─────────────────────────────────────────────────────────────────

#define RING_SIZE        4096
#define RING_MASK        (RING_SIZE - 1)
#define MAX_SIGNAL_BYTES 256           // 2048 bits max per frame
#define SHM_NAME         "/virtual_can_bus"

// ── Frame slot ────────────────────────────────────────────────────────────────

// One frame slot in the ring buffer.
// __attribute__((packed)) ensures no compiler padding between fields
// so the Python ctypes struct matches exactly.
typedef struct __attribute__((packed)) {
    uint64_t timestamp_ns;
    uint32_t arbitration_id;
    uint16_t bit_count;
    uint8_t  protocol;        // 0 = CAN, 1 = CAN FD
    uint8_t  is_extended;
    uint8_t  is_remote;
    uint16_t brs_index;       // bit index where data-rate phase begins
                              // 0 = classic CAN (no rate switch)
    uint8_t  _pad[1];         // explicit padding to align next field
    uint8_t  canh_bytes[MAX_SIGNAL_BYTES];
    uint8_t  canl_bytes[MAX_SIGNAL_BYTES];
} BusFrameSlot;

// ── Shared memory layout ──────────────────────────────────────────────────────

// Cache-line-aligned layout to avoid false sharing.
//
// write_index: monotonically increasing counter (never wraps to 0).
//   Actual slot = write_index & RING_MASK.
//   This lets readers detect overwritten slots by comparing distance.
//
// write_lock: spinlock that serialises concurrent writers.
//   Multiple ECU processes write to the same ring buffer.
//   Without this, two writers could claim the same slot.
typedef struct {
    // Cache line 0: producer writes here
    _Alignas(64) atomic_uint_fast64_t write_index;
    uint8_t _pad1[64 - sizeof(atomic_uint_fast64_t)];

    // Cache line 1: write serialisation lock
    _Alignas(64) atomic_int write_lock;
    uint8_t _pad2[64 - sizeof(atomic_int)];

    // Frame slots follow after the header
    _Alignas(64) BusFrameSlot slots[RING_SIZE];
} SharedBus;

// ── API ───────────────────────────────────────────────────────────────────────

// Create and map shared memory. Returns pointer or NULL on error.
SharedBus* shm_create(void);

// Open existing shared memory for reading. Returns pointer or NULL on error.
SharedBus* shm_open_existing(void);

// Unmap shared memory. Does not delete the shm object.
void shm_close(SharedBus* bus);

// Delete the shared memory object from the system.
void shm_unlink_bus(void);

// Write a frame. Always returns 0 (overwriting ring buffer, never full).
// Safe to call from multiple processes concurrently.
int shm_write(SharedBus* bus, const BusFrameSlot* frame);

// Force delete and recreate the shm.
SharedBus* shm_reset(void);

// Read the next frame for this consumer.
// consumer_index is per-ECU state (each ECU tracks its own position).
// Returns 0 on success, -1 if no new frames.
// If the consumer fell behind (slot overwritten), it advances
// automatically to the oldest available slot.
int shm_read(
    SharedBus*    bus,
    uint64_t*     consumer_index,
    BusFrameSlot* out
);

// How many frames are waiting for this consumer.
uint64_t shm_available(SharedBus* bus, uint64_t consumer_index);

#endif // RING_BUFFER_H
