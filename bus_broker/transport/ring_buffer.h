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

#define RING_SIZE        1024          // Must be power of 2
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
    uint8_t  _pad[3];         // explicit padding to align next field
    uint8_t  canh_bytes[MAX_SIGNAL_BYTES];
    uint8_t  canl_bytes[MAX_SIGNAL_BYTES];
} BusFrameSlot;

// ── Shared memory layout ──────────────────────────────────────────────────────

// We place write_index and read_index on separate cache lines (64 bytes each)
// to avoid false sharing between producer and consumer cores.
typedef struct {
    // Cache line 0: producer writes here
    _Alignas(64) atomic_uint_fast64_t write_index;
    uint8_t _pad1[64 - sizeof(atomic_uint_fast64_t)];

    // Cache line 1: consumer reads here
    _Alignas(64) atomic_uint_fast64_t read_index;
    uint8_t _pad2[64 - sizeof(atomic_uint_fast64_t)];

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

// Write a frame. Returns 0 on success, -1 if buffer is full.
int shm_write(SharedBus* bus, const BusFrameSlot* frame);

// Force delete and recreate the shm. Used by tests only.
SharedBus* shm_reset(void);

// Read the next frame for this consumer.
// consumer_index is per-ECU state (each ECU tracks its own position).
// Returns 0 on success, -1 if no new frames.
int shm_read(
    SharedBus*    bus,
    uint64_t*     consumer_index,
    BusFrameSlot* out
);

// How many frames are waiting for this consumer.
uint64_t shm_available(SharedBus* bus, uint64_t consumer_index);

#endif // RING_BUFFER_H
