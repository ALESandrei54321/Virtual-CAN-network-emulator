// bus_broker/transport/ring_buffer.c

#include "ring_buffer.h"

// ── Spinlock helpers ──────────────────────────────────────────────────────────
// TTAS (Test-and-Test-And-Set) spinlock for writer serialisation.
// Only writers lock; readers are lock-free.

static inline void spin_lock(atomic_int* lock) {
    for (;;) {
        // Optimistic test (relaxed load avoids bus traffic)
        if (atomic_load_explicit(lock, memory_order_relaxed) == 0) {
            // Try to acquire
            if (atomic_exchange_explicit(lock, 1, memory_order_acquire) == 0) {
                return;  // Got the lock
            }
        }
        // Spin — could add __builtin_ia32_pause() for x86 but
        // this is a simulation, not a latency-critical path.
    }
}

static inline void spin_unlock(atomic_int* lock) {
    atomic_store_explicit(lock, 0, memory_order_release);
}

// ── Shared memory management ──────────────────────────────────────────────────

SharedBus* shm_create(void) {
    // Try to open existing first
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    
    if (fd < 0) {
        // Does not exist yet - create it fresh
        fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            perror("shm_open create");
            return NULL;
        }

        if (ftruncate(fd, sizeof(SharedBus)) < 0) {
            perror("ftruncate");
            close(fd);
            return NULL;
        }

        SharedBus* bus = mmap(
            NULL, sizeof(SharedBus),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd, 0
        );
        close(fd);

        if (bus == MAP_FAILED) {
            perror("mmap create");
            return NULL;
        }

        // Fresh shm - zero and initialise
        memset(bus, 0, sizeof(SharedBus));
        atomic_store(&bus->write_index, 0);
        atomic_store(&bus->write_lock, 0);
        return bus;

    } else {
        // Already exists - just map it
        SharedBus* bus = mmap(
            NULL, sizeof(SharedBus),
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd, 0
        );
        close(fd);

        if (bus == MAP_FAILED) {
            perror("mmap existing");
            return NULL;
        }

        return bus;
    }
}

SharedBus* shm_open_existing(void) {
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        // No perror here - caller handles the missing shm case
        return NULL;
    }

    SharedBus* bus = mmap(
        NULL, sizeof(SharedBus),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd, 0
    );
    close(fd);

    if (bus == MAP_FAILED) {
        perror("mmap open");
        return NULL;
    }

    return bus;
}

SharedBus* shm_reset(void) {
    // Force remove whatever exists
    shm_unlink(SHM_NAME);

    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        perror("shm_open reset");
        return NULL;
    }

    if (ftruncate(fd, sizeof(SharedBus)) < 0) {
        perror("ftruncate reset");
        close(fd);
        return NULL;
    }

    SharedBus* bus = mmap(
        NULL, sizeof(SharedBus),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd, 0
    );
    close(fd);

    if (bus == MAP_FAILED) {
        perror("mmap reset");
        return NULL;
    }

    memset(bus, 0, sizeof(SharedBus));
    atomic_store(&bus->write_index, 0);
    atomic_store(&bus->write_lock, 0);
    return bus;
}

void shm_close(SharedBus* bus) {
    if (bus) {
        munmap(bus, sizeof(SharedBus));
    }
}

void shm_unlink_bus(void) {
    shm_unlink(SHM_NAME);
}

// ── Ring buffer operations ────────────────────────────────────────────────────

int shm_write(SharedBus* bus, const BusFrameSlot* frame) {
    // Acquire the write lock — serialises concurrent writers
    // from different ECU processes.
    spin_lock(&bus->write_lock);

    uint64_t write = atomic_load_explicit(
        &bus->write_index, memory_order_relaxed
    );

    // Copy frame into the correct slot (overwriting whatever was there)
    uint64_t slot = write & RING_MASK;
    memcpy(&bus->slots[slot], frame, sizeof(BusFrameSlot));

    // Release write index — makes frame visible to consumers.
    // memory_order_release ensures memcpy completes before
    // the index is visible to readers.
    atomic_store_explicit(
        &bus->write_index, write + 1, memory_order_release
    );

    spin_unlock(&bus->write_lock);

    return 0;  // Always succeeds — overwriting ring buffer, never full.
}

int shm_read(
    SharedBus*    bus,
    uint64_t*     consumer_index,
    BusFrameSlot* out
) {
    uint64_t write = atomic_load_explicit(
        &bus->write_index, memory_order_acquire
    );

    if (*consumer_index >= write) {
        return -1;   // No new frames
    }

    // Check if consumer has fallen behind (slots overwritten).
    // This happens if a reader is too slow and the writer has
    // lapped it. We skip ahead to the oldest slot that hasn't
    // been overwritten yet, leaving a margin of 1 to avoid
    // reading a slot that's currently being written.
    if (write - *consumer_index > RING_SIZE) {
        uint64_t skipped = (write - RING_SIZE) - *consumer_index;
        *consumer_index  = write - RING_SIZE + 1;
        // In a real system you'd log this.
        // For the simulation it means a slow reader lost frames,
        // which matches real CAN bus behaviour.
        (void)skipped;
    }

    uint64_t slot = *consumer_index & RING_MASK;
    memcpy(out, &bus->slots[slot], sizeof(BusFrameSlot));
    (*consumer_index)++;

    return 0;
}

uint64_t shm_available(SharedBus* bus, uint64_t consumer_index) {
    uint64_t write = atomic_load_explicit(
        &bus->write_index, memory_order_acquire
    );
    if (write > consumer_index) {
        uint64_t avail = write - consumer_index;
        // Cap at RING_SIZE — can't read more than what's in the buffer
        return (avail > RING_SIZE) ? RING_SIZE : avail;
    }
    return 0;
}
