// bus_broker/transport/ring_buffer.c

#include "ring_buffer.h"

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
        atomic_store(&bus->read_index,  0);
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
    atomic_store(&bus->read_index,  0);
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
    uint64_t write = atomic_load_explicit(
        &bus->write_index, memory_order_relaxed
    );
    uint64_t read = atomic_load_explicit(
        &bus->read_index, memory_order_acquire
    );

    // Buffer full check
    if (write - read >= RING_SIZE) {
        return -1;
    }

    // Copy frame into the correct slot
    uint64_t slot = write & RING_MASK;
    memcpy(&bus->slots[slot], frame, sizeof(BusFrameSlot));

    // Release write index - makes frame visible to consumers
    atomic_store_explicit(
        &bus->write_index, write + 1, memory_order_release
    );

    return 0;
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

    uint64_t slot = *consumer_index & RING_MASK;
    memcpy(out, &bus->slots[slot], sizeof(BusFrameSlot));
    (*consumer_index)++;

    // Update global read index to the minimum across all consumers.
    // For simplicity we track it as a single value here.
    // In a multi-ECU setup each ECU manages its own consumer_index.
    atomic_store_explicit(
        &bus->read_index, *consumer_index, memory_order_release
    );

    return 0;
}

uint64_t shm_available(SharedBus* bus, uint64_t consumer_index) {
    uint64_t write = atomic_load_explicit(
        &bus->write_index, memory_order_acquire
    );
    return (write > consumer_index) ? (write - consumer_index) : 0;
}
