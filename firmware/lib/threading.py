# firmware/lib/threading.py
import _thread

class Lock:
    def __init__(self):
        self._lock = _thread.allocate_lock()

    def acquire(self, waitflag=1, timeout=-1):
        return self._lock.acquire(waitflag, timeout)

    def release(self):
        self._lock.release()

    def __enter__(self):
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._lock.release()

class Thread:
    def __init__(self, target, args=(), name=None, daemon=None):
        self.target = target
        self.args = args
        self.name = name
        self.daemon = daemon

    def start(self):
        _thread.start_new_thread(self.target, self.args)
