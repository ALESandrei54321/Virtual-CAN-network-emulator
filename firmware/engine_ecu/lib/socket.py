# firmware/lib/socket.py
import sys
import uselect

AF_INET = 2
SOCK_STREAM = 1
SOL_SOCKET = 65535
SO_REUSEADDR = 4

class timeout(OSError):
    pass

class socket:
    def __init__(self, af=AF_INET, sock=SOCK_STREAM):
        self._timeout = None

    def setsockopt(self, level, optname, value):
        pass

    def bind(self, addr):
        pass

    def listen(self, backlog):
        pass

    def settimeout(self, value):
        self._timeout = value

    def accept(self):
        # Return self as the connection object and a dummy address
        return self, ('127.0.0.1', 5555)

    def recv(self, bufsize):
        poll = uselect.poll()
        poll.register(sys.stdin, uselect.POLLIN)
        
        timeout_ms = int(self._timeout * 1000) if self._timeout is not None else -1
        events = poll.poll(timeout_ms)
        if not events:
            raise timeout("timed out")
        
        res = sys.stdin.read(1)
        if not res:
            return b""
            
        # Draining any other waiting characters immediately
        chars = [res]
        while len(chars) < bufsize:
            if poll.poll(0):
                c = sys.stdin.read(1)
                if c:
                    chars.append(c)
                else:
                    break
            else:
                break
        return "".join(chars).encode('utf-8')

    def sendall(self, data):
        if isinstance(data, str):
            sys.stdout.write(data)
        else:
            sys.stdout.write(data.decode('utf-8'))

    def close(self):
        pass
