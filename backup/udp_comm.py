import socket
import threading
import numpy as np

pc_addr = '10.110.1.3'
pc_port = 3000

gl_addr = '10.110.1.2'
gl_port = 2000

# pc_addr = '10.110.1.3'
# pc_port = 3000

# gl_addr = '10.110.1.2'
# gl_port = 2000


class UdpReaderThread(threading.Thread):
    def __init__(self, protocol_factory):
        super(UdpReaderThread, self).__init__()

        self.daemon = True
        self.alive = True
        self.protocol_factory = protocol_factory

        self._lock = threading.Lock()
        self._connection_made = threading.Event()
        self.protocol = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((pc_addr, pc_port))
        self.sock.setblocking(True)
        self.sock.settimeout(1)

        print('::: Udp communication start!!')

    def stop(self):
        """Stop the reader thread"""
        self.alive = False
        self.join(2)


    def run(self):
        """Reader loop"""
        self.protocol = self.protocol_factory()
        try:
            self.protocol.connection_made(self)
        except Exception as e:
            self.alive = False
            self.protocol.connection_lost(e)
            self._connection_made.set()
            return
        error = None
        self._connection_made.set()

        while self.alive:
            try:
                # read all that is there or wait for one byte (blocking)
                data = self.sock.recvfrom(1100)[0]
            except socket.error as e:
                error = e
                break
            else:
                if data:
                    # make a separated try-except for called used code
                    try:              
                        self.protocol.data_received(data)
                    except Exception as e:
                        error = e
                        break

        self.alive = False
        self.protocol.connection_lost(error)
        self.protocol = None


    def write(self, data):
        """Thread safe writing (uses lock)"""
        with self._lock:   
            self.sock.sendto(data, (gl_addr, gl_port))


    def close(self):
        """Close the serial port and exit reader thread (uses lock)"""
        # use the lock to let other threads finish writing
        with self._lock:
            # first stop reading, so that closing can be done on idle port
            self.stop()
            self.sock.close()

    def connect(self):
        """
        Wait until connection is set up and return the transport and protocol
        instances.
        """
        if self.alive:
            self._connection_made.wait()
            if not self.alive:
                raise RuntimeError('connection_lost already called')
            return (self, self.protocol)
        else:
            raise RuntimeError('already stopped')

    # - -  context manager, returns protocol

    def __enter__(self):
        """\
        Enter context handler. May raise RuntimeError in case the connection
        could not be created.
        """
        self.start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')
        return self.protocol

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Leave context: close port"""
        self.close()
