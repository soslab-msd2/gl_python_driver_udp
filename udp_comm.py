import socket
import threading
import select

pc_addr = '10.110.1.3'
pc_port = 3000

gl_addr = '10.110.1.2'
gl_port = 2000


class UdpReaderThread(threading.Thread):
    def __init__(self, protocol_factory):
        super(UdpReaderThread, self).__init__()
        self.daemon = True
        self.alive = True	
        self.protocol_factory = protocol_factory
        self.protocol = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((pc_addr, pc_port))
        
        print('::: Udp communication start!!')

    
    def run(self):
        """Reader loop"""
        self.protocol = self.protocol_factory()
        try:
            self.protocol.connection_made(self)
        except Exception as e:
            self.alive = False
            self.protocol.connection_lost(e)
            RuntimeError('connection_made error')
            return

        error = None
        while self.alive:
            try:
                data = self.sock.recvfrom(2048)[0]
            except socket.error as e:
                error = e
                break
            else:
                if data:
                    try:              
                        self.protocol.data_received(data)
                    except Exception as e:
                        error = e
                        break

        self.alive = False
        self.protocol.connection_lost(error)
        self.protocol = None


    def write(self, data):
        self.sock.sendto(data, (gl_addr, gl_port))


    # - -  context manager, returns protocol

    def __enter__(self):
        """\
        Enter context handler. May raise RuntimeError in case the connection
        could not be created.
        """
        self.start()
        if not self.alive:
            raise RuntimeError('connection_lost already called')
        return self.protocol

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Leave context: close port"""
        self.alive = False
        self.sock.close()
        print('::: Udp communication End!!')
