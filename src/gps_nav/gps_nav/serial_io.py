"""POSIX UART adapter, including WSL ttyUSB devices; no pyserial dependency."""
import os
import errno
import select
import termios

BAUD = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
        57600: termios.B57600, 115200: termios.B115200}


class SerialPort:
    def __init__(self, path, baud=19200):
        if baud not in BAUD:
            raise ValueError(f'unsupported baud rate: {baud}')
        self.fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            flags = termios.tcgetattr(self.fd)
            flags[0] = 0; flags[1] = 0
            flags[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            flags[3] = 0
            flags[4] = BAUD[baud]; flags[5] = BAUD[baud]
            flags[6][termios.VMIN] = 0; flags[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, flags)
        except Exception:
            self.close(); raise

    def read(self, size=4096, timeout=0.02):
        if select.select([self.fd], [], [], timeout)[0]:
            data = os.read(self.fd, size)
            if not data:
                raise OSError(errno.EIO, 'serial EOF/disconnected')
            return data
        return b''

    def close(self):
        if self.fd is not None:
            os.close(self.fd); self.fd = None
