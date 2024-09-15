import subprocess, serial, time

class SerialEmulator(object):
    def __init__(self, device_port='./ttyNewDevice', client_port='./ttyNewClient', baudrate=115200):
        self.device_port = device_port
        self.client_port = client_port
        cmd=['/usr/bin/socat','-d','-d','PTY,link=%s,raw,echo=0' %
                self.device_port, 'PTY,link=%s,raw,echo=0' % self.client_port]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(1)
        self.serial = serial.Serial(
            self.device_port, 
            baudrate=baudrate, 
            rtscts=True, 
            dsrdtr=True, 
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )
        self.err = ''
        self.out = ''

    def write(self, out):
        try: 
            ret = self.serial.write(out)
            # print(ret)
        except:
            serial.SerialTimeoutException

    def read(self):
        data = self.serial.read(1)
        return data

    def __del__(self):
        self.stop()

    def stop(self):
        self.proc.kill()
        self.out, self.err = self.proc.communicate()