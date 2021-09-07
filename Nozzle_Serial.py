import serial
import threading
import queue

# Packet (PC -> Machine)
Packet_Tx = {
    'CMD_PING_REQ': "&ping_req&",
    'CMD_CONNECTED': "&connected&",
    'CMD_NOZZLE_OFFSET': "&nozzle_offset&",
    }

# Packet (Machine -> PC)
Packet_Rx = {
    'MSG_PING' : "$PING$",
    'MSG_READY' : "$READY$",
    'MSG_WAITING_PARAM' : "$WAITING_PARAM$",
    }
    
# Rx Thread
class SerialThread(threading.Thread):
    '''
    Control RS-xxx Ports
    '''
    seq = serial.Serial('COM5', 115200, timeout=1)  # MS-Windows
    # seq = serial.Serial('/dev/ttyUSB0', 9600) # Linux
    is_run = True
    
    def __init__(self, que):
        threading.Thread.__init__(self)
        self._lock = threading.Lock()
        self.queue = que
        
    def run(self):
        while self.is_run:
            if self.seq.inWaiting():
                #text = self.seq.readline(self.seq.inWaiting())
                #self.queue.put(text)

                res = self.seq.readline(self.seq.inWaiting())
                res_packet = res.decode()[:len(res)-1]
                print('Rx :', res_packet)
                #if '$PING$' in res_packet:
                #    data = "&response_sub&!"
                #    self.seq.write(bytes(data, encoding='ascii'))
                # if Packet_Rx['MSG_PING'] in res_packet:
                #     self.seq.write(bytes(Packet_Tx['CMD_CONNECTED'], encoding='ascii'))
                # elif Packet_Rx['MSG_READY'] in res_packet:
                #     self.seq.write(bytes(Packet_Tx['CMD_NOZZLE_OFFSET'], encoding='ascii'))
                # elif Packet_Rx['MSG_WAITING_PARAM'] in res_packet:
                #     data = "10.005!"
                #     self.seq.write(bytes(data, encoding='ascii'))
                #     data = "20.005!"
                #     self.seq.write(bytes(data, encoding='ascii'))
                #     data = "10000!"
                #     self.seq.write(bytes(data, encoding='ascii'))
                #     data = "Nozzle_Cal_End"
                #     self.seq.write(bytes(data, encoding='ascii'))
                if Packet_Rx['MSG_PING'] in res_packet:
                    self.write(Packet_Tx['CMD_CONNECTED'])
                elif Packet_Rx['MSG_READY'] in res_packet:
                    self.write(Packet_Tx['CMD_NOZZLE_OFFSET'])
                elif Packet_Rx['MSG_WAITING_PARAM'] in res_packet:
                    self.write("10.005")
                    self.write("20.005")
                    self.write("10000")
                    self.write("Nozzle_Cal_End")

    def write(self, data):
        """Thread safe writing (uses lock)"""
        print('Tx :', data)
        delimiter = '\n'
        data = data + delimiter
        
        self.seq.write(bytes(data, encoding='ascii'))

