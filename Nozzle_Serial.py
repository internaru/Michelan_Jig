import serial
import threading
import serial.tools.list_ports as sp

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic

import sys

from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QBoxLayout
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QComboBox
from PyQt5.QtWidgets import QGroupBox
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QThread
from PyQt5.QtSerialPort import QSerialPort
from PyQt5.QtSerialPort import QSerialPortInfo
from PyQt5.QtCore import QIODevice
from PyQt5.QtCore import QWaitCondition
from PyQt5.QtCore import QMutex
from PyQt5.QtCore import QByteArray
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QTextEdit

__author__ = "Seunghwan Lee <seunghwanlee@sindoh.com>"
__platform__ = sys.platform


form_class_serial = uic.loadUiType("Nozzle_Serial.ui")[0]

# # Packet (PC -> Machine)
# Packet_Tx = {
#     'CMD_PING_REQ': "&ping_req&",
#     'CMD_CONNECTED': "&connected&",
#     'CMD_NOZZLE_OFFSET': "&nozzle_offset&",
#     }

# # Packet (Machine -> PC)
# Packet_Rx = {
#     'MSG_PING' : "$PING$",
#     'MSG_READY' : "$READY$",
#     'MSG_WAITING_PARAM' : "$WAITING_PARAM$",
#     }
    
# COM_port = 0

# Rx Thread
# class SerialThread(threading.Thread):
#     '''
#     Control RS-xxx Ports
#     '''
#     is_run = False

#     list = sp.comports()
#     connected = []
#     for i in list:
#         connected.append(i.device)

#     print("Connected COM ports : " + str(connected))

#     #seq = serial.Serial('COM5', 115200, timeout=1)  # MS-Windows
#     ch = 5
#     seq = serial.Serial('COM'+str(ch), 115200, timeout=1)  # MS-Windows
#     # seq = serial.Serial('/dev/ttyUSB0', 9600) # Linux
        
#     def __init__(self, que):
#         threading.Thread.__init__(self)
#         self._lock = threading.Lock()
#         self.queue = que
        
#     def connect(self, ch):
#         print('connect start... :', ch)
#         self.is_run = True
#         #self.seq = serial.Serial('COM'+str(ch), 115200, timeout=1)  # MS-Windows
#         print('connect end... :', ch)

#     def run(self):
#         print('run...start : ', self.is_run)
#         while self.is_run:
#             if self.seq.inWaiting():

#                 res = self.seq.readline(self.seq.inWaiting())
#                 res_packet = res.decode()[:len(res)-1]
#                 print('Rx :', res_packet)

#                 if Packet_Rx['MSG_PING'] in res_packet:
#                     self.write(Packet_Tx['CMD_CONNECTED'])
#                 elif Packet_Rx['MSG_READY'] in res_packet:
#                     self.write(Packet_Tx['CMD_NOZZLE_OFFSET'])
#                 elif Packet_Rx['MSG_WAITING_PARAM'] in res_packet:
#                     self.write("-10.005")
#                     self.write("-20.005")
#                     self.write("-10000")
#                     self.write("Nozzle_Cal_End")
#         print('run...end : ', self.is_run)

#     def write(self, data):
#         """Thread safe writing (uses lock)"""
#         print('Tx :', data)
#         delimiter = '\n'
#         data = data + delimiter
        
#         self.seq.write(bytes(data, encoding='ascii'))

class SerialReadThread(QThread):
    """
    시리얼 연결이 성공하면 항상 데이터를 수신할 수 있어야 하므로
    스레드로 만들어야 한다.
    """
    # 사용자 정의 시그널 선언
    # 받은 데이터 그대로를 전달 해주기 위해 QByteArray 형태로 전달
    received_data = pyqtSignal(QByteArray, name="receivedData")

    def __init__(self, serial):
        print('SerialReadThread __init__ enter')
        QThread.__init__(self)
        print('QWaitCondition enter...')
        self.cond = QWaitCondition()
        print('QWaitCondition exit...')
        self._status = False
        self.mutex = QMutex()
        self.serial = serial
        print('SerialReadThread __init__ exit')

    def __del__(self):
        print('__del__')
        self.wait()

    def run(self):
        print('run')
        """
        들어온 데이터가 있다면 시그널을 발생
        :return:
        """
        while True:
            self.mutex.lock()
            if not self._status:
                self.cond.wait(self.mutex)

            buf = self.serial.readAll()
            if buf:
                self.received_data.emit(buf)
            self.usleep(1)
            self.mutex.unlock()

    def toggle_status(self):
        print('toggle_status')
        self._status = not self._status
        if self._status:
            self.cond.wakeAll()

    @pyqtSlot(bool, name='setStatus')
    def set_status(self, status):
        print('set_status : ', status)
        self._status = status
        if self._status:
            self.cond.wakeAll()

#class SerialController(QWidget):
class SerialDialog(QDialog, form_class_serial):

    # 시리얼포트 상수 값
    BAUDRATES = (
        QSerialPort.Baud115200,
        QSerialPort.Baud57600,
        QSerialPort.Baud38400,
        QSerialPort.Baud19200,
        QSerialPort.Baud9600,
        QSerialPort.Baud4800,
        QSerialPort.Baud2400,
        QSerialPort.Baud1200,
    )

    DATABITS = (
        QSerialPort.Data8,
        QSerialPort.Data7,
        QSerialPort.Data6,
        QSerialPort.Data5,
    )

    FLOWCONTROL = (
        QSerialPort.NoFlowControl,
        QSerialPort.HardwareControl,
        QSerialPort.SoftwareControl,
    )

    PARITY = (
        QSerialPort.NoParity,
        QSerialPort.EvenParity,
        QSerialPort.OddParity,
        QSerialPort.SpaceParity,
        QSerialPort.MarkParity,
    )

    STOPBITS = (
        QSerialPort.OneStop,
        QSerialPort.OneAndHalfStop,
        QSerialPort.TwoStop,
    )

    received_data = pyqtSignal(QByteArray, name="receivedData")
    sent_data = pyqtSignal(str, name="sentData")

    def __init__(self):
        print('SerialDialog __init__ enter')
        super().__init__()
        self.setupUi(self)

        # 시리얼 인스턴스 생성
        # 시리얼 스레드 설정 및 시작
        self.serial = QSerialPort()
        self.serial_info = QSerialPortInfo()
        self.serial_read_thread = SerialReadThread(self.serial)
        self.serial_read_thread.received_data.connect(lambda v: self.received_data.emit(v))
        self.serial_read_thread.start()

        self.pushButton_Connect.clicked.connect(self.slot_clicked_connect_button)
        self.received_data.connect(self.read_data)
        self.pushButton_Send.clicked.connect(self.write_data)
        
        self._fill_serial_info()
        print('SerialDialog __init__ exit')

    def _fill_serial_info(self):
        print('_fill_serial_info')
        # 시리얼 상수 값들을 위젯에 채운다
        self.cb_port.insertItems(0, self._get_available_port())
        self.cb_baud_rate.insertItems(0, [str(x) for x in self.BAUDRATES])
        self.cb_data_bits.insertItems(0, [str(x) for x in self.DATABITS])
        flow_name = {0: "None", 1: "Hardware", 2: "Software"}
        self.cb_flow_control.insertItems(0, [flow_name[x] for x in self.FLOWCONTROL])
        parity_name = {0: "None", 2: "Even", 3: "Odd", 4: "Space", 5: "Mark"}
        self.cb_parity.insertItems(0, [parity_name[x] for x in self.PARITY])
        stop_bits_name = {1: "1", 3: "1.5", 2: "2"}
        self.cb_stop_bits.insertItems(0, [stop_bits_name[x] for x in self.STOPBITS])

    @staticmethod
    def get_port_path():
        print('get_port_path')
        return {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]

    def _get_available_port(self):
        print('_get_available_port')
        available_port = list()
        port_path = self.get_port_path()

        for number in range(10):    # port 1~10
            port_name = port_path + str(number)
            if not self._open(port_name):
                continue
            available_port.append(port_name)
            self.serial.close()
        return available_port

    def _open(self, port_name, baudrate=QSerialPort.Baud115200, data_bits=QSerialPort.Data8,
              flow_control=QSerialPort.NoFlowControl, parity=QSerialPort.NoParity, stop_bits=QSerialPort.OneStop):
        print('_open')
        info = QSerialPortInfo(port_name)
        self.serial.setPort(info)
        self.serial.setBaudRate(baudrate)
        self.serial.setDataBits(data_bits)
        self.serial.setFlowControl(flow_control)
        self.serial.setParity(parity)
        self.serial.setStopBits(stop_bits)
        return self.serial.open(QIODevice.ReadWrite)

    def slot_clicked_connect_button(self):
        if self.serial.isOpen():
            self.disconnect_serial()
        else:
            self.connect_serial()
        self.serial_read_thread.setStatus(self.serial.isOpen())
        self.pushButton_Connect.setText({False: 'Connect', True: 'Disconnect'}[self.serial.isOpen()])

    def connect_serial(self):
        print('connect_serial')
        serial_info = {
            "port_name": self.cb_port.currentText(),
            "baudrate": self.BAUDRATES[self.cb_baud_rate.currentIndex()],
            "data_bits": self.DATABITS[self.cb_data_bits.currentIndex()],
            "flow_control": self.FLOWCONTROL[self.cb_flow_control.currentIndex()],
            "parity": self.PARITY[self.cb_parity.currentIndex()],
            "stop_bits": self.STOPBITS[self.cb_stop_bits.currentIndex()],
        }
        print('serial_info :', serial_info)
        status = self._open(**serial_info)
        return status

    def disconnect_serial(self):
        print('disconnect_serial')
        return self.serial.close()

    @pyqtSlot(QByteArray, name="readData")
    def read_data(self, rd):
        self.textEdit_Comm.insertPlainText(str(rd, 'ascii', 'replace'))
        
    def write_data(self):
        print('write_data')
        data = bytes([0x02]) + bytes("connected!", "utf-8") + bytes([0x03])
        self.serial.writeData(data)

# class SerialDialog(QDialog, form_class_serial):
#     def __init__(self):
#         super().__init__()
#         self.setupUi(self)
#         self.serial = SerialController()

    

