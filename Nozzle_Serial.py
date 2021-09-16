import serial
import threading
import serial.tools.list_ports as sp

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import uic
from PyQt5.QtGui import *

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

# Packet (Machine -> PC)
Packet_Ack = {
    'MSG_PING' : "$PING$",
    'MSG_READY' : "$READY$",
    }

# Packet (PC -> Machine)
Packet_Cmd = { 
    # Command
    'PING_REQ': "&ping_req&",
    'SETUP_OFFSET': "&setup_offset&",
    'MACHINE_CONTROL': "&machine_ctrl&",
    }

Packet_Info = { 
    # Command
    'CONNECTED': "&connected&",
    'PARAM_END': "&param_end&",
    }

Packet_Param = {
    # Machine Control
    'pb_Machine_Abs': ["1", "G90"],
    'pb_Machine_Move_Home': ["2", "G28"],
    'pb_Machine_Sel_N1': ["3", "T0"],
    'pb_Machine_Move_Bed': ["4", "G0", "Z200", "F600"],
    'pb_Machine_Move_N1': ["5", "G0", "X160", "Y180", "F2400"],
    'pb_Machine_Move_N1_1': ["6", "G0", "X165", "Y180", "F2400"],
    'pb_Machine_Sel_N2': ["7", "T1"],
    'pb_Machine_Move_N2': ["8", "G0", "X160", "Y180", "F2400"],
    'pb_Machine_Gcode': ["9"],
    }

color_blue = QColor(0,0,255)
color_red = QColor(255,0,0)


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
        self.connected = False
        self.ready = False
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
            if not self.connected:
                self.cond.wait(self.mutex)

            buf = self.serial.readAll()
            if buf:
                self.received_data.emit(buf)
            self.usleep(1)
            self.mutex.unlock()

    @pyqtSlot(bool, name='setStatus')
    def set_status(self, status):
        print('set_status : ', status)
        self.connected = status
        if self.connected:
            self.cond.wakeAll()

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

        self.received_data.connect(self.read_data)
        self.pb_connect.clicked.connect(self.on_connect_button)
        self.pb_ping.clicked.connect(self.ping_request)
        self.pb_clear.clicked.connect(self.on_comm_log_clear)
        self.pb_setup_offset.clicked.connect(self.on_setup_offset_button)

        self.pb_Machine_Abs.clicked.connect(self.on_machine_button)
        self.pb_Machine_Move_Home.clicked.connect(self.on_machine_button)
        self.pb_Machine_Move_Bed.clicked.connect(self.on_machine_button)
        self.pb_Machine_Move_N1.clicked.connect(self.on_machine_button)
        self.pb_Machine_Move_N1_1.clicked.connect(self.on_machine_button)
        self.pb_Machine_Move_N2.clicked.connect(self.on_machine_button)
        self.pb_Machine_Sel_N1.clicked.connect(self.on_machine_button)
        self.pb_Machine_Sel_N2.clicked.connect(self.on_machine_button)
        self.pb_Machine_Gcode.clicked.connect(self.on_machine_button)
        
        self.N2_offset = {'name':'nozzle_offset', 'x':0, 'y':0}

        self._fill_serial_info()
        self.button_status_change(False)
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

    def on_connect_button(self):
        if self.serial.isOpen():
            self.disconnect_serial()
        else:
            self.connect_serial()
        self.serial_read_thread.setStatus(self.serial.isOpen())
        self.pb_ping.setEnabled(self.serial.isOpen())
        self.pb_connect.setText({False: 'Connect', True: 'Disconnect'}[self.serial.isOpen()])

    def on_machine_button(self):
        # Command
        self.write_data(Packet_Cmd['MACHINE_CONTROL'])
        # Param
        if self.sender().objectName() == 'pb_Machine_Gcode':
            # index
            index = Packet_Param[self.sender().objectName()][0]
            self.write_data(index)
            # User Gcode
            gcode = self.lineEdit_Gcode.text()
            gcode = gcode.replace(' ', '\n')
            print(gcode)
            self.write_data(gcode)
        else:
            # index & Fixed Gcode
            for str in Packet_Param[self.sender().objectName()]:
                self.write_data(str)
        # Param End
        self.write_data(Packet_Info['PARAM_END'])

    def on_setup_offset_button(self):
        # Command
        self.write_data(Packet_Cmd['SETUP_OFFSET'])
        # Param
        self.write_data(str(round(self.N2_offset['x'], 3)))
        self.write_data(str(round(self.N2_offset['y'], 3)))

    def	button_status_change(self, status):
        self.pb_Machine_Abs.setEnabled(status)
        self.pb_Machine_Move_Home.setEnabled(status)
        self.pb_Machine_Move_Bed.setEnabled(status)
        self.pb_Machine_Move_N1.setEnabled(status)
        self.pb_Machine_Move_N1_1.setEnabled(status)
        self.pb_Machine_Move_N2.setEnabled(status)
        self.pb_Machine_Sel_N1.setEnabled(status)
        self.pb_Machine_Sel_N2.setEnabled(status)
        self.pb_Machine_Gcode.setEnabled(status)
        self.pb_setup_offset.setEnabled(status)
        self.ready = status

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
        if status == True: self.pb_connect.setStyleSheet("background-color: rgb(50, 255, 50)")
        return status

    def disconnect_serial(self):
        print('disconnect_serial')
        self.button_status_change(False)
        self.pb_connect.setStyleSheet("background-color: rgb(255, 50, 50)")
        return self.serial.close()

    @pyqtSlot(QByteArray, name="readData")
    def read_data(self, rd):
        str(rd, 'ascii', 'replace')
        rx_data = str(rd, 'ascii', 'replace')
        print('Rx :', rx_data, len(rx_data))
        rx_packet = rx_data[:len(rx_data)-2]

        self.textEdit_Comm.setTextColor(color_red)
        self.textEdit_Comm.insertPlainText(str(rd, 'ascii', 'replace'))
        self.textEdit_Comm.moveCursor(QtGui.QTextCursor.End)

        # Parsing Packet
        if Packet_Ack['MSG_PING'] in rx_packet:
            self.write_data(Packet_Info['CONNECTED'])
        elif Packet_Ack['MSG_READY'] in rx_packet:
            self.button_status_change(True)
    
    def write_data(self, data):
        print('Tx :', data)
        delimiter = '\n'
        tx_packet = data + delimiter
        self.serial.writeData(bytes(tx_packet, "utf-8"))

        self.textEdit_Comm.setTextColor(color_blue)
        self.textEdit_Comm.insertPlainText(data+'\r\n')
        self.textEdit_Comm.moveCursor(QtGui.QTextCursor.End)

    def ping_request(self):
        self.write_data(Packet_Cmd['PING_REQ'])

    def on_comm_log_clear(self):
        self.textEdit_Comm.clear()

    
    
