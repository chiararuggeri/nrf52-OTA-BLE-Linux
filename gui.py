# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui.ui'
#
# Created: Thu Oct  5 03:48:50 2017
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from PyQt4.QtGui import QFileDialog
from PyQt4.QtCore import pyqtSignal

from bluepy.btle import *
from intelhex import IntelHex
from array    import array
from unpacker import Unpacker

import os, re
import sys
import optparse
import time
import thread

VERBOSE=0

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)


# DFU Opcodes
class Commands:
    START_DFU                    = 1
    INITIALIZE_DFU               = 2
    RECEIVE_FIRMWARE_IMAGE       = 3
    VALIDATE_FIRMWARE_IMAGE      = 4
    ACTIVATE_FIRMWARE_AND_RESET  = 5
    SYSTEM_RESET                 = 6
    PKT_RCPT_NOTIF_REQ           = 8

# DFU Procedures values
DFU_proc_to_str = {
    "01" : "START",
    "02" : "INIT",
    "03" : "RECEIVE_APP",
    "04" : "VALIDATE",
    "08" : "PKT_RCPT_REQ",
}

# DFU Operations values
DFU_oper_to_str = {
    "01" : "START_DFU",
    "02" : "RECEIVE_INIT",
    "03" : "RECEIVE_FW",
    "04" : "VALIDATE",
    "05" : "ACTIVATE_N_RESET",
    "06" : "SYS_RESET",
    "07" : "IMAGE_SIZE_REQ",
    "08" : "PKT_RCPT_REQ",
    "10" : "RESPONSE",
    "11" : "PKT_RCPT_NOTIF",
}

# DFU Status values
DFU_status_to_str = {
    "01" : "SUCCESS",
    "02" : "INVALID_STATE",
    "03" : "NOT_SUPPORTED",
    "04" : "DATA_SIZE",
    "05" : "CRC_ERROR",
    "06" : "OPER_FAILED",
}

class UUID:
    CCCD 				= "00002902-0000-1000-8000-00805f9b34fb"
    DFU_Control_Point 	= "00001531-1212-efde-1523-785feabcd123"
    DFU_Packet			= "00001532-1212-efde-1523-785feabcd123"
    DFU_Version			= "00001534-1212-efde-1523-785feabcd123"

"""
------------------------------------------------------------------------------
 Convert a number into an array of 4 bytes (LSB).
 This has been modified to prepend 8 zero bytes per the new DFU spec.
------------------------------------------------------------------------------
"""
def convert_uint32_to_array(value):
    return [0,0,0,0,0,0,0,0,
           (value >> 0  & 0xFF),
           (value >> 8  & 0xFF),
           (value >> 16 & 0xFF),
           (value >> 24 & 0xFF)
    ]

"""
------------------------------------------------------------------------------
 Convert a number into an array of 2 bytes (LSB).
------------------------------------------------------------------------------
"""
def convert_uint16_to_array(value):
    return [
        (value >> 0 & 0xFF),
        (value >> 8 & 0xFF)
    ]

"""
------------------------------------------------------------------------------

------------------------------------------------------------------------------
"""
def convert_array_to_hex_string(arr):
    hex_str = ""
    for val in arr:
        if val > 255:
            raise Exception("Value is greater than it is possible to represent with one byte")
        hex_str += "%02x" % val

    value = binascii.a2b_hex(hex_str)
    value = str(value)
    return value
    
    

def debug_msg(message):
	"""
	Print messages in verbose mode
	"""
	if VERBOSE:
		print "[DEBUG]" + message



def getDFUControlPointHandle(ble_connection, uuid):
    dfu_service = None
    try:
        dfu_service = ble_connection.getServiceByUUID("00001530-1212-efde-1523-785feabcd123")
    except Exception, e:
	self.write.emit("[Error]: DFU service was not found on this device!")
	return 0, 0		    

    characteristics = dfu_service.getCharacteristics()
    for char in characteristics:
	if char.uuid == "00001531-1212-efde-1523-785feabcd123":
	    return char.getHandle() + 1, char.getHandle()

    self.write.emit("[Error]: DFU service was not found on this device!")   
    return 0, 0 	    

        
def getHandle(ble_connection, uuid):
    in_characteristic = True
    
    descriptors = ble_connection.getDescriptors()

    for desc in descriptors:
	if(desc.uuid == uuid):
	    return desc.handle
    return False



"""
------------------------------------------------------------------------------
Callback class to get scan information
------------------------------------------------------------------------------
"""
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

    def handleDiscovery(self, dev, isNewDev, isNewData):
	pass



"""
------------------------------------------------------------------------------
Callback class to receive notification
------------------------------------------------------------------------------
"""
class MyDelegate(DefaultDelegate):
    instance = None
    def __init__(self, instance):
	DefaultDelegate.__init__(self)
	self.instance = instance

    def handleNotification(self, cHandle, data):
	if self.instance is not None:
	    method = getattr(self.instance, 'setNotification') 
	    method(data)



class Ui_MainWindow(QtCore.QObject):

    bin_file = ""
    dat_file = ""
    hex_file = ""
    zip_file = ""
    address = ""

    ctrlpt_handle      = 0x10
    ctrlpt_cccd_handle = 0x11
    data_handle        = 0x0e
    reset_handle      = 0x13
    ctrlpt_cccd_handle_buttonless  = 0x14

    #TODO Check this parameter to speed up the rprocedure
    pkt_receipt_interval = 10 #DEFAULT=10
    pkt_payload_size     = 20 #DEFAULT=20

    dfu_in_progress = False
    value_written_success_msg='Characteristic value was written successfully'
    value_written_success_msg_alt='.* Characteristic value was written successfully'
    updated = pyqtSignal(int)
    write = pyqtSignal(str)



    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
	MainWindow.setFixedSize(800, 600)

        self.centralwidget = QtGui.QWidget(MainWindow)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))

        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(0, 0, 301, 71))
        font = QtGui.QFont()
        font.setPointSize(32)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))

        self.pushButton = QtGui.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(150, 120, 98, 27))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
	self.pushButton.clicked.connect(self.scanClicked)

        self.listWidget = QtGui.QListWidget(self.centralwidget)
        self.listWidget.setGeometry(QtCore.QRect(0, 160, 251, 381))
        self.listWidget.setObjectName(_fromUtf8("listWidget"))
	self.listWidget.itemClicked.connect(self.start)
	self.listWidget.setStyleSheet("QListWidget::item::hover {background: lightgray} QListWidget::item::selected {background: lightblue; border: 1px solid blue}")


        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(0, 110, 111, 31))
        font = QtGui.QFont()
        font.setPointSize(21)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setObjectName(_fromUtf8("label_2"))

        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(620, 100, 171, 51))
        self.label_3.setObjectName(_fromUtf8("label_3"))

        self.pushButton_2 = QtGui.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(510, 120, 98, 27))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))
	self.pushButton_2.clicked.connect(self.selectFile)

        self.textEdit = QtGui.QTextEdit(self.centralwidget)
        self.textEdit.setEnabled(True)
        self.textEdit.setGeometry(QtCore.QRect(500, 160, 271, 381))
        self.textEdit.setObjectName(_fromUtf8("textEdit"))
	self.textEdit.setReadOnly(True)

        self.progressBar = QtGui.QProgressBar(self.centralwidget)
        self.progressBar.setGeometry(QtCore.QRect(0, 570, 800, 23))
        self.progressBar.setProperty("value", 0)
	self.progressBar.setTextVisible(False)
        self.progressBar.setObjectName(_fromUtf8("progressBar"))

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

	self.delegate = MyDelegate(self)

	self.updated.connect(self.updateProgressBar)
	self.write.connect(self.log)


    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "nRF OTA BLE", None))
        self.pushButton.setText(_translate("MainWindow", "Scan", None))
        self.label_2.setText(_translate("MainWindow", "Devices", None))
        self.label_3.setText(_translate("MainWindow", "Selected file:", None))
        self.pushButton_2.setText(_translate("MainWindow", "Select File", None))


    def updateProgressBar(self, value):
	self.progressBar.setProperty("value", value)


    #Callback called when Scan button is pressed
    def scanClicked(self):
	self.listWidget.clear()
	self.textEdit.clear()
	#disable push button
	#self.pushButton.(False)
	self.log("Scanning...")
	self.updateProgressBar(0)
	try:
	    #Start scanning in another thread so that GUI can be reactive in the meantime
	    thread.start_new_thread(self.scan, ())
	except Exception, e:
	    print e
	    self.log("[Error]: Error during scan");
	

    def scan(self):

	scanner = Scanner().withDelegate(ScanDelegate())
	try:
	    devices = scanner.scan(5.0)
	except:
	    self.wite.emit("[Error]: Error during scan")

	for dev in devices:
	    label = ""
	    for (adtype, desc, value) in dev.getScanData():
		if desc == "Complete Local Name":
		    label = value


	    item = QtGui.QListWidgetItem(label + "\n" + dev.addr + "\n")
	    self.listWidget.addItem(item)
	self.write.emit("Scanning complete.")
	    

    def log(self, message):
	self.textEdit.append(_fromUtf8(message))
	self.textEdit.update()

    #Callback called when Select File button is pressed
    def selectFile(self):
	dlg = QFileDialog()
	dlg.setFileMode(QFileDialog.ExistingFiles)
	dlg.setFilters(["Uploadable File (*.zip *.hex *.bin *.dat)"])

	if dlg.exec_():
	    self.zip_file = ""
	    self.hex_file = ""
	    self.bin_file = ""
	    self.dat_file = ""

	    self.textEdit.clear()

	    filenames = dlg.selectedFiles()
	    for name in filenames:
		if name[-4:] == ".zip":
		    self.zip_file = name
		if name[-4:] == ".hex":
		    self.hex_file = name
		if name[-4:] == ".bin":
		    self.bin_file = name
		if name[-4:] == ".dat":
		    self.dat_file = name 
	    if (self.zip_file != "" and self.bin_file != "") or (self.zip_file != "" and self.hex_file != "") or (self.bin_file != "" and self.hex_file != ""):
	        self.log("[Error]: Please select just one .zip, .hex, .bin file or both .bin and .dat files!")
	        self.zip_file = ""
	        self.hex_file = ""
	        self.bin_file = ""
	        self.dat_file = ""
		return
	    if self.zip_file != "":
		self.unzip(self.zip_file)
		zip_name = self.zip_file.split("/")
		self.label_3.setText("Selected File:\n" + zip_name[len(zip_name)-1])

		return

	    if self.bin_file != "" and self.dat_file != "":
		bin_name = self.bin_file.split("/")
		dat_name = self.dat_file.split("/")
		self.label_3.setText("Selected File:\n" + bin_name[len(bin_name)-1] + "\n" + dat_name[len(dat_name)-1])
		self.bin_file = str(self.bin_file)
		self.dat_file = str(self.dat_file)
		return

	    if self.hex_file != "":
		# start nrfutil to create the package 
		hex_name = self.hex_file.split("/")

		#remove old generated file if present
		try:
		    os.remove("out.zip")
		except:
		    pass
		
		os.system("nrfutil dfu genpkg out.zip --application " + str(hex_name[len(hex_name)-1]))
		time.sleep(2)
		zip_file = "out.zip"
		self.log("Starting nrfutil to generate the package...")
		if not os.path.isfile(zip_file):
		    self.log("[Error]: nrfutil failed to create the package. Please try again.")
		    self.zip_file = ""
		    self.bin_file = ""
		    self.dat_file = ""
		    self.hex_file = ""
		    return

		self.label_3.setText("Selected File:\n" + hex_name[len(hex_name)-1])
		self.unzip(zip_file)
		return

	    if self.bin_file != "":
		# start nrfutil to create the package
		bin_name = self.bin_file.split("/")

		#remove old generated file if present
		try:
		    os.remove("out.zip")
		except:
		    pass

 
		os.system("nrfutil dfu genpkg out.zip --application " + str(bin_name[len(bin_name)-1]))
		time.sleep(2)
		zip_file = "out.zip"
		self.log("Starting nrfutil to generate the package...")
		if not os.path.isfile(zip_file):
		    self.log("[Error]: nrfutil failed to create the package. Please try again.")
		    self.zip_file = ""
		    self.bin_file = ""
		    self.dat_file = ""
		    self.hex_file = ""
		    return
		
		self.log("Package created!")
		self.label_3.setText("Selected File:\n" + bin_name[len(bin_name)-1])
		self.unzip(zip_file)
		return


    def unzip(self, zip_file):
	unpacker = Unpacker()
	try:
            self.bin_file, self.dat_file = unpacker.unpack_zipfile(str(zip_file))
	except Exception, e:
	    self.log("[Error]: Error while parsing .zip file. Does it contain .bin (or .hex) and .dat files?")
	    self.zip_file = ""
	    self.bin_file = ""
	    self.dat_file = ""
	    self.hex_file = ""
	    return


    #Callback called when user selects a device from the list. Start DFU
    def start(self):
	if self.bin_file == "" and self.dat_file == "":
	    self.log("[Error]: Please first select a valid file to be uploaded.")
	    self.listWidget.currentItem().setSelected(False)
	    return

	item = self.listWidget.currentItem()
	self.address = str(item.text()).split("\n")[1]
	self.address = self.address.upper()
	self.device = str(item.text()).split("\n")[0] + " " + str(item.text()).split("\n")[1]

	if not self.dfu_in_progress:
	    try:
		thread.start_new_thread(self.dfuProcess, ())
	    except:
		self.log("\n[ERROR]: Failed to start firmware update. Please try again.")

    def dfuProcess(self):
	
	self.dfu_in_progress = True
	''' Start of Device Firmware Update processing '''
        self.write.emit("Connecting to " + self.address + "...")

        # Initialize inputs
        self.input_setup()

        # Connect to peer device.
        if self.scan_and_connect():

	    self.write.emit("Device " + self.device + " connected. Updating firmware...")

            # Transmit the hex image to peer device.
            self.dfu_send_image()

	    self.updated.emit(100)

            # Wait to receive the disconnect event from peripheral device.
            time.sleep(1)
    
            # Disconnect from peer device if not done already and clean up. 
            self.disconnect()

	    self.write.emit("Done.")

	    self.dfu_in_progress = False

    """
--------------------------------------------------------------------------
     Connect to peripheral device.
    --------------------------------------------------------------------------
    """
    def scan_and_connect(self):
   		
		debug_msg("scan_and_connect")

		try:
		    self.ble_conn = Peripheral(self.address, "random")
		except:
		    self.write.emit("[Error]: Failed connecting device " + self.address)
		self.ble_conn.setDelegate(self.delegate)
		debug_msg("connected!")

                self.ctrlpt_cccd_handle_buttonless, self.reset_handle = getDFUControlPointHandle(self.ble_conn, UUID.CCCD)
			
		debug_msg("END scan_and_connect")
		
		return True        


    """
--------------------------------------------------------------------------
     Receive notification from the callback class.
    --------------------------------------------------------------------------
    """
    def setNotification(self, notify):
	self.notification = notify


    """
    --------------------------------------------------------------------------
     Parse notification status results
    --------------------------------------------------------------------------
    """
    def _dfu_parse_notify(self, notify):

        if len(notify) < 3:
            return None

	dfu_oper = hex(ord(notify[0]))
	dfu_oper = str(dfu_oper[2:])
	oper_str = DFU_oper_to_str[dfu_oper]
        debug_msg("_dfu_parse_notify:" + str(notify) + " dfu_oper:" + str(dfu_oper))
        if oper_str == "RESPONSE":
       	    proc = hex(ord(notify[1]))
	    proc = str(proc[2:])
	    dfu_process = ""
	    if len(proc) == 1:
		dfu_process = "0" + proc
	    else:
		dfu_process = proc

	    stat = hex(ord(notify[2]))
	    stat = str(stat[2:])
	    dfu_status = ""
	    if len(stat) == 1:
		dfu_status = "0" + stat
	    else:
		dfu_status = stat

            process_str = DFU_proc_to_str[dfu_process]
            status_str  = DFU_status_to_str[dfu_status]

            debug_msg(str("oper: {0}, proc: {1}, status: {2}".format(oper_str, process_str, status_str)))

            if oper_str == "RESPONSE" and status_str == "SUCCESS":
                return "OK"
            else:
                self.write.emit("[Error]: Notification error. Please retry.")
                sys.exit(1)
                return "FAIL"


        if oper_str == "PKT_RCPT_NOTIF":

            byte1 = ord(notify[4])
            byte2 = ord(notify[3])
            byte3 = ord(notify[2])
            byte4 = ord(notify[1])


            receipt = 0
            receipt = receipt + (byte1 << 24)
            receipt = receipt + (byte2 << 16)
            receipt = receipt + (byte3 << 8)
            receipt = receipt + (byte4 << 0)

	    #update progress bar
	    self.updated.emit((receipt * 100)/self.hex_size)

            return "OK"


    """
    --------------------------------------------------------------------------
     Send two bytes: command + option
    --------------------------------------------------------------------------
    """
    def _dfu_state_set(self, opcode):
		try:
		    opcode = hex(opcode)
		except e:
		    pass
		debug_msg(str('char-write-req 0x%04x %s' % (self.ctrlpt_handle, opcode)))
		if len(opcode) == 5:
		    send_opcode = opcode[0:2] + '0' + opcode[2:]

		else:
		    send_opcode = opcode 
     		self.ble_conn.writeCharacteristic(self.ctrlpt_handle, str(unichr(int(send_opcode[2:4], 16))) + str(unichr(int(send_opcode[4:6], 16))), True)


    #--------------------------------------------------------------------------
    # Send one byte: command
    #--------------------------------------------------------------------------
    def _dfu_state_set_byte(self, opcode):
	try:
	    opcode = hex(opcode)
	except e:
	    pass
	
	self.ble_conn.writeCharacteristic(self.ctrlpt_handle, str(unichr(int(opcode[2:4], 16))), True)
       
    #--------------------------------------------------------------------------
    # Send 3 bytes: PKT_RCPT_NOTIF_REQ with interval of 10 (0x0a)
    #--------------------------------------------------------------------------
    def _dfu_pkt_rcpt_notif_req(self):

        opcode = 0x080000
        opcode = opcode + (self.pkt_receipt_interval << 8)

	self.ble_conn.writeCharacteristic(self.ctrlpt_handle, str(unichr(0x08)) + str(unichr(0x0A)) + str(unichr(0x00)), True) 


    #--------------------------------------------------------------------------
    # Send an array of bytes: request mode
    #--------------------------------------------------------------------------
    def _dfu_data_send_req(self, data_arr):
        hex_str = convert_array_to_hex_string(data_arr)
        
	self.ble_conn.writeCharacteristic(self.data_handle, hex_str, True)
        	

    #--------------------------------------------------------------------------
    # Send an array of bytes: command mode
    #--------------------------------------------------------------------------
    def _dfu_data_send_cmd(self, data_arr):
        hex_str = convert_array_to_hex_string(data_arr)

	self.ble_conn.writeCharacteristic(self.data_handle, hex_str, True)

    #--------------------------------------------------------------------------
    # Enable DFU Control Point CCCD (Notifications)
    #--------------------------------------------------------------------------
    def _dfu_enable_cccd(self, alreadyDfuMode):
		handle=self.ctrlpt_cccd_handle
		if(alreadyDfuMode==False):
		   handle=self.ctrlpt_cccd_handle_buttonless
		
		debug_msg("_dfu_enable_cccd")
		cccd_enable_value_array_lsb = convert_uint16_to_array(0x0001)
		cccd_enable_value_hex_string = convert_array_to_hex_string(cccd_enable_value_array_lsb)
		command=str('char-write-req 0x%04x %s' % (handle, cccd_enable_value_hex_string))
		debug_msg(command)

		descs = self.ble_conn.getDescriptors()
		handle = None
		for desc in descs:
		    if desc.uuid == "00001531-1212-efde-1523-785feabcd123":
			handle = desc.handle + 1
			
		#self.ble_conn.writeCharacteristic(handle,  str(unichr(0x01)) + str(unichr(0x01)), True)
		try:
		    self.ble_conn.writeCharacteristic(handle,  cccd_enable_value_hex_string, True)
		except Exception, e:
		    return False
		return True
		

    #--------------------------------------------------------------------------
    # Send the Init info (*.dat file contents) to peripheral device.
    #--------------------------------------------------------------------------
    def _dfu_send_init(self):
        debug_msg("dfu_send_info")

        # Open the DAT file and create array of its contents
        bin_array = array('B', open(self.dat_file, 'rb').read())

        # Transmit Init info
        self._dfu_data_send_req(bin_array)



    #--------------------------------------------------------------------------
    # Initialize: 
    #    Hex: read and convert hexfile into bin_array 
    #    Bin: read binfile into bin_array
    #--------------------------------------------------------------------------
    def input_setup(self):

        if self.bin_file == None:
            raise Exception("input invalid")

        name, extent = os.path.splitext(self.bin_file)

        if extent == ".bin":
            self.bin_array = array('B', open(self.bin_file, 'rb').read())
            self.hex_size = len(self.bin_array)
            return

        if extent == ".hex":
            intelhex = IntelHex(self.hex_file)
            self.bin_array = intelhex.tobinarray()
            self.hex_size = len(self.bin_array)
            return

        raise Exception("input invalid")
    
    def _dfu_check_mode(self):
        
        self._dfu_get_handles()
        
        debug_msg("_dfu_check_mode")
        #look for DFU switch characteristic
		
        resetHandle = getHandle(self.ble_conn, UUID.DFU_Control_Point)  
                
        self.ctrlpt_cccd_handle=None
        
        if not resetHandle:
            # maybe it already is IN DFU mode
            self.ctrlpt_handle = getHandle(self.ble_conn, UUID.DFU_Control_Point)
            if not self.ctrlpt_handle:
                debug_msg("Not in DFU, nor has the toggle characteristic, aborting..")
                return False
        
        if resetHandle or self.ctrlpt_handle:
            if resetHandle:
                debug_msg("Switching device into DFU mode")
                debug_msg('char-write-cmd 0x%02s %02x' % (resetHandle, 1))
		self.ble_conn.writeCharacteristic(resetHandle, str(unichr(0x01)), True)
               # self.ble_conn.sendline('char-write-cmd 0x%02s %02x' % (resetHandle, 1))
                time.sleep(0.2)
        
                debug_msg("Node is being restarted")
                self.ble_conn.disconnect()
                time.sleep(0.2)
        
                # wait for restart
                time.sleep(5)
                debug_msg("Reconnecting...")
        

                # reconnect
                connected = self.scan_and_connect()
                        
                if not connected:
                    return False
        
                return self._dfu_check_mode()
            else:
                debug_msg("Node is in DFU mode")
            return True
        else:
        
            return False

    def _dfu_get_handles(self):
        #s132
        self.ctrlpt_cccd_handle = '10'
        self.data_handle = '0e'
        
        
        ctrlpt_cccd_handle = getHandle(self.ble_conn,"00002902-0000-1000-8000-00805f9b34fb")
        data_handle = getHandle(self.ble_conn,"00001532-1212-efde-1523-785feabcd123")
        
        if ctrlpt_cccd_handle:
            self.ctrlpt_cccd_handle = ctrlpt_cccd_handle
        if data_handle:
            self.data_handle = data_handle

    def switch_in_dfu_mode(self):
		"""
		Enable CCD to switch in DFU mode
		"""
		debug_msg("switch_in_dfu_mode")


		if(self._dfu_enable_cccd(False)==False): #Try this
			return False
		time.sleep(0.5)

		#TODO handle softdevice and bootloader upgrade
		#Reset the board in DFU mode. After reset the board will be disconnected
		debug_msg(str('char-write-req 0x%02x 0104' % (self.reset_handle))) #char-write-req 0x0013 0104
		self.ble_conn.writeCharacteristic(self.reset_handle, str(unichr(0x01)) + str(unichr(0x04)), True)
		self.ble_conn.disconnect()
		time.sleep(1)
#		self.ble_conn.sendline('char-write-req 0x%02x 0104' % (self.reset_handle))  #Reset
#		self.ble_conn.sendline('') #BR

		debug_msg("END switch_in_dfu_mode")
		#Reconnect the board.
		ret = self.scan_and_connect()
        
    def switch_in_dfu_mode_alt(self):
		debug_msg("switch_in_dfu_mode")
		# scan for characteristics:
		status = self._dfu_check_mode()
		if not status:
			return False

    """
    --------------------------------------------------------------------------
     Send the binary firmware image to peripheral device.
    --------------------------------------------------------------------------
    """
    def dfu_send_image(self):
		debug_msg("dfu_send_image")

		if not self._check_DFU_mode():
			self.switch_in_dfu_mode()

		debug_msg("Enable Notifications in DFU mode")
		self._dfu_enable_cccd(True)

		#TODO Handle softdevice and bootloader upgrade
		# Send 'START DFU' + Application Command
		self._dfu_state_set(0x0104)

		# Transmit binary image size
		hex_size_array_lsb = convert_uint32_to_array(len(self.bin_array))
		
		self._dfu_data_send_req(hex_size_array_lsb)
		debug_msg("Sending hex file size")

		received = self.ble_conn.waitForNotifications(10.0)
                        
		if not received:
		    raise Exception("Notification not received. Device didn't reply")
		else:
		    dfu_status = self._dfu_parse_notify(self.notification)
		    if dfu_status != "OK":
			raise Exception("bad notification status")
                
		#notify = self._dfu_wait_for_notify()
		
		# Send 'INIT DFU' Command
		self._dfu_state_set(0x0200)


		# Transmit the Init image (DAT).
		self._dfu_send_init()

		# Send 'INIT DFU' + Complete Command
		self._dfu_state_set(0x0201)

		# Send packet receipt notification interval (currently 10)
		self._dfu_pkt_rcpt_notif_req()

		# Send 'RECEIVE FIRMWARE IMAGE' command to set DFU in firmware receive state. 
		self._dfu_state_set_byte(Commands.RECEIVE_FIRMWARE_IMAGE)

		'''
		Send bin_array contents as as series of packets (burst mode).
		Each segment is pkt_payload_size bytes long.
		For every pkt_receipt_interval sends, wait for notification.
		'''
		segment_count = 1
		for i in range(0, self.hex_size, self.pkt_payload_size):

			segment = self.bin_array[i:i + self.pkt_payload_size]
			self._dfu_data_send_cmd(segment)


			if (segment_count % self.pkt_receipt_interval) == 0:
				#notify = self._dfu_wait_for_notify()
				#time.sleep(2)

				self.ble_conn.waitForNotifications(30.0)

				if self.notification == None:
					raise Exception("no notification received")

				dfu_status = self._dfu_parse_notify(self.notification)

				if dfu_status == None or dfu_status != "OK":
					raise Exception("bad notification status")

			segment_count += 1

		# Send Validate Command
		self._dfu_state_set_byte(Commands.VALIDATE_FIRMWARE_IMAGE)

		# Wait a bit for copy on the peer to be finished
		time.sleep(1)

		# Send Activate and Reset Command
		self._dfu_state_set_byte(Commands.ACTIVATE_FIRMWARE_AND_RESET)
		
		"""
		--------------------------------------------------------------------------
			Return True if is already in DFU mode
		--------------------------------------------------------------------------
		"""
    def _check_DFU_mode(self):
		res = False
		DFUversion_char = self.ble_conn.getCharacteristics(uuid="00001534-1212-efde-1523-785feabcd123")[0]
		if DFUversion_char != None:
		    version = DFUversion_char.read()
		    if ord(version[0]) == 8:
			res=True
		    elif ord(version[0]) == 1:
			pass
		return res
    """
    --------------------------------------------------------------------------
     Disconnect from peer device if not done already and clean up. 
    --------------------------------------------------------------------------
    """
    def disconnect(self):
        self.ble_conn.disconnect()



if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = QtGui.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

