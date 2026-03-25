'''
Robot communication dispatch layer,
The module sets the serial port rules, 
and analyzes the communication data 
between the onboard computer and MCU.
机器人通信调度层，模块设置串口规则，并分析通信数据，在车载计算机和 MCU 之间。
'''

import queue
import struct
import time
import threading
import serial
import rclpy


from bubble_protocol.protocol import *
from rclpy.node import Node
from std_msgs.msg import Int32
from rm_interfaces.msg import SerialReceiveData

RX_BUFFER_MAX_SIZE = 500
TX_BUFFER_MAX_SIZE = 500


class RobotSerial(serial.Serial, Node):
    def __init__(self, name, *, port="", baudrate=115200, timeout_T=0):
        '''UART device initialization
            设备初始化
        Parameters
        ------------
            port: `string`
                Com port path
            baudrate: `int`
                Serial port baudrate
            timeout_T: `int`
                Timeout

        Returns
        -----------
            `serial.Serial`
            Class return a serial port object.
        '''
        Node.__init__(self, "enemy_color")

        self.init_device(port, baudrate, timeout_T)  # 初始化时保存参数

        # self.RB_info_pub_ = self.create_publisher(Int32, "red_blue_info", 10)
        self.Imu_gimbal_pub_ = self.create_publisher(SerialReceiveData, "serial/receive", 10)
        self.init_device(port, baudrate, timeout_T)
        self.init_protocol(name)
        self.tx_thread = threading.Thread(target=self.run_tx_thread)
        self.tx_thread.daemon = True
        self.tx_thread.start()
        self.status = STATUS
        self.serial_done = False
        self.realtime_pub = dict()
        self.red_blue_msg = None
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        
        
        # 定时检测串口状态
        self.check_serial_timer = threading.Thread(target=self.check_serial_status)
        self.check_serial_timer.daemon = True
        self.check_serial_timer.start()
    
    def init_device(self, port: str, baudrate: int, timeout_T: int) -> None:
        super().__init__(port=port, baudrate=baudrate, timeout=timeout_T)
        # 保存参数到成员变量
        self.port_name = port
        self.baudrate = baudrate
        self.timeout_T = timeout_T
        self.rx_count = 0
        self.tx_count = 0
        self.reset_rx_buffer()
    
    def run_tx_thread(self):
        '''threading to continuously process the transmit buffer'''
        while True:
            try:
                self.process_tx_buffer()
            except Exception as e:
                self.get_logger().error(f"TX Thread error: {e}")
            time.sleep(0.01)

    def process_tx_buffer(self) -> None:
        '''process the transmit buffer and send data'''
        while not self.tx_buffer.empty():
            try:
                data_to_send = self.tx_buffer.get()
                self.send(data_to_send)
            except Exception as e:
                self.get_logger().error(f"Error sending data: {e}")
                
    def check_serial_status(self):
        '''Periodically check and recover the serial connection'''
        while True:
            try:
                if not self.is_open:
                    self.get_logger().warn("Serial port disconnected. Attempting to reconnect...")
                    self.init_device(self.port_name, self.baudrate, self.timeout_T)
            except Exception as e:
                self.get_logger().error(f"Error reconnecting serial port: {e}")
            time.sleep(3)

    def init_device(self, port: str, baudrate: int, timeout_T: int) -> None:
        '''UART device initialization parameters setting
            UART设备初始化参数设置
        Parameters
        ------------
            port: `str`
                Com port path
            baudrate: `int`
                Serial port baudrate
            timeout_T: `int`
                Timeout
        '''
        super().__init__(port=port, baudrate=baudrate, timeout=timeout_T)
        self.rx_count = 0
        self.tx_count = 0
        self.reset_rx_buffer()

    def init_protocol(self, name: str) -> None:
        '''Initialize protocol frame param used robot name.
            初始化协议帧参数使用的机器人名称。
        Parameters
        ------------
            name: `str`
                robot name
        '''
        BCP_TX_FRAME.d_addr = D_ADDR[name]
        BCP_TX_FRAME.INFO = bytearray([HEAD, D_ADDR[name]])
        print("BCP_FRAME set d_addr: {}".format(BCP_TX_FRAME.d_addr))
        print("BCP_FRAME set INFO: {}".format(BCP_TX_FRAME.INFO))
        print("now init {} robot completed!".format(name))
        self.tx_buffer = queue.Queue()
        self.rx_buffer = queue.Queue()

    def send(self, data: bytearray) -> None:
        '''Write string to UART.
            将字符串写入 UART。
        Parameters
        ------------
        data: `bytearray`
            The data stream to be send
        '''
        # print(data)
        self.write(data)
        self.tx_count += 1

    def reset_rx_buffer(self) -> None:
        '''Reset receive buffer
            重置接收缓冲区
        '''
        self.current_packet = BCP_FRAME()
        self.rx_status = 0
        self.rx_datalen = 0

    def rx_function(self) -> None:
        '''The data in the buffer is parsed periodically, 
            the function will put BCP freme to ``current_packet`` if data is parsed successfully.
            缓冲区中的数据被周期性地解析，如果数据解析成功，该函数会将 BCP freme 放入 ``current_packet``。
        '''
        try:
            rx_bytes = self.readall()
        except serial.SerialException as e:
            # 打印错误类型
            print("SerialException: {0}".format(e))
            print("device reports readiness to read but returned no data")
            try:
                self.close()
                self.open()
                self.flushInput()
            except serial.SerialException as e:
                print("SerialException: {0}".format(e))
            return
        except TypeError as e:
            print("disconnect occured")
            return

        #print([hex(i) for i in rx_bytes])
        for rx_byte in rx_bytes:
            if self.rx_status == 0:  # wait HEAD
                if rx_byte == HEAD:
                    self.current_packet.setData(rx_byte)
                    self.rx_status = 1
            elif self.rx_status == 1:  # wait D_ADDR
                # for normal:
                if rx_byte == D_ADDR["mainfold"]:
                    # for debug:
                    # if rx_byte ==  D_ADDR["standard"]:
                    self.current_packet.setData(rx_byte)
                    self.rx_status = 2
                else:
                    self.reset_rx_buffer()
            elif self.rx_status == 2:  # wait ID
                self.current_packet.setData(rx_byte)
                self.rx_status = 3
            elif self.rx_status == 3:  # wait LEN
                self.current_packet.setData(rx_byte)
                self.rx_status = 4
            elif self.rx_status == 4:  # wait DATA
                self.current_packet.setData(rx_byte)
                self.rx_datalen += 1
                if self.rx_datalen >= self.current_packet.info[LEN_POSE]:
                    self.rx_status = 5
                    self.current_packet.combineCheck()
            elif self.rx_status == 5:  # wait SUM_CHECK
                if rx_byte == self.current_packet.info[SUM_CHECK_POSE]:
                    self.rx_status = 6
                else:  # check fail
                    #print("check fail")
                    self.rx_status = 6
                    self.reset_rx_buffer()
            elif self.rx_status == 6:  # wait ADD_CHECK
                if rx_byte == self.current_packet.info[ADD_CHECK_POSE]:
                    self.rx_buffer.put(copy.deepcopy(self.current_packet.info))
                    #print(self.current_packet)

                #debug
                self.rx_buffer.put(copy.deepcopy(self.current_packet.info))

                self.reset_rx_buffer()

    def onboard_data_analysis(self, current_packet: BCP_FRAME) -> None:
        '''
        Parameters
        ------------
        current_packet: `BCP_FRAME`
            Update robot status by successfully parsed BCP frame.
        '''
        def getFrameFmt(info_list: list) -> str:
            '''
            Parameters
            ------------
            info_list: `list`
                Gets the type of  BCP frame to parse.

            Returns
            -----------
            `str`
                 A format string that needs to be passed to ``struct`` moudle for parsing
            '''
            return "<"+"".join([info_list[key][IDX_BCP_TYPE] for key in info_list])
        
        def getFrameRatio(info_list):
            '''Gets the ratio of frame data
                获取帧数据的比例
            Parameters
            ------------
            info_list: `list`
                Gets the ratio of  BCP frame to parse.

            Returns
            -----------
            `list`
                A list of all parsed radio.
            '''
            return [info_list[key][IDX_BCPID_RATIO] for key in info_list]


        
        for key in STATUS:
            # print([hex(i) for i in current_packet])
            # unpack packet id
            if self.serial_done is False:
                return
            if current_packet[ID_POSE] == ID[key][IDX_BCPID]:
                data_info = ID[key][IDX_BCP_DETAIL]
                #-------------------------------------------------------------------
                #print("===POSE====", DATA_POSE, SUM_CHECK_POSE)
                #print("===data_info====", getFrameFmt(data_info))
                try:
                    unpack_info = struct.unpack(
                        getFrameFmt(data_info),
                        current_packet[DATA_POSE: SUM_CHECK_POSE]
                    )
                except struct.error as e:
                    self.get_logger().error(
                        f"Failed to unpack data: {e}. Current packet: {current_packet}")
                    return
                # ratio_list = getFrameRatio(data_info)
                # #print(unpack_info)
                # # if (int(unpack_info[4])/1000) == 1:
                # #     self.red_blue_msg = 0
                # # else:
                # self.red_blue_msg = int(int(unpack_info[4])/1000)
                # self.imu_yaw = int(unpack_info[1])/1000.0
                # self.imu_pitch = int(unpack_info[2])/1000.0
                # self.imu_roll = int(unpack_info[3])/1000.0


                # #print(self.red_blue_msg)
                # #set_param('/armor_detector','detect_color',int(unpack_info[4]/1000))#change 11.24
                # #set_parameter(rclpy.Parameter('detect_color', int(self.unpack_info[4] / 1000)))
                
                # res = list(
                #     map(lambda info, ratio: info/ratio, unpack_info, ratio_list))
                # for idx, detail_key in enumerate(self.status[key]):
                #     self.status[key][detail_key][IDX_VAL] = res[idx]
                # for pub_key in self.realtime_pub:
                #     if key == pub_key and self.realtime_pub[pub_key] is not None:
                #         self.realtime_pub[pub_key]()
                # # print(self.status["manifold_ctrl"],self.status["gimbal"])
                # return
                # 0315
                ratio_list = getFrameRatio(data_info)
                
                # =================== 修复点 ===================
                # 只有当这个包的元素大于等于 5 个时，才去强行读取 IMU 等数据
                # 这样就不会干扰 sentry_posture 等短包的正常解析了！
                if len(unpack_info) >= 5:
                    try:
                        self.red_blue_msg = int(int(unpack_info[4])/1000)
                        self.imu_yaw = int(unpack_info[1])/1000.0
                        self.imu_pitch = int(unpack_info[2])/1000.0
                        self.imu_roll = int(unpack_info[3])/1000.0
                    except Exception as e:
                        pass
                # ==============================================

                res = list(
                    map(lambda info, ratio: info/ratio, unpack_info, ratio_list))
                for idx, detail_key in enumerate(self.status[key]):
                    self.status[key][detail_key][IDX_VAL] = res[idx]
                    
                for pub_key in self.realtime_pub:
                    if key == pub_key and self.realtime_pub[pub_key] is not None:
                        self.realtime_pub[pub_key]()
                return
        print(f"Received a vaild frame, but not defined by robot status.\n\
                BCP core refused to process this frame data.\n\
                process time:{time.time()}, current packtet id:{current_packet[ID_POSE]}, \
                packtet detail: {[i for i in current_packet]}")
        return

    def red_blue_info_callback(self):
        if self.red_blue_msg is not None:
            msg = Int32()
            msg.data = self.red_blue_msg
            self.RB_info_pub_.publish(msg)
            #print(msg.data)
            #self.get_logger().info(f"enemy_color:{msg.data}")
        #else:
            #self.get_logger().warn("Red-blue message is None. No message published.")

    # 攫取C板的IMU數據
    def imu_gimbal_callback(self):
        msg = SerialReceiveData() # 统一数据格式
        msg.yaw = float(self.imu_yaw)
        msg.pitch = float(self.imu_pitch)
        msg.roll = float(self.imu_roll)
        self.Imu_gimbal_pub_.publish(msg)


    def process(self):
        '''Thread handing functions. 
        The buffer data is periodically taken out by timer 
        for updating robot status or sending to MCU.
            线程处理函数。
            缓存数据由定时器周期性取出，用于更新机器人状态或发送给 MCU。

        Parameters
        ------------

        Returns
        -----------
        '''
        rx_buffer_size = self.rx_buffer.qsize()
        #tx_buffer_size = self.tx_buffer.qsize()
        for _ in range(min(rx_buffer_size, RX_BUFFER_MAX_SIZE)):
            self.onboard_data_analysis(self.rx_buffer.get())
        #for _ in range(min(tx_buffer_size, TX_BUFFER_MAX_SIZE)):
            #self.send(self.tx_buffer.get())

    def setFrameData(self, frame: BCP_TX_FRAME, info: list, data: OrderedDict):
        frame_data_fmt = ""
        frame_data_list = []
        for index, key in enumerate(data):
            frame_data_fmt += data[key][IDX_BCP_TYPE]
            # print(info,data)

            frame_data_list.append(
                int(info[index] * data[key][IDX_BCPID_RATIO]))

        # cvt data to little-endian send  to uart
        # defined by protocol, send data are must a int numeber
        # cvt data to int before send
        res_data = struct.pack("<"+frame_data_fmt, *frame_data_list)
        # set LEN
        frame.setData(len(res_data))
        # set DATA
        for itm in res_data:
            frame.setData(itm)
        # set check data
        frame.combineCheck()

    def send_data(self, name: str, info: list):
        '''Convert data that needs to be send to MCU into BCP frame datas.
            将需要发送给MCU的数据转换成BCP帧数据。
        Parameters
        ------------
        name: `str`
            robot name.
        info: `list`
            A list of data to send.
        '''
        frame = BCP_TX_FRAME()
        detail = ID[name]
        # set ID
        frame.setData(detail[IDX_BCPID])
        # set frame data
        self.setFrameData(frame, info, detail[IDX_BCP_DETAIL])

        self.tx_buffer.put(frame.getData())
        # print("UART send data {}, now transmit count: {}".format([hex(i) for i in frame.getData()], self.tx_count))
        # print("UART send data {}, now transmit count: {}".format([i for i in frame.getData()], self.tx_count))

