#-*- coding:utf-8 -*-

import sys
import time
import numpy as np

import udp_comm
import cv2

PS1             = 0xC3	
PS2             = 0x51	
PS3             = 0xA1	
PS4             = 0xF8	
PE              = 0xC2	
SM_SET          = 0	
SM_GET          = 1	
SM_STREAM       = 2	
SM_ERROR        = 255	
BI_PC2GL        = 0x21	
BI_GL2PC        = 0x12	
STATE_INIT      = 0	
STATE_PS1       = 1	
STATE_PS2       = 2	
STATE_PS3       = 3	
STATE_PS4       = 4	
STATE_TL        = 5	
STATE_PAYLOAD   = 6	
STATE_CS        = 7	

class GL(object):	
    def connection_made(self, transport):	
        """Called when reader thread is started"""	
        self.transport = transport	
    def data_received(self, data):	
        """Called with snippets received from the serial port"""	
        for a in data:	
            self.AddPacketElement(a)	
        	
    def connection_lost(self, exc):	
        """\	
        Called when the serial port is closed or the reader loop terminated	
        otherwise.	
        """	
        self.transport = None	
        if isinstance(exc, Exception):	

            raise exc	
    #############################################################	
    #  Constructor and Deconstructor for GL Class	
    #############################################################	
    def __init__(self):	
        self.RecvPacketClear()	
        self.serial_num = ''	
        self.lidar_data = []	
        self.frame_data_in = []	
        	
        	
    #############################################################	
    #  Functions for Serial Comm	
    #############################################################	
    def read_cs_update(self, data):	
        self.read_cs = self.read_cs ^ (data & 0xff)	
    def read_cs_get(self):	
        return self.read_cs & 0xff	
    def read_cs_clear(self):	
        self.read_cs = 0	
    def write_cs_update(self, data):	
        self.write_cs = self.write_cs ^ (data & 0xff)	
    def write_cs_get(self):	
        return self.write_cs & 0xff	
    def write_cs_clear(self):	
        self.write_cs = 0	
    def write(self, data):	
        self.send_packet.append(data)	
        self.write_cs_update(data)	
    def write_PS(self):	
        PS = np.array([PS1, PS2, PS3, PS4])	
        for i in PS:	
            self.write(i)	
    def SendPacket(self, packet):	
        for data in packet:	
            self.transport.write(bytes(bytearray([data])))	
    def WritePacket(self, PI, PL, SM, CAT0, CAT1, DTn):	
        
    
        self.send_packet = []	
        self.write_cs_clear()	
        self.write_PS()	
        TL = DTn.shape[0] + 14	
        buff = TL & 0xff	
        self.write(buff)	
        buff = (TL >> 8) & 0xff	
        self.write(buff)	
        self.write(PI)	
        self.write(PL)	
        self.write(SM)	
        self.write(BI_PC2GL)	
        self.write(CAT0)	
        self.write(CAT1)	
        for a in DTn:	
            self.write(a)	
        self.write(PE)	
        self.write(self.write_cs_get())	
        
        self.SendPacket(self.send_packet)


    def RecvPacketClear(self):
        self.read_cs_clear()
        self.recv_state = STATE_INIT
        self.recv_packet = []


    def CheckPS(self, data):
        if self.recv_state==STATE_INIT and data==PS1:
            self.RecvPacketClear()
            self.read_cs_update(data)
            self.recv_state = STATE_PS1
            return
        elif self.recv_state==STATE_PS1 and data==PS2:
            self.read_cs_update(data)
            self.recv_state = STATE_PS2
            return
        elif self.recv_state==STATE_PS2 and data==PS3:
            self.read_cs_update(data)
            self.recv_state = STATE_PS3
            return
        elif self.recv_state==STATE_PS3 and data==PS4:
            self.read_cs_update(data)
            self.recv_state = STATE_PS4
            return
        self.recv_state = STATE_INIT


    def FrameData(self, recv_data, PI, PL, SM):
        if SM!=SM_STREAM or len(recv_data)==0:
            return
        
        #print('len:',len(recv_data),'type:',recv_data)

        if PI==0:
            self.lidar_data = []
            self.lidar_data.append(recv_data)
        elif PI==len(self.lidar_data):
            self.lidar_data.append(recv_data)
        else:
            self.lidar_data = []
            return

        if len(self.lidar_data)==PL:
            if len(self.lidar_data[0])<3:
                self.lidar_data = []
                return
             
            #print('len:',len(self.lidar_data),'lidar:',self.lidar_data)
            
            data = np.reshape(sum(self.lidar_data, []), (1, -1)).T
            #data.flatten()
            
            
            #print('data size:',data.shape,data)
            

            frame_data_size = int(data[0] & 0xff)
            frame_data_size = frame_data_size | int(((data[1]&0xff)<<8))

            if len(data)!=(frame_data_size*4+22):
                self.lidar_data = []
                return

            frame_data = []
            dist_array = np.zeros((frame_data_size,1), dtype=float)
            pulse_array = np.zeros((frame_data_size,1), dtype=float)
            angle_array = np.zeros((frame_data_size,1), dtype=float)

            for i in range(frame_data_size):
                distance = data[i*4+2]&0xff
                distance = distance | ((data[i*4+3]&0xff)<<8)

                pulse_width = data[i*4+4]&0xff
                pulse_width = pulse_width | ((data[i*4+5]&0xff)<<8)

                if distance>30000:
                    distance = 0.0
                    
                dist_array[i] = distance/1000.0
                pulse_array[i] = pulse_width
                angle_array[i] = i*180.0/(frame_data_size-1)*3.141592/180.0

            frame_data.append(dist_array)
            frame_data.append(pulse_array)
            frame_data.append(angle_array)

            self.frame_data_in = frame_data
            self.lidar_data = []


    def SerialNum(self, recv_data, PI, PL, SM):
        if SM!=SM_GET or len(recv_data)==0:
            return

        self.serial_num = ''.join(chr(e) for e in recv_data)


    def ParsingData(self, recv_data, PI, PL, SM, BI, CAT0, CAT1):
        # print('')
        # print('Recv Data')
        # print('PI = ' + str(PI))
        # print('PL = ' + str(PL))
        # print('SM = ' + str(SM))
        # print('BI = ' + str(BI))
        # print('CAT0 = ' + str(CAT0))
        # print('CAT1 = ' + str(CAT1))
        # print('DTL = ' + str(len(recv_data)))

        if BI!=BI_GL2PC:
            return
        
        if CAT0==0x01 and CAT1==0x02:
            self.FrameData(recv_data, PI, PL, SM)
        elif CAT0==0x02 and CAT1==0x0A:
            self.SerialNum(recv_data, PI, PL, SM)


    def ParsingPayload(self, recv_packet):
        TL = self.recv_packet[0] & 0xff
        TL = TL | ((self.recv_packet[1]&0xff)<<8)

        PI = self.recv_packet[2] & 0xff
        PL = self.recv_packet[3] & 0xff
        SM = self.recv_packet[4] & 0xff
        BI = self.recv_packet[5] & 0xff
        CAT0 = self.recv_packet[6] & 0xff
        CAT1 = self.recv_packet[7] & 0xff

        recv_data = []
        for i in range(TL-14):
            recv_data.append(self.recv_packet[8+i])

        self.ParsingData(recv_data, PI, PL, SM, BI, CAT0, CAT1)


    def AddPacketElement(self, data):
        if self.recv_state==STATE_INIT or self.recv_state==STATE_PS1 or self.recv_state==STATE_PS2 or self.recv_state==STATE_PS3:
            self.CheckPS(data)
        elif self.recv_state==STATE_PS4:
            self.recv_state = STATE_TL
            self.recv_packet.append(data)
            self.read_cs_update(data)
        elif self.recv_state==STATE_TL:
            self.recv_state = STATE_PAYLOAD
            self.recv_packet.append(data)
            self.read_cs_update(data)
        elif self.recv_state==STATE_PAYLOAD:
            if len(self.recv_packet)>=8:
                recv_TL = self.recv_packet[0] & 0xff
                recv_TL = recv_TL | ((self.recv_packet[1]&0xff)<<8)

                if len(self.recv_packet)==(recv_TL - 6):
                    if data==PE:
                        self.recv_state = STATE_CS
                        self.read_cs_update(data)
                    else:
                        self.recv_state = STATE_INIT
                else:
                    self.recv_packet.append(data)
                    self.read_cs_update(data)
            else:
                self.recv_packet.append(data)
                self.read_cs_update(data)
        elif self.recv_state==STATE_CS:
            if data==self.read_cs_get():
                self.ParsingPayload(self.recv_packet)
            self.RecvPacketClear()
            return

        if self.recv_state==STATE_INIT:
            packet = self.recv_packet
            self.RecvPacketClear()
            for v in packet:
                self.AddPacketElement(v)


    #############################################################
    #  Read GL Conditions
    #############################################################
    def GetSerialNum(self):
        PI = 0
        PL = 1
        SM = SM_GET
        CAT0 = 0x02
        CAT1 = 0x0A

        DTn = np.array([1])
        
        self.serial_num = ''
        for i in range(50):
            
            self.WritePacket(PI, PL, SM, CAT0, CAT1, DTn)
            time.sleep(0.1)

            if len(self.serial_num)>0:
                return self.serial_num
    
        return '[ERROR] Serial Number is not received'

    
    def ReadFrameData(self):
        if len(self.frame_data_in)!=3:
            return np.array([]), np.array([]), np.array([])

        frame_data = self.frame_data_in
        self.frame_data_in = []

        dist_array = frame_data[0]
        pulse_array = frame_data[1]
        angle_array = frame_data[2]

        # for i in range(len(dist_array)-1):
        #     if dist_array[i]>0.0 and dist_array[i+1]>0.0:
        #         diff = (dist_array[i] - dist_array[i+1]) / 2.0
        #         if diff>0.01*dist_array[i]:
        #             dist_array[i] = 0.0
        #         if diff<-0.01*dist_array[i]:
        #             dist_array[i] = 0.0
            
        return dist_array, pulse_array, angle_array


    #############################################################
    #  Set GL Conditions
    #############################################################
    def SetFrameDataEnable(self, framedata_enable):
        PI = 0
        PL = 1
        SM = SM_SET
        CAT0 = 0x1
        CAT1 = 0x3

        DTn = np.array([int(framedata_enable)])
        self.WritePacket(PI, PL, SM, CAT0, CAT1, DTn)


# main
if __name__ == '__main__':
    

    cv2.namedWindow('lidar_img')
    with udp_comm.UdpReaderThread(GL) as udp_gl:
        try:
            print('Start GL Python Driver')
            udp_gl.SetFrameDataEnable(False)
            print('Serial Num : ' + udp_gl.GetSerialNum())
            time.sleep(0.1)

            udp_gl.SetFrameDataEnable(True)
            time.sleep(0.1)

            width = 1280
            height = 720
            meter_to_pixel = 32.0/0.6

            start = time.time()
            

            while True:
                distance, pulse_width, angle = udp_gl.ReadFrameData()

                if distance.shape[0]>0 and pulse_width.shape[0]>0 and angle.shape[0]>0:
                    time_delta = time.time()-start
                    if time_delta != 0:
                        print(1/time_delta)
                    start = time.time()


                img_view = np.zeros((height,width,3), np.uint8)

                
                x = distance*np.cos(angle)
                y = distance*np.sin(angle)
                
                img_x = x*meter_to_pixel + height/2
                img_y = y*meter_to_pixel + width/2

                for i in range(len(distance)):
                    img_view[int(img_x[i]), int(img_y[i]), 2]= 255

                # print('loop hz:',1/(a-loop_time_prev).total_seconds())
                # loop_time_prev = a
                if len(distance):

                    cv2.imshow('lidar_img',img_view)
                    cv2.waitKey(1)
                    
                time.sleep(0.0125)
        except KeyboardInterrupt:
            udp_gl.SetFrameDataEnable(False)
            print('End GL Python Driver')
            sys.exit()

        
                
