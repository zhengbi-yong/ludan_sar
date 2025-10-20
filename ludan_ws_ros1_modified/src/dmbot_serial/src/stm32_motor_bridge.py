#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import serial.tools.list_ports
import threading
import time
from std_msgs.msg import Float64MultiArray, Float32

FRAME_HEADER = 0x7B
PACKET_LENGTH = 11  # 1B头 + 1B id + 8B payload + 1B XOR

# ========= 新增：参数化 =========
NUM_MOTORS = rospy.get_param("/stm32_motor_bridge/num_motors", 14)
# ==============================

def xor_checksum(data_bytes):
    c = 0
    for b in data_bytes:
        c ^= b
    return c & 0xFF

def build_packet(motor_id, payload_8bytes):
    assert 0 <= motor_id < NUM_MOTORS, "motor_id 越界"
    if len(payload_8bytes) != 8:
        raise ValueError("payload 必须 8 字节")
    buf = bytearray(10)
    buf[0] = FRAME_HEADER
    buf[1] = motor_id
    buf[2:10] = payload_8bytes
    chksum = xor_checksum(buf)
    return bytes(buf) + bytes([chksum])

def clamp_int16(v): return max(-32768, min(32767, int(round(v))))

def encode_pd_q15(pos, vel, kp, kd, scale=1000, big_endian=True):
    # 注意：如果下位机忽略 vel，你也可以置 0
    p = clamp_int16(pos * scale)
    v = clamp_int16(vel * scale)
    kpp = clamp_int16(kp  * scale)
    kdd = clamp_int16(kd  * scale)
    vals = [p, v, kpp, kdd]
    b = bytearray(8)
    for i, val in enumerate(vals):
        if big_endian:
            b[2*i]   = (val >> 8) & 0xFF
            b[2*i+1] =  val       & 0xFF
        else:
            b[2*i]   =  val       & 0xFF
            b[2*i+1] = (val >> 8) & 0xFF
    return b

class Stm32Bridge(object):
    def __init__(self):
        self.port   = rospy.get_param("~port", "/dev/mcu_rightarm")
        self.baud   = int(rospy.get_param("~baud", 921600))       # 改成 921600
        self.rate_hz= float(rospy.get_param("~rate", 100.0))
        self.mode   = rospy.get_param("~mode", "pd_q15")
        self.scale  = float(rospy.get_param("~scale", 1000.0))
        self.big_endian = bool(rospy.get_param("~big_endian", True))

        # /hybrid_cmd 长度 = 5*NUM_MOTORS
        self.cmd = [0.0] * (5 * NUM_MOTORS)
        self.cmd_lock = threading.Lock()
        self.last_cmd_time = 0.0
        self.timeout = float(rospy.get_param("~timeout", 0.2))

        self.emergency_stop = False

        # 方向映射长度改成 NUM_MOTORS（按你实际填充）
        default_dir = [1]*NUM_MOTORS
        # 例：原来 10 路是 [ 1,1,-1,1,1, -1,1,-1,-1,-1 ]
        # 你可以在参数服务器覆盖，也可以这里直接写 14 路的表
        self.direction = rospy.get_param("~direction", default_dir)

        # raw8 的默认 payload
        self.raw_payload_default = [0x7F, 0xFF, 0x84, 0x30, 0x00, 0x33, 0x37, 0xFF]
        self.raw_payloads = [bytearray(self.raw_payload_default) for _ in range(NUM_MOTORS)]

        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        rospy.loginfo("stm32_motor_bridge 串口已打开 %s @ %d", self.port, self.baud)

        rospy.Subscriber("/hybrid_cmd", Float64MultiArray, self.hybrid_cb, queue_size=1)
        rospy.Subscriber("/emergency_stop", Float32, self.emg_cb, queue_size=1)

        # raw8 订阅也扩到 NUM_MOTORS
        for mid in range(NUM_MOTORS):
            rospy.Subscriber(f"/stm32/raw8/motor{mid}", Float64MultiArray,
                             lambda msg, i=mid: self.raw8_cb(i, msg), queue_size=1)

        self.run = True
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()

    def hybrid_cb(self, msg: Float64MultiArray):
        data = list(msg.data)
        need = 5 * NUM_MOTORS
        if len(data) != need:
            rospy.logwarn_throttle(1.0, "/hybrid_cmd 长度=%d（需 %d），忽略", len(data), need)
            return
        with self.cmd_lock:
            self.cmd = data
            self.last_cmd_time = time.time()

    def emg_cb(self, msg: Float32):
        self.emergency_stop = (msg.data > 0.5)
        rospy.logwarn("收到急停: %s", self.emergency_stop)

    def raw8_cb(self, motor_id, msg: Float64MultiArray):
        arr = msg.data
        if len(arr) != 8:
            rospy.logwarn("raw8 电机 %d 长度=%d(需8)，忽略", motor_id, len(arr))
            return
        b = bytearray(8)
        for i, v in enumerate(arr):
            b[i] = int(max(0, min(255, round(v))))
        self.raw_payloads[motor_id] = b

    def loop(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and self.run:
            now = time.time()
            if self.emergency_stop or (now - self.last_cmd_time) > self.timeout:
                rate.sleep()
                continue

            with self.cmd_lock:
                d = self.cmd[:]

            # 切片都按 NUM_MOTORS
            pos = d[0:NUM_MOTORS]
            vel = d[NUM_MOTORS:2*NUM_MOTORS]
            kp  = d[2*NUM_MOTORS:3*NUM_MOTORS]
            kd  = d[3*NUM_MOTORS:4*NUM_MOTORS]
            # tau = d[4*NUM_MOTORS:5*NUM_MOTORS]  # 如固件不用可忽略

            for mid in range(NUM_MOTORS):
                if self.mode == "raw8":
                    payload = self.raw_payloads[mid]
                else:
                    p = pos[mid] * self.direction[mid]
                    v = vel[mid] * self.direction[mid]
                    kpp = kp[mid]
                    kdd = kd[mid]
                    payload = encode_pd_q15(p, v, kpp, kdd, scale=self.scale, big_endian=self.big_endian)

                pkt = build_packet(mid, payload)
                try:
                    self.ser.write(pkt)
                except Exception as e:
                    rospy.logerr_throttle(1.0, "串口写失败: %s", e)
                    break

            rate.sleep()

    def shutdown(self):
        self.run = False
        try:
            self.th.join(timeout=1.0)
        except:
            pass
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("stm32_motor_bridge 已关闭")

if __name__ == "__main__":
    rospy.init_node("stm32_motor_bridge")
    try:
        node = Stm32Bridge()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except Exception as e:
        rospy.logfatal("stm32_motor_bridge 异常: %s", e)
