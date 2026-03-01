#!/usr/bin/env python3
import math
import time
import socket
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from unitree_go.msg import WirelessController

IP = "172.28.216.7"
PORT = 9101

class Go2_Move(Node):
    def __init__(self):
        super().__init__('go2_move')

        # TCP
        self.declare_parameter('unity_host', IP)
        self.declare_parameter('unity_port', PORT)
        self.declare_parameter('unity_timeout_s', 0.5)

        self.declare_parameter('unity_forward_norm', 0.1) # 유니티에 의한 전진값
        self.declare_parameter('unity_turn_speed', 0.1)
        self.declare_parameter('unity_invert_forward', False) # 전진/후진 반전여부

        self.declare_parameter('hz', 50.0) # 퍼블리시 주기
        #self.declare_parameter('mode', 0) # 조작 모드

        # 스케일 한계
        self.declare_parameter('max_lin', 1.0) # lx/ly 최대 절대값
        self.declare_parameter('max_ang', 1.0) # rx/ry 최대 절대값

        # manual 값들
        self.declare_parameter('lx', 0.0)
        self.declare_parameter('ly', 0.0)
        self.declare_parameter('rx', 0.0)
        self.declare_parameter('ry', 0.0)
        self.declare_parameter('keys', 0) # 조이스틱 입력 키값

        # pattern 파라미터
        self.declare_parameter('circle_amp', 0.6)
        self.declare_parameter('circle_omega', 0.4)
        self.declare_parameter('osc_amp', 0.8)
        self.declare_parameter('osc_omega', 0.6)
        self.declare_parameter('rot_speed', 0.5)

        # 안전 데드존/옵션
        self.declare_parameter('deadzone', 0.02) # 최종값의 절댓값이 이보다 작으면 0으로
        self.declare_parameter('no_reverse', False) # 후진 금지여부
        self.declare_parameter('no_strafe', False) # 측이동 금지여부

        self.add_on_set_parameters_callback(self.IsParamGood) # 파라미터 변경 시 유효성 검사

        self.pub = self.create_publisher(WirelessController, '/wirelesscontroller', 10)

        # 타이머
        hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / max(1.0, hz)
        self.t0 = time.time()
        self.timer = self.create_timer(self.dt, self._loop)

        self.joint0_turn = 0

        self.get_logger().info(f"[moving_well] started: hz={hz}")

    def IsParamGood(self, params):
        for p in params:
            if p.name in ('lx', 'ly', 'rx', 'ry'):
                try:
                    v = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason=f"{p.name} must be float in [-1,1]")
                if not (-1.0 <= v <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{p.name} must be in [-1,1]")
            if p.name in ('max_lin', 'max_ang'):
                if float(p.value) <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{p.name} must be > 0")
            if p.name == 'unity_forward_norm':
                v = float(p.value)
                if not (0.0 <= v <= 1.0):
                    return SetParametersResult(successful=False, reason="unity_forward_norm must be in [0,1]")
            if p.name == 'unity_turn_speed':
                v = float(p.value)
                if not (0.0 <= v <= 1.0):
                    return SetParametersResult(successful=False, reason="unity_turn_speed must be in [0,1]")
        return SetParametersResult(successful=True)

    def Unity_TCP(self):
        host = str(self.get_parameter('unity_host').value)
        port = int(self.get_parameter('unity_port').value)
        timeout_s = float(self.get_parameter('unity_timeout_s').value)
        try:
            with socket.create_connection((host, port), timeout=timeout_s) as s:
                s.sendall(b"flag\n")
                data = s.recv(4096)
                if not data:
                    return False
                txt = data.decode(errors='ignore').strip()
                parts = txt.split(',')
                flag = parts[0].lower() == 'true'
                if len(parts) > 1:
                    try:
                        self.joint0_turn = int(parts[1])
                    except ValueError:
                        self.joint0_turn = 0
                return flag
        except Exception as e:
            self.get_logger().debug(f"Unity TCP status query failed: {e}")
            self.joint0_turn = 0
            return False

    def _loop(self):
        now = time.time()
        t = now - self.t0 # 노드 시작 기준 경과 시간

        #mode = int(self.get_parameter('mode').value)
        max_lin = float(self.get_parameter('max_lin').value)
        max_ang = float(self.get_parameter('max_ang').value)
        deadzone = float(self.get_parameter('deadzone').value)
        no_rev = bool(self.get_parameter('no_reverse').value) # 후진 금지
        no_strafe = bool(self.get_parameter('no_strafe').value) # 측이동 금지
        lx = ly = rx = ry = 0.0
        keys = int(self.get_parameter('keys').value)
        unity_turn_speed = float(self.get_parameter('unity_turn_speed').value)

        flag = self.Unity_TCP()
        #self.get_logger().info(f"flag={flag}")

        if (flag):
            ly = float(self.get_parameter('unity_forward_norm').value)
            if bool(self.get_parameter('unity_invert_forward').value):
                ly = -ly

            if self.joint0_turn == 1: # 왼쪽
                lx = -unity_turn_speed
                ly = unity_turn_speed

            elif self.joint0_turn == 2: # 오른쪽
                lx = unity_turn_speed
                ly = unity_turn_speed
        else:
            lx = 0.0
            ly = 0.0

        if no_strafe: # 측이동 금지
            lx = 0.0
        if no_rev and ly < 0.0: # 후진 금지
            ly = 0.0

        # 클램프
        lx = self.clamp(lx, -1.0, 1.0)
        ly = self.clamp(ly, -1.0, 1.0)
        rx = self.clamp(rx, -1.0, 1.0)
        ry = self.clamp(ry, -1.0, 1.0)

        # 데드존
        lx = self.value_dz(lx, deadzone)
        ly = self.value_dz(ly, deadzone)
        rx = self.value_dz(rx, deadzone)
        ry = self.value_dz(ry, deadzone)

        # 퍼블리시
        msg = WirelessController()
        msg.lx = float(self.clamp(lx, -max_lin, max_lin))
        msg.ly = float(self.clamp(ly, -max_lin, max_lin))
        msg.rx = float(self.clamp(rx, -max_ang, max_ang))
        msg.ry = float(self.clamp(ry, -max_ang, max_ang))
        try:
            msg.keys = int(keys)
        except AttributeError:
            pass
        self.pub.publish(msg)

    @staticmethod
    def clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    @staticmethod
    def value_dz(v, d):
        # v가 d보다 작으면 0으로 처리
        if abs(v) < d:
            return 0.0
        else:
            return v

def main():
    rclpy.init()
    node = Go2_Move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: # 종료 시 정지 명령 퍼블리시
        stop_msg = WirelessController()
        stop_msg.lx = stop_msg.ly = stop_msg.rx = stop_msg.ry = 0.0
        stop_msg.keys = 0
        node.pub.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()