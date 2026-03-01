#!/usr/bin/env python3
import time
import socket
import json
import math
from piper_sdk import C_PiperInterface_V2

IP = "192.168.68.51"
PORT = 9101

piper_speed = 5
gripper_speed = 1000

def Connect_Unity(piper):
    while True:
        try:
            s = socket.create_connection((IP, PORT), timeout=5.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            f = s.makefile("rwb")
            print(f"[Unity] Connect to {IP}:{PORT}")

            while True:
                try:
                    data=Unity_Get_Date(f)
                    if data is None:
                        break
                    #print(f"[Unity] Received: {data}")
                    Piper_Joint_Move(piper,data)
                    Piper_Gripper_Move(piper,data)
                    time.sleep(0.05)
                except KeyboardInterrupt:
                    raise
                except Exception:
                    break
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"[Unity] {e}: Reconnect in 2s...")
            time.sleep(2.0)
            continue
        finally:
            try:
                if f is not None:
                    f.close()
            except:
                pass
            try:
                if s is not None:
                    s.close()
            except:
                pass

def Unity_Get_Date(f=None):
    try:
        f.write(b"get\n") # get 요청을 보냄
        f.flush()

        line = f.readline() # 한 줄 읽어옴
        if not line:
            return None

        msg = line.decode("utf-8").strip() # 수신한 바이트 데이터를 UTF-8 문자열로 변환 후 양쪽 공백/개행 제거
        if isinstance(msg,dict) and int(msg.get("seq",0))<=0:
            return None
        return json.loads(msg)
    except KeyboardInterrupt:
        raise
    except Exception:
        return None
    
def Piper_Joint_Move(piper, data):
    try:
        arr = None
        if isinstance(data, dict):
            if isinstance(data.get("target_deg"), list):
                arr = data.get("target_deg")
            elif isinstance(data.get("current_deg"), list):
                arr = data.get("current_deg")
        if not isinstance(arr, list) or len(arr) < 6:
            return False

        vals = [float(arr[i]) for i in range(6)]
        max_abs = max(abs(v) for v in vals)
        factor = 57295.7795 if max_abs <= 3.2 else 1000.0
        mdeg = [int(round(v * factor)) for v in vals]

        piper.JointCtrl(mdeg[0], mdeg[1], mdeg[2], mdeg[3], mdeg[4], mdeg[5])
        return True
    except Exception:
        return False
    
def Piper_Gripper_Move(piper, data):
    try:
        um = None
        if isinstance(data, dict):
            if "piper_jaw" in data:
                um = int(data["piper_jaw"])

        if um is None:
            return False
        if um<0:
            um=0

        piper.GripperCtrl(abs(um), gripper_speed, 0x01, 0x00)
        return True
    except Exception:
        return False

def Piper_Ready(piper):
    try:
        # MotionCtrl_1 : piper의 모드를 켜거나 끄는 신호 보낼 때 사용
        piper.MotionCtrl_1(grag_teach_ctrl=0x02) # 티칭모드 종료
        time.sleep(0.05)
        piper.MotionCtrl_1(grag_teach_ctrl=0x00) # 티칭모드 비활성화
        time.sleep(0.05)
        piper.MotionCtrl_1(track_ctrl=0x01) # 트랙모드 일시정지
        time.sleep(0.05)
        #piper.MotionCtrl_1(track_ctrl=0x06) # 트랙모드 종료
        #time.sleep(0.05)
        '''
        grag_teach_ctrl
        - 0x00 : 티칭모드 비활성화(OFF)
        - 0X01 : 티칭모드 진입(ON)
        - 0X02 : 티칭모드 종료(Exit)

        track_ctrl
        - 0x01 : 현재 트랙 일시정지
        - 0x02 : 트랙 실행 재개
        - 0x03 : 현재 트랙 삭제
        - 0x04 : 모든 트랙 삭제
        - 0x06 : 현재 트랙 종료
        '''
    except Exception:
        pass
    
    # 로봇팔 전원인가 + 구동 준비
    piper.EnableArm(0x07, 0x01) # 0x07 : 로봇팔 전체
    time.sleep(0.2)
    piper.EnableArm(0x07, 0x02)
    time.sleep(0.2)
    '''
    EnableArm
    [첫 번째 인자]
    - 0x01, 0x02, 0x04, 0x08, 0x10, 0x20 : joint 1~6
    - 0x40 : gripper
    - 0x07 : joint 전체 (gripper 제외)

    [두 번째 인자]
    - 0x01 : Enable 준비
    - 0x02 : Enable 완료
    '''
    Piper_Set_MotionCtrl_2(piper)
    time.sleep(0.1)

    piper.GripperCtrl(0,1000,0x02,0x00)
    time.sleep(0.05)
    piper.GripperCtrl(0,1000,0x01,0x00)
    time.sleep(0.05)

# 로봇팔과 그리퍼를 초기위치로 이동
def SetZeroPos(piper):
    try:
        piper.JointCtrl(0, 0, 0, 0, 0, 0)
        piper.GripperCtrl(0, gripper_speed, 0x01, 0)
        time.sleep(0.05)
    except Exception:
        pass

def Piper_Set_MotionCtrl_2(piper):
    # MotionCtrl_2 : piper의 메인 제어 모드를 설정
    piper.MotionCtrl_2(
        ctrl_mode=0x01, # 위치 제어
        move_mode=0x01, # 조인트 이동 모드
        is_mit_mode=0x00, # MIT 모드 비활성화
        move_spd_rate_ctrl=piper_speed # 이동 속도 (1~100)
    )
    time.sleep(0.05)
    '''
    ctrl_mode
    - 0x01 : 위치 제어
    - 0x02 : 속도 제어
    - 0x03 : 토크 제어

    move_mode
    - 0x01 : 조인트 이동 (move j)
    - 0x02 : Cartesian / Task-space 이동 (move l)
    - 0x03 : 원호 이동 (move c)

    is_mit_mode
    - 0x00 : 비활성화
    - 0x01 : 활성화

    move_spd_rate_ctrl
    이동 속도 (%단위, 1~100)
    내부적으로는 최대 속도를 기준으로 퍼센트 비율을 곱해서 사용
    '''

def main():
    piper = C_PiperInterface_V2("can1")
    #print("[PiPER] Connecting...")
    piper.ConnectPort()
    #print("[PiPER] Connect Successful")
    
    Piper_Ready(piper)
    #print("[PiPER] Ready Successful")

    try:
        Connect_Unity(piper)
    except KeyboardInterrupt:
        print("\n[PiPER] Ctrl+C Detected. Stopping...")
    finally:
        SetZeroPos(piper)

if __name__ == "__main__":
    main()
