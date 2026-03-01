using UnityEngine;
using System;
using System.Net.Sockets;
using System.Net;
using System.IO;
using System.Threading;
using System.Globalization;

public class Unity_Python_TCP : MonoBehaviour
{
    [Header("Ref Code")]
    public PiperIK ik;
    public Piper_Joint_Limit_Check checker;

    [Header("TCP")]
    int port = 9101; // 서버가 열릴 TCP 포트 번호
    TcpListener listener; // 서버 소켓 객체
    Thread acceptThread;
    volatile bool running; // 서버 실행 유무

    long seq = 0; // 송신 메시지 일련번호
    string latestJson = "{\"seq\":0,\"ts\":0.0,\"target_deg\":[0,0,0,0,0,0],\"current_deg\":[0,0,0,0,0,0],\"piper_jaw\":0}"; // 이전 송신 메시지

    // readonly는 런타임 중에 변경 불가
    readonly object cacheLock = new object(); // 스레드 간 데이터 보호용 락

    [Header("Unity Gripper")]
    // Unity 그리퍼의 개구 간격 기준점
    float unityClosedM = 0.000f;
    float unityOpenM = 0.035f * 2f;

    [Header("PiPER Gripper")]
    // PiPER 그리퍼 개구 간격 기준점
    float piper_jaw_min = 0f;
    float piper_jaw_max = 80080f;

    bool latestFlag = false;
    int joint0_check = 0;

    void Start()
    {
        running = true;

        // TCP 서버 시작
        listener = new TcpListener(IPAddress.Any, port);
        listener.Start();

        // 별도의 스레드에서 AcceptLoop 실행
        acceptThread = new Thread(AcceptLoop) { IsBackground = true };
        acceptThread.Start();

        Debug.Log($"[Unity_Python_TCP] TCP Connected! {port}");
    }

    void FixedUpdate()
    {
        if (ik == null) return;

        var inv = CultureInfo.InvariantCulture; // 소수점 구분자 '.' 고정
        
        // 로봇팔 IK 정보 가져오기
        float[] target_deg = ik.GetJointAnglesDeg(); // 목표 각도
        float[] current_deg = ik.GetCurrentAnglesDeg(); // 현재 각도

        if (target_deg == null || target_deg.Length < 6) return;
        if (current_deg == null || current_deg.Length < 6) return;

        // Nan, Inf 방지
        for (int i = 0; i < 6; i++)
        {
            if (!float.IsFinite(target_deg[i])) target_deg[i] = 0f;
            if (!float.IsFinite(current_deg[i])) current_deg[i] = 0f;
        }

        // 그리퍼 개구 간격 정보 가져오기
        float gripper_gap = ik.GetGripperPosition(); // 그리퍼 개구 간격 (단위: m)

        float jawOpenRatio = Mathf.InverseLerp(unityClosedM, unityOpenM, gripper_gap); // 그리퍼가 열린 정도를 0~1 사이 값으로 정규화

        int piper_jaw = Mathf.RoundToInt(Mathf.Lerp(piper_jaw_min, piper_jaw_max, jawOpenRatio)); // piper 기준으로 변경
        //float piper_jaw_m = piper_jaw / 1_000_000f; // 미터 단위로 변경

        latestFlag = (checker != null) && checker.jointLimitAndFar;
        joint0_check = (checker != null) ? checker.joint0_turn : 0;

        seq++; // 메시지 일련번호 증가

        // string.Join : 배열의 각 요소를 지정한 구분자로 연결하여 하나의 문자열로 만듦
        // Array.ConvertAll : 배열의 각 요소에 지정한 변환 함수를 적용하여 새로운 배열을 만듦
        // x.ToString("F4", inv) : 소수점 4자리까지 표현, inv는 CultureInfo.InvariantCulture (소수점 구분자 '.')
        string ts = Time.realtimeSinceStartupAsDouble.ToString("F6", inv); // 타임스탬프 (초 단위, 소수점 6자리)
        string jt = string.Join(",", Array.ConvertAll(target_deg, x => x.ToString("F4", inv))); // 목표 각도
        string jm = string.Join(",", Array.ConvertAll(current_deg, x => x.ToString("F4", inv))); // 현재 각도

        string json = $"{{\"seq\":{seq},\"ts\":{ts},\"target_deg\":[{jt}],\"current_deg\":[{jm}],\"piper_jaw\":{piper_jaw}}}";

        lock (cacheLock) latestJson = json; // 최신 스냅샷 갱신
    }

    void AcceptLoop()
    {
        try
        {
            while (running)
            {
                TcpClient c = listener.AcceptTcpClient(); // 클라이언트가 서버에 접속할 때까지 기다렸다가 수락하는 역할
                c.NoDelay = true; // Nagle 알고리즘 비활성화 (지연 시간 감소)
                Thread t = new Thread(() => Handle(c)) { IsBackground = true };
                t.Start();
            }
        }
        catch (SocketException e) { Debug.Log(e); }
        catch (Exception e) { Debug.LogWarning(e); }
    }

    void Handle(TcpClient c)
    {
        using (var ns = c.GetStream()) // 양방향 바이트 스트림
        using (var r = new StreamReader(ns)) // 문자 스트림 리더
        using (var w = new StreamWriter(ns) { AutoFlush = true }) // 문자 스트림 라이터
        {
            try
            {
                string line;
                while ((line = r.ReadLine()) != null && running)
                {
                    // get 명령을 받으면 최신 JSON 스냅샷 전송
                    if (line == "get")
                    {
                        string snapshot;
                        lock (cacheLock) snapshot = latestJson;
                        w.WriteLine(snapshot);
                    }
                    // flag 명령을 받으면 관절 한계 및 거리도달 여부 전송
                    else if (line == "flag")
                    {
                        bool flag = latestFlag;
                        int turn = joint0_check;
                        w.WriteLine($"{(flag ? "true" : "false")},{turn}");
                    }
                    else
                    {
                        w.WriteLine("Error : unknown_command!");
                    }
                }
            }
            catch (Exception e) { Debug.LogError(e); }
            finally
            {
                try { c.Close(); }
                catch (Exception e) { Debug.LogWarning(e); }
            }
        }
    }

    void OnDestroy()
    {
        Shutdown();
    }

    void OnApplicationQuit()
    {
        Shutdown();
    }

    void Shutdown()
    {
        running = false;
        try { listener?.Stop(); } catch { }
        try { acceptThread?.Join(100); } catch (Exception e) { Debug.LogWarning(e); }
    }
}
