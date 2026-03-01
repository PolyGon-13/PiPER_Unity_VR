using UnityEngine;
using System.Diagnostics;

public class Piper_Joint_Limit_Check : MonoBehaviour
{
    [Header("Objects")]
    public ArticulationBody joint0;
    public ArticulationBody joint1;
    public Transform ee_Target;
    public Transform vr_Target;

    [Header("Thresholds")]
    public float limitMarginDeg = 2f; // 관절 한계 여유각 (deg)
    public float distanceThreshold = 0.15f; // VR Hand와 EE Target 간 거리 임계값 (m)
    public float joint0ThresholdDeg = 5f;

    [Header("State")]
    public bool jointLimitAndFar = false;
    public int joint0_turn = 0;

    void Update()
    {
        if (joint0 == null || joint1 == null || ee_Target == null || vr_Target == null) return;

        float currentDeg = joint1.jointPosition[0] * Mathf.Rad2Deg; // 현재 관절 각도 읽기
        bool nearMax = Mathf.Abs(currentDeg - joint1.xDrive.upperLimit) <= limitMarginDeg; // 관절 최대 한계 근처

        float dist = Vector3.Distance(ee_Target.position, vr_Target.position); // EE Target과 VR Hand 간 거리
        bool tooFar = dist >= distanceThreshold;

        jointLimitAndFar = (nearMax && tooFar);

        if (jointLimitAndFar) UnityEngine.Debug.Log("[Piper_JointLimitCheck] Joint near limit and VR hand too far!");

        float joint0Deg = joint0.jointPosition[0] * Mathf.Rad2Deg;
        if (joint0Deg > joint0ThresholdDeg) // 왼쪽
            joint0_turn = 1;
        else if (joint0Deg < -joint0ThresholdDeg) // 오른쪽
            joint0_turn = 2;
        else
            joint0_turn = 0;
    }
}
