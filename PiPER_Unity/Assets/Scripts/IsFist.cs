using UnityEngine;

public class IsFist : MonoBehaviour
{
    [Header("VR")]
    public OVRHand leftHand;
    public VR_Move vr;
    public Transform palmCenter;
    public Transform[] fingerTips = new Transform[4];

    [Header("Fist")]
    public bool isFist = false; // 주먹 동작 여부
    float hold_second = 3.0f; // 해당 초 이상 주먹 동작이면 주먹이라고 인식
    [HideInInspector] public float fist_last_time = 0f; // 주먹 동작 유지 시간
    float tipsDistance = 0.045f;

    void Update()
    {
        if (isFist) return;
        if (leftHand == null || !leftHand.IsTracked) return;

        bool curled = true;
        for (int i = 0; i < fingerTips.Length; i++)
        {
            var tip = fingerTips[i];
            if (!tip)
            {
                curled = false;
                break;
            }

            if (Vector3.Distance(tip.position, palmCenter.position) > tipsDistance)
            {
                curled = false;
                break;
            }
        }

        if (curled) fist_last_time += Time.deltaTime;
        else fist_last_time = 0f;

        if (fist_last_time >= hold_second) isFist = true;
    }
}