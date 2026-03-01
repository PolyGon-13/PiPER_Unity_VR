using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class PiperIK : MonoBehaviour
{
    [Header("Ref Code")]
    public VR_Move vr;

    [Header("Joint")]
    public ArticulationBody[] joint = new ArticulationBody[6];
    public ArticulationBody jawLeft;
    public ArticulationBody jawRight;
    float[] angle = new float[6];
    float limitMarginDeg = 2f;

    [Header("Target")]
    public GameObject target;
    public GameObject ee_Target;

    [Header("PID")]
    public float[] KpJoint = new float[6] { 1.2f, 1.1f, 1.0f, 0.8f, 0.7f, 0.6f };

    float?[] minAngle = new float?[6];
    float?[] maxAngle = new float?[6];

    float[] minAngleVelocity = new float[6];
    float[] maxAngleVelocity = new float[6];

    float d_tolerance = 0.01f;
    float r_tolerance = 0.5f;
    int maxIterations = 30;
    float[] preAngle = new float[6];

    float sigma = 0.5f;
    float mu = 0f;

    Vector3 prevEePos, prevTgtPos;
    Quaternion prevEeRot, prevTgtRot;
    double prevTime;

    // 목표 각도 반환
    public float[] GetJointAnglesDeg()
    {
        return (float[])angle.Clone();
    }

    // 현재 각도 반환
    public float[] GetCurrentAnglesDeg()
    {
        if (joint == null) return null;

        int n = joint.Length; // 관절 개수
        var current_deg = new float[n];

        for (int i = 0; i < n; i++)
        {
            var ab = joint[i];
            if (ab == null)
            {
                Debug.LogWarning($"[PiperIK] joint[{i}] 미할당");
                current_deg[i] = 0f;
                continue;
            }

            var jp = ab.jointPosition;
            if (jp.dofCount <= 0)
            {
                current_deg[i] = 0f;
                Debug.LogWarning($"[PiperIK] joint[{i}]의 자유도가 0");
                continue;
            }

            current_deg[i] = jp[0] * Mathf.Rad2Deg; // 라디안을 도 단위로 변환
        }
        return current_deg;
    }

    // 그리퍼의 개방폭 (단위: m)
    public float GetGripperPosition()
    {
        if (jawLeft == null && jawRight == null) return 0f;

        float Read_AB(ArticulationBody ab)
        {
            if (ab == null) return 0f;

            var d = ab.xDrive;
            if (float.IsFinite(d.target)) return d.target; // target 값이 유효한지 확인

            var jp = ab.jointPosition; // 현재 조인트의 상태를 읽음
            if (jp.dofCount > 0 && float.IsFinite((float)jp[0])) return (float)jp[0];
            // dofCount : 조인트의 자유도 수 (Revolute=1, Fixed=0, Spherical=3, Prismatic=1)
            // jp[0] : 조인트의 첫 번째 자유도 값
            Debug.LogWarning("[PiperIK] 그리퍼 조인트의 위치를 읽을 수 없습니다.");
            return 0f;
        }

        // 좌우 그리퍼의 위치를 읽음
        float l = Read_AB(jawLeft);
        float r = Read_AB(jawRight);

        float gap = Mathf.Abs(l - r);
        float unity_jaw_max_gap = 0.035f * 2f; // 유니티 그리퍼의 최대 개방폭

        if (gap < 0f) gap = 0f;
        if (unity_jaw_max_gap > 0f) gap = Mathf.Min(gap, unity_jaw_max_gap);

        return gap;
    }

    void Start()
    {
        if (ee_Target == null || target == null) Debug.LogError("[PiperIK] target이 할당되지 않았습니다.");

        if (KpJoint == null || KpJoint.Length !=6)
        {
            KpJoint = new float[6] { 2f, 2f, 2f, 2f, 2f, 2f };
            Debug.LogWarning("[PiperIK] KpJoint 미설정 → 기본값으로 설정");
        }

        for (int i = 0; i < joint.Length; i++)
        {
            var ab = joint[i];

            if (ab == null)
            {
                Debug.LogError($"[PiperIK] joint[{i}] 미할당 → IK 비활성화");
                return;
            }

            var j = ab.xDrive;
            angle[i] = j.target;
            preAngle[i] = j.target;

            if (j.lowerLimit < j.upperLimit)
            {
                minAngle[i] = j.lowerLimit;
                maxAngle[i] = j.upperLimit;
            }
            else
            {
                minAngle[i] = null;
                maxAngle[i] = null;
            }
        }

        SetLimitVelocity();

        prevEePos = ee_Target.transform.position;
        prevTgtPos = target.transform.position;
        prevEeRot = ee_Target.transform.rotation;
        prevTgtRot = target.transform.rotation;
        prevTime = Time.realtimeSinceStartupAsDouble;
    }

    void FixedUpdate()
    {
        if (ee_Target == null || target == null) return;

        if (target.CompareTag("Target"))
        {
            SolveIK();
        }
        else if (target.CompareTag("Target_VR"))
        {
            if (vr.vr_start)
                SolveIK();
        }
    }

    void SolveIK()
    {
        for (int iterations = 0; iterations < maxIterations; iterations++)
        {
            bool reachedTarget = true;

            Matrix<double> jacobian = CalculateJacobian();
            Vector<double> posErrForLambda;
            Vector<double> xdot_des = ComputeDesiredTwist(out posErrForLambda);
            Vector<double> jointVelocities = GDLSWithSVD_Twist(jacobian, xdot_des, posErrForLambda);
/*
            for (int i = 0; i < 6; i++)
            {
                float k = (i < KpJoint.Length) ? Mathf.Max(0f, KpJoint[i]) : 1f;
                jointVelocities[i] *= k;
            }
*/

            for (int i = 0; i < 6; i++)
            {
                float q = preAngle[i];

                if (minAngle[i].HasValue && q <= minAngle[i].Value + limitMarginDeg && jointVelocities[i] < 0.0)
                    jointVelocities[i] = 0.0;

                if (maxAngle[i].HasValue && q >= maxAngle[i].Value - limitMarginDeg && jointVelocities[i] > 0.0)
                    jointVelocities[i] = 0.0;
            }

            for (int i = 0; i < 6; i++)
            {
                float angleChange = (float)jointVelocities[i] * Time.fixedDeltaTime * Mathf.Rad2Deg; // 라디안을 도 단위로 변환
                angleChange = Mathf.Clamp(angleChange, minAngleVelocity[i] * Time.fixedDeltaTime, maxAngleVelocity[i] * Time.fixedDeltaTime); // 속도 제한

                float newAngle = preAngle[i] + angleChange; // 새로운 목표 각도 계산

                if (minAngle[i].HasValue && maxAngle[i].HasValue)
                    newAngle = Mathf.Clamp(newAngle, minAngle[i].Value, maxAngle[i].Value);

                var d = joint[i].xDrive;

                float maxDegPerSec = Mathf.Abs(maxAngleVelocity[i]);
                float smoothed = Mathf.MoveTowardsAngle(d.target, newAngle, maxDegPerSec * Time.fixedDeltaTime);
                d.target = smoothed;
                joint[i].xDrive = d;

                preAngle[i] = d.target;
                angle[i] = d.target;

                if (Vector3.Distance(ee_Target.transform.position, target.transform.position) >= d_tolerance ||
                    Quaternion.Angle(ee_Target.transform.rotation, target.transform.rotation) >= r_tolerance)
                {
                    reachedTarget = false;
                }
            }

            if (reachedTarget)
            {
                return;
            }
        }
    }

    // 자코비안 행렬 계산
    Matrix<double> CalculateJacobian()
    {
        Matrix<double> J = DenseMatrix.OfArray(new double[6, 6]);

        for (int i = 0; i < 6; i++)
        {
            var ab = joint[i];
            if (ab == null) continue;

            // 부모 프레임 기준 앵커/회전을 월드로
            Transform pt = ab.transform.parent != null ? ab.transform.parent : ab.transform;
            Vector3 jointPosition = pt.TransformPoint(ab.parentAnchorPosition);
            Quaternion parentRot  = pt.rotation * ab.parentAnchorRotation;

            // Unity Articulation에서 Revolute 축은 로컬 X
            Vector3 axis = (parentRot * Vector3.right).normalized;

            Vector3 r = ee_Target.transform.position - jointPosition;
            Vector3 lin = Vector3.Cross(axis, r);

            J[0,i]=lin.x; J[1,i]=lin.y; J[2,i]=lin.z;
            J[3,i]=axis.x; J[4,i]=axis.y; J[5,i]=axis.z;
        }
        return J;
    }

    // Target과 엔드이펙터의 위치 오차
    Vector<double> CalculatePositionError()
    {
        Vector3 positionError = target.transform.position - ee_Target.transform.position;
        return DenseVector.OfArray(new double[] { positionError.x, positionError.y, positionError.z, 0, 0, 0 });
    }

    // Target과 엔드이펙터의 회전 오차
    Vector<double> CalculateRotationError()
    {
        Quaternion targetRotation = target.transform.rotation;
        Quaternion currentRotation = ee_Target.transform.rotation;

        Quaternion rotationDiff = targetRotation * Quaternion.Inverse(currentRotation); // 두 회전의 상대회전
        Vector3 rotationError = rotationDiff.eulerAngles; // 오일러 각으로 변환

        // 오일러각은 0~360 범위이므로 -180~180 범위로 변환 (WrapAngle)
        rotationError.x = WrapAngle(rotationError.x);
        rotationError.y = WrapAngle(rotationError.y);
        rotationError.z = WrapAngle(rotationError.z);

        return DenseVector.OfArray(new double[] { 0, 0, 0, rotationError.x, rotationError.y, rotationError.z });
    }

    float WrapAngle(float angle)
    {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        else if (angle < -180) angle += 360;
        return angle;
    }

    Vector3 AngularVelocity(Quaternion q_prev, Quaternion q_curr, float dt)
    {
        Quaternion dq = q_curr * Quaternion.Inverse(q_prev);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (float.IsNaN(axis.x)) return Vector3.zero;
        float angleRad = angleDeg * Mathf.Deg2Rad;
        if (angleRad > Mathf.PI) angleRad -= 2f * Mathf.PI;
        return axis.normalized * (angleRad / Mathf.Max(dt, 1e-6f));
    }

    Vector<double> ComputeDesiredTwist(out Vector<double> posErrOut)
    {
        double now = Time.realtimeSinceStartupAsDouble; // 유니티 현재 시각
        float dt=Time.fixedDeltaTime;
        //float dt = (float)(now - prevTime); // 프레임 간 시간 간격
        //dt = Mathf.Max(dt, 1e-6f);

        Vector3 eePos = ee_Target.transform.position;
        Vector3 tgtPos = target.transform.position;
        Quaternion eeRot = ee_Target.transform.rotation;
        Quaternion tgtRot = target.transform.rotation;

        // 선속도/각속도 계산
        Vector3 v_ee = (eePos - prevEePos) / dt;
        Vector3 v_tgt = (tgtPos - prevTgtPos) / dt;
        Vector3 w_ee = AngularVelocity(prevEeRot, eeRot, dt);
        Vector3 w_tgt = AngularVelocity(prevTgtRot, tgtRot, dt);

        float Kp = 2.0f;
        float Kr = 1.0f;

        Vector3 p_err = tgtPos - eePos;

        Quaternion q_err = tgtRot * Quaternion.Inverse(eeRot); // EE 기준 상대 회전
        q_err.ToAngleAxis(out float angDeg, out Vector3 axis); // 상대 회전을 축,각으로 분해
        float angRad = (angDeg > 180f) ? (angDeg - 360f) * Mathf.Deg2Rad : angDeg * Mathf.Deg2Rad; // -180~180 범위로 변환
        Vector3 r_err = axis.normalized * angRad; // 회전 오차 벡터

        Vector3 v_des = v_tgt + Kp * p_err; // 목표 선속도
        Vector3 w_des = w_tgt + Kr * r_err; // 목표 각속도

        var xdot_des = DenseVector.OfArray(new double[] {
            v_des.x, v_des.y, v_des.z,  w_des.x, w_des.y, w_des.z
        }); // 목표 트위스트 벡터

        posErrOut = DenseVector.OfArray(new double[] { p_err.x, p_err.y, p_err.z, 0, 0, 0 }); // 위치 오차 벡터

        prevEePos = eePos;
        prevTgtPos = tgtPos;
        prevEeRot = eeRot;
        prevTgtRot = tgtRot;
        prevTime = now;

        return xdot_des;
    }

    Vector<double> GDLSWithSVD_Twist(Matrix<double> J, Vector<double> xdot_des, Vector<double> posErrForLambda)
    {
        double wPos = 1.0, wRot = 1.0;
        var W = DiagonalMatrix.OfDiagonal(6, 6, new double[] { wPos, wPos, wPos, wRot, wRot, wRot });
        var e = W * xdot_des;

        var svd = J.Svd();
        Matrix<double> U = svd.U;
        Vector<double> S = svd.S.Clone();
        Matrix<double> V = svd.VT.Transpose();

        double lambda = CalculateGaussianDampingFactor(posErrForLambda);

        for (int i = 0; i < S.Count; i++)
        {
            double si = S[i];
            S[i] = si / (si * si + lambda * lambda);
        }

        Matrix<double> dampedS = DiagonalMatrix.OfDiagonal(S.Count, S.Count, S);
        Matrix<double> J_pinv = V * dampedS * U.Transpose();

        return J_pinv * e;
    }

    Vector<double> GDLSWithSVD(Matrix<double> jacobian, Vector<double> positionError, Vector<double> rotationError)
    {
        double wPos = 1.0; // 위치 오차 가중치
        double wRot = 0.3; // 회전 오차 가중치

        double rx_rad = Mathf.Deg2Rad * (float)rotationError[3]; // roll
        double ry_rad = Mathf.Deg2Rad * (float)rotationError[4]; // pitch
        double rz_rad = Mathf.Deg2Rad * (float)rotationError[5]; // yaw

        var e = DenseVector.OfArray(new double[] {
            wPos * positionError[0],
            wPos * positionError[1],
            wPos * positionError[2],
            wRot * rx_rad,
            wRot * ry_rad,
            wRot * rz_rad
        }); // 가중치로 스케일 조정

        var svd = jacobian.Svd(); // 특이값 분해(SVD)
        Matrix<double> U = svd.U;
        Vector<double> S = svd.S.Clone();
        Matrix<double> V = svd.VT.Transpose();

        double lambda = CalculateGaussianDampingFactor(positionError); // 감쇠 계수 계산

        // 감쇠된 특이값 계산
        for (int i = 0; i < S.Count; i++)
        {
            double si = S[i];
            S[i] = si / (si * si + lambda * lambda);
        }

        Matrix<double> dampedS = DiagonalMatrix.OfDiagonal(S.Count, S.Count, S); // 대각 행렬 생성
        Matrix<double> dampedJacobian = V * dampedS * U.Transpose(); // 감쇠된 자코비안 행렬 계산
        Vector<double> dampedVelocity = dampedJacobian * e; // 감쇠된 관절 속도 계산

        return dampedVelocity;
    }

    double CalculateGaussianDampingFactor(Vector<double> positionError)
    {
        double errorMagnitude = positionError.SubVector(0, 3).L2Norm();
        double sigma = this.sigma;
        double mu = this.mu;

        double lambda = Mathf.Exp(-(float)(errorMagnitude - mu) * (float)(errorMagnitude - mu) / (2.0f * (float)sigma * (float)sigma));
        return lambda;
    }

    void SetLimitVelocity()
    {
        minAngleVelocity[0] = -30f;
        maxAngleVelocity[0] = 30f;

        minAngleVelocity[1] = -30f;
        maxAngleVelocity[1] = 30f;

        minAngleVelocity[2] = -30f;
        maxAngleVelocity[2] = 30f;

        minAngleVelocity[3] = -30f;
        maxAngleVelocity[3] = 30f;

        minAngleVelocity[4] = -30f;
        maxAngleVelocity[4] = 30f;

        minAngleVelocity[5] = -30f;
        maxAngleVelocity[5] = 30f;
    }
}
