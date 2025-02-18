using UnityEngine;
using System;
using System.Linq;
using System.Collections.Generic;

public class BasketballController : MonoBehaviour
{
    public Transform cameraTransform;
    public CameraController cameraController;
    public NetController netController;
    public ScoreDisplay scoreDisplay;

    // 篮球返回手中的最大间隔
    float gapTime = 4.0f;
    int currentScore = 2;
    float baseShootForce = 1.0f;
    Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    float dt = 0.05f;
    Vector3 initialPosition;
    bool isShooting = false;
    Vector3 v = new Vector3(0, 0, 0);
    Vector3 w = new Vector3(0, 0, 0);

    float mass;
    Matrix4x4 I_ref;

    float linear_decay = 0.999f;
    float angular_decay = 0.98f;
    float restitution = 0.5f;
    float friction = 0.2f;

    float verticalOffset = -1.0f; // 摄像机的垂直偏移量
    float distance = 2.0f; // 摄像机与篮球的距离
    float ballRadius;


    void Start()
    {
        initialPosition = transform.position;

        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        // 计算篮球半径
        float maxDistance = 0f;
        foreach (Vector3 vertex in vertices)
        {
            float d = vertex.magnitude;
            if (d > maxDistance)
            {
                maxDistance = d;
            }
        }
        // 考虑物体的缩放
        ballRadius = maxDistance * Mathf.Max(transform.localScale.x, transform.localScale.y, transform.localScale.z);

        float m = 1;
        mass = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            mass += m;
            float diag = m * vertices[i].sqrMagnitude; // diag = mv^2
            I_ref[0, 0] += diag;
            I_ref[1, 1] += diag;
            I_ref[2, 2] += diag;
            I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
            I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
            I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
            I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
            I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
            I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
            I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
            I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
            I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
        }
        I_ref[3, 3] = 1;

        int n = 20;
        Vector3[] X = new Vector3[n * n];
        int[] T = new int[(n - 1) * (n - 1) * 6];

        // 在开始时设置摄像机位置
        UpdateBasketballPosition();
    }

    Matrix4x4 Get_Cross_Matrix(Vector3 a)//得到向量a的叉乘矩阵
    {
        //Get the cross product matrix of vector a
        Matrix4x4 A = Matrix4x4.zero;
        A[0, 0] = 0;
        A[0, 1] = -a[2];
        A[0, 2] = a[1];
        A[1, 0] = a[2];
        A[1, 1] = 0;
        A[1, 2] = -a[0];
        A[2, 0] = -a[1];
        A[2, 1] = a[0];
        A[2, 2] = 0;
        A[3, 3] = 1;
        return A;
    }

    private Quaternion Add(Quaternion a, Quaternion b)
    {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
        a.w += b.w;
        return a;
    }

    private Matrix4x4 Matrix_subtraction(Matrix4x4 a, Matrix4x4 b)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                a[i, j] -= b[i, j];
            }
        }
        return a;
    }


    private Matrix4x4 Matrix_miltiply_float(Matrix4x4 a, float b)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                a[i, j] *= b;
            }
        }
        return a;
    }

    void Collision_Impulse(Vector3 P, Vector3 N, bool isNet, out Vector3 J)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;

        // 旋转矩阵
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);

        // 平移向量
        Vector3 T = transform.position;
        Vector3 sum = new Vector3(0, 0, 0);

        int collisionNum = 0;

        for(int i = 0; i < vertices.Length; i++)
        {
            Vector3 r_i = vertices[i];
            Vector3 Rri = R.MultiplyVector(r_i);
            Vector3 x_i = T + Rri;
            float d = Vector3.Dot(x_i - P, N);

            if (d < 0.0f)
            {
                Vector3 v_i = v + Vector3.Cross(w, Rri);
                float v_N_size = Vector3.Dot(v_i, N);
                if (v_N_size < 0.0f)
                {
                    sum += r_i;
                    collisionNum++;
                }
            }
        }
        if (collisionNum == 0)
        {
            J = Vector3.zero;
            return;
        }

        Matrix4x4 I_rot = R * I_ref * R.transpose;
        Matrix4x4 I_inverse = I_rot.inverse;
        Vector3 r_collision = sum / (float)collisionNum;
        Vector3 Rr_collision = R.MultiplyVector(r_collision);
        //Vector3 x_collision = T + Rr_collision;							 // virtual collision point（global coordination）
        Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);

        // 计算v_N
        Vector3 v_N = Vector3.Dot(v_collision, N) * N;
        Vector3 v_T = v_collision - v_N;
        Vector3 v_N_new = -1.0f * restitution * v_N;
        float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
        Vector3 v_T_new = a * v_T;
        Vector3 v_new = v_N_new + v_T_new;

        // 计算冲量J
        Matrix4x4 Rri_star = Get_Cross_Matrix(Rr_collision);
        Matrix4x4 K = Matrix_subtraction(Matrix_miltiply_float(Matrix4x4.identity, 1.0f / mass),
                                        Rri_star * I_inverse * Rri_star);
        J = K.inverse.MultiplyVector(v_new - v_collision);

        if(isNet)
        {
            J *= 0.2f;
        }

        // 更新v和w
        v = v + 1.0f / mass * J;
        w = w + I_inverse.MultiplyVector(Vector3.Cross(Rr_collision, J));
    }

    void HandleCollisionWithRing(Collider meshCollider, Vector3 ballPosition, float ballRadius)
    {
        Mesh mesh = meshCollider.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        Vector3 sumP = Vector3.zero;
        Vector3 sumN = Vector3.zero;
        int collisionCount = 0;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0 = meshCollider.transform.TransformPoint(vertices[triangles[i]]);
            Vector3 v1 = meshCollider.transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 v2 = meshCollider.transform.TransformPoint(vertices[triangles[i + 2]]);

            // 小球与篮筐的碰撞检测
            if(SphereTriangleCollision(ballPosition, ballRadius, v0, v1, v2, out Vector3 P, out Vector3 N))
            {
                sumP += P;
                sumN += N;
                collisionCount++;
            }
        }

        if (collisionCount > 0)
        {
            Vector3 averageP = sumP / collisionCount;
            Vector3 averageN = sumN.normalized;
            Collision_Impulse(averageP, averageN, false, out Vector3 J);
        }
    }

    bool SphereTriangleCollision(Vector3 sphereCenter, float sphereRadius, Vector3 v0, Vector3 v1, Vector3 v2, out Vector3 collisionPoint, out Vector3 collisionNormal)
    {
        // 计算三角形的法向量
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 triangleNormal = Vector3.Cross(edge1, edge2).normalized;

        // 计算球心到三角形平面的距离
        float d = Vector3.Dot(sphereCenter - v0, triangleNormal);

        // d > 半径：没有碰撞
        if(Mathf.Abs(d) > sphereRadius)
        {
            collisionPoint = Vector3.zero;
            collisionNormal = Vector3.zero;
            return false;
        }

        // 球心投影到三角形平面的点并检测投影是否在三角形内
        Vector3 projection = sphereCenter - d * triangleNormal;
        if(PointInTriangle(projection, v0, v1, v2))
        {
            collisionPoint = projection + triangleNormal * Mathf.Sign(d) * sphereRadius;
            collisionNormal = triangleNormal;
            return true;
        }

        // 如果投影点不在三角形内，则检查球与三角形的边或顶点的碰撞
        // 检查与三条边的碰撞
        bool collided = false;
        float minDistance = float.MaxValue;
        Vector3 closestPoint = Vector3.zero;
        Vector3 closestNormal = Vector3.zero;

        Vector3[] edges = { v0, v1, v2, v0 };
        for (int i = 0; i < 3; i++)
        {
            Vector3 point;
            Vector3 edgeStart = edges[i];
            Vector3 edgeEnd = edges[i + 1];
            ClosestPointOnLineSegment(projection, edgeStart, edgeEnd, out point);

            float dist = Vector3.Distance(sphereCenter, point);
            if (dist < sphereRadius && dist < minDistance)
            {
                minDistance = dist;
                closestPoint = point;
                closestNormal = (sphereCenter - point).normalized;
                collided = true;
            }
        }

        if (collided)
        {
            collisionPoint = closestPoint + closestNormal * sphereRadius;
            collisionNormal = closestNormal;
            return true;
        }

        collisionPoint = Vector3.zero;
        collisionNormal = Vector3.zero;
        return false;
    }

    // 检查点是否在三角形内
    bool PointInTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = p - a;

        float dot00 = Vector3.Dot(v0, v0);
        float dot01 = Vector3.Dot(v0, v1);
        float dot02 = Vector3.Dot(v0, v2);
        float dot11 = Vector3.Dot(v1, v1);
        float dot12 = Vector3.Dot(v1, v2);

        float denom = dot00 * dot11 - dot01 * dot01;
        if (denom == 0) return false;

        float u = (dot11 * dot02 - dot01 * dot12) / denom;
        float v = (dot00 * dot12 - dot01 * dot02) / denom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }


    // 找到点在线段上的最近点
    void ClosestPointOnLineSegment(Vector3 p, Vector3 a, Vector3 b, out Vector3 closest)
    {
        Vector3 ab = b - a;
        float denominator = Vector3.Dot(ab, ab);
        if (denominator < 1e-6f)
        {
            closest = a; // 如果 a 和 b 非常接近，直接取 a 作为最近点
            return;
        }
        float t = Vector3.Dot(p - a, ab) / denominator;
        t = Mathf.Clamp(t, 0, 1);
        closest = a + t * ab;
    }

    void HandleCollisionWithNet(Collider meshCollider)
    {
        Mesh mesh = meshCollider.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        Vector3 sumP = Vector3.zero;
        Vector3 sumN = Vector3.zero;
        int collisionCount = 0;
        List<Vector3> collisionPoint = new List<Vector3>();

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0 = meshCollider.transform.TransformPoint(vertices[triangles[i]]);
            Vector3 v1 = meshCollider.transform.TransformPoint(vertices[triangles[i + 1]]);
            Vector3 v2 = meshCollider.transform.TransformPoint(vertices[triangles[i + 2]]);

            // 小球与篮网的碰撞检测
            if (SphereTriangleCollision(transform.position, ballRadius, v0, v1, v2, out Vector3 P, out Vector3 N))
            {
                sumP += P;
                sumN += N;
                collisionCount++;
                collisionPoint.Add(vertices[triangles[i]]);
                collisionPoint.Add(vertices[triangles[i + 1]]);
                collisionPoint.Add(vertices[triangles[i + 2]]);
            }
        }
        
        if (collisionCount > 0)
        {
            Vector3 averageP = sumP / collisionCount;
            Vector3 averageN = sumN.normalized;
            Collision_Impulse(averageP, averageN, true, out Vector3 J);
            netController.ApplyJ(collisionPoint, -J);
        }
    }


    void Update()
    {
        //print(v);
        //print(w);
        if (!isShooting)
        {
            UpdateBasketballPosition();
        }

        if (Input.GetKeyDown(KeyCode.Space) && !isShooting)
        {
            Shoot();
            scoreDisplay.StartGame();
        }

        if(isShooting)
        {
            // float d = Vector3.Distance(cameraTransform.position, transform.position);
            // 篮球快停止时或固定时间后回到手中
            gapTime -= Time.deltaTime;
            if (v.magnitude < 0.4f || gapTime < 0)
            {
                v = Vector3.zero;
                w = Vector3.zero;
                isShooting = false;

                // 当篮球速度几乎为零时更新摄像机位置
                UpdateBasketballPosition();
                CameraController.isCameraTracking = true;
                gapTime = 4.0f;
            }
            else
            {
                SimulateMotion();
            }
        }
        
    }

    void Shoot()
    {
        Vector3 netVector = netController.transform.position;
        Vector3 ballVector = transform.position;
        Vector2 netVectorXZ = new Vector3(netVector.x, netVector.z);
        Vector2 ballVectorXZ = new Vector3(ballVector.x, ballVector.z);
        float d = Vector2.Distance(netVectorXZ, ballVectorXZ);
        // if d >= 8.6 三分
        if (d >= 8.6f)
        {
            currentScore = 3;
        }
        else
        {
            currentScore = 2;
        }
        float dFactor;
        float randomFactor = UnityEngine.Random.Range(0.8f, 1.2f);
        if(d < 10.0f)
        {
            dFactor = 0.5f * randomFactor;
        }
        else
        {
            dFactor = 0.4f * randomFactor;
        }

        float shootForce = baseShootForce + d * dFactor;

        // 获取投篮方向
        Vector3 shootDirection = cameraTransform.forward;
        shootDirection.y = MathF.Max(shootDirection.y, 0.5f);

        // 初速度
        v = shootDirection.normalized * shootForce;
        v.y += 10.0f;
        w = new Vector3(0, 1, 0);

        isShooting = true;

        CameraController.isCameraTracking = false;
    }

    void SimulateMotion()
    {
        v += gravity * dt;
        v *= linear_decay;
        w *= angular_decay;

        // 与地面的碰撞
        Collision_Impulse(new Vector3(0, 0.05f, 0), new Vector3(0, 1, 0), false, out Vector3 J);
        // 隐形墙的碰撞
        Collision_Impulse(new Vector3(0, 0, 32.01f), new Vector3(0, 0, -1), false, out J);// Wall1
        Collision_Impulse(new Vector3(-17.79f, 0.01f, 0), new Vector3(1, 0, 0), false, out J);// Wall2
        Collision_Impulse(new Vector3(17.95f, 0.01f, 0), new Vector3(-1, 0, 0), false, out J);// Wall3
        Collision_Impulse(new Vector3(0, 0, -31.57f), new Vector3(0, 0, 1), false, out J);// Wall4

        Vector3 ballPosition = transform.position;
        // 宽阶段直接用unity提供的了，具体采用了Spacial Partitioning
        Collider[] colliders = Physics.OverlapSphere(ballPosition, ballRadius);
        foreach (Collider collider in colliders)
        {
            // 与篮筐的碰撞
            if (collider.CompareTag("Ring"))
            {
                HandleCollisionWithRing(collider, ballPosition, ballRadius);
            }

            // 与篮网的碰撞
            if (collider.CompareTag("Net"))
            {
                HandleCollisionWithNet(collider);
            }
        }


        Vector3 x_0 = transform.position;
        Quaternion q_0 = transform.rotation;

        Vector3 x = x_0 + v * dt;

        Vector3 dw = 0.5f * w * dt;
        Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
        Quaternion q = Add(q_0, qw * q_0);

        transform.position = x;
        transform.rotation = q;
    }

    void UpdateBasketballPosition()
    {
        if (cameraTransform != null)
        {
            // 获取摄像机的旋转角度
            float rotationX = cameraController.getRotationX();
            float rotationY = cameraController.getRotationY();

            // 计算篮球的新位置：在摄像机前方，具有 verticalOffset
            Quaternion rotation = Quaternion.Euler(rotationY, rotationX, 0);
            Vector3 offset = rotation * new Vector3(0, verticalOffset, distance);
            transform.position = cameraTransform.position + offset;

        }
    }

    public Vector3 GetVelocity()
    {
        return v;
    }

    public int getCurrentScore()
    {
        return currentScore;
    }

}


