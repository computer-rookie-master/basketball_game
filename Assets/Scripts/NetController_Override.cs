using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class NetController_Override : MonoBehaviour
{
    float t = 0.02f;
    float damping = 0.99f;
    Vector3 gravity = new Vector3(0, -9.8f, 0);
    Vector3[] V;
    int[] E;
    float[] L;

    float maxY, minY;
    float avgX, avgZ;

    int rows = 50;
    int cols = 30;

    bool[] fixedVertices;
    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] origVertices = mesh.vertices;

        maxY = origVertices.Max(v => v.y);
        minY = origVertices.Min(v => v.y);
        avgX = origVertices.Average(v => v.x);
        avgZ = origVertices.Average(v => v.z);
        float heightStep = (maxY - minY) / (rows - 1);

        // 对每一层，计算该高度下原始网格的平均半径
        // 可通过筛选origVertices中y坐标接近该层的顶点来估计半径
        // radiusAtHeight[h] = 该高度截面的平均半径
        List<Vector3> newVertices = new List<Vector3>();
        for (int r = 0; r < rows; r++)
        {
            float y = maxY - r * heightStep;
            float radius = ComputeRadiusAtHeight(y, origVertices, avgX, avgZ);
            // 在该层生成cols个点，环绕一圈
            for (int c = 0; c < cols; c++)
            {
                float angle = 2 * Mathf.PI * c / cols;
                float x = radius * Mathf.Cos(angle);
                float z = radius * Mathf.Sin(angle);
                newVertices.Add(new Vector3(x, y, z));
            }
        }

        // 2. 构建三角形索引
        List<int> newTriangles = new List<int>();

        for (int r = 0; r < rows - 1; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                int curr = r * cols + c;
                int nextC = r * cols + ((c + 1) % cols);
                int currBelow = (r + 1) * cols + c;
                int nextBelow = (r + 1) * cols + ((c + 1) % cols);

                // 三角形1
                newTriangles.Add(curr);
                newTriangles.Add(nextC);
                newTriangles.Add(nextBelow);

                // 三角形2
                newTriangles.Add(curr);
                newTriangles.Add(nextBelow);
                newTriangles.Add(currBelow);
            }
        }

        // 封顶或封底可视需要决定
        // 创建新网格并赋值给MeshFilter
        Mesh newMesh = new Mesh();
        newMesh.vertices = newVertices.ToArray();
        newMesh.triangles = newTriangles.ToArray();
        newMesh.RecalculateNormals();

        //GetComponent<MeshFilter>().mesh = newMesh;

        E = newTriangles.ToArray();
        // 计算每条边的初始长度
        L = new float[E.Length / 2];
        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[e * 2];
            int j = E[e * 2 + 1];
            L[e] = (newMesh.vertices[i] - newMesh.vertices[j]).magnitude;
        }

        // 初始化速度数组
        V = new Vector3[newMesh.vertices.Length];
        for (int i = 0; i < V.Length; i++)
        {
            V[i] = Vector3.zero;
        }

        // 初始化固定顶点
        fixedVertices = new bool[newMesh.vertices.Length];
        InitializeFixedVertices(newMesh.vertices);
    }

    private void InitializeFixedVertices(Vector3[] vertices)
    {
        float maxY = vertices.Max(v => v.y);

        for (int i = 0; i < vertices.Length; i++)
        {
            if (vertices[i].y <= maxY && vertices[i].y >= maxY - 0.1f)
            {
                fixedVertices[i] = true;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X_old = (Vector3[])mesh.vertices.Clone();
        Vector3[] X = mesh.vertices;

        // 1. 预测位置
        for (int i = 0; i < X.Length; i++)
        {
            if (!fixedVertices[i])
            {
                V[i] *= damping;
                X[i] = X_old[i] + V[i] * t + gravity * t * t;
            }
            else
            {
                // 固定点不动
                X[i] = X_old[i];
            }
        }
        mesh.vertices = X;

        // 2. 多次迭代应变限制
        for (int i = 0; i < 8; i++)
            Strain_Limiting();

        // 应变限制完成后获取最终位置
        X = mesh.vertices;

        mesh.RecalculateNormals();
    }

    void Strain_Limiting()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        Vector3[] sum_x = new Vector3[vertices.Length];
        int[] sum_n = new int[vertices.Length];
        //
        for (int i = 0; i < vertices.Length; i++)
        {
            sum_x[i] = new Vector3(0, 0, 0);
            sum_n[i] = 0;
        }
        for (int e = 0; e < L.Length; e++)
        {
            int i = E[e * 2];
            int j = E[e * 2 + 1];
            Vector3 xij = vertices[i] - vertices[j];
            sum_x[i] = sum_x[i] + vertices[i] - 0.5f * (xij.magnitude - L[e]) * xij.normalized;
            sum_x[j] = sum_x[j] + vertices[j] + 0.5f * (xij.magnitude - L[e]) * xij.normalized;
            sum_n[i]++;
            sum_n[j]++;
        }
        for (int i = 0; i < vertices.Length; i++)
        {
            if (fixedVertices[i]) continue;
            V[i] = V[i] + (1.0f / t) * ((0.2f * vertices[i] + sum_x[i]) / (0.2f + (float)sum_n[i]) - vertices[i]);
            vertices[i] = (0.2f * vertices[i] + sum_x[i]) / (0.2f + sum_n[i]);
        }
        mesh.vertices = vertices;
    }

    public static float ComputeRadiusAtHeight(float queryY, Vector3[] origVertices, float cx, float cz)
    {
        float searchRange = 0.01f;

        // 收集在指定高度附近的顶点的半径
        List<float> foundRadii = new List<float>();

        foreach (var v in origVertices)
        {
            if (Mathf.Abs(v.y - queryY) < searchRange)
            {
                float dx = v.x - cx;
                float dz = v.z - cz;
                float radius = Mathf.Sqrt(dx * dx + dz * dz);
                foundRadii.Add(radius);
            }
        }

        // 若没有在该范围内找到顶点，可选择以下策略：
        // 1. 返回0或默认值
        // 2. 寻找最邻近的层进行插值（可选）
        if (foundRadii.Count == 0)
        {
            // 没有找到点，可返回一个默认半径（例如0）或根据需求进行插值。
            return 0f;
        }


        return foundRadii.Average();
    }
}
