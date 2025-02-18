using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;

public class NetController : MonoBehaviour
{
    float t = 0.2f;
    float damping = 0.99f;
    Vector3 gravity = new Vector3(0.0f, -9.8f, 0);
    int[] E;
    float[] L;
    Vector3[] V; // 每个顶点的速度
    bool[] fixedVertices;
    int mass;


    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        mass = vertices.Length;
        int[] triangles = mesh.triangles;

        // 提取边列表
        List<(int, int)> edgeList = new List<(int, int)>();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int v0 = triangles[i];
            int v1 = triangles[i + 1];
            int v2 = triangles[i + 2];

            edgeList.Add(SortEdge(v0, v1));
            edgeList.Add(SortEdge(v1, v2));
            edgeList.Add(SortEdge(v2, v0));
        }

        //// 去重边
        var uniqueEdges = new HashSet<(int, int)>(edgeList);
        E = uniqueEdges.SelectMany(edge => new int[] { edge.Item1, edge.Item2 }).ToArray();

        // 计算每条边的初始长度
        L = new float[E.Length / 2];
        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[e * 2];
            int j = E[e * 2 + 1];
            L[e] = (vertices[i] - vertices[j]).magnitude;
        }

        // 初始化速度和外力数组
        V = new Vector3[vertices.Length];
        for (int i = 0; i < V.Length; i++)
        {
            V[i] = Vector3.zero;
        }

        // 初始化固定顶点
        fixedVertices = new bool[vertices.Length];
        InitializeFixedVertices(vertices);
    }

    private (int, int) SortEdge(int a, int b)
    {
        return a < b ? (a, b) : (b, a);
    }

    private void InitializeFixedVertices(Vector3[] vertices)
    {
        float maxY = vertices.Max(v => v.y);
        float minY = vertices.Min(v => v.y);
        float rangeY = maxY - minY;

        for (int i = 0; i < vertices.Length; i++)
        {
            if (vertices[i].y <= maxY && vertices[i].y >= maxY - 0.2f)
            {
                fixedVertices[i] = true;
            }
        }
    }

    public void ApplyJ(List<Vector3> collisionPoint, Vector3 J)
    {
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] X = mesh.vertices;
        for (int i = 0; i < collisionPoint.Count; i++)
        {
            for (int j = 0; j < X.Length; j++)
            {
                if (collisionPoint[i] == X[j])
                {
                    V[j] = V[j] + 1.0f / collisionPoint.Count * J;
                    X[j] = X[j] + V[j] * t;
                }
            }
        }
    }


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
                X[i] = X_old[i] + V[i] * t + 0.5f * gravity * t * t;
            }
            else
            {
                // 固定点不动
                X[i] = X_old[i];
            }
        }
        mesh.vertices = X;

        // 2. 多次迭代应变限制
        for (int i = 0; i < 16; i++)
            Strain_Limiting();

        // 应变限制完成后获取最终位置
        X = mesh.vertices;

        mesh.RecalculateNormals();
    }


    // PBD
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
            //sum_x[i] = sum_x[i] + 0.5f * (vertices[i] + vertices[j] + L[e] * xij * (1.0f / xij.magnitude));
            //sum_x[j] = sum_x[j] + 0.5f * (vertices[i] + vertices[j] - L[e] * xij * (1.0f / xij.magnitude));
            sum_x[i] = sum_x[i] + vertices[i] - 0.5f * (xij.magnitude - L[e]) * xij.normalized;
            sum_x[j] = sum_x[j] + vertices[j] + 0.5f * (xij.magnitude - L[e]) * xij.normalized;
            sum_n[i]++;
            sum_n[j]++;
        }
        for (int i = 0; i < vertices.Length; i++)
        {
            if (fixedVertices[i]) continue;
            V[i] = V[i] + (1.0f / t) * ((0.2f * vertices[i] + sum_x[i]) / (0.2f + (float)sum_n[i]) - vertices[i]);
            vertices[i] = (0.2f * vertices[i] + sum_x[i]) / (0.2f + (float)sum_n[i]);
        }
        mesh.vertices = vertices;
    }

    public int[] getE()
    {
        return E;
    }

    public float[] getL()
    {
        return L;
    }

}