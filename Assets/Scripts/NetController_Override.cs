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

        // ��ÿһ�㣬����ø߶���ԭʼ�����ƽ���뾶
        // ��ͨ��ɸѡorigVertices��y����ӽ��ò�Ķ��������ư뾶
        // radiusAtHeight[h] = �ø߶Ƚ����ƽ���뾶
        List<Vector3> newVertices = new List<Vector3>();
        for (int r = 0; r < rows; r++)
        {
            float y = maxY - r * heightStep;
            float radius = ComputeRadiusAtHeight(y, origVertices, avgX, avgZ);
            // �ڸò�����cols���㣬����һȦ
            for (int c = 0; c < cols; c++)
            {
                float angle = 2 * Mathf.PI * c / cols;
                float x = radius * Mathf.Cos(angle);
                float z = radius * Mathf.Sin(angle);
                newVertices.Add(new Vector3(x, y, z));
            }
        }

        // 2. ��������������
        List<int> newTriangles = new List<int>();

        for (int r = 0; r < rows - 1; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                int curr = r * cols + c;
                int nextC = r * cols + ((c + 1) % cols);
                int currBelow = (r + 1) * cols + c;
                int nextBelow = (r + 1) * cols + ((c + 1) % cols);

                // ������1
                newTriangles.Add(curr);
                newTriangles.Add(nextC);
                newTriangles.Add(nextBelow);

                // ������2
                newTriangles.Add(curr);
                newTriangles.Add(nextBelow);
                newTriangles.Add(currBelow);
            }
        }

        // �ⶥ���׿�����Ҫ����
        // ���������񲢸�ֵ��MeshFilter
        Mesh newMesh = new Mesh();
        newMesh.vertices = newVertices.ToArray();
        newMesh.triangles = newTriangles.ToArray();
        newMesh.RecalculateNormals();

        //GetComponent<MeshFilter>().mesh = newMesh;

        E = newTriangles.ToArray();
        // ����ÿ���ߵĳ�ʼ����
        L = new float[E.Length / 2];
        for (int e = 0; e < E.Length / 2; e++)
        {
            int i = E[e * 2];
            int j = E[e * 2 + 1];
            L[e] = (newMesh.vertices[i] - newMesh.vertices[j]).magnitude;
        }

        // ��ʼ���ٶ�����
        V = new Vector3[newMesh.vertices.Length];
        for (int i = 0; i < V.Length; i++)
        {
            V[i] = Vector3.zero;
        }

        // ��ʼ���̶�����
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

        // 1. Ԥ��λ��
        for (int i = 0; i < X.Length; i++)
        {
            if (!fixedVertices[i])
            {
                V[i] *= damping;
                X[i] = X_old[i] + V[i] * t + gravity * t * t;
            }
            else
            {
                // �̶��㲻��
                X[i] = X_old[i];
            }
        }
        mesh.vertices = X;

        // 2. ��ε���Ӧ������
        for (int i = 0; i < 8; i++)
            Strain_Limiting();

        // Ӧ��������ɺ��ȡ����λ��
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

        // �ռ���ָ���߶ȸ����Ķ���İ뾶
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

        // ��û���ڸ÷�Χ���ҵ����㣬��ѡ�����²��ԣ�
        // 1. ����0��Ĭ��ֵ
        // 2. Ѱ�����ڽ��Ĳ���в�ֵ����ѡ��
        if (foundRadii.Count == 0)
        {
            // û���ҵ��㣬�ɷ���һ��Ĭ�ϰ뾶������0�������������в�ֵ��
            return 0f;
        }


        return foundRadii.Average();
    }
}
