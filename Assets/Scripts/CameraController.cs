using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform basketball; // ����� Transform
    float mouseSensitivity = 100.0f; // ���������
    float moveSpeed = 5.0f; // �����ƽ���ٶ�

    float rotationX = 0.0f; // ˮƽ������ת�Ƕ�
    float rotationY = 0.0f; // ��ֱ�����ʼ��ת�Ƕ�

    public static bool isCameraTracking = true;

    void Start()
    {
        // ��ʼ�����������ת�Ƕ�
        rotationX = transform.eulerAngles.y;
        rotationY = transform.eulerAngles.x;
    }

    void Update()
    {
        HandleMouseInput(); // �������������ת
        HandleWASDMovement(); // WASD ���������ƽ��
    }

    // �������������ת
    void HandleMouseInput()
    {
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity * Time.deltaTime;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity * Time.deltaTime;

        rotationX += mouseX; // ˮƽ��ת
        rotationY -= mouseY; // ��ֱ��ת
        rotationY = Mathf.Clamp(rotationY, -30f, 90f); // ���ƴ�ֱ�Ƕȷ�Χ

        // �������������ת
        transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
    }

    // WASD ���������ƽ��
    void HandleWASDMovement()
    {
        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.W)) move += transform.forward; // ǰ��
        if (Input.GetKey(KeyCode.S)) move -= transform.forward; // ����
        if (Input.GetKey(KeyCode.A)) move -= transform.right;   // ����
        if (Input.GetKey(KeyCode.D)) move += transform.right;   // ����

        // ���Ƹ߶ȣ���Ҫ����y���ƽ��
        move.y = 0;

        // �ƶ������
        transform.position += move * moveSpeed * Time.deltaTime;
    }

    public float getRotationX()
    {
        return rotationX;
    }

    public float getRotationY()
    {
        return rotationY;
    }
}



