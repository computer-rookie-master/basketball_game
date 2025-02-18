using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public Transform basketball; // 篮球的 Transform
    float mouseSensitivity = 100.0f; // 鼠标灵敏度
    float moveSpeed = 5.0f; // 摄像机平移速度

    float rotationX = 0.0f; // 水平方向旋转角度
    float rotationY = 0.0f; // 垂直方向初始旋转角度

    public static bool isCameraTracking = true;

    void Start()
    {
        // 初始化摄像机的旋转角度
        rotationX = transform.eulerAngles.y;
        rotationY = transform.eulerAngles.x;
    }

    void Update()
    {
        HandleMouseInput(); // 鼠标控制摄像机旋转
        HandleWASDMovement(); // WASD 控制摄像机平移
    }

    // 鼠标控制摄像机旋转
    void HandleMouseInput()
    {
        float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity * Time.deltaTime;
        float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity * Time.deltaTime;

        rotationX += mouseX; // 水平旋转
        rotationY -= mouseY; // 垂直旋转
        rotationY = Mathf.Clamp(rotationY, -30f, 90f); // 限制垂直角度范围

        // 更新摄像机的旋转
        transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
    }

    // WASD 控制摄像机平移
    void HandleWASDMovement()
    {
        Vector3 move = Vector3.zero;

        if (Input.GetKey(KeyCode.W)) move += transform.forward; // 前移
        if (Input.GetKey(KeyCode.S)) move -= transform.forward; // 后移
        if (Input.GetKey(KeyCode.A)) move -= transform.right;   // 左移
        if (Input.GetKey(KeyCode.D)) move += transform.right;   // 右移

        // 限制高度，不要进行y轴的平移
        move.y = 0;

        // 移动摄像机
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



