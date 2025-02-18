using UnityEngine;
using TMPro;

public class ScoreDisplay : MonoBehaviour
{
    public TextMeshPro textMeshPro; // 引用TextMeshPro组件
    public float countdownTime = 60.0f; // 初始倒计时时间（秒）
    float currentTime;
    int score = 0;
    bool isStart = false;

    void Start()
    {
        currentTime = countdownTime;
        textMeshPro.text = $"SHOOT!";
    }

    void Update()
    {
        UpdateDisplay();
    }

    void UpdateDisplay()
    {
        if (isStart && currentTime > 0)
        {
            currentTime -= Time.deltaTime;
            if (currentTime < 0)
            {
                isStart = false;
                currentTime = 60.0f;
                textMeshPro.text = $"CONGRATULATIONS!\nYOUR SCORE IS: {score}\n SHOOT FOR CONTINUE";
                score = 0;
            }
            else
            {
                textMeshPro.text = $"TIME: {Mathf.CeilToInt(currentTime)}s\nSCORE: {score}";
            }
        }
    }

    // 增加得分的方法
    public void AddScore(int points)
    {
        score += points;
        UpdateDisplay();
    }

    public void StartGame()
    {
        isStart = true;
    }
}
