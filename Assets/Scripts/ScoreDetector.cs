using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScoreDetector : MonoBehaviour
{
    public BasketballController basketballController; // 引用篮球控制器
    public ScoreDisplay scoreDisplay; // 引用ScoreDisplay脚本
    bool hasScored = false; // 防止重复计分

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("ball") && !hasScored)
        {
            // 获取篮球的速度方向
            Vector3 velocity = basketballController.GetVelocity();
            int currentScore = basketballController.getCurrentScore();
            print("velocity:" + velocity);
            // 判断篮球是否是向下运动（即从上方向下穿过）
            if (velocity.y < 0)
            {
                // 记录得分
                hasScored = true;
                scoreDisplay.AddScore(currentScore);
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("ball"))
        {
            hasScored = false; // 重置计分状态
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
