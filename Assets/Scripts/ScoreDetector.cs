using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ScoreDetector : MonoBehaviour
{
    public BasketballController basketballController; // �������������
    public ScoreDisplay scoreDisplay; // ����ScoreDisplay�ű�
    bool hasScored = false; // ��ֹ�ظ��Ʒ�

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("ball") && !hasScored)
        {
            // ��ȡ������ٶȷ���
            Vector3 velocity = basketballController.GetVelocity();
            int currentScore = basketballController.getCurrentScore();
            print("velocity:" + velocity);
            // �ж������Ƿ��������˶��������Ϸ����´�����
            if (velocity.y < 0)
            {
                // ��¼�÷�
                hasScored = true;
                scoreDisplay.AddScore(currentScore);
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.CompareTag("ball"))
        {
            hasScored = false; // ���üƷ�״̬
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
