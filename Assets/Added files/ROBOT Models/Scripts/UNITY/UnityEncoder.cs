using UnityEngine;

public class UnityEncoder : MonoBehaviour
{
    [SerializeField] private ArticulationBody[] joints = new ArticulationBody[6];
    
    

    void Start()
    {
        GetUnityAngles();
    }

    void Update()
    {
        GetUnityAngles();
    }


    public float[] GetUnityAngles()
    {
        float[] angles = new float[6];
        
        for (int i = 0; i < 6; i++)
        {
            angles[i] = joints[i].jointPosition[0];
            angles[i] = angles[i] * Mathf.Rad2Deg;
        }
        
        //Debug.Log("Angles: " + string.Join(", ", angles));
        return angles;
    }
} 