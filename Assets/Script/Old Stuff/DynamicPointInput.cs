
using UnityEngine;

class DynamicPointInput
{
    public Vector2 accel;
    public float time;
    public TreeNode resultNode;

    public DynamicPointInput(Vector2 accel, float time, TreeNode resultNode)
    {
        this.accel = accel;
        this.time = time;
        this.resultNode = resultNode;
    }
}