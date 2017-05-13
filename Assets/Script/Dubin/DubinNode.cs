using System.Collections.Generic;
using System.Runtime.Remoting.Channels;
using UnityEngine;

public class DubinNode : Node
{
    public float vel;
    public float theta;
    public float omega;
    public List<DubinNode> children;
    public DubinNode parent;
    public DubinNode()
    {
        this.pos = Vector2.zero;
        this.vel = 0F;
        this.theta = 0F;
        this.omega = 0F;
        this.time = 0F;
        this.children = new List<DubinNode>();
        this.parent = null;
    }
    public DubinNode(Vector2 pos, float vel, float theta, float omega, float time)
    {
        this.pos = pos;
        this.vel = vel;
        this.theta = theta;
        this.omega = omega;
        this.time = time;
        this.children = new List<DubinNode>();
        this.parent = null;
    }
    public void SetParent(DubinNode p)
    {
        if (this.parent != null) this.parent.children.Remove(this);
        this.parent = p;
    }
}