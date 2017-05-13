using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DubinState
{
    public Vector2 pos;
    public float v;
    public float theta;
    public float omega;

    public DubinState()
    {
        this.pos = Vector2.zero;
        this.v = 0;
        this.theta = 0;
    }

    public DubinState(Vector2 pos, float v, float theta, float omega)
    {
        this.pos = pos;
        this.v = v;
        this.theta = theta;
        this.omega = omega;
    }

    public DubinState(DubinNode node)
    {
        this.pos = node.pos;
        this.v = node.vel;
        this.theta = node.theta;
        this.omega = node.omega;
    }
}
