using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Circle
{
    public Vector2 pos;
    public float r;

    public Circle(double x, double y, double r)
    {
        pos = new Vector2((float) x, (float) y);
        this.r = (float) r;
    }
}
