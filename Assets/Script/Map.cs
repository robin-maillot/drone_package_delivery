using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

public class Map : MonoBehaviour
{
    //public float[] startPos, goalPos, startVel, goalVel;
    //public float MAX_SPEED, MAX_ACCEL, MAX_OMEGA, MAX_PHI, L_CAR, K_FRICTION;

    public Vector2[][] polygons;
    public Vector2[] boundaryPolygon;
    public float[][] items;
    public float[][] seen;
    public float sensor_range;
    //public bool useSaved; // hack

    //public List<Node> path;
    //public int curPos = 0;
}