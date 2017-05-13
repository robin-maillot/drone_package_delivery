using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

public class VelPoint : MonoBehaviour
{
    public float[] startPos, goalPos, startVel, goalVel;
    public float MAX_SPEED, MAX_ACCEL, MAX_OMEGA, MAX_PHI, L_CAR, K_FRICTION;

    public Vector2[][] polygons;
    public Vector2[] boundaryPolygon;
    public bool useSaved; // hack

    public List<VelNode> path;
    public int curPos = 0;
    public int guardID;
    public List<Waypoint> waypoint;
}