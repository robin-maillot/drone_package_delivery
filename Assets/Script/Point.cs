using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

public class Point : MonoBehaviour
{
    public float[] startPos, goalPos, startVel, goalVel;
    public float MAX_SPEED, MAX_ACCEL, MAX_OMEGA, MAX_PHI, L_CAR, K_FRICTION;
    public Vector3 vel;

    public Vector2[][] polygons;
    public Vector2[] boundaryPolygon;
    public bool useSaved; // hack

    public List<Node> path;
    public int curPos = 0;
    public int guardID;
    public List<Waypoint> waypoint;
    public List<Vector3> formation; //goes from guard 0 - 4, skipping itself
    public Vector2 formationError;
    public float Kp, Kd, Ki;
    public Vector3 closestBuildingPoint;
}