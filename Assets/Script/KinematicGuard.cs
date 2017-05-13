using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicGuard : Point {
    //private float time = 0.0F;

    private Vector2[] points;
    
    private int N = 0;

    private const float DELTA_T = 0.01F;

    List<Node> Plan() {

        //var nodes = Utils.bf(N, 0, N-1, edges);  //vertices
        var pathw = new List<Waypoint>();
        var path = new List<Node>();

        path.Add(new Node(this.waypoint[0].point, 0));
        for (int i = 1, len = this.waypoint.Count; i < len; i++)
        {
            path.Add(new Node(this.waypoint[i].point, path[i-1].time + Vector2.Distance(this.waypoint[i].point, this.waypoint[i-1].point) / MAX_SPEED));
        }
        Debug.Log("Guard:" + this.guardID + "goal time " + path[path.Count - 1].time);
        var filledPath = GetFilledPath(path);
        Utils.SaveFilledPath(filledPath);
        return filledPath;  //List of Nodes
    }

    Vector2 KinematicInput(Node from, Node to)
    {
        return (to.pos - from.pos).normalized * MAX_SPEED;
    }

    List<Node> GetFilledPath(List<Node> path)
    {
        List<Node> filledPath = new List<Node>();
        filledPath.Add(path[0]);
        for (var i = 0; i < path.Count - 1; i++)
        {
            var v = KinematicInput(path[i], path[i + 1]);
            var totalT = path[i + 1].time - path[i].time;
            float sumT = 0;
            while (sumT < totalT)
            {
                var dt = Mathf.Min(DELTA_T, totalT - sumT);
                sumT += dt;
                filledPath.Add(new Node(path[i].pos + v * sumT, path[i].time + sumT));
            }
        }
        return filledPath;
    }

    // Use this for initialization
    void Start () {
 //       transform.position = new Vector3(startPos[0], startPos[1], 1);
        if (useSaved)
        {
            path = Utils.LoadFilledPath();
            if (path != null) return;
        }
        var t = DateTime.Now;
        path = Plan();
        Debug.Log("Computation time: " + (DateTime.Now - t));
    }

    // Update is called once per frame
    private float totalTime = 0F;
    void Update()
    {
        if (path != null && path.Count > 0)
        {
            totalTime += Time.deltaTime;
            UpdatePosition();
        }
    }

    void UpdatePosition()
    {
        float time = totalTime;
        while (curPos + 1 < path.Count && time > path[curPos + 1].time) curPos++;
        if (curPos >= path.Count)
        {
            return;
        }
        var pos = path[curPos].pos;
        transform.position = new Vector3(pos.x, pos.y, 1);
    }
}
