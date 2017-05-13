using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StaticGuard : Point {
    //private float time = 0.0F;

    private Vector2[] points;
    
    private int N = 0;

    private const float DELTA_T = 0.01F;

    List<Node> Plan() {
        foreach (var polygon in polygons)
            N += polygon.Length;

        N += 2;
        points = new Vector2[N];
        points[0] = new Vector2(startPos[0], startPos[1]);
        int n = 1;
        foreach (var polygon in polygons)
            foreach (var point in polygon)
                points[n++] = point;
        points[n++] = new Vector2(goalPos[0], goalPos[1]);

        List<Utils.Edge> edges = new List<Utils.Edge>();

        for (int i = 0; i < N; i++)
        {
            for (int j = i + 1; j < N; j++)
            {
                bool intersect = false;
                for (int k = 0; k < boundaryPolygon.Length; k++)
                {
                    Vector2 p3 = boundaryPolygon[k], p4 = boundaryPolygon[(k + 1) % boundaryPolygon.Length];

                    intersect = intersect || Utils.FasterLineSegmentIntersection(points[i], points[j], p3, p4);
                }
                foreach (var polygon in polygons)
                {
                    for (int k = 0; k < polygon.Length; k++)
                    {
                        Vector2 p3 = polygon[k], p4 = polygon[(k + 1) % polygon.Length];
                        if (points[i] == p3 || points[j] == p4 || points[i] == p4 || points[j] == p3) continue;
                        
                        intersect = intersect || Utils.FasterLineSegmentIntersection(points[i], points[j], p3, p4);
                    }
                }
                if (!intersect) {
                    float dist = Vector2.Distance(points[i], points[j]);
                    edges.Add(new Utils.Edge(i, j, dist));
                    edges.Add(new Utils.Edge(j, i, dist));
                    Debug.Log("edge " + i + " " + j + " " + dist);
                }
            }
        }

        var nodes = Utils.bf(N, 0, N-1, edges);
        var path = new List<Node>();
//            Debug.Log(points[nodes[0]]);
        path.Add(new Node(points[nodes[0]], 0));
        for (int i = 1, len = nodes.Count; i < len; i++)
        {
            path.Add(new Node(points[nodes[i]], path[i-1].time + Vector2.Distance(points[nodes[i]], points[nodes[i-1]]) / MAX_SPEED));
        }
        Debug.Log("goal time " + path[path.Count - 1].time);
        var filledPath = GetFilledPath(path);
        Utils.SaveFilledPath(filledPath);
        return filledPath;
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
    /*void Start () {
        transform.position = new Vector3(guardID, guardID, 1);
        //transform.position = new Vector3(8, 8, 1);
        //transform.position = new Vector3(startPos[0], startPos[1], 1);
        if (useSaved)
        {
            path = Utils.LoadFilledPath();
            if (path != null) return;
        }
        var t = DateTime.Now;
        path = Plan();
        Debug.Log("Computation time: " + (DateTime.Now - t));
    }*/

    // Update is called once per frame
    //	void Update () {
    //        Debug.Log("pathCnt " + path.Count);
    //        Debug.Log("curPos " + curPos);
    //        if (curPos == path.Count) return;
    //
    //        Vector2 v2 = points[path[curPos]];
    //        if (transform.position == new Vector3(v2.x, v2.y, 1)) curPos++;
    //
    //
    //        if (curPos == path.Count) return;
    //        float step = MAX_SPEED * Time.deltaTime;
    //            
    //        transform.position = Vector3.MoveTowards(transform.position, new Vector3(v2.x, v2.y, 1), step);
    //    }
    private float totalTime = 0F;
    void Update()   {

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
