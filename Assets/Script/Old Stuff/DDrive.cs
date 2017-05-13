using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KDTreeDLL;
using Random = UnityEngine.Random;

public class DDrive : Point
{
    public float startTheta;
    public float goalTheta;


    public int task;


    const float DELTA_T = 0.01F;
    //private float time = 0.0F;

    private List<DubinNode> nodes = new List<DubinNode>();
    private KDTree kdTree;

    private int N = 0;
    private int reparentCnt = 0;

    private float startTime;

    private float informedLength = Mathf.Infinity;
    private float curBestTime = Mathf.Infinity;

    private DubinNode startNode, goalNode;

    private void OnDrawGizmos()
    {
        foreach (var p in nodes)
        {
            Gizmos.DrawCube(new Vector3(p.pos.x, p.pos.y, 20), new Vector3(0.2F, 0.2F, 0));
        }
    }

    bool fulfillInformed(Vector2 p)
    {
        //return true;
        bool res = informedLength >
                   (p - new Vector2(startPos[0], startPos[1])).magnitude +
                   (p - new Vector2(goalPos[0], goalPos[1])).magnitude;
        //        Debug.Log("useful");
        //        Debug.Log(informedLength);
        //        Debug.Log((p - new Vector2(startPos[0], startPos[1])).magnitude + (p - new Vector2(goalPos[0], goalPos[1])).magnitude);
        if (!res)
        {
            Debug.Log("im useful");
        }
        return res;
    }

    //I've got to say that this is really elegant and clean, nicely done.
    Vector2 randomVel()
    {
        var x = Random.Range(-1.0F, 1.0F);
        var y = Random.Range(-1.0F, 1.0F);
        var v = new Vector2(x, y);
        v.Normalize();
        v *= MAX_SPEED;
        return v;
    }

    DubinNode randomNode()
    {
        /*var r = Random.Range(0F, 1.0F);
                if (r < 0.01F) return goalNode;*/
        Vector2 p = Utils.randomPoint(startNode, goalNode, boundaryPolygon, polygons);
        float theta = Random.Range(0, 2 * Mathf.PI);
        float v;
        float omega;
        switch (task)
        {
            case 2:
                v = Random.Range(0, MAX_SPEED);
                return new DubinNode(p, v, theta, MAX_ACCEL / v, 0);
                break;
            case 3:
                v = Random.Range(0, MAX_SPEED);
                return new DubinNode(p, v, theta, MAX_OMEGA, 0);
                break;
            case 4:
                v = MAX_SPEED;
                omega = (v / L_CAR) * Mathf.Tan(MAX_PHI);
                return new DubinNode(p, v, theta, omega, 0);
                break;
            case 5:
                v = Random.Range(0, MAX_SPEED);
                omega = Mathf.Min(MAX_ACCEL / v, (v / L_CAR) * Mathf.Tan(MAX_PHI));
                return new DubinNode(p, v, theta, omega, 0);
            case 6:
                v = Random.Range(0, MAX_SPEED);
                float g = 9.81F;
//                omega = Mathf.Min(MAX_ACCEL / v,
//                    Mathf.Sqrt(g * g * K_FRICTION * K_FRICTION -
//                               v * v * v * v / (L_CAR * L_CAR) * Mathf.Atan(MAX_PHI) * Mathf.Atan(MAX_PHI)) / v);
                omega = Mathf.Min((v / L_CAR) * Mathf.Tan(MAX_PHI), Mathf.Min(MAX_ACCEL / v, (g * K_FRICTION) / (Mathf.Sqrt(2) * v)));
                return new DubinNode(p, v, theta, omega, 0);
            default:
                break;
        }
        return new DubinNode();
    }

    bool pathFeasible(DubinNode from, DubinNode to, DubinInput input)
    {
        // TODO: check for max condition
        foreach (DubinInput.Steer steer in input.steer)
        {
            if (Mathf.Abs(steer.a) > MAX_ACCEL)
            {
                return false;
            }

            if (task == 6 && Mathf.Abs(steer.a) > 9.81F * K_FRICTION)
            {
                return false;
            }
        }
        var dubinsCollision = dubinsTransition(true, null, from, to, input);
        return dubinsCollision;
    }

    Pair<DubinNode, float> findCheapestPoint(List<DubinNode> nodes, DubinNode target)
    {
        float minCost = Mathf.Infinity;
        DubinNode cheapest = new DubinNode();
        //var knn = kdTree.nearest(new double[2] {target.pos.x, target.pos.y}, Math.Min(nodes.Count, 100));
        var knn = nodes;
        foreach (var nn in knn)
        {
            DubinNode p = (DubinNode) nn;
            var input = dubinInput(p, target);
            if (input.time + p.time < minCost && pathFeasible(p, target, input))
            {
                minCost = p.time + input.time;
                cheapest = p;
            }
        }
        return new Pair<DubinNode, float>(cheapest, minCost);
    }

    void reparent(DubinNode root, DubinNode parent, DubinNode goal)
    {
        var nodes = new List<Pair<DubinNode, float>>();
        Utils.TopoSort(root, nodes, 0F);
        for (int i = 0; i < nodes.Count; i++)
        {
            DubinNode p = nodes[i].first;
            float dt = nodes[i].second;
            var input = dubinInput(parent, p);
            if (p.parent != null && p.parent.time + dt < p.time)
                p.time = p.parent.time + dt;
            if (input.time + parent.time < p.time && pathFeasible(parent, p, input))
            {
                p.time = input.time + parent.time;
                p.SetParent(parent);
                reparentCnt++;
            }
        }
    }

    void addNode(List<DubinNode> nodes, DubinNode node)
    {
        nodes.Add(node);
        kdTree.insert(new double[2] {node.pos.x, node.pos.y}, node);
    }

    private int usefulCnt = 0;

    List<Node> GetFilledPath(List<DubinNode> path)
    {
        List<Node> filledPath = new List<Node>();
        for (var i = 0; i < path.Count - 1; i++)
        {
            var input = dubinInput(path[i], path[i + 1]);
            filledPath.Add(path[i]);
            dubinsTransition(false, filledPath, path[i], path[i + 1], input);
        }

        return filledPath;
    }


    /*
     * dubins transition function, multiple uses
     * 1. if collision param is true: check for collision, will return bool
     * 2. else: put interpolated nodes into list
     */

    public static bool dubinsTransition(bool collision, List<Node> list, DubinNode from, DubinNode to, DubinInput input)
    {
        DubinNode prev = from;
        double tx = from.pos.x, ty = from.pos.y, ttheta = from.theta; // HACK: to improve precision
        foreach (DubinInput.Steer steer in input.steer)
        {
            float sumT = 0;
            double v = steer.v;
            while (sumT < steer.time)
            {
                var dt = Mathf.Min(DELTA_T, steer.time);
                tx = tx + dt * v * Math.Cos(prev.theta);
                ty = ty + dt * v * Math.Sin(prev.theta);
                var pos = new Vector2((float) tx, (float) ty);
                v += dt * steer.a;
                ttheta = ttheta + dt * steer.omega;
                while (ttheta > 2 * Mathf.PI)
                    ttheta -= 2 * Mathf.PI;
                while (ttheta < 0)
                    ttheta += 2 * Mathf.PI;
                if (collision && !Cheetah.instance.IsValid(pos)) return false;
                DubinNode nextPoint = new DubinNode(pos, steer.v, (float) ttheta, steer.omega, prev.time + dt);
                if (!collision) list.Add(nextPoint);
//                if ((nextPoint.pos - prev.pos).magnitude > 0.5) // TODO: debug
//                {
//                    Debug.Log("WTF");
//                    dubinInput(from, to);
//                    return false;
//                }
                prev = nextPoint;
                sumT += dt;
            }
        }
//        // TODO: debug
//        if ((prev.pos - to.pos).magnitude > 0.5 || Mathf.Abs(prev.theta - to.theta) > 0.3)
//        {
//            Debug.Log("WTF");
//            dubinInput(from, to);
//            return false;
//        }
        return true;
    }

    public int iter;

    List<Node> PlanRrtStar()
    {
        // RRT*
        startNode = new DubinNode(new Vector2(startPos[0], startPos[1]), new Vector2(startVel[0], startVel[1]).magnitude,
            startTheta, MAX_OMEGA, 0);
        addNode(nodes, startNode);
        goalNode = new DubinNode(new Vector2(goalPos[0], goalPos[1]), new Vector2(goalVel[0], goalVel[1]).magnitude, goalTheta, MAX_OMEGA, 0);
        int cnt = iter, i = 0;
        float r = Mathf.Infinity; // TODO: Dynamic compute this r
        var solnCnt = 0;
        while (i++ < cnt)
        {
            DubinNode target = i == cnt ? goalNode : randomNode();
            var cheapest = findCheapestPoint(nodes, target);
            var cheapestNode = cheapest.first;
            var cheapestCost = cheapest.second;
            if (cheapestCost == Mathf.Infinity) continue;
            // TODO: Change the magic number
            if ((target.pos - goalNode.pos).magnitude <= 0.5)
            {
                Debug.Log("found a soln");
                solnCnt++;
                informedLength = Mathf.Min(informedLength, cheapestCost * MAX_SPEED);
                curBestTime = Mathf.Min(curBestTime, cheapestCost);
                Debug.Log(target.pos);
                Debug.Log(cheapestCost);
            }
            if (cheapestCost + (goalNode.pos - target.pos).magnitude / MAX_SPEED > curBestTime + 0.5)
            {
                usefulCnt++;
//                i--;
//                continue;
            }
            cheapestNode.children.Add(target);
            target.SetParent(cheapestNode);
            target.time = cheapestCost;
            addNode(nodes, target);
            reparent(startNode, target, goalNode);
        }
        Debug.Log("reparent cnt = " + reparentCnt);
        Debug.Log("useful count = " + usefulCnt);
        Debug.Log("goalNode parent count " + (goalNode.parent != null));

        List<DubinNode> path = new List<DubinNode>();
        Utils.backtrackPath(nodes, startNode, goalNode, path);

        for (i = 0; i < path.Count - 1; i++)
        {
            var f = path[i];
            Debug.Log("vel " + i);

            Debug.Log(f.vel);

            Debug.Log("pos");
            Debug.Log(f.pos);
        }

        var filledPath = GetFilledPath(path);
        Utils.SaveFilledPath(filledPath);

        Debug.Log("path " + path.Count);
        Debug.Log("path[cnt] " + path[path.Count - 1].pos.x + " " + path[path.Count - 1].pos.y + " " +
                  path[path.Count - 1].time);
        return filledPath;
    }

    private DubinInput dubinInput(DubinNode near, DubinNode target)
    {
        return DubinUtils.DubinSP(new DubinState(near), new DubinState(target), task == 2 || task == 5);
    }

    // Use this for initialization
    void Start()
    {
        if (useSaved)
        {
            path = Utils.LoadFilledPath();
            if (path != null) return;
        }
        transform.position = new Vector3(startPos[0], startPos[1], 1);

        const float EPS = 0.0001F;
        startTheta = Mathf.Atan2(startVel[1], startVel[0]);
        if (startTheta < -EPS)
            startTheta += 2F * Mathf.PI;
        goalTheta = Mathf.Atan2(goalVel[1], goalVel[0]);
        if (goalTheta < -EPS)
            goalTheta += 2F * Mathf.PI;
        //TimerText.text = "0";
        kdTree = new KDTree(2);
        DubinUtils.MAX_SPEED = MAX_SPEED;


        var t = DateTime.Now;
        path = PlanRrtStar();

        startTime = Time.time;
        Debug.Log(DateTime.Now - t);
    }

    private bool init = false;
    private float totalTime = 0F;
    // Update is called once per frame
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
        //        var cur = path[curPos];
        //        var next = path[curPos + 1];
        //        float dt = time - cur.time;
        //        var input = dynamicPointInput(cur, next);
        //        var pos = path[curPos].pos + dt * cur.vel + 0.5F * input.accel * dt * dt;
        var pos = path[curPos].pos;
        transform.position = new Vector3(pos.x, pos.y, 1);
        if (curPos > 1)
        {
            var nnode = path[curPos];
            var onode = path[curPos - 1];
            Vector2 v = (nnode.pos - onode.pos) / (nnode.time - onode.time);
            float theta = Mathf.Atan2(v.y, v.x);
            theta = theta < 0 ? theta + Mathf.PI * 2 : theta;
            transform.rotation = Quaternion.Euler(0F, 0F, theta * 180 / Mathf.PI);

        }
    }
}