using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KDTreeDLL;
using UnityEngine.Assertions;
using Random = UnityEngine.Random;

public class DynamicPoint : Point
{
    //private float time = 0.0F;

    const float DELTA_T = 0.01F;

    private List<TreeNode> nodes = new List<TreeNode>();
    private KDTree kdTree;

    private int N = 0;

    private float startTime;

    private float informedLength = Mathf.Infinity;
    private float curBestTime = Mathf.Infinity;

    private TreeNode startNode, goalNode;
    private int reparentCnt = 0;

    private void OnDrawGizmos()
    {
        foreach (var p in nodes)
        {
            Gizmos.DrawCube(new Vector3(p.pos.x, p.pos.y, 20), new Vector3(0.2F, 0.2F, 0));
                }
//        if (path == null) return;
//        foreach (var p in path)
//        {
//            Gizmos.DrawCube(new Vector3(p.pos.x, p.pos.y, 20), new Vector3(0.1F, 0.1F, 0));
//        }
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

    TreeNode randomNode()
    {
        /*var r = Random.Range(0F, 1.0F);
                if (r < 0.01F) return goalNode;*/
        Vector2 p = Utils.randomPoint(startNode, goalNode, boundaryPolygon, polygons), v = randomVel();
        return new TreeNode(p, v, 0);
    }

    TreeNode findClosestPoint(List<TreeNode> nodes, TreeNode target)
    {
        float minDist = Mathf.Infinity;
        TreeNode closest = new TreeNode();
        foreach (var p in nodes)
        {
            Vector2 v = target.pos - p.pos;
            //So right now, we're doing this based off of position difference and not the time difference, so we should incorporate that somewhere
            if (v.magnitude < minDist)
            {
                closest = p;
                minDist = v.magnitude; //we should really store these values somewhere to save repeated computations
            }
        }
        return closest;
    }

    bool pathFeasible(TreeNode from, TreeNode to, DynamicPointInput input)
    {
        if (input.accel.magnitude > MAX_ACCEL) return false;
        if (to.vel.magnitude > MAX_SPEED) return false;
        TreeNode prev = from;
        float sumT = 0;
        double tx = prev.pos.x, ty = prev.pos.y, tvx = prev.vel.x, tvy = prev.vel.y;
        while (sumT < input.time)
        {
            var dt = Mathf.Min(DELTA_T, input.time);
//            var pos = prev.pos + prev.vel * dt + input.accel * dt * dt * 0.5F;
            tx = tx + tvx * dt + input.accel.x * dt * dt * 0.5;
            ty = ty + tvy * dt + input.accel.y * dt * dt * 0.5;
            var pos = new Vector2((float) tx, (float)ty);
            if (!Cheetah.instance.IsValid(pos)) return false;
            tvx = tvx + input.accel.x * dt;
            tvy = tvy + input.accel.y * dt;
//            var vel = prev.vel + input.accel * dt;
            var vel = new Vector2((float)tvx, (float)tvy);
            TreeNode nextPoint = new TreeNode(pos, vel, prev.time + dt);
            prev = nextPoint;
            sumT += dt;
        }
        if (Vector2.Distance(prev.pos, to.pos) > 0.01)
        {
            Debug.Log("WTF dynamic dist" + Vector2.Distance(prev.pos, to.pos));
            Assert.IsTrue(false);
        }
        return true;
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
        }
        var dubinsCollision = DDrive.dubinsTransition(true, null, from, to, input);
        return dubinsCollision;
    }

    Pair<Pair<TreeNode, TreeNode>, float> findCheapestPoint(List<TreeNode> nodes, TreeNode target, bool approx = false)
    {
        float minCost = Mathf.Infinity;
        TreeNode cheapest = null;
        TreeNode dest = null;
        //var knn = kdTree.nearest(new double[2] {target.pos.x, target.pos.y}, Math.Min(nodes.Count, 100));
        var knn = nodes;
        foreach (var nn in knn)
        {
            TreeNode p = nn;
            if (p.time > curBestTime) continue;
            if (approx)
            {
                var input = dynamicPointInput(p, target);
                if (input.time + p.time < minCost && pathFeasible(p, input.resultNode, input))
                {
                    minCost = p.time + input.time;
                    cheapest = p;
                    dest = input.resultNode;
                }
            }
            else
            {
                // TODO: is omega cal optimal?
                var dubinP = nodeToDubinNode(p);
                var dubinTarget = nodeToDubinNode(target);
                var input = dubinInput(dubinP, dubinTarget);
                if (input.time + p.time < minCost && pathFeasible(dubinP, dubinTarget, input))
                {
                    minCost = p.time + input.time;
                    cheapest = p;
                    target.isDubins = true;
                    dest = target;
                }
            }
        }
        return new Pair<Pair<TreeNode, TreeNode>, float>(new Pair<TreeNode, TreeNode>(cheapest, dest), minCost);
    }

    DubinNode nodeToDubinNode(TreeNode node)
    {
        var theta = Mathf.Atan2(node.vel.y, node.vel.x);
        if (theta < 0)
            theta += 2F * Mathf.PI;
        return new DubinNode(node.pos, node.vel.magnitude, theta,
            MAX_ACCEL / node.vel.magnitude,
            node.time);
    }

    void reparent(TreeNode root, TreeNode parent, TreeNode goal)
    {
        var nodes = new List<Pair<TreeNode, float>>();
        var goalDt = goal.parent == null? Mathf.Infinity : goal.time - goal.parent.time;
        TopoSort(root, nodes, 0F);
        for (int i = 0; i <= nodes.Count; i++)
        {
            TreeNode p = i == nodes.Count ? goal : nodes[i].first;
            float dt = i == nodes.Count ? goalDt : nodes[i].second;
            var dubinP = nodeToDubinNode(p);
            var dubinParent = nodeToDubinNode(parent);
            if (p.parent != null && p.parent.time + dt < p.time)
                p.time = p.parent.time + dt;
            var input = dubinInput(dubinParent, dubinP);
            if (input.time + parent.time < p.time && pathFeasible(dubinParent, dubinP, input))
            {
                p.isDubins = true;
                p.time = input.time + parent.time;
                p.SetParent(parent);
                reparentCnt++;
            }
            if (p == goal)
            {
//                Debug.Log("p is goal, curBest = " + curBestTime);
                curBestTime = Mathf.Min(curBestTime, p.time);
            }
        }
    }

    static void TopoSort(TreeNode root, List<Pair<TreeNode, float>> sorted, float dt)
    {
        foreach (var child in root.children)
        {
            TopoSort(child, sorted, child.time - root.time);
        }
        sorted.Add(new Pair<TreeNode, float>(root, dt));
        if (root.parent == null)
        {
            sorted.Reverse();
        }
    }

    void addNode(List<TreeNode> nodes, TreeNode node)
    {
        nodes.Add(node);
        kdTree.insert(new double[2] {node.pos.x, node.pos.y}, node);
    }

    private int usefulCnt = 0;

    List<Node> GetFilledPath(List<TreeNode> path)
    {
        List<Node> filledPath = new List<Node>();
        for (var i = 0; i < path.Count - 1; i++)
        {
            if (path[i + 1].isDubins)
            {
                DubinNode from = nodeToDubinNode(path[i]);
                DubinNode to = nodeToDubinNode(path[i + 1]);
                var input = dubinInput(from, to);
                filledPath.Add(from);
                DDrive.dubinsTransition(false, filledPath, from, to, input);
            }
            else
            {
                var input = dynamicPointInput(path[i], path[i + 1]);
                //Debug.Log(path[i + 1].vel.magnitude + " " + (path[i].vel + input.first * input.second).magnitude);

                float sumT = 0;
                var prev = path[i];
                filledPath.Add(prev);
                while (sumT < input.time)
                {
                    var dt = Mathf.Min(DELTA_T, input.time);
                    var pos = prev.pos + prev.vel * dt + 0.5F * input.accel * dt * dt;
                    var vel = prev.vel + input.accel * dt;
                    TreeNode nextPoint = new TreeNode(pos, vel, prev.time + dt);
                    filledPath.Add((Node) nextPoint);
                    prev = nextPoint;
                    sumT += dt;
                }
            }
        }
        return filledPath;
    }


    List<Node> PlanRrtStar()
    {
        // RRT*
        startNode = new TreeNode(new Vector2(startPos[0], startPos[1]), new Vector2(startVel[0], startVel[1]), 0);
        addNode(nodes, startNode);
        goalNode = new TreeNode(new Vector2(goalPos[0], goalPos[1]), new Vector2(goalVel[0], goalVel[1]), Mathf.Infinity);
        goalNode.isDubins = true;
        int cnt = 5000, i = 0;
        float r = Mathf.Infinity; // TODO: Dynamic compute this r
        var solnCnt = 0;
        while (i++ < cnt)
        {
            TreeNode target = i == cnt ? goalNode : randomNode();
            var cheapest = findCheapestPoint(nodes, target, i != cnt);
            var cheapestNode = cheapest.first.first;
            var cheapestCost = cheapest.second;
            if (cheapestCost == Mathf.Infinity) continue;
            target = cheapest.first.second; // change to the target approximated
//            if ((target.pos - goalNode.pos).magnitude <= 0.5)
//            {
//                Debug.Log("found a soln");
//                solnCnt++;
//                informedLength = Mathf.Min(informedLength, cheapestCost * MAX_SPEED);
//                curBestTime = Mathf.Min(curBestTime, cheapestCost);
//                Debug.Log(target.pos);
//                Debug.Log(cheapestCost);
//            }
//            if (cheapestCost + (goalNode.pos - target.pos).magnitude / MAX_SPEED > curBestTime + 0.5)
//            {
//                usefulCnt++;
//                //i--;
//                continue;
//            }
            target.SetParent(cheapestNode);
            cheapestNode.children.Add(target);
            target.time = cheapestCost;
            addNode(nodes, target);
            //nodes.Add(target);
            reparent(startNode, target, goalNode);
        }
        Debug.Log("useful count = " + usefulCnt);
        Debug.Log("goalNode parent count " + (goalNode.parent != null));

        List<TreeNode> path = new List<TreeNode>();
        Utils.backtrackPath(nodes, startNode, goalNode, path);

        var filledPath = GetFilledPath(path);
        Utils.SaveFilledPath(filledPath);

        Debug.Log("path " + path.Count);
        Debug.Log("goal time " + goalNode.time);
        Debug.Log("path[cnt] " + path[path.Count - 1].pos.x + " " + path[path.Count - 1].pos.y + " " +
                  path[path.Count - 1].time);
        Debug.Log("reparentCnt " + reparentCnt);

        return filledPath;
    }

    private DubinInput dubinInput(DubinNode near, DubinNode target)
    {
        return DubinUtils.DubinSP(new DubinState(near), new DubinState(target), true);
    }

    private DynamicPointInput dynamicPointInput(TreeNode near, TreeNode target)
    {
        float t = 2 * (target.pos - near.pos).magnitude / (target.vel + near.vel).magnitude;
        var a = 2 * (target.pos - near.pos - near.vel * t) / (t * t);
        TreeNode dest = new TreeNode(target.pos, near.vel + a * t, 0);
        return new DynamicPointInput(a, t, dest);
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
        //TimerText.text = "0";
        kdTree = new KDTree(2);
        var t = DateTime.Now;
        path = PlanRrtStar();
        startTime = Time.time;
        Debug.Log("Computation time: " + (DateTime.Now - t));
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
        var pos = path[curPos].pos;
        transform.position = new Vector3(pos.x, pos.y, 1);
    }
}