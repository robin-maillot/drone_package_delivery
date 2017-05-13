using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Newtonsoft.Json;
using UnityEngine;

public class Set
{
    public List<int> set;
    public int score;
    public Vector2 center;
    public float time;
    public int closestGuard;

    public Set(List<int> s, int si, Vector2 c)
    {
        this.set = s;
        this.score = si;
        this.center = c;
        this.time = 1000;
        this.closestGuard = -1;
    }

    //Returns the shortest time to get to the enter of the set s and the id of which guard would go there
    public float[] updateSetScores(Waypoint[] waypoints, Vector2[][] polygons)
    {
        float[] score = new float[2];
        score[1] = Mathf.Infinity;
        score[0] = -1;
        for (int i = 0; i < waypoints.Length; i++)
        {
            Waypoint w = waypoints[i];

            bool intersect = false;
            foreach (var polygon in polygons)
            {
                for (int k = 0; k < polygon.Length; k++)
                {
                    Vector2 p3 = polygon[k], p4 = polygon[(k + 1) % polygon.Length];
                    intersect = intersect || Utils.FasterLineSegmentIntersection(w.point, this.center, p3, p4);
                }
            }
            if (!intersect)
            {
                if ((w.time + Vector2.Distance(w.point, this.center)) < score[1])
                {
                    score[0] = i;
                    score[1] = (w.time + Vector2.Distance(w.point, this.center));
                    //Debug.Log(score[1]);
                }
            }
        }
        this.closestGuard = (int)score[0];
        this.time = score[1];
        return score;
    }

    public void getVisiblePointsFromPath(Waypoint[] guardPos, Vector2[] interestPoints, List<int> pointsLeftToSee, float r, Vector2[][] polygons)
    {
        float DELTA_X = (float)0.5;
        List<int> pointsSeen = new List<int>();
        int closestGuard = -1;
        float mind = Mathf.Infinity;
        for (var i = 0; i < guardPos.Length; i++)
        {
            float d = Vector2.Distance(guardPos[i].point, this.center);
            if (d < mind)
                closestGuard = i;
        }

        Vector2 v = (guardPos[closestGuard].point - this.center).normalized;
        var totalD = Vector2.Distance(guardPos[closestGuard].point ,this.center);
        float sumD = 0;
        while (sumD < totalD)
        {
            var dx = Mathf.Min(DELTA_X, totalD - sumD);
            sumD += dx;
            Set s = Utils.pointsInSight(guardPos[closestGuard].point + v * sumD, r, interestPoints, polygons);
            foreach (int j in s.set)
            {
                if (!pointsSeen.Contains(j))
                    pointsSeen.Add(j);
            }
        }
        this.set = pointsSeen;
        this.score = pointsSeen.Count;
        return;
    }
}

public struct Waypoint
{
    public Vector2 point;
    public float time;
    public List<int> set;

    public Waypoint(Vector2 p, float t,List<int> s)
    {
        this.point = p;
        this.time = t;
        this.set = s;
    }

    public void print()
    {
        Debug.Log("Point: ("+point[0]+","+point[1]+"), time: "+time+", can see "+set.Count);
    }
}

public struct Datastruct
{
    public List<Set> sets;
    public List<Waypoint>[] waypoints;  //Guards 1-N, list of waypoints [x,y]

    public Datastruct(List<Waypoint>[] w, List<Set> s)
    {
        this.waypoints = w;
        this.sets = s;
    }
}

class Utils
{
    public struct Edge
    {
        public int from, to;
        public float dist;

        public Edge(int f, int t, float d)
        {
            this.from = f;
            this.to = t;
            this.dist = d;
        }
    }

    public struct GreedyNode
    {
        public Set set;
        public int nbCandidates;
        public int depth;
        public GreedyNode[] bestCandidates;

        public GreedyNode(Set s, int N,int d)
        {
            this.set = s;
            this.nbCandidates = 0;
            this.bestCandidates = new GreedyNode[N];
            this.depth=d;
        }

        public void addCandidate(GreedyNode gn)
        {
            this.bestCandidates[nbCandidates] = gn;
            this.nbCandidates++;
        }
    }

    public struct GreedyTree
    {
        public GreedyNode root;

        public GreedyTree(GreedyNode root)
        {
            this.root = root;
        }

    }

    public static List<int> bf(int N, int s, int t, List<Edge> edges)
    {
        List<int> ans = new List<int>();
        float[] dist = new float[N];
        int[] pre = new int[N];
        for (int i = 0; i < N; i++) pre[i] = -1;
        for (int i = 0; i < N; i++) dist[i] = Mathf.Infinity;

        dist[s] = 0;
        for (int i = 0; i < N; i++)
        {
            foreach (Edge edge in edges)
            {
                if (dist[edge.from] + edge.dist < dist[edge.to])
                {
                    dist[edge.to] = dist[edge.from] + edge.dist;
                    pre[edge.to] = edge.from;
                }
            }
        }

        int cur = t;
        while (cur != s)
        {
            ans.Add(cur);
            cur = pre[cur];
        }
        ans.Add(s);
        ans.Reverse();
        Debug.Log("ans length " + ans.Count);
        return ans;
    }

    public static bool FasterLineSegmentIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
    {
        Vector2 a = p2 - p1;
        Vector2 b = p3 - p4;
        Vector2 c = p1 - p3;

        float alphaNumerator = b.y * c.x - b.x * c.y;
        float alphaDenominator = a.y * b.x - a.x * b.y;
        float betaNumerator = a.x * c.y - a.y * c.x;
        float betaDenominator = alphaDenominator; /*2013/07/05, fix by Deniz*/

        bool doIntersect = true;

        if (alphaDenominator == 0 || betaDenominator == 0)
        {
            doIntersect = false;
        }
        else
        {
            if (alphaDenominator >= 0)
            {
                if (alphaNumerator <= 0 || alphaNumerator >= alphaDenominator)
                {
                    doIntersect = false;
                }
            }
            else if (alphaNumerator >= 0 || alphaNumerator <= alphaDenominator)
            {
                doIntersect = false;
            }

            if (doIntersect && betaDenominator >= 0)
            {
                if (betaNumerator <= 0 || betaNumerator >= betaDenominator)
                {
                    doIntersect = false;
                }
            }
            else if (betaNumerator >= 0 || betaNumerator <= betaDenominator)
            {
                doIntersect = false;
            }
        }

        return doIntersect;
    }

    //returns indexes of visible points from interestPoints. This corresponds to a subset of all the interestPoints.
    public static Set pointsInSight(Vector2 c, float r,Vector2[] interestPoints,Vector2[][] polygons)
    {
        List<int> visiblePoints = new List<int>();
        int l = 0;
        for (int i = 0; i < interestPoints.Length; i++)
        {
            float dist = Vector2.Distance(c, interestPoints[i]);
            if (dist > r) continue;

            bool intersect = false;
            foreach (var polygon in polygons)
            {
                for (int k = 0; k < polygon.Length; k++)
                {
                    Vector2 p3 = polygon[k], p4 = polygon[(k + 1) % polygon.Length];
                    intersect = intersect || Utils.FasterLineSegmentIntersection(c, interestPoints[i], p3, p4);
                }
            }
            if (!intersect)
            {
                visiblePoints.Add(i);
                l++;
            }
        }
        return new Set(visiblePoints, l,c);
    }

    //returns indexes of visible points from interestPoints. This corresponds to a subset of all the interestPoints.
    public static Set pointsInSight(Vector2 c, float r, Vector2[] interestPoints, List<int> interestPointsUnseen, Vector2[][] polygons)
    {
        List<int> visiblePoints = new List<int>();
        int l = 0;
        for (int i = 0; i < interestPoints.Length; i++)
        {
            if (interestPointsUnseen.Contains(i) == false) continue;
            float dist = Vector2.Distance(c, interestPoints[i]);
            if (dist > r) continue;

            bool intersect = false;
            foreach (var polygon in polygons)
            {
                for (int k = 0; k < polygon.Length; k++)
                {
                    Vector2 p3 = polygon[k], p4 = polygon[(k + 1) % polygon.Length];
                    intersect = intersect || Utils.FasterLineSegmentIntersection(c, interestPoints[i], p3, p4);
                }
            }
            if (!intersect)
            {
                visiblePoints.Add(i);
                l++;
            }
        }
        return new Set(visiblePoints, l, c);
    }

    // Use Greedy algorithm to find N best subsets to cover the interestPoints. Needs to be improved to run multiple Greedies in parallel.
    // I wanna test this first but I'm not sure how.
    public static Set[] findBestSetsUsingGreedy(List<Set> subsets,int N)
    {
        int[] bestIndex = new int[N];
        int[] bestScore = new int[N];
        Set s, bestS, tempS;
        List<Set> tempSubsets = subsets;
        Set[] bestSets = new Set[N];
        for (int n=0;n<N; n++)
        {
            for (int i = 0; i < subsets.Count; i++)
            {
                s = subsets[i];
                if (s.score > bestScore[n])
                {
                    bestIndex[n] = i;
                    bestScore[n] = s.score;
                }
            }

            bestS = subsets[bestIndex[n]];
            bestSets[n] = bestS;
            subsets.RemoveAt(bestIndex[n]);
            for (int i = 0; i < subsets.Count; i++)
            {
                s = subsets[i];
                tempS = s;
                //Debug.Log("center"+s.center[0]+","+s.center[1]);
                List<int> elementsToRemove = new List<int>();
                foreach (int j in s.set)
                {
                    if (bestS.set.Contains(j))
                    {
                        //tempS.set.Remove(j);
                        elementsToRemove.Add(j);
                        tempS.score--;
                    }
                }
                foreach (int j in elementsToRemove)
                {
                    tempS.set.Remove(j);
                }
                subsets[i] = tempS;
            }
        }
        for(int i = 0; i < N; i++)
        {
            Debug.Log("Score: " + bestScore[i] + "Center: (" + bestSets[i].center[0] + "," + bestSets[i].center[1] + ")");
        }
        Debug.Log("Total covered:" + (float)bestScore.Sum());
        return bestSets;
        
    }

    public static Set[] getStartPositions(float[][] items, int numberofGuards,Map map)
    {
        float[][] start_positions = new float[numberofGuards][];
        Vector2[] interestPoints = new Vector2[items.Length];
        List<Set> subsets = new List<Set>();
        float t1 = Time.realtimeSinceStartup;
        for(int i=0;i<items.Length;i++)
        {
            interestPoints[i] = new Vector2(items[i][0], items[i][1]);
        }
        Debug.Log("Time transforming array to vector2:"+ (Time.realtimeSinceStartup-t1));
        t1 = Time.realtimeSinceStartup;
        foreach (var c in interestPoints)
        {
            Set s = pointsInSight(c, map.sensor_range, interestPoints, map.polygons);
            subsets.Add(s);
        }
        Debug.Log("Time creating sets:" + (Time.realtimeSinceStartup - t1));
        t1 = Time.realtimeSinceStartup;
        Set[] bestSets = findBestSetsUsingGreedy(subsets, numberofGuards);

        Debug.Log("Time finding running Greedy:" + (Time.realtimeSinceStartup - t1));

        return bestSets;
    }


    // CODE FOR TASK 2 //

    // Use Greedy algorithm to find N best subsets to cover the interestPoints. Needs to be improved to run multiple Greedies in parallel.
    // I wanna test this first but I'm not sure how.
    public static Waypoint[] weightedGreedy(List<Set> subsets, Waypoint[] guardPositions, Vector2[][] polygons,float max_speed)
    {
        int bestIndex=-1;
        float bestWeightedScore=Mathf.Infinity;
        float bestTime = Mathf.Infinity;
        Set s, bestS, tempS;
        List<Set> tempSubsets = subsets;
        Waypoint[] newGuardPositions = (Waypoint[])guardPositions.Clone();
        for (int i = 0; i < subsets.Count; i++)
        {
            s = subsets[i];
            if (s.closestGuard == -1) continue;
            if (s.time/ s.score < bestWeightedScore || bestWeightedScore==Mathf.Infinity)
            {
                bestIndex = i;
                bestWeightedScore = s.time / s.score;
                bestTime = s.time;
                //Debug.Log((s.time / s.score));
            }
            else if(s.time / s.score == bestWeightedScore)
            {
                if(s.time<bestTime)
                {
                    bestIndex = i;
                    bestWeightedScore = s.time / s.score;
                    bestTime = s.time;
                }
            }
        }
        //Debug.Log(bestIndex);
        if (bestIndex == -1)
            return null;
        bestS = subsets[bestIndex];
        subsets.RemoveAt(bestIndex);
        float[] updatedScore = bestS.updateSetScores(newGuardPositions, polygons);
        newGuardPositions[(int)updatedScore[0]].time = updatedScore[1]/max_speed;
        newGuardPositions[(int)updatedScore[0]].point = bestS.center;
        newGuardPositions[(int)updatedScore[0]].set = bestS.set;
        //Debug.Log("Moved guard " + (int)updatedScore[0]);
        //newGuardPositions[(int)updatedScore[0]].print();
        for (int i = 0; i < subsets.Count; i++)
        {
            s = subsets[i];
            tempS = s;
            List<int> elementsToRemove = new List<int>();
            foreach (int j in s.set)
            {
                if (bestS.set.Contains(j))
                {
                    elementsToRemove.Add(j);
                    tempS.score--;
                }
            }
            foreach (int j in elementsToRemove)
            {
                tempS.set.Remove(j);
            }
            tempS.time = updatedScore[1];
            subsets[i] = tempS;
        }
        return newGuardPositions;
    }

    public static Waypoint[] weightedGreedy2(List<Set> subsets, Waypoint[] guardPositions, Vector2[][] polygons,float r,Vector2[] interestPoints,float max_speed)
    {
        int bestIndex = -1;
        float bestWeightedScore = Mathf.Infinity;
        float bestTime = Mathf.Infinity;
        Set s, bestS, tempS;
        List<Set> tempSubsets = subsets;
        Waypoint[] newGuardPositions = (Waypoint[])guardPositions.Clone();
        for (int i = 0; i < subsets.Count; i++)
        {
            s = subsets[i];
            if (s.closestGuard == -1) continue;
            if (s.time / s.score < bestWeightedScore || bestWeightedScore == Mathf.Infinity)
            {
                bestIndex = i;
                bestWeightedScore = s.time / s.score;
                bestTime = s.time;
                //Debug.Log((s.time / s.score));
            }
            else if (s.time / s.score == bestWeightedScore)
            {
                if (s.time < bestTime)
                {
                    bestIndex = i;
                    bestWeightedScore = s.time / s.score;
                    bestTime = s.time;
                }
            }
        }
        //Debug.Log(bestIndex);
        if (bestIndex == -1)
            return null;
        bestS = subsets[bestIndex];
        subsets.RemoveAt(bestIndex);
        float[] updatedScore = bestS.updateSetScores(newGuardPositions, polygons);
        newGuardPositions[(int)updatedScore[0]].time = updatedScore[1]/max_speed;
        newGuardPositions[(int)updatedScore[0]].point = bestS.center;
        newGuardPositions[(int)updatedScore[0]].set = bestS.set;
        //Debug.Log("Moved guard " + (int)updatedScore[0]);
        //newGuardPositions[(int)updatedScore[0]].print();

        float DELTA_X = (float)0.05;
        List<int> interestPointsToRemove = new List<int>();
        Vector2 p1 = guardPositions[(int)updatedScore[0]].point;
        Vector2 p2 = newGuardPositions[(int)updatedScore[0]].point;
        Vector2 v = (p2 - p1).normalized;
        float totalD = Vector2.Distance(p1,p2);
        float sumD = 0;
        while (sumD < totalD)
        {
            float dx = Mathf.Min(DELTA_X, totalD - sumD);
            sumD += dx;
            Set s1 = pointsInSight(p1 + v * sumD, r, interestPoints, polygons);
            foreach (int j in s1.set)
            {
                if (!interestPointsToRemove.Contains(j))
                    interestPointsToRemove.Add(j);
            }
        }

        for (int i = 0; i < subsets.Count; i++)
        {
            s = subsets[i];
            tempS = s;
            List<int> elementsToRemove = new List<int>();

            foreach (int j in s.set)
            {
                if (interestPointsToRemove.Contains(j))
                {
                    elementsToRemove.Add(j);
                    tempS.score--;
                }
            }
            foreach (int j in elementsToRemove)
            {
                tempS.set.Remove(j);
            }
            tempS.time = updatedScore[1];
            subsets[i] = tempS;
        }
        Debug.Log("Greedy: " + interestPointsToRemove.Count + " points seen.");
        return newGuardPositions;
    }

    public static List<int> removeSeenPoints(List<int> s,List<int> interestPointsUnseen)
    {
        List<int> interestPointsUnseenNew = interestPointsUnseen;
        foreach (var i in s)
        {
            interestPointsUnseenNew.Remove(i);
        }
        return interestPointsUnseenNew;
    }



    public static List<int> removeSeenPointsFromPath(Waypoint guardPos, Vector2 endPoint,Vector2[] interestPoints, List<int> interestPointsUnseen, float r, Vector2[][] polygons)
    {
        float DELTA_X = (float)0.05;
        List<int> interestPointsUnseenNew = interestPointsUnseen;
        int s = interestPointsUnseen.Count;
        Vector2 v = (endPoint- guardPos.point).normalized;
        float totalD = Vector2.Distance(guardPos.point, endPoint);
        float sumD = 0;
        while (sumD < totalD)
        {
            float dx = Mathf.Min(DELTA_X, totalD - sumD);
            sumD += dx;
            Set s1 = Utils.pointsInSight(guardPos.point + v * sumD, r, interestPoints, polygons);
            foreach (int j in s1.set)
            {
                if (interestPointsUnseenNew.Contains(j))
                    interestPointsUnseenNew.Remove(j);
            }
        }
        Debug.Log("removeSeenPointsFromPath: " + (s - interestPointsUnseenNew.Count) + " points seen.");
        return interestPointsUnseenNew;
    }

    List<Waypoint> GetFilledWaypoints(List<Waypoint> path,List<int> interestPoints)
    {
        float MAX_SPEED = 1;
        float DELTA_T = (float)0.5;
        List<Waypoint> filledPath = new List<Waypoint>();
        filledPath.Add(path[0]);
        for (var i = 0; i < path.Count - 1; i++)
        {
            Vector2 v = (path[i].point - path[i].point).normalized * MAX_SPEED;
            var totalT = path[i + 1].time - path[i].time;
            float sumT = 0;
            while (sumT < totalT)
            {
                var dt = Mathf.Min(DELTA_T, totalT - sumT);
                sumT += dt;
                filledPath.Add(new Waypoint(path[i].point + v * sumT, path[i].time + sumT,interestPoints));
            }
        }
        return filledPath;
    }

    //public static Set[] getPositionsT2(float[][] items, int numberofGuards, Map map,float r, float[][] start_position)
    public static Datastruct getPositionsT2(float[][] items, int numberofGuards, Map map, float[][] start_position,float max_speed)
    {
        Vector2[] interestPoints = new Vector2[items.Length];
        List<int> interestPointsUnseen = new List<int>();
        List<Set> subsets = new List<Set>();
        List<Waypoint>[] guardWaypoints = new List<Waypoint>[numberofGuards];
        Waypoint[] guardCurrentPositions = new Waypoint[numberofGuards];
        Waypoint[] newGuardPositions = new Waypoint[numberofGuards];
        int runs = 0;

        // Initialize items
        for (int i = 0; i < items.Length; i++)
        {
            interestPoints[i] = new Vector2(items[i][0], items[i][1]);
            interestPointsUnseen.Add(i);

        }

        //Initialize position of the guards and remove initial points that are seen
        for (int i = 0; i < numberofGuards; i++)
        {
            Vector2 c = new Vector2(start_position[i][0], start_position[i][1]);
            Set s = pointsInSight(c, map.sensor_range, interestPoints, map.polygons);
            interestPointsUnseen = removeSeenPoints(s.set, interestPointsUnseen);
            Waypoint w = new Waypoint(new Vector2(start_position[i][0], start_position[i][1]),0, s.set);
            guardWaypoints[i] = new List<Waypoint>();
            guardWaypoints[i].Add(w);
            guardCurrentPositions[i] = w;
        }

        //Generate subsets
        foreach (var c in interestPoints)
        {
            Set s = pointsInSight(c, map.sensor_range, interestPoints, interestPointsUnseen, map.polygons);
            //s.getVisiblePointsFromPath(guardCurrentPositions, interestPoints, interestPointsUnseen, r, map.polygons);
            subsets.Add(s);
        }

        //While whole set isn't covered
        while (interestPointsUnseen.Count >= 1 && runs<30)
        {
            Debug.Log("Total Interest points to see: " + interestPoints.Length);
            Debug.Log("left Interest points to see: "+ interestPointsUnseen.Count);

            for (int i = 0; i < numberofGuards; i++)
            {
                guardCurrentPositions[i] = guardWaypoints[i].Last();
            }
            //Update distance from each subset to the closest guard
            foreach (Set s in subsets)
            {
                s.updateSetScores(guardCurrentPositions, map.polygons);
            }

            //Run weighted greedy and get new guard position
            newGuardPositions = weightedGreedy2(subsets, guardCurrentPositions,map.polygons,map.sensor_range,interestPoints,max_speed);
            if (newGuardPositions == null)
                break;
            int h=-1;
            for (int i = 0; i < numberofGuards; i++)
            {
                //Debug.Log("Guard: " + i + " moved to (" + newGuardPositions[i].point[0] + "," + newGuardPositions[i].point[1] + ")");
                //Debug.Log("Saw: " + newGuardPositions[i].set.Count);
                //Remove the new seen points from the unseenpoints
                //interestPointsUnseen = removeSeenPoints(newGuardPositions[i].set, interestPointsUnseen);
                if (guardCurrentPositions[i].point != newGuardPositions[i].point)
                {
                    h = i;
                    Debug.Log("Guard " + i + " moved");
                    interestPointsUnseen = removeSeenPointsFromPath(guardCurrentPositions[i], newGuardPositions[i].point, interestPoints, interestPointsUnseen, map.sensor_range, map.polygons);
                    //guardCurrentPositions[i] = newGuardPositions[i];
                }
            }
            guardCurrentPositions = (Waypoint[])newGuardPositions.Clone();
            guardWaypoints[h].Add(guardCurrentPositions[h]);

            //for (int i = 0; i < numberofGuards; i++)
            //{
            //    guardWaypoints[i].Add(guardCurrentPositions[i]);
            //}
            runs++;
        }
        for(int i = 0; i < guardWaypoints.Length; i++)
        {
            string s1 = "Guard "+i+": ";
            string s2 = "Guard " + i + ": ";
            for (int j = 0; j < guardWaypoints[i].Count; j++)
            {
                s1 += guardWaypoints[i][j].time+", ";
                s2 += guardWaypoints[i][j].point + ", ";

            }
            Debug.Log(s1);
            Debug.Log(s2);

        }
        float totalTime = 0;
        for (int i = 0; i < guardWaypoints.Length; i++)
        {
            if (guardWaypoints[i].Last().time > totalTime)
                totalTime = guardWaypoints[i].Last().time;
        }
        Debug.Log("Final: Total Interest points to see: " + interestPoints.Length);
        Debug.Log("Final: left Interest points to see: " + interestPointsUnseen.Count);
        Debug.Log("Runs: " + runs);
        Debug.Log("Task completed in: " + totalTime);
        return new Datastruct(guardWaypoints, subsets);
    }

    public static Datastruct getPositionsT4(float[][] items, int numberofGuards, Map map, float[][] start_position, float[][] end_position, float max_speed)
    {
        Vector2[] interestPoints = new Vector2[items.Length];
        List<int> interestPointsUnseen = new List<int>();
        List<Set> subsets = new List<Set>();
        List<Waypoint>[] guardWaypoints = new List<Waypoint>[numberofGuards];
        Waypoint[] guardCurrentPositions = new Waypoint[numberofGuards];
        Waypoint[] newGuardPositions = new Waypoint[numberofGuards];
        int runs = 0;

        // Initialize items
        for (int i = 0; i < items.Length; i++)
        {
            interestPoints[i] = new Vector2(items[i][0], items[i][1]);
            interestPointsUnseen.Add(i);

        }

        //Initialize position of the guards and remove initial points that are seen and points that will be seen at the end
        for (int i = 0; i < numberofGuards; i++)
        {
            Vector2 c_start = new Vector2(start_position[i][0], start_position[i][1]);
            Vector2 c_end = new Vector2(end_position[i][0], end_position[i][1]);
            Set s_start = pointsInSight(c_start, map.sensor_range, interestPoints, map.polygons);
            Set s_end = pointsInSight(c_end, map.sensor_range, interestPoints, map.polygons);
            interestPointsUnseen = removeSeenPoints(s_start.set, interestPointsUnseen);
            interestPointsUnseen = removeSeenPoints(s_end.set, interestPointsUnseen);

            Waypoint w = new Waypoint(new Vector2(start_position[i][0], start_position[i][1]), 0, s_start.set);
            guardWaypoints[i] = new List<Waypoint>();
            guardWaypoints[i].Add(w);
            guardCurrentPositions[i] = w;
        }

        //Generate subsets
        foreach (var c in interestPoints)
        {
            Set s = pointsInSight(c, map.sensor_range, interestPoints, interestPointsUnseen, map.polygons);
            //s.getVisiblePointsFromPath(guardCurrentPositions, interestPoints, interestPointsUnseen, r, map.polygons);
            subsets.Add(s);
        }

        //While whole set isn't covered
        while (interestPointsUnseen.Count >= 1 && runs < 50)
        {
            Debug.Log("Total Interest points to see: " + interestPoints.Length);
            Debug.Log("left Interest points to see: " + interestPointsUnseen.Count);

            for (int i = 0; i < numberofGuards; i++)
            {
                guardCurrentPositions[i] = guardWaypoints[i].Last();
            }
            //Update distance from each subset to the closest guard
            foreach (Set s in subsets)
            {
                s.updateSetScores(guardCurrentPositions, map.polygons);
            }

            //Run weighted greedy and get new guard position
            newGuardPositions = weightedGreedy2(subsets, guardCurrentPositions, map.polygons, map.sensor_range, interestPoints, max_speed);
            if (newGuardPositions == null)
                break;
            int h = -1;
            for (int i = 0; i < numberofGuards; i++)
            {
                if (guardCurrentPositions[i].point != newGuardPositions[i].point)
                {
                    h = i;
                    Debug.Log("Guard " + i + " moved");
                    interestPointsUnseen = removeSeenPointsFromPath(guardCurrentPositions[i], newGuardPositions[i].point, interestPoints, interestPointsUnseen, map.sensor_range, map.polygons);
                }
            }
            guardCurrentPositions = (Waypoint[])newGuardPositions.Clone();
            guardWaypoints[h].Add(guardCurrentPositions[h]);
            runs++;
        }

        //guardWaypoints = findEndPositions(end_position, guardWaypoints);

        for (int i = 0; i < numberofGuards; i++)
        {
            Vector2 c_end = new Vector2(end_position[i][0], end_position[i][1]);
            Set s_end = pointsInSight(c_end, map.sensor_range, interestPoints, map.polygons);

            Waypoint w = new Waypoint(c_end, Vector2.Distance(c_end,guardWaypoints[i].Last().point)/max_speed, s_end.set);
            guardWaypoints[i].Add(w);
        }

        for (int i = 0; i < guardWaypoints.Length; i++)
        {
            string s1 = "Guard " + i + ": ";
            string s2 = "Guard " + i + ": ";
            for (int j = 0; j < guardWaypoints[i].Count; j++)
            {
                s1 += guardWaypoints[i][j].time + ", ";
                s2 += guardWaypoints[i][j].point + ", ";

            }
            Debug.Log(s1);
            Debug.Log(s2);

        }
        float totalTime = 0;
        for (int i = 0; i < guardWaypoints.Length; i++)
        {
            if (guardWaypoints[i].Last().time > totalTime)
                totalTime = guardWaypoints[i].Last().time;
        }
        Debug.Log("Final: Total Interest points to see: " + interestPoints.Length);
        Debug.Log("Final: left Interest points to see: " + interestPointsUnseen.Count);
        Debug.Log("Runs: " + runs);
        Debug.Log("Task completed in: " + totalTime);
        return new Datastruct(guardWaypoints, subsets);
    }


    public static List<Waypoint>[] findEndPositions(float[][] end_position, List<Waypoint>[] guardWaypoints,float max_speed)
    {
        List<Waypoint>[] newGuardWaypoints = (List<Waypoint>[])guardWaypoints.Clone();
        int n = guardWaypoints.Length;
        int bestRow = -1;
        float totalTime = Mathf.Infinity;
        float[][] dist = new float[n][];
        for (int i = 0; i < n; i++)
        {
            dist[i] = new float[n];
        }

        for (int i = 0; i < n; i++)
        {
            Vector2 endVector = new Vector2(end_position[i][0], end_position[i][1]);
            for (int j = 0; j < n; j++)
            {
                dist[i][j] = Vector2.Distance(endVector, guardWaypoints[j].Last().point)/max_speed;
            }
        }
        for (int i = 0; i < n; i++)
        {
            if(dist[i].Max()< totalTime)
            {
                totalTime = dist[i].Max();
                bestRow = i;
            }
        }

         return newGuardWaypoints;
    }
    // Tree Greedy, NOT USED FOR NOW
    /*
    public static Set[] findNBestSetsUsingGreedy2(List<Set> subsets, int N)
    {
        Set[] bestSubsets = new Set[N];
        int[] bestScore = new int[N];
        Set s;
        for (int i = 0; i < subsets.Count; i++)
        {
            s = subsets[i];
            for(int j = 0; j < bestSubsets.Length; j++)
            {
                if (s.score > bestScore[j])
                {
                    bestSubsets[j] = s;
                    bestScore[j] = s.score;
                }
            }
        }
        return bestSubsets;
    }

    public static GreedyTree findBestSetsUsingImprovedGreedy(List<Set> subsets, int N,int Ncandidates)
    {
        GreedyNode start = new GreedyNode(new Set(new List<int>(),new int(),new Vector2()),Ncandidates,0);
        GreedyTree gt = new GreedyTree(start);
        GreedyNode parent;
        parent = start;
        improvedGreedyReccursive(subsets, start, 0, N, Ncandidates);
        return gt;
    }

    public static void improvedGreedyReccursive(List<Set> subsets, GreedyNode parent,int depth,int maxDepth,int Ncandidates)
    {
        if (parent.depth == maxDepth)
        {
            return;
        }
        List<Set> tempSubsets = subsets;
        GreedyNode child;
        Set[] s = findNBestSetsUsingGreedy2(tempSubsets, Ncandidates);
        for (int j = 0; j < Ncandidates; j++)
        {
            tempSubsets = subsets;
            child = new GreedyNode(s[j], Ncandidates, depth+1);
            parent.addCandidate(child);
            tempSubsets.Remove(s[j]);
            improvedGreedyReccursive(tempSubsets, child, depth, maxDepth, Ncandidates);
        }

        return;
    }


    // Useless for now (Can be modified to calulate intersection point between an obstacle and a circle later if needed)
    public static bool LineSegmentCircleIntersection(Vector2 p1, Vector2 p2, Vector2 center, float r)
    {
        Vector2 d = p2 - p1;
        Vector2 f = p1 - center;

        float a = Vector2.Dot(d,d);
        float b = 2 * Vector2.Dot(f,d);
        float c = Vector2.Dot(f,f) - r * r;
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            // no intersection
            return false;
        }
        else
        {
            // ray didn't totally miss sphere,
            // so there is a solution to
            // the equation.

            discriminant = (float)System.Math.Sqrt(discriminant);

            // either solution may be on or off the ray so need to test both
            // t1 is always the smaller value, because BOTH discriminant and
            // a are nonnegative.
            float t1 = (-b - discriminant) / (2 * a);
            float t2 = (-b + discriminant) / (2 * a);

            // 3x HIT cases:
            //          -o->             --|-->  |            |  --|->
            // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 

            // 3x MISS cases:
            //       ->  o                     o ->              | -> |
            // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

            if (t1 >= 0 && t1 <= 1)
            {
                // t1 is the intersection, and it's closer than t2
                // (since t1 uses -b - discriminant)
                // Impale, Poke
                return true;
            }

            // here t1 didn't intersect so we are either started
            // inside the sphere or completely past it
            if (t2 >= 0 && t2 <= 1)
            {
                // ExitWound
                return true;
            }

            // no intn: FallShort, Past, CompletelyInside
            return false;
        }

    }

        */
    /*static bool ContainsPoint(Vector2[] polyPoints, Vector2 p)
        {
            var j = polyPoints.Length - 1;
            var inside = false;
            for (var i = 0; i < polyPoints.Length; j = i++)
            {
                if (((polyPoints[i].y <= p.y && p.y < polyPoints[j].y) || (polyPoints[j].y <= p.y && p.y < polyPoints[i].y)) &&
                   (p.x < (polyPoints[j].x - polyPoints[i].x) * (p.y - polyPoints[i].y) / (polyPoints[j].y - polyPoints[i].y) + polyPoints[i].x))
                    inside = !inside;
            }
            return inside;
        }*/

    public static bool ContainsPoint(Vector2[] p, Vector2 v)
    {
        int j = p.Length - 1;
        bool c = false;
        for (int i = 0; i < p.Length; j = i++)
            c ^= p[i].y > v.y ^ p[j].y > v.y && v.x < (p[j].x - p[i].x) * (v.y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
        return c;
    }

    public static float gaussianRandom(float mean, float stdDev)
    {
        float u1 = Random.Range(0.0F, 1.0F); //these are uniform(0,1) random doubles
        float u2 = Random.Range(0.0F, 1.0F);
        float randStdNormal = Mathf.Sqrt(-2.0F * Mathf.Log(u1)) *
                              Mathf.Sin(2.0F * Mathf.PI * u2); //random normal(0,1)
        float randNormal =
            mean + stdDev * randStdNormal; //random normal(mean,stdDev^2)
        return randNormal;
    }

    public static void backtrackPath(List<TreeNode> nodes, TreeNode startNode, TreeNode current, List<TreeNode> path)
    {
        if (current.parent != null) backtrackPath(nodes, startNode, current.parent, path);
        path.Add(current);
    }

    public static void backtrackPath(List<DubinNode> nodes, DubinNode startNode, DubinNode current, List<DubinNode> path)
    {
        if (current.parent != null) backtrackPath(nodes, startNode, current.parent, path);
        path.Add(current);
    }

    public static void SaveFilledPath(List<Node> path)
    {
        string json = JsonConvert.SerializeObject(path, new Vector2Converter(), new NodeConverter());
        System.IO.File.WriteAllText(@"output.json", json);
    }


    public static void TopoSort(DubinNode root, List<Pair<DubinNode, float>> sorted, float dt)
    {
        foreach (var child in root.children)
        {
            TopoSort(child, sorted, child.time - root.time);
        }
        sorted.Add(new Pair<DubinNode, float>(root, dt));
        if (root.parent == null)
        {
            sorted.Reverse();
        }
    }


    public static List<Node> LoadFilledPath()
    {
        List<Node> path = null;
        try
        {
            var json = System.IO.File.ReadAllText(@"output.json");
            path =
                (List<Node>)
                JsonConvert.DeserializeObject(json, typeof(List<Node>), new Vector2Converter(), new NodeConverter());
        }
        catch
        {
        }
        return path;
    }

    static float minX = Mathf.Infinity,
        minY = Mathf.Infinity,
        maxX = Mathf.NegativeInfinity,
        maxY = Mathf.NegativeInfinity;


    //Generates a random point, then checks to see if it fits the boundary
    public static Vector2 randomPoint(Node startNode, Node goalNode, Vector2[] boundaryPolygon, Vector2[][] polygons)
    {
        if (minX == Mathf.Infinity)
        {
            foreach (Vector2 p in boundaryPolygon)
            {
                minX = Mathf.Min(minX, p.x);
                minY = Mathf.Min(minY, p.y);
                maxX = Mathf.Max(maxX, p.x);
                maxY = Mathf.Max(maxY, p.y);
            }
        }

        float x, y;
        do
        {
            Vector2 vertex;

            var rand = Random.Range(0.0F, 1.0F);
            if (rand < 0.05)
            {
                vertex = startNode.pos;
            }
            else if (rand < 0.1)
            {
                vertex = goalNode.pos;
            }
            else
            {
                // our optimization
                var polygonId = Random.Range(0, polygons.Length);
                var vertexId = Random.Range(0, polygons[polygonId].Length);
                //bias to obstacle vertex
                vertex = polygons[polygonId][vertexId];
            }

            const float GAUSSIAN_SIZE = 3.0F;
            x = Utils.gaussianRandom(vertex.x, GAUSSIAN_SIZE);
            y = Utils.gaussianRandom(vertex.y, GAUSSIAN_SIZE);
            //            x = Random.Range(minX, maxX);
            //            y = Random.Range(minY, maxY);
        } while (!(x > minX && x < maxX && y > minY && y < maxY) ||
                 !Cheetah.instance.IsBounded(new Vector2(x, y)));
        return new Vector2(x, y);
    }
}