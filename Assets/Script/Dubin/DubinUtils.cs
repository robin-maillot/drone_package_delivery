using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

class DubinUtils
{

    public static DubinState start, goal; // this is bad bad :)
    public static double r1, r2;
    public static double o1, o2;
    public static float MAX_SPEED;
    public static bool constAcc;

    public static List<Pair<Vector2, Vector2>> TangentLines(Circle c1, Circle c2)
    {
        
        double x1 = c1.pos.x;
        double y1 = c1.pos.y;
        double x2 = c2.pos.x;
        double y2 = c2.pos.y;
        double r1 = c1.r;
        double r2 = c2.r;
        double EPS = 0.00001f;
        double d_sq = Math.Pow(x2 - x1, 2) + Math.Pow(y2 - y1, 2);
        List<Pair<Vector2, Vector2>> res = new List<Pair<Vector2, Vector2>>();
        if (d_sq < Math.Pow(r1 - r2, 2))
        {
            if (Math.Abs(d_sq - Math.Max(r1, r2)) > EPS && d_sq - EPS < Math.Max(r1, r2))
            {
                return res;
            }
        }
        double d = Math.Sqrt(d_sq);
        double vx = (x2 - x1) / d;
        double vy = (y2 - y1) / d;
        for (int sign1 = +1; sign1 >= -1; sign1 -= 2)
        {
            double c = (r1 - sign1 * r2) / d;
            if (c * c > 1.0) continue; //want to be subtracting small from large, not adding
            double h = Math.Sqrt(Math.Max(0.0, 1.0 - c * c));

            for (int sign2 = +1; sign2 >= -1; sign2 -= 2)
            {
                double nx = vx * c - sign2 * h * vy;
                double ny = vy * c + sign2 * h * vx;
                res.Add(new Pair<Vector2, Vector2>(new Vector2((float)(x1 + r1 * nx), (float)(y1 + r1 * ny)),
                      new Vector2((float)(x2 + sign1 * r2 * nx), (float)(y2 + sign1 * r2 * ny))));
            }
        }
        return res;
    }

    public static double ArcLength(Vector2 center, Vector2 lhs, Vector2 rhs, double r, bool left)
    {
        const double EPS = 0.000001f;
        Vector2 vec1, vec2;
        vec1 = new Vector2(lhs.x - center.x, lhs.y - center.y);
        vec2 = new Vector2(rhs.x - center.x, rhs.y - center.y);

        double theta = Math.Atan2(vec2.y, vec2.x) - Math.Atan2(vec1.y, vec1.x);
        if (theta < EPS && left)
        {
            theta += 2 * Math.PI;
        } else if (theta > EPS && !left)
        {
            theta -= 2 * Math.PI;
        }

        return Math.Abs(theta * r);
    }

    public static DubinInput DubinSP(DubinState start, DubinState goal, bool constAcc = false)
    {
        DubinUtils.constAcc = constAcc;

        DubinUtils.start = start;
        DubinUtils.goal = goal;

        DubinUtils.o1 = start.omega;
        DubinUtils.o2 = goal.omega;

        DubinUtils.r1 = start.v / o1;
        DubinUtils.r2 = goal.v / o2;

        Circle agentLeft;
        Circle agentRight;
        Circle queryLeft;
        Circle queryRight;

        double theta = start.theta;
        theta += Math.PI / 2.0;
        if (theta > Math.PI)
            theta -= 2.0 * Math.PI;

        agentLeft = new Circle(start.pos.x + r1 * Math.Cos(theta), start.pos.y + r1 * Math.Sin(theta), r1);

        theta = start.theta;
        theta -= Math.PI / 2.0;

        if (theta < -Math.PI)
        {
            theta += 2.0 * Math.PI;
        }

        agentRight = new Circle(start.pos.x + r1 * Math.Cos(theta), start.pos.y + r1 * Math.Sin(theta), r1);

        theta = goal.theta;
        theta += Math.PI / 2.0;

        if (theta > Math.PI)
        {
            theta -= 2.0 * Math.PI;
        }

        queryLeft = new Circle(goal.pos.x + r2 * Math.Cos(theta), goal.pos.y + r2 * Math.Sin(theta), r2);

        theta = goal.theta;
        theta -= Math.PI / 2.0;

        if (theta < -Math.PI)
        {
            theta += 2.0 * Math.PI;
        }

        queryRight = new Circle(goal.pos.x + r2 * Math.Cos(theta), goal.pos.y + r2 * Math.Sin(theta), r2);

        DubinInput bestCSC = bestCSCInput(agentLeft, agentRight, queryLeft, queryRight);
//        DubinInput bestCCC = bestCCCInput(agentLeft, agentRight, queryLeft, queryRight);
//        DubinInput shortest = bestCCC.time > bestCSC.time ? bestCSC : bestCCC;
        DubinInput shortest = bestCSC;

        return shortest;
    }

    public static DubinInput bestCSCInput(Circle agentLeft, Circle agentRight, Circle queryLeft, Circle queryRight)
    {

        List<Pair<Vector2, Vector2>> RRTangents = TangentLines(agentRight, queryRight);
        List<Pair<Vector2, Vector2>> LLTangents = TangentLines(agentLeft, queryLeft);
        List<Pair<Vector2, Vector2>> RLTangents = TangentLines(agentRight, queryLeft);
        List<Pair<Vector2, Vector2>> LRTangents = TangentLines(agentLeft, queryRight);

        DubinInput best = new DubinInput(), next;
        next = RSRInput(RRTangents, agentRight, queryRight);
        if (next.time < best.time)
            best = next;
        next = LSLInput(LLTangents, agentLeft, queryLeft);
        if (next.time < best.time)
            best = next;
        next = RSLInput(RLTangents, agentRight, queryLeft);
        if (next.time < best.time)
            best = next;
        next = LSRInput(LRTangents, agentLeft, queryRight);
        if (next.time < best.time)
            best = next;
        return best;
    }

    public static DubinInput bestCCCInput(Circle agentLeft, Circle agentRight, Circle queryLeft, Circle queryRight)
    {
        DubinInput best = new DubinInput();
        return best;
    }

    public static Pair<double, double> computeAT(double s, double u, double v)
    {
        double a = (v * v - u * u) / (2 * s);
        double t = (v - u) / a;
        return new Pair<double, double>(a, t);
    }

    public static DubinInput RSRInput(List<Pair<Vector2, Vector2>> RRTangents, Circle agentRight, Circle queryRight)
    {
        DubinInput next = new DubinInput();

        DubinInput.Steer nextInput;

        if (RRTangents.Count > 0)
        {
            double arc1, arc2, arc3;
            // R
            arc1 = ArcLength(agentRight.pos, start.pos, RRTangents[0].first, r1, false);
            nextInput = new DubinInput.Steer(start.v, (float) -o1, (float)(arc1 / start.v));
            next.steer.Add(nextInput);
            // S
            arc2 = (RRTangents[0].first - RRTangents[0].second).magnitude;
            nextInput = new DubinInput.Steer(MAX_SPEED, 0, (float) (arc2 / MAX_SPEED));
            if (constAcc)
            {
                var at = computeAT(arc2, start.v, goal.v);
                nextInput = new DubinInput.Steer(start.v, 0, (float) at.second, (float) at.first);
            }
            next.steer.Add(nextInput);
            // R
            arc3 = ArcLength(queryRight.pos, RRTangents[0].second, goal.pos, r2, false);
            nextInput = new DubinInput.Steer(goal.v, (float) -o2, (float) (arc3 / goal.v));
            next.steer.Add(nextInput);
            next.updateTime();
        }
        return next;
    }

    public static DubinInput LSLInput(List<Pair<Vector2, Vector2>> LLTangents, Circle agentLeft, Circle queryLeft)
    {
        DubinInput next = new DubinInput();

        DubinInput.Steer nextInput;

        if (LLTangents.Count > 1)
        {
            double arc1, arc2, arc3;
            // L
            arc1 = ArcLength(agentLeft.pos, start.pos, LLTangents[1].first, r1, true);
            nextInput = new DubinInput.Steer(start.v, (float) o1, (float)(arc1 / start.v));
            next.steer.Add(nextInput);
            // S
            arc2 = (LLTangents[1].first - LLTangents[1].second).magnitude;
            nextInput = new DubinInput.Steer(MAX_SPEED, 0, (float)(arc2 / MAX_SPEED));
            if (constAcc)
            {
                var at = computeAT(arc2, start.v, goal.v);
                nextInput = new DubinInput.Steer(start.v, 0, (float)at.second, (float)at.first);
            }
            next.steer.Add(nextInput);
            // L
            arc3 = ArcLength(queryLeft.pos, LLTangents[1].second, goal.pos, r2, true);
            nextInput = new DubinInput.Steer(goal.v, (float) o2, (float)(arc3 / goal.v));
            next.steer.Add(nextInput);
            next.updateTime();
        }
        return next;
    }

    public static DubinInput RSLInput(List<Pair<Vector2, Vector2>> RLTangents, Circle agentRight, Circle queryLeft)
    {
        DubinInput next = new DubinInput();

        DubinInput.Steer nextInput;

        if (RLTangents.Count > 2)
        {
            double arc1, arc2, arc3;
            // R
            arc1 = ArcLength(agentRight.pos, start.pos, RLTangents[2].first, r1, false);
            nextInput = new DubinInput.Steer(start.v, (float)-o1, (float)(arc1 / start.v));
            next.steer.Add(nextInput);
            // S
            arc2 = (RLTangents[2].first - RLTangents[2].second).magnitude;
            nextInput = new DubinInput.Steer(MAX_SPEED, 0, (float)(arc2 / MAX_SPEED));

            if (constAcc)
            {
                var at = computeAT(arc2, start.v, goal.v);
                nextInput = new DubinInput.Steer(start.v, 0, (float)at.second, (float)at.first);
            }
            next.steer.Add(nextInput);
            // L
            arc3 = ArcLength(queryLeft.pos, RLTangents[2].second, goal.pos, r2, true);
            nextInput = new DubinInput.Steer(goal.v, (float) o2, (float)(arc3 / goal.v));
            next.steer.Add(nextInput);
            next.updateTime();
        }
        return next;
    }

    public static DubinInput LSRInput(List<Pair<Vector2, Vector2>> LRTangents, Circle agentLeft, Circle queryRight)
    {
        DubinInput next = new DubinInput();

        DubinInput.Steer nextInput;

        if (LRTangents.Count > 3)
        {
            double arc1, arc2, arc3;
            // L
            arc1 = ArcLength(agentLeft.pos, start.pos, LRTangents[3].first, r1, true);
            nextInput = new DubinInput.Steer(start.v, (float) o1, (float)(arc1 / start.v));
            next.steer.Add(nextInput);
            // S
            arc2 = (LRTangents[3].first - LRTangents[3].second).magnitude;
            nextInput = new DubinInput.Steer(MAX_SPEED, 0, (float)(arc2 / MAX_SPEED));
            if (constAcc)
            {
                var at = computeAT(arc2, start.v, goal.v);
                nextInput = new DubinInput.Steer(start.v, 0, (float)at.second, (float)at.first);
            }
            next.steer.Add(nextInput);
            // R
            arc3 = ArcLength(queryRight.pos, LRTangents[3].second, goal.pos, r2, false);
            nextInput = new DubinInput.Steer(goal.v, (float)-o2, (float)(arc3 / goal.v));
            next.steer.Add(nextInput);
            next.updateTime();
        }
        return next;
    }

//    public static DubinInput LRLInput(double interiorTheta, Circle agentLeft, Circle queryLeft)
//    {
//        DubinInput next;
//        double arc1, arc2, arc3;
//        DubinInput.Steer nextInput;
//        Vector2 agentTan, queryTan;
//
//        Circle rCircle = new Circle(agentLeft.pos.x + (2 * r1))
//    }
}
