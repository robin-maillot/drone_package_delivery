using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KDTreeDLL;
using UnityEngine.Assertions;
using Random = UnityEngine.Random;

public class DynamicGuard : Point
{
    private const float DELTA_T = 0.01F;
    public Vector2 guardPos = new Vector2(0F,0F);
    private bool inFinalInput = false;
    private Vector2 acc;
    private float vox, voy;
    private PID[] pid;
    private bool collision = false;
    private float t_run = 0F;
    private float t_dur = 0F;
    private Vector2 coll_acc;
    public bool initialRush = true;
    public bool finished = false;
    private float finalrad = 0;
    public float goalMag = 1;
    public float formMag = 20;
    public float obsMultiplier = 3;
    public Color velcolour = Color.green;

    private float[][] integral;
    private float[][] prev_error;

    /*private Vector2 acc = new Vector2();

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.black;  //why the hell do they have the American spelling of color with the English spelling of grey?
        Gizmos.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + acc.x*100, transform.position.y + acc.y * 100, 20));
    }*/

    Vector2 ObstacleAvoid()
    {
        var avoid = new Vector2(0F,0F);
        foreach (var poly in this.polygons)     
        {
            float dist = Mathf.Infinity;
            var dirn = new Vector2(Mathf.Infinity, Mathf.Infinity);
            for (int i = 0; i < poly.Length; i++)   //each poly defines a new polygon, only need to return closest side
            {  
                int j = (i+1)% poly.Length;

                var closestpnt = ClosestPointOnLine(new Vector3(poly[i][0], poly[i][1],0F), new Vector3(poly[j][0], poly[j][1], 0F), transform.position);
                var dist2 = Mathf.Abs(Vector2.Distance(transform.position, closestpnt));
                if (dist2 < dist)
                {
                    dist = dist2;
                    if (closestpnt == new Vector3(poly[i][0], poly[i][1], closestpnt.z))  //if closest point is the vertices, it is not between the vertices
                    {
                        dirn = new Vector2(this.transform.position.y - poly[i][1], this.transform.position.x - poly[i][0]);
                    }
                    else if (closestpnt == new Vector3(poly[j][0], poly[j][1], closestpnt.z))  //if closest point is the vertices, it is not between the vertices
                    {
                        dirn = new Vector2(this.transform.position.y - poly[j][1], this.transform.position.x - poly[j][0]); // might need to swap one of these
                    }
                    else
                    {
                        //dirn = new Vector2(poly[j][0] - poly[i][0], poly[i][1] - poly[j][1]);        //if the guard decides to suicide into the wall, this is the reason
                        dirn = new Vector2(poly[j][1] - poly[i][1], poly[i][0] - poly[j][0]);
                    }
                    dirn.Normalize();                       // ^ p2x - p1x, p1y - p2y
                    dirn *= Mathf.Pow(obsMultiplier / (dist), 2);
                }                                 
            }
            
            if (Vector2.Dot(vel, dirn) < 0 && dist < 5)
            {
                Debug.Log("Distance: " + dist + " dirn " + dirn + " Dot: " + Vector2.Dot(vel, dirn));
                Debug.DrawLine(transform.position, transform.position + new Vector3(dirn.x, dirn.y, 0F), Color.magenta);
                avoid += dirn;
            }
                

        }
        return avoid;
    }

    Vector2 AvoidWalls()
    {
        float dist = Mathf.Infinity;
        var avoid = new Vector2(0F, 0F);
        for (int i = 0; i < boundaryPolygon.Length; i++)    //for each wall segment
        {
            int j = (i + 1) % (boundaryPolygon.Length);
            var dirn = new Vector2(Mathf.Infinity, Mathf.Infinity);
            var closestpnt = ClosestPointOnLine(new Vector3(boundaryPolygon[i][0], boundaryPolygon[i][1], 0F), new Vector3(boundaryPolygon[j][0], boundaryPolygon[j][1], 0F), transform.position);
            var dist2 = Vector2.Distance(transform.position, closestpnt);
            if (dist2 < dist)
            {
                dist = dist2;
                dirn = new Vector2(boundaryPolygon[i][1] - boundaryPolygon[j][1], boundaryPolygon[j][0] - boundaryPolygon[i][0]);        //if the guard decides to suicide into the wall, this is the reason
                dirn.Normalize();                       // ^ p2x - p1x, p1y - p2y
                dirn *= dist2;
            }
            if (dirn.x != 0)
                dirn.x = Mathf.Pow(obsMultiplier / (dirn.x), 2);

            if (dirn.y != 0)
                dirn.y = Mathf.Pow(obsMultiplier / (dirn.y), 2);
            avoid += dirn;
        }
        return avoid;
    }

    Vector2 GoalComponent()
    {
        var x = this.goalPos[0] - this.transform.position.x;
        var y = this.goalPos[1] - this.transform.position.y;
        var comp = new Vector2(x, y);
        comp.Normalize();
        return comp;
    }


    Vector2 FormationComponent(bool rush)
    {
        float x = 0, y = 0;
        int j = 0, iteration = 0;
        for (int i = 0; i < this.formation.Count+1; i++) //9 max guards
        {
            if(i == this.guardID)
            {
                j++;    //hax
                continue;
            }
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                var pos = gObj.transform.position;
                var xerr = (pos.x - this.transform.position.x) - this.formation[i-j].x;
                var yerr = (pos.y - this.transform.position.y) - this.formation[i-j].y;
                x += PIDs(xerr, iteration, 0, rush);  //i = 0-4, need 0-6 -> doesn't matter, so long as consistent
                y += PIDs(yerr, iteration, 1, rush);


                if (float.IsNaN(x) || float.IsNaN(y))
                {
                    Debug.Log("Fak");
                }
                //PID fubar = new PID();
                //fubar.error;
                //Debug.Log("x Error between " + this.guardID + " and " + i + " is " + x);
                //Debug.Log("y Error between " + this.guardID + " and " + i + " is " + y);
            }
            iteration++;
        }
        this.formationError = new Vector2(x, y);
        return formationError;
    }

    float PIDs(float error, int connection, int xy, bool rush) //0 = x, 1 = y
    {
        if (!rush && Ki > 0)
        {
            if (error < 1)
                integral[connection][xy] = 0F;
            integral[connection][xy] += integral[connection][xy] + (error * Time.deltaTime)/10000;
            if (integral[connection][xy] == Mathf.Infinity)
                integral[connection][xy] = 0F;
        }
        var derivative = (error - prev_error[connection][xy]) / Time.deltaTime;
        var output = Kp * error + Ki * integral[connection][xy] * 10000 + Kd * derivative;
        prev_error[connection][xy] = error;
        return output;
    }


    void InitiatePIDs()
    {
        //int j = 0;
        integral = new float[formation.Count][];
        prev_error = new float[formation.Count][];
        for (int i = 0; i < integral.Length; i++)
        {
            integral[i] = new float[] { 0, 0 };
            prev_error[i] = new float[] { 0, 0 };
        }
    }

    Vector3 GetInput()
    {
        //var pid: PID;    // Set values in the inspector.
        //var correction = pid.Update(setSpeed, actualSpeed, Time.deltaTime);

        var goalcomp = GoalComponent();
        var formcomp = FormationComponent(false);
        var obsavoid = ObstacleAvoid();
        var edgeavoid = AvoidWalls();
        //Debug.Log("Guard ID: " + guardID + ", Error: " + Mathf.Sqrt(Mathf.Pow(obsavoid.x,2) + Mathf.Pow(obsavoid.y,2)));
        //Debug.Log("Guard: "+guardID+" Edge: (" + edgeavoid.x +", "+edgeavoid.y +")");

        var x = goalcomp.x * goalMag + formcomp.x * formMag + obsavoid.x;// + edgeavoid.x;
        var y = goalcomp.y * goalMag + formcomp.y * formMag + obsavoid.y;// + edgeavoid.y;

        var acc = new Vector2(x, y);
        if (acc.magnitude > MAX_ACCEL)
        {
            acc.Normalize();
            acc *= MAX_ACCEL;
        }

        var dt = Time.deltaTime;
        vel += acc * dt;
        if (vel.magnitude > MAX_SPEED)
        {
            vel.Normalize();
            vel *= MAX_SPEED;
        }

        //we're feeding it position + acceleration componenent, which is wrong
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, 20), velcolour);
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + acc.x, transform.position.y + acc.y, 20), Color.black);

        return transform.position + new Vector3(vel.x, vel.y, 0F) * dt;
    }

    Vector3 GetInitialInput()
    {
        var formcomp = FormationComponent(true);
        var obsavoid = ObstacleAvoid();
        var edgeavoid = AvoidWalls();

        var x = formcomp.x;// + edgeavoid.x; // + obsavoid.x;// + edgeavoid.x;
        var y = formcomp.y;// + edgeavoid.y;// + obsavoid.y;// + edgeavoid.y;

        var acc = new Vector2(x, y);
        if (acc.magnitude > MAX_ACCEL)
        {
            acc.Normalize();
            acc *= MAX_ACCEL;
        }

        var dt = Time.deltaTime;
        vel += acc * dt;
        if (vel.magnitude > MAX_SPEED)
        {
            vel.Normalize();
            vel *= MAX_SPEED;
        }

        //we're feeding it position + acceleration componenent, which is wrong
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, 20), velcolour);
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + acc.x, transform.position.y + acc.y, 20), Color.black);
        return transform.position + new Vector3(vel.x, vel.y, 0F) * dt;
    }

    void Formcheck()
    {
        float x = 0, y = 0;
        int j = 0;
        for (int i = 0; i < this.formation.Count + 1; i++) //9 max guards
        {
            if (i == this.guardID)
            {
                j++;    //hax
                continue;
            }
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                var pos = gObj.transform.position;
                x += (pos.x - this.transform.position.x) - this.formation[i - j].x;
                y += (pos.y - this.transform.position.y) - this.formation[i - j].y;

                if (float.IsNaN(x) || float.IsNaN(y))
                {
                    Debug.Log("Fak");
                }
            }
        }
        var error = new Vector2(x, y).magnitude;
        if (error < 1) //arbitrary
            initialRush = false;
    }

    // Use this for initialization
    void Start()
    {
        vel = new Vector2(startVel[0], startVel[1]);
        goalMag = GameManager.goalMag;
        formMag = GameManager.formMag;
        obsMultiplier = GameManager.obsMultiplier;
        InitiatePIDs();
    }

    // Update is called once per frame
    private float totalTime = 0F;
    void Update()
    {
        totalTime += Time.deltaTime;
        UpdatePosition();
    }

    
    void UpdatePosition()
    {
        float time = totalTime;
        Vector3 input = new Vector3();
        var distance = Vector3.Distance(transform.position, new Vector3(goalPos[0], goalPos[1], transform.position.z));
        if (inFinalInput == true || distance < GameManager.endRange)
        {

            if (!finished)
                input = getFinalInputOld();    //Final Input
                //input = getFinalInputOld();    //Final Input
            else
            {
                input = transform.position;
                var x = Mathf.Sin(finalrad);
                var y = Mathf.Cos(finalrad);
                Debug.DrawLine(transform.position, transform.position + 2 * new Vector3(x, y, 0F), Color.white);
                Debug.DrawLine(transform.position, transform.position - 2 * new Vector3(x, y, 0F), Color.white);
                finalrad += Time.deltaTime*10F;
                finalrad %= 2F*Mathf.PI;
            }
            if (distance < 0.01 || t_run > t_dur)
                finished = true;
        }
        else
        {
            input = CollisionImminant();
            if (!collision)
            {
                if(initialRush)
                {
                    input = GetInitialInput();
                    Formcheck();   //not inform, in form
                }
                else
                {
                    velcolour = Color.blue;
                    input = GetInput();     //Regurlar Input
                }
            }
            
        }
        

        transform.position = input;
    }


    Vector3 CollisionImminant()
    {
        //bool collision = false;
        var dt = Time.deltaTime;

        var t_col = this.vel.magnitude / MAX_ACCEL;
        var d = Mathf.Abs(vel.magnitude) * t_col - MAX_ACCEL * t_col * t_col * 0.5;

        var t_2col = this.vel.magnitude / (Mathf.Sqrt(MAX_ACCEL));  //assumes that polygons are not more than a 90 degree angle
        var d_2poly = Mathf.Abs(vel.magnitude) * t_col - MAX_ACCEL * t_col * t_col * 0.5;
        int poly_colnumber = 0;
        var sign = 1;
        Vector2[] dir_2poly = new Vector2[2];


        float dist = Mathf.Infinity;
        if (!collision)
        { 
            var dirn = new Vector2(Mathf.Infinity, Mathf.Infinity);
            //foreach (var poly in this.polygons)
            for(int p = 0; p < polygons.Length+1; p++)
            {
                Vector2[] poly;
                if (p < polygons.Length)
                    poly = polygons[p];
                else
                {
                    poly = boundaryPolygon;
                    sign = -1;

                }
                for (int i = 0; i < poly.Length; i++)   //each poly defines a new polygon, only need to return closest side
                {
                    int j = (i + 1)% poly.Length;
                    var closestpnt = ClosestPointOnLine(new Vector3(poly[i][0], poly[i][1], 0F), new Vector3(poly[j][0], poly[j][1], 0F), transform.position);
                    var dist2 = Vector2.Distance(transform.position, closestpnt);
                    if(dist2 < d_2poly) //might have collisions with two polygons
                    {
                        var dirp = new Vector2(poly[j][1] - poly[i][1], poly[i][0] - poly[j][0]); //This is backwards. It works and I don't know why
                        dirp.Normalize();
                        dirp *= MAX_ACCEL;
                        dir_2poly[poly_colnumber] = dirp;
                        poly_colnumber++;
                        if (poly_colnumber > 1)
                            break;
                    }

                    if (dist2 < dist)  //check shortest distance
                    {
                        dist = dist2;
                        if (closestpnt == new Vector3(poly[i][0], poly[i][1], 0F))  //if closest point is the vertices, it is not between the vertices
                        {
                            dirn = new Vector2(this.transform.position.x - poly[i][0], this.transform.position.y - poly[i][1]);
                        }
                        else if (closestpnt == new Vector3(poly[j][0], poly[j][1], 0F))  //if closest point is the vertices, it is not between the vertices
                        {
                            dirn = new Vector2(this.transform.position.x - poly[j][0], this.transform.position.y - poly[j][1]); // might need to swap one of these
                        }
                        else
                        {
                            dirn = new Vector2(poly[j][1] - poly[i][1], poly[i][0] - poly[j][0]); //This is backwards. It works and I don't know why
                        }
                        dirn.Normalize();
                        dirn *= MAX_ACCEL * sign;
                    }
                }
                if (poly_colnumber > 1)
                {
                    collision = true;
                    t_dur = t_2col;
                    t_run = 0;
                    dirn = dir_2poly[0] + dir_2poly[1];
                    dirn.Normalize();
                    dirn *= MAX_ACCEL;
                    coll_acc = dirn;
                    Debug.Log("TWO COLLISIONS IMMINANT! FUCK!");
                    Debug.Log("btw the guard is number " + guardID);
                    break;
                }
            }
            //Debug.Log("dist: " + dist + " Guard: " + guardID);
            if (dist < d + 0.5 && Vector2.Dot(vel, dirn) < 0)
            {
                collision = true;
                t_dur = t_col;
                t_run = 0;
                coll_acc = dirn;
                //Debug.Log("COLLISION IMMINANT: GUARD " + guardID);
                //Debug.Log("d: " + d + ", dist: " + dist + ", velmag: " + vel.magnitude );
                //Debug.Log("dirn: " + Vector2.Dot(vel, dirn));
            }
        }
        if (t_run > t_dur)
            collision = false;
        if (collision) //can't do else, since we need to check this after the first point
        {
            vel += coll_acc * dt;
            t_run += dt;
            Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, 20), velcolour);
            Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + coll_acc.x, transform.position.y + coll_acc.y, 20), Color.red);

            return transform.position + new Vector3(vel.x, vel.y, 0F) * dt;
        }
        return new Vector3(0F, 0F, 0F);
    }







    Vector3 ClosestPointOnLine(Vector3 vA, Vector3 vB, Vector3 vPoint)
    {
        var vVector1 = vPoint - vA;
        var vVector2 = (vB - vA).normalized;

        var d = Vector3.Distance(vA, vB);
        var t = Vector3.Dot(vVector2, vVector1);
        if (t <= 0)
            return vA;

        if (t >= d)
            return vB;

        var vVector3 = vVector2 * t;

        var vClosestPoint = vA + vVector3;

        return vClosestPoint;
    }


    Vector3 getFinalInputOld()
    {
     
        if (inFinalInput == false)
        {
            //double tx = prev.pos.x, ty = prev.pos.y, tvx = prev.vel.x, tvy = prev.vel.y;
            var goalpos = new Vector3(goalPos[0], goalPos[1], 0F);
            var goalvel = new Vector2(goalVel[0], goalVel[1]);

            //float t = 2 * Vector3.Distance(goalpos, transform.position) / (goalvel + vel).magnitude;
            float timex = 2 * (goalPos[0] - transform.position.x) / (2F + vel.x);//(goalvel[0] + vel.x);
            float timey = 2 * (goalPos[1] - transform.position.y) / (2F + vel.y);
            float t = Mathf.Max(timex, timey);
            Debug.Log("Time to dest: " + t);
            var accel = 2 * (goalpos - transform.position - new Vector3(vel[0], vel[1], 0F) * t) / (t * t);

            acc = new Vector2(accel.x, accel.y);
            t_run = 0F;
            t_dur = t;
            if (acc.magnitude > MAX_ACCEL)
            {
                acc.Normalize();
                acc *= MAX_ACCEL;
            }
            vox = vel.x;//goalVel[0];//vel.x;
            voy = vel.y; //goalVel[1];//vel.y;
            inFinalInput = true;
        }
        //var dt = Time.deltaTime;
        var dt = Time.deltaTime;
        t_run += dt;
        float tx = transform.position.x, ty = transform.position.y;
        tx = tx + vel.x * dt + acc.x * dt * dt * 0.5F;
        ty = ty + vel.y * dt + acc.y * dt * dt * 0.5F;

        vel += acc * dt;
        if (vel.magnitude > MAX_SPEED)
        {
            vel.Normalize();
            vel *= MAX_SPEED;
        }

        //we're feeding it position + acceleration componenent, which is wrong
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, 20), velcolour);
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + acc.x, transform.position.y + acc.y, 20), Color.red);

        return new Vector3(tx, ty, 20F);// * Time.deltaTime;
    }


    private DubinInput input;
    private int finalstatus = 0;
    private float theta;
    //private Vector2 prevel;
    Vector3 getFinalInput()
    {

        if (inFinalInput == false)  //plan Dubins curves
        {


            //cost
            var position = new float[] { transform.position.x, transform.position.y };
            var velocity = new float[] { vel.x, vel.y };
            var current = nodeToDubinNode(position, velocity);
            var goal = nodeToDubinNode(goalPos, goalVel);
            input = dubinInput(current, goal);
            t_run = 0;
            theta = current.theta;
            inFinalInput = true;
            var magacc = new Vector2(input.steer[0].v * input.steer[0].omega, input.steer[0].v * input.steer[0].omega).magnitude;
            Debug.Log("Enter Case 0, Guard: " + guardID + " Time: " + input.steer[0].time + " Omega: " + input.steer[0].omega + " Acc: " + magacc);
            //Debug.Break();
        }
        var dt = Time.deltaTime;

        var sonic = new Vector2(0F, 0F);
        switch (finalstatus)
        {
            case 0:
                theta += input.steer[0].omega*dt;
                t_run += dt;
                vel.x = input.steer[0].v * Mathf.Cos(theta);
                vel.y = input.steer[0].v * Mathf.Sin(theta);
                sonic = new Vector2(input.steer[0].v*input.steer[0].omega, input.steer[0].v * input.steer[0].omega);
                if (input.steer[0].omega > 0)
                    sonic = -sonic;
                sonic.Normalize();
                sonic *= MAX_ACCEL;
                if (t_run >= input.steer[0].time)
                {
                    Debug.Log("Enter Case 1, Guard: " + guardID + " Time: " + input.steer[1].time + " Omega: " + input.steer[1].omega + " Acc: " + input.steer[1].a);
                    //var appxacc = new Vector2((vel.x - prevel.x),(vel.y- prevel.y)).magnitude / dt;
                    //Debug.Log("Sonic.mag: " + sonic.magnitude + " Appx Accel: " + appxacc);
                    t_run = 0;
                    finalstatus++;
                }
                //prevel = vel;
                break;
            case 1:
                theta += input.steer[1].omega * dt;
                t_run += dt;
                vel.x += Mathf.Cos(theta) * input.steer[1].a * dt;
                vel.y += Mathf.Sin(theta) * input.steer[1].a * dt;
                sonic = new Vector2(Mathf.Cos(theta) * input.steer[1].a, Mathf.Sin(theta) * input.steer[1].a);
                //sonic = -sonic;
                //Debug.Break();
                if (t_run >= input.steer[1].time)
                {
                    t_run = 0;
                    finalstatus++;
                    Debug.Log("Enter Case 0, Guard: " + guardID + " Time: " + input.steer[2].time + " Omega: " + input.steer[2].omega + " Acc: " + sonic.magnitude);
                    //Debug.Break();
                }
                break;
            case 2:
                theta += input.steer[2].omega * dt;
                t_run += dt;
                vel.x = input.steer[2].v * Mathf.Cos(theta);
                vel.y = input.steer[2].v * Mathf.Sin(theta);
                sonic = new Vector2(input.steer[2].v * input.steer[2].omega, input.steer[2].v * input.steer[2].omega);
                if (input.steer[2].omega > 0)
                    sonic = -sonic;
                sonic.Normalize();
                sonic *= MAX_ACCEL;
                //Debug.Break();
                if (t_run >= input.steer[2].time)
                {
                    Debug.Log("Finished! Guard: " + guardID);
                    finished = true;
                    t_run = 0;
                    finalstatus++;
                    //Debug.Break();
                }
                break;
            default:
                vel.x = 0F;
                vel.y = 0F;
                finished = true;
                break;
        }

        //var dt = Time.deltaTime;
        float tx = transform.position.x, ty = transform.position.y;
        tx = tx + vel.x * dt;// + acc.x * dt * dt * 0.5F;
        ty = ty + vel.y * dt;// + acc.y * dt * dt * 0.5F;

//        vel += acc * dt;
        if (vel.magnitude > MAX_SPEED)
        {
            //Debug.Log("MAXIMUM VELOCITY FOR GUARD " + guardID);
            //Debug.Log("...fuck");
            vel.Normalize();
            vel *= MAX_SPEED;
        }

        //we're feeding it position + acceleration componenent, which is wrong
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, 20), velcolour);
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + sonic.x, transform.position.y + sonic.y, 20), Color.red);

        return new Vector3(tx, ty, 20F);// * Time.deltaTime;
    }

    private DubinInput dubinInput(DubinNode near, DubinNode target)
    {
        return DubinUtils.DubinSP(new DubinState(near), new DubinState(target), true);
    }

    DubinNode nodeToDubinNode(float[] pos, float[] velo)
    {
        var theta = Mathf.Atan2(velo[1], velo[0]);
        if (theta < 0)
            theta += 2F * Mathf.PI;
        var magni = new Vector2(velo[0], velo[1]).magnitude;
        return new DubinNode(new Vector2(pos[0], pos[1]), magni, theta,
            MAX_ACCEL / magni, 0F);
    }

}


