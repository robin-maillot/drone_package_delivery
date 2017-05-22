using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KDTreeDLL;
using UnityEngine.Assertions;
using Random = UnityEngine.Random;

public class Drone : Point
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



    //-----------------------------------------------------------------------------------------------------------
    //---------------------------------------- Input Components -------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Component - General Bias Against walls
    Vector2 ObstacleAvoid()
    {
        var avoid = new Vector2(0F,0F);
        foreach (var poly in this.polygons)     
        {
            float dist = Mathf.Infinity;

            // Uses the unity function to calculate closest point
            Vector2 dirn = new Vector2(this.transform.position.x - this.closestBuildingPoint.x, this.transform.position.y - this.closestBuildingPoint.y); // x and y used to be inversed
            dist = Mathf.Abs(Vector2.Distance(transform.position, this.closestBuildingPoint));
            dirn.Normalize();                       // ^ p2x - p1x, p1y - p2y

            dirn *= Mathf.Pow(obsMultiplier / (dist), 2);

            if (Vector2.Dot(vel, dirn) < 0 && dist < 5)
            {
                Debug.Log("Distance: " + dist + " dirn " + dirn + " Dot: " + Vector2.Dot(vel, dirn));
                Debug.DrawLine(transform.position, transform.position + new Vector3(dirn.x, dirn.y, 0F), Color.magenta);
                avoid += dirn;
            }
                

        }
        return avoid;
    }

    //Component - Not in current use
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


    //Component - Pull toward goal/waypoint
    Vector2 GoalComponent()
    {
        var x = this.goalPos[0] - this.transform.position.x;
        var y = this.goalPos[1] - this.transform.position.y;
        var comp = new Vector2(x, y);
        comp.Normalize();
        return comp;
    }

    //Component - vector to keep formation
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


    //-----------------------------------------------------------------------------------------------------------
    //----------------------------------------- Component Utils -------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Used for formation maintanence. Note that I is not used
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

    //Initiates PIDs for the first time
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


    //-----------------------------------------------------------------------------------------------------------
    //----------------------------------------- Input Control ---------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Function using vector weights to give direction. 
    Vector3 GetInput()
    {
        var goalcomp = GoalComponent();
        var formcomp = FormationComponent(false);
        var obsavoid = ObstacleAvoid();
        var edgeavoid = AvoidWalls();

        //main weights. xMag means the component of x
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

        //Shows directions
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, transform.position.z), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, transform.position.z), velcolour);
        Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, transform.position.z), new Vector3(transform.position.x + acc.x, transform.position.y + acc.y, transform.position.z), Color.black);

        return transform.position + new Vector3(vel.x, vel.y, 0F) * dt;
    }


    //-----------------------------------------------------------------------------------------------------------
    //----------------------------------------- Collision Checks ------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Checks to see if we will collide with a wall using standard kinematics (using unity collision). 
    Vector3 CollisionImminant()
    {
        //bool collision = false;
        var dt = Time.deltaTime;

        var t_col = this.vel.magnitude / MAX_ACCEL;
        var d = Mathf.Abs(vel.magnitude) * t_col - MAX_ACCEL * t_col * t_col * 0.5; //maximum possible distance travelled

        float dist = Mathf.Infinity;
            if (!collision)
        {
            dist = Vector3.Distance(transform.position, closestBuildingPoint);
            Vector3 dirn = closestBuildingPoint - transform.position;
            //Debug.Log("dist: " + dist + " Guard: " + guardID);
            if (dist < d + 0.5 && Vector2.Dot(vel, new Vector2(dirn.x, dirn.y)) < 0)
            {
                collision = true;
                t_dur = t_col;
                t_run = 0;
                coll_acc = new Vector2(dirn.x, dirn.y);
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
            Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, transform.position.z), new Vector3(transform.position.x + vel.x, transform.position.y + vel.y, transform.position.z), velcolour);
            Debug.DrawLine(new Vector3(transform.position.x, transform.position.y, transform.position.z), new Vector3(transform.position.x + coll_acc.x, transform.position.y + coll_acc.y, transform.position.z), Color.red);

            return transform.position + new Vector3(vel.x, vel.y, 0F) * dt;
        }
        return new Vector3(0F, 0F, 0F);
    }


    //-----------------------------------------------------------------------------------------------------------
    //-------------------------------------------- Utilaties ----------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Function borrowed from the internet. Used a few times. 
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



    //-----------------------------------------------------------------------------------------------------------
    //------------------------------------------- Game Instances ------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

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

    //Defines the choice of input
    void UpdatePosition()
    {
        float time = totalTime;
        Vector3 input = new Vector3();
        var dt = Time.deltaTime;

        input = CollisionImminant();    //This function is the red one that really avoids the walls. Unless we are in the final stretch, we check for collisions. This changes the "collision" variable to true (think like holding a value in a behavoir tree)
        if (!collision)     //If we don't expect a collision in the walls
        {
            velcolour = Color.blue;
            input = GetInput();     //Regurlar Input
        }

        // Add force of wind (assumes acc = F, ie m = 1)

        input += GameManager.wind * dt * dt;

        transform.position = input;
    }

}


