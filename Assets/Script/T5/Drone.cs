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
    private Vector3 coll_acc;
    public bool initialRush = true;
    public bool finished = false;
    private float finalrad = 0;
    public float mass = 0.5f;
    public float goalMag = 1;
    public float formMag = 20;
    public float zMag = 10;
    public float heightMag = 1;
    public float obsMultiplier = 3;
    public Color velcolour = Color.green;
    private float g = 9.81f;
    private float[][] integral;
    private float[][] prev_error;
    private float prev_height_error = 0f;

   /*private void OnDrawGizmos()
    {
        //Gizmos.color = Color.black;  //why the hell do they have the American spelling of color with the English spelling of grey?
        //Gizmos.DrawLine(new Vector3(transform.position.x, transform.position.y, 20), new Vector3(transform.position.x + acc.x*100, transform.position.y + acc.y * 100, 20));

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, 2);

    }*/



    //-----------------------------------------------------------------------------------------------------------
    //---------------------------------------- Input Components -------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Component - General Bias Against obsticles
    Vector3 ObstacleAvoid()
    {
        var avoid = new Vector3(0F,0F, 0F);
        foreach (var poly in this.polygons)     
        {
            float dist = Mathf.Infinity;

            // Uses the unity function to calculate closest point
            var dirn = this.transform.position - this.closestBuildingPoint; // hopefully OK
            dist = Mathf.Abs(Vector3.Distance(transform.position, this.closestBuildingPoint));
            dirn.Normalize();                       // ^ p2x - p1x, p1y - p2y

            dirn *= Mathf.Pow(obsMultiplier / (dist), 2);

            if (Vector3.Dot(vel, dirn) < 0 && dist < 5)
            {
                //Debug.Log("Distance: " + dist + " dirn " + dirn + " Dot: " + Vector3.Dot(vel, dirn));
                //Debug.DrawLine(transform.position, transform.position + new Vector3(dirn.x, dirn.y, 0F), Color.magenta);
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
    Vector3 GoalComponent()
    {
        var x = this.goalPos[0] - this.transform.position.x;
        var y = this.goalPos[1] - this.transform.position.y;
        var z = this.goalPos[2] - this.transform.position.z;
        var comp = new Vector3(x, y, z);
        comp.Normalize();
        return comp;
    }

    //Component - vector to keep formation
    Vector3 FormationComponent(bool rush)
    {
        float x = 0, y = 0, z = 0;
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
                var zerr = (pos.z - this.transform.position.z) - this.formation[i-j].z;

                x += PIDs(xerr, iteration, 0, rush);  //i = 0-4, need 0-6 -> doesn't matter, so long as consistent
                y += PIDs(yerr, iteration, 1, rush);
                z += PIDs(zerr, iteration, 2, rush);

                if (float.IsNaN(x) || float.IsNaN(y))
                {
                    Debug.Log("Fak");
                }
            }
            iteration++;
        }
        this.formationError = new Vector3(x, y, z);
        return formationError;
    }


    //Component - vector to keep level height
    Vector3 HorizontalComponent()
    {
        float zavg = 0, iteration = 0;
        int j = 0;
        for (int i = 0; i < this.formation.Count + 1; i++) //9 max guards
        {
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                zavg += gObj.transform.position.z;
            }
            iteration++;
        }
        zavg /= iteration;
        var diff = zavg - transform.position.z;
        var zcomponent = new Vector3(0, 0, diff);
        return zcomponent;
    }


    //Component - Keep height above goalPos (assumes goalPos all at the same height)
    Vector3 HeightComponent()
    {
        var z = this.goalPos[2];
        float kp = 0f, kd = 0f;
        var error = goalPos[2] - (transform.position.z + vel.z * Time.deltaTime);
        if (error > 0)
        {
            kp = 1.0f;
            kd = 0.3f;
        } else {
            kp = 0.8f;
            kd = 0.7f;
        }
        
        var derivative = (error - prev_height_error) / Time.deltaTime;
        var pd = kp * error + kd * derivative;

        //Debug.Log("error: " + error + " pd: " + pd);
        prev_height_error = error;
        return new Vector3(0, 0, pd);
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
            integral[i] = new float[] { 0, 0, 0 };
            prev_error[i] = new float[] { 0, 0, 0 };
        }
    }


    //-----------------------------------------------------------------------------------------------------------
    //----------------------------------------- Input Control ---------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Function using vector weights to give direction. 
    Vector3 GetInput()
    {
        var dt = Time.deltaTime;

        var goalcomp = GoalComponent();
        var formcomp = FormationComponent(false);
        var obsavoid = ObstacleAvoid();
        var edgeavoid = AvoidWalls();
        var zcomp = HorizontalComponent();
        var heightcomp = HeightComponent();

        //main weights. xMag means the component of x
        var acc = new Vector3(0f, 0f, 0f);
        acc = goalcomp * goalMag + formcomp * formMag + obsavoid + zMag * zcomp + heightcomp * heightMag;

        if (acc.magnitude > MAX_ACCEL)
        {
            acc.Normalize();
            acc *= MAX_ACCEL;
        }

        var vel2 = new Vector2(vel.x, vel.y);
        var acc2 = new Vector2(acc.x, acc.y);

        if ( (vel2 + acc2 * dt).magnitude  > MAX_SPEED && Vector2.Dot(vel2, acc2) > 0)
        {
            var normvel = vel2;
            normvel.Normalize();
            normvel *= MAX_ACCEL;
            acc = new Vector3(-normvel.x, -normvel.y, acc.z);
        }

        return acc;
    }


    //-----------------------------------------------------------------------------------------------------------
    //----------------------------------------- Collision Checks ------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    //Checks to see if we will collide with a wall using standard kinematics (using unity collision). 
    Vector3 CollisionImminant()
    {
        //bool collision = false;
        var dt = Time.deltaTime;
        var pos = transform.position;

        var t_col = this.vel.magnitude / MAX_ACCEL;
        var breakDist = Mathf.Abs(vel.magnitude) * t_col - MAX_ACCEL * t_col * t_col * 0.5f; //maximum possible distance travelled

        float dist = Mathf.Infinity;
        if (!collision)
        {
            var veln = vel;
            veln.Normalize();
            //Debug.DrawLine(pos, pos + veln*breakDist, Color.yellow);
            if (Physics.Raycast(pos, vel, breakDist+2))
            {
                collision = true;
                t_dur = t_col;
                t_run = 0;
                coll_acc = -veln*MAX_ACCEL;   //negative??
                Debug.Log("COLLISION IMMINANT: GUARD " + guardID);
                Debug.Log(coll_acc + " mag: " + coll_acc.magnitude);
                //Debug.Break();
            }
        }
        if (t_run > t_dur)
            collision = false;
        if (collision) //can't do else, since we need to check this after the first point
        {
            t_run += dt;
            //Debug.DrawLine(transform.position, transform.position + vel*100f, Color.green);
            //Debug.DrawLine(transform.position, transform.position + coll_acc, Color.red);

            return coll_acc;
        }
        return new Vector3(0F, 0F, 0F);
    }


    //-----------------------------------------------------------------------------------------------------------
    //-------------------------------------------- Utilities ----------------------------------------------------
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
    //------------------------------------------ External Factors -----------------------------------------------
    //-----------------------------------------------------------------------------------------------------------
    Vector3 PizzaWeight()
    {
        var pizzaForce = new Vector3(0F, 0F, 1);


        var z = g * 1 /4; //1 = weight of pizza

        var avg = new Vector3 (0, 0, 0);
        float iteration = 0;
        int j = 0;
        /*for (int i = 0; i < this.formation.Count + 1; i++) //9 max guards
        {
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                avg += gObj.transform.position;
                avg.z += 1f;
            }
            iteration++;
        }*/
        //avg /= iteration;
        avg = GameManager.packagePos;// + new Vector3(0,0,1f);
        var diff = avg - transform.position;
        diff /= diff.z;
        pizzaForce = diff * z;
        //Debug.Log(diff);

        Debug.DrawLine(transform.position, avg, Color.white);
        //Debug.DrawLine(transform.position, transform.position + pizzaForce, Color.green);

        return pizzaForce;
    }

    Vector3 compensateGravity(Vector3 force)
    {
        Vector3 new_force = force;
        if (g > MAX_ACCEL)
        {
            Debug.Log("error max acceleration cannot compensate for gravity");
            Debug.Log("Max Accel: " + MAX_ACCEL + " Gravity: " + g);
            //Debug.Break();
        }
        else
        {
            new_force = force + new Vector3(0, 0, -g * (mass + 1f / 4f)); // 1+mass = (mass of pizza) + (mass of drone)
            while (new_force.magnitude / mass > MAX_ACCEL)
            {
                new_force = new_force - force.normalized * 0.1f;
            }
        }
        
        return new_force;
    }

    //-----------------------------------------------------------------------------------------------------------
    //------------------------------------------- Game Instances ------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------

    // Use this for initialization
    void Start()
    {
        vel = new Vector3(startVel[0], startVel[1], 0);
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
        var dt = Time.deltaTime;

        Vector3 input_force = CollisionImminant() * mass;    //This function is the red one that really avoids the walls. Unless we are in the final stretch, we check for collisions. This changes the "collision" variable to true (think like holding a value in a behavoir tree)
        if (!collision)     //If we don't expect a collision in the walls
        {
            velcolour = Color.blue;
            input_force = GetInput() * mass;     //Regurlar Input
        }

        Vector3 new_input_force = compensateGravity(input_force);
        //var new_input_force = input_force;
        // Add force of wind and gravity(assumes acc = F, ie m = 1)
        var new_input_force2 = new_input_force;
        new_input_force += GameManager.wind;
        new_input_force += g * (new Vector3(0, 0, 1)) *mass;
        new_input_force += PizzaWeight();
        //Debug.Log("Guard: " + guardID + " velocity: " + vel.magnitude + " Input Acc: " + new_input_force2.magnitude +  ", Old Old Input: " + input_force + " Old Input: " + new_input_force2 + " Final Acc: "+ new_input_force + " Wind: " + GameManager.wind + " gravity: " + (g * (new Vector3(0, 0, 1))) * mass + " pizza: " + PizzaWeight()*mass);

        //Shows directions
        //Debug.DrawLine(transform.position, transform.position + new_input_force, velcolour);
        //Debug.DrawLine(transform.position, transform.position + new_input_force2, Color.black);
        vel += (new_input_force / mass) * dt;

        //Debug.DrawLine(transform.position, transform.position + vel, Color.green);

        transform.position = transform.position + vel * dt + new_input_force / mass * dt * dt;
    }

}


