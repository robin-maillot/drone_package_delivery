using System;
using Newtonsoft.Json;
using System.Collections.Generic;

[Serializable]
public class Input2
{
    //these are the things the json reads
    public float L_car;
    public float a_max;
    public float[][] boundary_polygon;
    public float k_friction, omega_max, phi_max, v_max;
    public float[] goal_vel, start_vel;

    //I hate to hardcode something like this, but w/e
    /*public float[] goal_pos, goal_vel, start_pos, start_vel;
    public float[] goal_pos1, goal_vel1, start_pos1, start_vel1;
    public float[] goal_pos2, goal_vel2, start_pos2, start_vel2;
    public float[] goal_pos3, goal_vel3, start_pos3, start_vel3;
    public float[] goal_pos4, goal_vel4, start_pos4, start_vel4;
    public float[] goal_pos5, goal_vel5, start_pos5, start_vel5;
    public float[] goal_pos6, goal_vel6, start_pos6, start_vel6;
    public float[] goal_pos7, goal_vel7, start_pos7, start_vel7;
    public float[] goal_pos8, goal_vel8, start_pos8, start_vel8;*/


    [JsonExtensionData]
    public IDictionary<string, Newtonsoft.Json.Linq.JToken> polygon;
    //public IDictionary<string, Newtonsoft.Json.Linq.JToken> poly_rot;
}