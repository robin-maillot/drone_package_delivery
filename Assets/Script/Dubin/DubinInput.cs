using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DubinInput
{

    public class Steer
    {
        public float v;
        public float omega;
        public float time;
        public float a;

        public Steer(float v, float omega, float time, float a = 0F)
        {
            this.v = v;
            this.omega = omega;
            this.time = time;
            this.a = a;
        }

    }

    public List<Steer> steer;
    public float time;

    public DubinInput(List<Steer> steer)
    {
        this.steer = steer;
        updateTime();
    }

    public DubinInput()
    {
        time = Mathf.Infinity;
        steer = new List<Steer>();
    }

    public void updateTime()
    {
        time = 0;
        foreach (var s in steer)
        {
            time += s.time;
        }
    }
}
