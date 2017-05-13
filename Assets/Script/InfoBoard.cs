using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class InfoBoard : MonoBehaviour
{
    public Text info;

    public bool showTime;
    public bool showPos;
    public bool showTheta;
    public bool showVel;
    public bool showAcc;
    public bool showOmega;
    public bool showPhi;
    public bool showGK;

    private Point ai;

    private Vector2 pos;
    private Vector2 vel;
    private Vector2 acc;

    private float theta;
    private float phi;
    private float omega;
    private float f;
    private float gk;

    private float time;

    private bool init = false;
    private int cnt = 0;

    string ParseFloat(float f)
    {
        return  ((float)((int) (f * 100 + 0.000001)) / 100).ToString("F2");
    }
    

    string ParseVector(Vector2 v, string heading, bool mag)
    {
        return string.Format("{0}: ({1}, {2})", heading, ParseFloat(v.x), ParseFloat(v.y)) + (mag ? (", " + ParseFloat(v.magnitude)) : "");
    }

    string ParseRadian(float ang, string heading, bool toDeg)
    {
        return string.Format("{0}: {1}", heading, toDeg ? ParseFloat(ang * 180F / Mathf.PI) + "°" : ParseFloat(ang));
    }

    void UpdateInfo()
    {
        info.text = showTime ? "t: " + ParseFloat(time) : "";
        info.text += showPos ? ("\n" + ParseVector(pos, "p", false)) : "";
        info.text += showVel ? ("\n" + ParseVector(vel, "v", true)) : "";
        info.text += showAcc ? ("\n" + ParseVector(acc, "a", true)) : "";
        info.text += showTheta ? ("\n" + ParseRadian(theta, "θ", true)) : "";
        info.text += showOmega ? ("\n" + ParseRadian(omega, "ω", false)) : "";
        info.text += showPhi ? ("\n" + ParseRadian(phi, "ϕ", false)) : "";
        info.text += showGK ? ("\nFric:" + ParseFloat(gk)) : "";
    }

    float ComputeAngle(Vector2 v)
    {
        float ang = Mathf.Atan2(v.y, v.x);
        ang = ang < 0 ? ang + 2 * Mathf.PI : ang;
        return ang;
    }

    void ComputeInfo()
    {
        int curPos = ai.curPos;
        if (curPos < 3) return;
        var ooState = ai.path[curPos - 2];
        var oState = ai.path[curPos - 1];
        var nState = ai.path[curPos];

        float odt = oState.time - ooState.time;
        float dt = nState.time - oState.time;
        time = nState.time;

        Vector2 oPos = oState.pos;
        Vector2 oVel = (oState.pos - ooState.pos) / odt;
        float oTheta = ComputeAngle(oVel);
        
        pos = new Vector2(nState.pos.x, nState.pos.y);
        vel = (pos - oPos) / dt;
        acc = (vel - oVel) / dt;
        theta = ComputeAngle(vel);
        omega = (theta - oTheta) / dt;
        phi = Mathf.Atan(omega * ai.L_CAR / vel.magnitude);
        gk =
            Mathf.Sqrt((Mathf.Pow(vel.magnitude, 4) / Mathf.Pow(ai.L_CAR, 2) * Mathf.Pow(Mathf.Tan(phi), 2)) +
                       Mathf.Pow(acc.magnitude, 2));

    }

	// Use this for initialization
	void Start ()
	{
	}

    void Setup()
    {
        var player = GameObject.FindGameObjectWithTag("Player");
        ai = player.GetComponent<Point>();

        showTime = true;
        showPos = true;
        showVel = true;

        showTheta = false;
        showAcc = false;
        showOmega = false;
        showPhi = false;
        showGK = false;

        var dp = player.GetComponent<DynamicPoint>();
        var dubin = player.GetComponent<DDrive>();
        
        if (dp)
        {
            showAcc = true;
        } else if (dubin)
        {
            switch (dubin.task)
            {
                case 2:
                    showVel = true;
                    showAcc = true;
                    break;
                case 3:
                    showTheta = true;
                    showOmega = true;
                    break;
                case 4:
                    showTheta = true;
                    showOmega = true;
                    showPhi = true;
                    break;
                case 5:
                    showTheta = true;
                    showOmega = true;
                    showPhi = true;
                    showAcc = true;

                    break;
                case 6:
                    showTheta = true;
                    showOmega = true;
                    showPhi = true;
                    showAcc = true;
                    showGK = true;

                    break;
                default:
                    break;
            }
        }
}

    // Update is called once per frame
    void Update ()
	{
	    cnt++;
	    if (cnt % 10 == 0)
	    {
	        if (!ai)
	        {
	            Setup();
	        }
            ComputeInfo();
            UpdateInfo();
	    }



	}
}
