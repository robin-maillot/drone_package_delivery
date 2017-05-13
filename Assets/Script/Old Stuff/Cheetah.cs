using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using System.Xml.XPath;
using UnityEngine;

public class Cheetah
{
    public static Cheetah instance;
    public static int resolution = 4000000;

    private byte[] s;
    private int ppr;
    private int ppc;
    private double px;
    private double py;
    private double minX, minY ,maxX, maxY;


    private Vector2 indexToCoord(int idx)
    {
        return new Vector2((float)((idx % ppr) / px + minX), (float) ((idx / ppr) / py + minY));
    }

    private int CoordToIndex(Vector2 p)
    {
        int res = (int) (Math.Round((p.y - minY) * py) * ppr + (int) (Math.Round((p.x - minX) * px)));
        if (res >= resolution)
        {
            Debug.Log("Dman");
        }
        return res;
    }

    private string problemStorage(string problemPath)
    {
        return problemPath + ".byte";
    }

    public bool IsValid(Vector2 p)
    {
        int idx = CoordToIndex(p);
        return idx >= 0 && idx < resolution && s[idx] == 1;
    }

    public bool IsBounded(Vector2 p)
    {
        int idx = CoordToIndex(p);
        return idx >= 0 && idx < resolution && s[idx] > 0;

    }

    public void CreateOrLoad(string problem, Vector2[] boundaryPolygon, Vector2[][] polygons)
    {
        var t = DateTime.Now;

        minX = double.PositiveInfinity;
        minY = double.PositiveInfinity;
        maxX = double.NegativeInfinity;
        maxY = double.NegativeInfinity;

        foreach (Vector2 p in boundaryPolygon)
        {
            minX = Math.Min(minX, p.x);
            minY = Math.Min(minY, p.y);
            maxX = Math.Max(maxX, p.x);
            maxY = Math.Max(maxY, p.y);
        }

        double xspan = maxX - minX;
        double yspan = maxY - minY;

        double k = yspan / xspan;


        ppr = (int) Math.Sqrt(resolution / k);
        ppc = resolution / ppr;

        px = ppr / xspan;
        py = ppc / yspan;
       
        if (Cheeted(problem))
        {
            Load(problem);
            var we = DateTime.Now - t;
            Debug.Log("Load map uses: " + we);
            return;
        }
        s = new byte[resolution];

        for (int i = 0; i < resolution; i++)
        {
            var coor = indexToCoord(i);
            s[i] = 0;
            if (Utils.ContainsPoint(boundaryPolygon, coor))
            {
                s[i] = 1;
                foreach (var p in polygons)
                {
                    if (Utils.ContainsPoint(p, coor))
                    {
                        s[i] = 2;
                        break;
                    }
                }
            }
        }
        var cst = DateTime.Now - t;
        Debug.Log("Create map uses: " + cst);
        Save(problem);
    }




    public void Save(string path)
    {
        var t = DateTime.Now;
        File.WriteAllBytes(problemStorage(path), s);
        var cst = DateTime.Now - t;
        Debug.Log("Saving map uses: " + cst);
    }

    public void Load(string path)
    {
        var t = DateTime.Now;
        s = File.ReadAllBytes(problemStorage(path));
        var cst = DateTime.Now - t;
        Debug.Log("Loading map uses: " + cst);
    }

    public bool Cheeted(string path)
    {
        return File.Exists(problemStorage(path));
    }
}

