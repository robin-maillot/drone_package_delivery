using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

[JsonConverter(typeof(NodeConverter))]
public class VelNode
{
    [JsonConverter(typeof(Vector2Converter))]
    public Vector2 pos;
    public Vector2 vel;
    public float time;

    public VelNode()
    {
    }

    public VelNode(Vector2 pos, Vector2 vel, float time)
    {
        this.pos = pos;
        this.time = time;
        this.vel = vel;
    }
}