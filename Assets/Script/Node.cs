using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

[JsonConverter(typeof(NodeConverter))]
public class Node
{
    [JsonConverter(typeof(Vector2Converter))]
    public Vector2 pos;
    public float time;

    public Node()
    {
    }

    public Node(Vector2 pos, float time)
    {
        this.pos = pos;
        this.time = time;
    }
}