using System;
using Newtonsoft.Json;
using UnityEngine;

public class NodeConverter : JsonConverter
{
    public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
    {
        var vec = (Node)value;
        writer.WriteStartObject();
        writer.WritePropertyName("pos");
        serializer.Serialize(writer, vec.pos);
//        writer.WriteValue();
        writer.WritePropertyName("time");
        writer.WriteValue(vec.time);
        writer.WriteEndObject();
    }

    public override bool CanConvert(Type objectType)
    {
        return objectType == typeof(Vector2);
    }

    public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
    {
        throw new NotImplementedException("Unnecessary because CanRead is false. The type will skip the converter.");
    }

    public override bool CanRead
    {
        get { return false; }
    }
}