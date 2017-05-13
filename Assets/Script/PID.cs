using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID : MonoBehaviour {

    public static float error = 0F;
    private float integral = 0F;
    private float prev_error = 0F;
    public static float Kp, Ki, Kd;
    public static float correction;
    //public PID()

    private void Update()
    {
        correction = PIDs();
    }
    float PIDs()
    {
        integral += integral + (error * Time.deltaTime);
        var derivative = (error - prev_error) / Time.deltaTime;
        var output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        //sleep(iteration_time)
        return output;
    }

}
