using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using KDTreeDLL;

public class Test : MonoBehaviour {



    void testDubin()
    {
        Circle c1 = new Circle(-5.0F, 0, 2);
        Circle c2 = new Circle(5.0F, 0, 2);
        var tangents = DubinUtils.TangentLines(c1, c2);
        Debug.Log(tangents.Count);
        Circle c3 = new Circle(-5.0F, 0, 5);
        Circle c4 = new Circle(5.0F, 0, 5);
        tangents = DubinUtils.TangentLines(c3, c4);
        Debug.Log(tangents.Count);
    }

    void testDubinPath()
    {
        // testing LL
        //        DubinInput input = DubinUtils.DubinSP(1, 1, new DubinState(new Vector2(-6, 1), 1, (float)(1.5 * Math.PI)),
        //            new DubinState(new Vector2(6, 1), 1, (float)(0.5 * Math.PI)));
        //        Debug.Log(input.time);

        // testing LR
//        DubinInput input = DubinUtils.DubinSP(1, 1, new DubinState(new Vector2(-6, 1), 1, (float)(1.5 * Math.PI)),
//            new DubinState(new Vector2(6, -1), 1, (float)(1.5 * Math.PI)));
//        Debug.Log(input.time);

        // testing RL
//        DubinInput input = DubinUtils.DubinSP(1, new DubinState(new Vector2(-6, -1), 1, (float)(0.5 * Math.PI)),
//            new DubinState(new Vector2(6, 1), 1, (float)(0.5 * Math.PI)));
//        Debug.Log(input.time);
    }

    void Awake()
    {
    }

    // Use this for initialization
    void Start ()
    {

        Cheetah.instance.Save(@"test3");
        Cheetah.instance.Load(@"test3");
        return;
        testDubinPath();

        KDTree t = new KDTree(2);
        t.insert(new double[2] { 2, 3 }, 2);
        t.insert(new double[2] { 5, 5 }, 1);

        var a = t.nearest(new double[2] { 1, 1 }, 2);
        Debug.Log(a);

        var b = t.nearest(new double[2] {5, 5}, 1);
        Debug.Log(b);

        var c = t.range(new double[2] {2, 2}, new double[2] {10, 10});
        Debug.Log(c);

        var d = t.range(new double[2] {3, 3}, new double[2] {1000, 1000});
        Debug.Log(d);
        Matrix A = new Matrix(new double[][] {
            new double[] { 1.0, 2.0, 3.0 },
            new double[] { 4.0, 5.0, 6.0 } });
        Matrix B = new Matrix(new double[][] {
            new double[] { 7, 8},
            new double[] { 9, 10},
            new double[] { 11, 12} });
        Matrix C = new Matrix(new double[][] {
            new double[] { 4.0, 7.0 },
            new double[] { 2.0, 6.0 } });
        Debug.Log(Matrix.ArrayPower(C, -1));

        Debug.Log(C.Inverse());
    }

    // Update is called once per frame
    void Update () {
		
	}
}
