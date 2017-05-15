using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine.UI;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class GameManager : MonoBehaviour {
    public Point[] point;
    public Set[] guardSets;
    public Map map;
    public Color[] colors;
    public GameObject ai;
    public GameObject mapai;
    public GameObject camera;
    public float Kp, Ki, Kd;
    public float goal = 1, form = 20, obs = 3, range = 2;
    public static float endRange = 2;
    public static float goalMag = 1;
    public static float formMag = 20;
    public static float obsMultiplier = 3;
    public Datastruct data;
    public bool useSaved;
    public string problem;
    public static int numberofGuards;
    private int itemcount;
    public static float[] dist;

    private void OnDrawGizmos()
    {
        for (int i = 0; i < numberofGuards; i++)
        {
            if (point[i])
            {
                //Gizmos.color = Color.blue;

                Gizmos.color = Color.blue;
                Gizmos.DrawCube(new Vector3(point[i].startPos[0], point[i].startPos[1], 2), new Vector3(0.5F, 0.5F, 0));
                Gizmos.color = Color.yellow;
                Gizmos.DrawCube(new Vector3(point[i].goalPos[0], point[i].goalPos[1], 2), new Vector3(0.5F, 0.5F, 0));
                //Gizmos.DrawIcon(new Vector3(point[i].goalPos[0], point[i].goalPos[1], 0), "doughnut.tif", false);

                List<float[]> unseenitems = new List<float[]>();
                unseenitems.AddRange(map.items);

            }
        }
        Vector3 packagepos = findPackage();
        Gizmos.DrawIcon(packagepos, "pizza.tif", true);
    }

    private Mesh GeneratePolygonMesh(float[][] polygon)
    {

        //int xSize = 10, ySize = 10;
        int l = polygon.Length;
        Vector2[] polygon2D = new Vector2[l];
        Vector3[] vertices = new Vector3[l * 3];

        for (int i = 0; i < l; i++)
        {
            polygon2D[i] = new Vector2(polygon[i][0], polygon[i][1]);
        }
        Triangulator tr = new Triangulator(polygon2D);
        int[] indices = tr.Triangulate();

        // Create the Vector3 vertices
        Vector3[] polygon3D = new Vector3[l];
        for (int i = 0; i < l; i++)
        {
            vertices[2*l+i] = new Vector3(polygon2D[i].x, polygon2D[i].y, 0);
        }


        for (int i = 0; i < l; i++)
        {
            vertices[i] = new Vector3(polygon[i][0], polygon[i][1],0);
            vertices[i+ l] = new Vector3(polygon[i][0], polygon[i][1], polygon[i][2]);
        }

        int[] triangles = new int[3*(4*l)+indices.Length];
        for (int i = 0; i < l; i++)
        {
            triangles[3 * i] = i;
            triangles[(3 * i) + 1] = (i+1)%l;
            triangles[(3 * i) + 2] = i+l;
            triangles[3 *(i+l)] = (i+1)%l;
            triangles[(3 * (i+l)) + 2] = i+l;
            triangles[(3 * (i+l)) + 1] = ((i + 1)%l + l);

            triangles[3*2*l+3 * i] = i;
            triangles[3*2 * l + (3 * i) + 2] = (i + 1) % l;
            triangles[3*2 * l + (3 * i) + 1] = i + l;
            triangles[3*2 * l + 3 * (i + l)] = (i + 1) % l;
            triangles[3*2 * l + (3 * (i + l)) + 1] = i + l;
            triangles[3*2 * l + (3 * (i + l)) + 2] = ((i + 1) % l + l);

            /*
            triangles = new int[3 * (l - 2)];
            for (int i = 0; i < l - 2; i++)
            {
                triangles[3 * i] = 0;
                triangles[(3 * i) + 1] = (i+1) % l;
                triangles[(3 * i) + 2] = (i + 2) % l;
                Debug.Log(i + " " + (l + i) + " " + (i + 1));
            }
            */
        }
        for (int i = 0; i < indices.Length; i++)
        {
            triangles[3 * (4 * l) +i] = indices[i];
        }

        // Create the mesh
        Mesh msh = new Mesh();
        msh.vertices = vertices;
        msh.triangles = triangles;
        msh.RecalculateNormals();
        msh.RecalculateBounds();

        GameObject obj = new GameObject();

        // Set up game object with mesh;
        obj.AddComponent(typeof(MeshRenderer));
        MeshFilter filter = obj.AddComponent(typeof(MeshFilter)) as MeshFilter;
        filter.mesh = msh;

        obj.transform.parent = camera.transform;
        obj.GetComponent<MeshRenderer>().material.SetColor("_Color", Color.black);
        return msh;
    }

    Point CreateAI()
    {

        var player = Instantiate(ai, transform.position, transform.rotation);   //why can I not just set transform.position to what I want?
        Point point = null;
        point = point ? point : player.GetComponent<DDrive>();
        point = point ? point : player.GetComponent<DynamicPoint>();
        point = point ? point : player.GetComponent<KinematicPoint>();
        point = point ? point : player.GetComponent<StaticGuard>();
        point = point ? point : player.GetComponent<KinematicGuard>();
        point = point ? point : player.GetComponent<DynamicGuard>();
        point = point ? point : player.GetComponent<DynamicCar>();
        return point;
    }

    Vector3 findPackage()
    {
        var pos = new Vector3[numberofGuards];
        var packagepos = new Vector3(0, 0, 0);
        var goalpos = new float[numberofGuards][];
        //var distances = new float[numberofGuards];
        for (int i = 0; i < numberofGuards; i++)
        {
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                pos[i] = gObj.transform.position;
            }
        }

        for (int i = 0; i < numberofGuards; i++)        //pos[i] = 0-3 (in order)
        {
            packagepos += pos[i];
        }
        packagepos = packagepos / numberofGuards;
        return packagepos;
    }

    Map CreateMap()
    {

        //var player = Instantiate(mapai, new Vector3 ( 0, 2, -9 ), new Quaternion (0,0,0,1) );//transform.position, transform.rotation);
        var player = Instantiate(mapai, transform.position, transform.rotation);
        Map field = null;
        field = field ? field : player.GetComponent<MapAI>();
        return field;
    }

    void Awake()
    {
        if (Cheetah.instance == null)
        {
            Cheetah.instance = new Cheetah();
        }
    }

    // Use this for initialization
    void Start ()
    {
        goalMag = goal;
        endRange = range;
        formMag = form;
        obsMultiplier = obs;


        //Generate();

        float[][] test_poly = new float[4][];
        test_poly[0] = new float[] { 5, 0, -20 };
        test_poly[1] = new float[] { 5, 10, -20 };
        test_poly[2] = new float[] { 20, 10, -20 };
        test_poly[3] = new float[] { 20, 0, -20 };
        //Mesh m1 = GeneratePolygonMesh(test_poly);
        test_poly[0] = new float[] { 30, 0, -20 };
        test_poly[1] = new float[] { 30, 40, -20 };
        test_poly[2] = new float[] { 40, 40, -20 };
        test_poly[3] = new float[] { 40, 0, -20 };
        //Mesh m2 = GeneratePolygonMesh(test_poly);
        test_poly[0] = new float[] { 0, 15, -10 };
        test_poly[1] = new float[] { 0, 25, -10 };
        test_poly[2] = new float[] { 20, 25, -10 };
        test_poly[3] = new float[] { 20, 15, -10 };
        //Mesh m3 = GeneratePolygonMesh(test_poly);


        colors = new Color[4];
        colors[0] = Color.green;
        colors[1] = Color.cyan;
        colors[2] = Color.yellow;
        colors[3] = Color.magenta;

        string json = "";
        using (StreamReader r = new StreamReader(problem))
        {
            json = r.ReadToEnd();
            // List<Item> items = JsonConvert.DeserializeObject<List<Item>>(json);
        }
        Input2 input = JsonConvert.DeserializeObject<Input2>(json);

        map = CreateMap();
        Vector2[] boundaryPolygon = new Vector2[input.boundary_polygon.Length];
        for (int i = 0; i < input.boundary_polygon.Length; i++) boundaryPolygon[i] = new Vector2(input.boundary_polygon[i][0], input.boundary_polygon[i][1]);
        map.boundaryPolygon = boundaryPolygon;

        Triangulator boundarytr = new Triangulator(boundaryPolygon);
        int[] boundaryindices = boundarytr.Triangulate();

        // Create the Vector3 vertices
        Vector3[] boundaryvertices = new Vector3[boundaryPolygon.Length];
        for (int i = 0; i < boundaryvertices.Length; i++)
        {
            boundaryvertices[i] = new Vector3(boundaryPolygon[i].x, boundaryPolygon[i].y, 0.01f);
        }
        // Create the mesh
        Mesh boundarymsh = new Mesh();
        boundarymsh.vertices = boundaryvertices;
        boundarymsh.triangles = boundaryindices;
        boundarymsh.RecalculateNormals();
        boundarymsh.RecalculateBounds();

        GameObject boundaryobj = new GameObject();

        // Set up game object with mesh;
        boundaryobj.AddComponent(typeof(MeshRenderer));
        MeshFilter filter = boundaryobj.AddComponent(typeof(MeshFilter)) as MeshFilter;
        filter.mesh = boundarymsh;

        boundaryobj.transform.parent = camera.transform;
        boundaryobj.GetComponent<MeshRenderer>().material.SetColor("_Color", Color.cyan);

        int polygonCnt = 0;
        numberofGuards = 0;
        Debug.Log(input.a_max);
        //Debug.Log(input.polygon["polygon0"] is JArray);

        //gets length to intialize each array (I know this is incredibly bad practice, but won't be too taxing in the end)
        int polycount = 0;
        this.itemcount = 0;
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("polygon")) {
                polycount++;
            } else if (pair.Key.StartsWith("start_pos")){
                numberofGuards++;
            } else if (pair.Key.StartsWith("item_")){
                itemcount++;
            }
        }

        Vector2[][] inputPolygon = new Vector2[polycount][];
        //gets each polygon opject
        Debug.Log(input.polygon);
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("polygon"))         //checks if name is polygon
            {
                Debug.Log(pair.Key);
                float[][] polygon = pair.Value.ToObject<float[][]>();       //extracts float object

                Mesh m = GeneratePolygonMesh(polygon);

                Vector2[] vertices2D = new Vector2[polygon.Length];
                for (int i = 0; i < polygon.Length; i++)
                {
                    vertices2D[i] = new Vector2(polygon[i][0], polygon[i][1]);      //puts float value to vertex
                }
                /*
                Triangulator tr = new Triangulator(vertices2D);
                int[] indices = tr.Triangulate();

                // Create the Vector3 vertices
                Vector3[] vertices = new Vector3[vertices2D.Length];
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertices[i] = new Vector3(vertices2D[i].x, vertices2D[i].y, 1);
                }

                // Create the mesh
                Mesh msh = new Mesh();
                msh.vertices = vertices;
                msh.triangles = indices;
                msh.RecalculateNormals();
                msh.RecalculateBounds();

                GameObject obj = new GameObject();

                // Set up game object with mesh;
                obj.AddComponent(typeof(MeshRenderer));
                MeshFilter filter2 = obj.AddComponent(typeof(MeshFilter)) as MeshFilter;
                filter2.mesh = msh;

                obj.transform.parent = camera.transform;
                */
                inputPolygon[polygonCnt++] = vertices2D;
            } 
        }
        map.polygons = inputPolygon;

        //get start positions of guards. Assumes:
                        //Number of guards = number of start pos
                        //Guard start & end pos arrive in same order
                        //Can throw this into above forloop later (but it's nice to have seperate)
        float[][] start_positions = new float[numberofGuards][];
        int grdID = 0;
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("start_pos"))
            {
 //               Debug.Log(pair.Key + " Number of Guards: " + grdID);
                var start_pos = pair.Value.ToObject<float[]>();
                start_positions[grdID] = start_pos;
                grdID++;
            }
        }

        float[][] end_positions = new float[numberofGuards][];
        grdID = 0;
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("goal_pos"))
            {
  //              Debug.Log(pair.Key + " Number of Guards: " + grdID);
                var end_pos = pair.Value.ToObject<float[]>();
                end_positions[grdID] = end_pos;
                grdID++;
            }
        }

        float[][] items = new float[itemcount][];
        grdID = 0;
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("item_"))
            {
 //               Debug.Log(pair.Key + " Number of Guards: " + grdID);
                var item_pos = pair.Value.ToObject<float[]>();
                items[grdID] = item_pos;
                grdID++;
            }
        }
        map.items = items;
        map.seen = new float[items.Length][];
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("sensor_range"))
            {
                map.sensor_range = pair.Value.ToObject<float>();
                break;
            }
        }

        //Create players

        this.point = new Point[numberofGuards];
        for (int i = 0; i < numberofGuards; i++)
        {
            Debug.Log("Guard Number: " + i);
            point[i] = CreateAI();
            point[i].name = ("Guard" + i);
            //point[i].useSaved = useSaved;

            //point[i].startPos = input.start_pos; //need to update for multiple
            point[i].startPos = start_positions[i];
            point[i].transform.position = new Vector3(start_positions[i][0], start_positions[i][1], -5);
            point[i].goalPos = end_positions[i];
            point[i].startVel = input.start_vel;
            point[i].goalVel = input.goal_vel;

            point[i].MAX_SPEED = input.v_max;
            point[i].MAX_ACCEL = input.a_max;
            point[i].MAX_OMEGA = input.omega_max;
            point[i].MAX_PHI = input.phi_max;
            point[i].L_CAR = input.L_car;
            point[i].K_FRICTION = input.k_friction;
            point[i].guardID = i;

            point[i].polygons = inputPolygon;

            point[i].startVel = input.start_vel;
            point[i].goalVel = input.goal_vel;
            point[i].boundaryPolygon = boundaryPolygon;
            point[i].Kp = Kp;
            point[i].Ki = Ki;
            point[i].Kd = Kd;
        }
        for (int i = 0; i < numberofGuards; i++)
        {
            for (int j = 0; j < numberofGuards; j++)
            {
                if (j != i)
                {
                    var x = point[j].goalPos[0] - point[i].goalPos[0];
                    var y = point[j].goalPos[1] - point[i].goalPos[1];
                    point[i].formation.Add(new Vector2(x, y));
                }
            }
        }


        // Power of Cheetah

        //Cheetah.instance.CreateOrLoad(problem, boundaryPolygon, inputPolygon);

        if (UnityEditor.SceneView.sceneViews.Count > 0)
        {
            UnityEditor.SceneView sceneView = (UnityEditor.SceneView)UnityEditor.SceneView.sceneViews[0];
            sceneView.Focus();
        }
        Debug.Break();

    }

    // Update is called once per frame
    private float totalTime = 0F;
    private float[] error = new float[numberofGuards];
    void Update()
    {
        //error
        totalTime += Time.deltaTime;
        //Debug.Log("Time Elapsed: " + totalTime);

        if (UnityEditor.SceneView.sceneViews.Count > 0)
        {
            UnityEditor.SceneView sceneView = (UnityEditor.SceneView)UnityEditor.SceneView.sceneViews[0];
            sceneView.Focus();
        }
    }
}
