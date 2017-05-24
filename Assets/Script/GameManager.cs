using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine.UI;
using System.Linq;
using System;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class GameManager : MonoBehaviour {
    public Point[] point;
    public Set[] guardSets;
    public Map map;
    public Color[] colors;
    public GameObject ai;
    public GameObject mapai;
    public GameObject camera;
    public GameObject moving_camera;
    public float max_wind;
    public float Kp, Ki, Kd;
    public float goal = 1, form = 20, level = 5, height = 1, obs = 3, range = 2;
    public static float endRange = 2;
    public static float goalMag = 1;
    public static float formMag = 20;
    public static float zMag = 10;
    public static float heightMag = 1;
    public static float obsMultiplier = 3;
    public Datastruct data;
    public bool useSaved;
    public string problem;
    public static int numberofGuards;
    private int itemcount;
    public static float[] dist;
    public static Vector3 wind;
    int buildingnumber = 0;
    private List<GameObject> buildingObjects = new List<GameObject>();
    private List<Quaternion> buildingRotations = new List<Quaternion>();
    private List<Vector3> buildingPivots = new List<Vector3>();
    public static Vector3 packagePos;
    private Vector3 packageSpeed;
    private Vector3 packageGoal;
    private List<Vector3> packageWaypoints = new List<Vector3>();
    private List<float> formation_sizes = new List<float>();
    private GameObject packageObject;
    public Material newMaterialRef;
    private Vector3[] drone_offset_positions = new Vector3[4];
    private int currentCheckpoint = 0;
    private void OnDrawGizmos()
    {
        for (int i = 0; i < point.Length; i++)
        {
            if (point[i])
            {
                //Gizmos.color = Color.blue;
                
                Gizmos.color = Color.blue;
                Gizmos.DrawCube(new Vector3(point[i].startPos[0], point[i].startPos[1], point[i].startPos[2]), new Vector3(0.5F, 0.5F, 0.5F));
                Gizmos.color = Color.yellow;
                Gizmos.DrawCube(new Vector3(point[i].goalPos[0], point[i].goalPos[1], point[i].goalPos[2]), new Vector3(0.5F, 0.5F, 0.5F));
                //Gizmos.DrawIcon(new Vector3(point[i].goalPos[0], point[i].goalPos[1], 0), "doughnut.tif", false);

                List<float[]> unseenitems = new List<float[]>();
                unseenitems.AddRange(map.items);

            }
        }
        Vector3 packagepos = findPackage();
        Gizmos.DrawIcon(packagepos, "pizza.tif", true);
        Gizmos.color = Color.blue;
        Gizmos.color = Color.yellow;

        for (int i = 0; i < buildingObjects.Count; i++)
        {
            Vector3 s = buildingObjects[i].GetComponent<BoxCollider>().size;
            Gizmos.DrawCube((buildingObjects[i].GetComponent<BoxCollider>().center), new Vector3(0.5F, 0.5F, 0));
            //Gizmos.DrawCube((buildingObjects[i].GetComponent<BoxCollider>().center+s/2), new Vector3(1F, 1F, 0));
            //Gizmos.DrawCube((buildingObjects[i].GetComponent<BoxCollider>().center - s / 2), new Vector3(1F, 1F, 0));

        }
        Gizmos.color = Color.red;
        for (int i = 0; i < point.Length; i++)
        {
            if (point[i])
            {
                Vector3 pos = point[i].transform.position;
                Vector3 vClosest = point[i].closestBuildingPoint;
                //Debug.DrawLine(pos, vClosest, Color.cyan);
                Gizmos.DrawCube(vClosest, new Vector3(0.5F, 0.5F, 0));
            }
        }
        Gizmos.DrawWireSphere(new Vector3(0, 0, -10), 5);
        Gizmos.color = Color.green;
        foreach (Vector3 w in packageWaypoints)
        {
            Gizmos.DrawCube(w, new Vector3(0.5F, 0.5F, 0.5F));
        }
    }

    private GameObject GeneratePolygonMesh(float[][] polygon, Color c)
    {

        GameObject obj = new GameObject();

        MeshFilter filter = obj.AddComponent<MeshFilter>();
        Mesh mesh = filter.mesh;
        mesh.Clear();

        float length = polygon[0][0];
        float width = polygon[0][1];
        float height = polygon[0][2];

        #region Vertices
        Vector3 p0 = new Vector3(-length * .5f, -width * .5f, height * .5f);
        Vector3 p1 = new Vector3(length * .5f, -width * .5f, height * .5f);
        Vector3 p2 = new Vector3(length * .5f, -width * .5f, -height * .5f);
        Vector3 p3 = new Vector3(-length * .5f, -width * .5f, -height * .5f);

        Vector3 p4 = new Vector3(-length * .5f, width * .5f, height * .5f);
        Vector3 p5 = new Vector3(length * .5f, width * .5f, height * .5f);
        Vector3 p6 = new Vector3(length * .5f, width * .5f, -height * .5f);
        Vector3 p7 = new Vector3(-length * .5f, width * .5f, -height * .5f);

        Vector3[] vertices = new Vector3[]
        {
	        
	        p0, p1, p2, p3, // Bottom
	        p7, p4, p0, p3, // Left 
	        p4, p5, p1, p0, // Front 
	        p6, p7, p3, p2, // Back
	        p5, p6, p2, p1, // Right
	        p7, p6, p5, p4  // Top
        };
        #endregion

        #region Normales
        Vector3 up = Vector3.up;
        Vector3 down = Vector3.down;
        Vector3 front = Vector3.forward;
        Vector3 back = Vector3.back;
        Vector3 left = Vector3.left;
        Vector3 right = Vector3.right;

        Vector3[] normales = new Vector3[]
        {
	        down, down, down, down, // Bottom
	        left, left, left, left, // Left 
	        front, front, front, front, // Front
	        back, back, back, back, // Back
	        right, right, right, right, // Right
	        up, up, up, up  // Top
        };
        #endregion

        #region UVs
        Vector2 _00 = new Vector2(0f, 0f);
        Vector2 _10 = new Vector2(1f, 0f);
        Vector2 _01 = new Vector2(0f, 1f);
        Vector2 _11 = new Vector2(1f, 1f);

        Vector2[] uvs = new Vector2[]
        {
	        _11, _01, _00, _10,  // Bottom  
	        _11, _01, _00, _10, // Left
	        _11, _01, _00, _10, // Front
	        _11, _01, _00, _10, // Back   
	        _11, _01, _00, _10, // Right
	        _11, _01, _00, _10, // Top
        };
        #endregion

        #region Triangles
        int[] triangles = new int[]
        {
	        // Bottom
	        3, 1, 0,
            3, 2, 1,			
 
	        // Left
	        3 + 4 * 1, 1 + 4 * 1, 0 + 4 * 1,
            3 + 4 * 1, 2 + 4 * 1, 1 + 4 * 1,
 
	        // Front
	        3 + 4 * 2, 1 + 4 * 2, 0 + 4 * 2,
            3 + 4 * 2, 2 + 4 * 2, 1 + 4 * 2,
 
	        // Back
	        3 + 4 * 3, 1 + 4 * 3, 0 + 4 * 3,
            3 + 4 * 3, 2 + 4 * 3, 1 + 4 * 3,
 
	        // Right
	        3 + 4 * 4, 1 + 4 * 4, 0 + 4 * 4,
            3 + 4 * 4, 2 + 4 * 4, 1 + 4 * 4,
 
	        // Top
	        3 + 4 * 5, 1 + 4 * 5, 0 + 4 * 5,
            3 + 4 * 5, 2 + 4 * 5, 1 + 4 * 5,
        };
        #endregion

        mesh.vertices = vertices;
        mesh.normals = normales;
        mesh.uv = uvs;
        mesh.triangles = triangles;

        mesh.RecalculateBounds();

        obj.AddComponent(typeof(MeshRenderer));
        obj.GetComponent<MeshRenderer>().material.SetColor("_Color", c);
        
        obj.transform.parent = camera.transform;
        obj.name = "Building" + buildingnumber;
        buildingnumber++;

       
        BoxCollider boxCollider = obj.AddComponent<BoxCollider>();
        //boxCollider.GetComponent<MeshRenderer>().material = newMaterialRef;

        boxCollider.size = new Vector3(length, width, height);

        obj.transform.position = new Vector3(polygon[1][0], polygon[1][1], polygon[1][2]);

        return obj;
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
        point = point ? point : player.GetComponent<Drone>();
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
        packagepos += new Vector3(0, 0, 1.0f);
        return packagepos;
    }


    void setPackageGoal()
    {
        var pos = new Vector3[numberofGuards];
        var packagegoalpos = new Vector3(0, 0, 0);
        var goalpos = new float[numberofGuards][];
        //var distances = new float[numberofGuards];
        for (int i = 0; i < numberofGuards; i++)
        {
            var gObj = GameObject.Find("Guard" + i);
            if (gObj)
            {
                pos[i] = new Vector3(point[i].goalPos[0], point[i].goalPos[1], point[i].goalPos[2]);
            }
        }

        for (int i = 0; i < numberofGuards; i++)        //pos[i] = 0-3 (in order)
        {
            packagegoalpos += pos[i];
        }
        packagegoalpos = packagegoalpos / numberofGuards;
        packageGoal = packagegoalpos;
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
        zMag = level;
        heightMag = height;

        formMag = form;
        obsMultiplier = obs;

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
        BoxCollider boxCollider = boundaryobj.AddComponent<BoxCollider>();
        boxCollider.size = new Vector3(100, 100, 10);
        boxCollider.center = new Vector3(50, 50, 5);

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
        //float[][] buildingpivots = new float[polycount][];
        
        foreach (var pair in input.polygon) //need to add rotation to json
        {
            if (pair.Key.StartsWith("polygon"))         //checks if name is polygon
            {
                Debug.Log(pair.Key);
                buildingRotations.Add(new Quaternion(0, 0, 0, 0));

                float[][] polygon = pair.Value.ToObject<float[][]>();       //extracts float object

                buildingObjects.Add(GeneratePolygonMesh(polygon, Color.gray));

                Vector2[] vertices2D = new Vector2[polygon.Length];
                for (int i = 0; i < polygon.Length; i++)
                {
                    vertices2D[i] = new Vector2(polygon[i][0], polygon[i][1]);      //puts float value to vertex
                }
                inputPolygon[polygonCnt++] = vertices2D;

                var tmp = new Vector3( 0F, 0F, 0F );
                for (int i = 0; i < polygon.Length; i++)
                {
                    tmp[0] += polygon[i][0];
                    tmp[1] += polygon[i][1];
                    tmp[2] += polygon[i][2];
                }
                tmp[0] /= polygon.Length;
                tmp[1] /= polygon.Length;
                tmp[2] /= polygon.Length;
                buildingPivots.Add(tmp);
            }
        }
        buildingRotations.Add(new Quaternion(0, 0, 0, 0));
        buildingObjects.Add(boundaryobj);
        buildingPivots.Add(new Vector3(50,50,5));

        // Create packageObject
        foreach (var pair in input.polygon) //need to add rotation to json
        {
            if (pair.Key.EndsWith("package"))         //checks if name is polygon
            {
                Debug.Log(pair.Key);
                float[][] package = pair.Value.ToObject<float[][]>();       //extracts float object

                packageObject = GeneratePolygonMesh(package, Color.white);
                break;
            }
        }

        foreach (var pair in input.polygon) //need to add rotation to json
        {
            if (pair.Key.EndsWith("_rot"))
            {
                var buildnum = pair.Key;
                int objnum = Int32.Parse(buildnum.Substring(4, 1));
                Debug.Log("hyperderp");
                Debug.Log(buildnum);
                Debug.Log(transform.position);
                float[] rot = pair.Value.ToObject<float[]>();
                buildingRotations[objnum] = Quaternion.Euler(rot[0], rot[1], rot[2]);
                buildingObjects[objnum].transform.rotation = Quaternion.Euler(rot[0], rot[1], rot[2]);
                Debug.Log(transform.position);

                //buildingObjects[objnum].transform.Rotate(new Vector3(rot[0], rot[1], rot[2]), Space.Self);
                /*buildingObjects[objnum].transform.RotateAround(buildingPivots[objnum], transform.right, rot[0]);

                buildingObjects[objnum].transform.RotateAround(buildingPivots[objnum], transform.forward, rot[1]);
                buildingObjects[objnum].transform.RotateAround(buildingPivots[objnum], transform.up, rot[2]);*/
            }
        }
        map.polygons = inputPolygon;

        //buildingObjects[0].transform.Rotate(new Vector3(45F, 0F, 0F)); //pivot over x, y, z

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
        foreach (var pair in input.polygon)
        {
            if (pair.Key.StartsWith("package_waypoints"))
            {
                //              Debug.Log(pair.Key + " Number of Guards: " + grdID);
                float[][] waypoints = pair.Value.ToObject<float[][]>();
                foreach (float[] waypoint in waypoints)
                {
                    Vector3 w = new Vector3(waypoint[0],waypoint[1],waypoint[2]);
                    packageWaypoints.Add(w);
                    formation_sizes.Add(waypoint[3]);
                    Debug.Log("Waypoint: " + w);
                }
            }
        }
        drone_offset_positions[0] = new Vector3(-formation_sizes[currentCheckpoint]/2, -formation_sizes[currentCheckpoint] / 2,0);
        drone_offset_positions[1] = new Vector3(formation_sizes[currentCheckpoint] / 2, -formation_sizes[currentCheckpoint] / 2, 0);
        drone_offset_positions[2] = new Vector3(formation_sizes[currentCheckpoint] / 2, formation_sizes[currentCheckpoint] / 2, 0);
        drone_offset_positions[3] = new Vector3(-formation_sizes[currentCheckpoint] / 2, formation_sizes[currentCheckpoint] / 2, 0);



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
            point[i].transform.position = new Vector3(start_positions[i][0], start_positions[i][1], start_positions[i][2]);
            //point[i].goalPos = end_positions[i];
            Vector3 goalV = drone_offset_positions[i] + packageWaypoints[currentCheckpoint];

            point[i].goalPos = new float[] { goalV.x,goalV.y,goalV.z };
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

            Vector3 pos = point[i].transform.position;
            Vector3 vClosest = new Vector3(100, 100, 100);
            for (int j = 0; j < buildingObjects.Count; j++)
            {
                Vector3 v = buildingObjects[j].GetComponent<BoxCollider>().ClosestPointOnBounds(pos);
                if (Vector3.Distance(v, pos) < Vector3.Distance(vClosest, pos))
                    vClosest = v;
            }
            point[i].closestBuildingPoint = vClosest;

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
        packagePos = findPackage();
        packageSpeed = packagePos;
        setPackageGoal();
        // Power of Cheetah

        //Cheetah.instance.CreateOrLoad(problem, boundaryPolygon, inputPolygon);

        if (UnityEditor.SceneView.sceneViews.Count > 0)
        {
            UnityEditor.SceneView sceneView = (UnityEditor.SceneView)UnityEditor.SceneView.sceneViews[0];
            sceneView.Focus();
        }
        Debug.Break();

    }

    // Updates wind direction and magnitude using a gaussian distribution and draws it on the map
    void updateWind()
    {
        Vector2 w = UnityEngine.Random.onUnitSphere;
        float theta = Vector3.Angle(new Vector3(1, 0, 0),wind);
        //Debug.Log(Vector3.Angle(new Vector3(1, 0, 0), new Vector3(0, 1, 0)));
        float new_theta_rad = Mathf.Deg2Rad*Utils.gaussianRandom(theta, 1);
        float new_mag = Utils.gaussianRandom(wind.magnitude, max_wind / 20);
        //wind = new Vector3(w.x, w.y, 0).normalized * max_wind;
        if(new_mag>max_wind)
            new_mag = max_wind;
        wind = new Vector3(Mathf.Cos(new_theta_rad), Mathf.Sin(new_theta_rad),0)*new_mag;
        Debug.DrawLine(new Vector3(0, 0, -10),new Vector3(wind.x * 5 / max_wind, wind.y * 5 / max_wind, -10), Color.green);
    }

    void updateCurrentCheckpoint()
    {
        if (currentCheckpoint < (packageWaypoints.Count - 1)){
            float d = Vector3.Distance(packagePos, packageWaypoints[currentCheckpoint]);
            if (d < formation_sizes[currentCheckpoint] / 2)
            {
                currentCheckpoint += 1;
                drone_offset_positions[0] = new Vector3(-formation_sizes[currentCheckpoint] / 2, -formation_sizes[currentCheckpoint] / 2, 0);
                drone_offset_positions[1] = new Vector3(formation_sizes[currentCheckpoint] / 2, -formation_sizes[currentCheckpoint] / 2, 0);
                drone_offset_positions[2] = new Vector3(formation_sizes[currentCheckpoint] / 2, formation_sizes[currentCheckpoint] / 2, 0);
                drone_offset_positions[3] = new Vector3(-formation_sizes[currentCheckpoint] / 2, formation_sizes[currentCheckpoint] / 2, 0);
                for (int i = 0; i < numberofGuards; i++)
                {
                    Vector3 goalV = drone_offset_positions[i] + packageWaypoints[currentCheckpoint];
                    point[i].goalPos = new float[] { goalV.x, goalV.y, goalV.z };
                    point[i].formation.Clear();
                }
                for (int i = 0; i < numberofGuards; i++)
                {
                    //point[i].formation = new List<Vector3>();
                    for (int j = 0; j < numberofGuards; j++)
                    {
                        if (j != i)
                        {
                            var x = point[j].goalPos[0] - point[i].goalPos[0];
                            var y = point[j].goalPos[1] - point[i].goalPos[1];
                            point[i].formation.Add(new Vector3(x, y,0f));
                        }
                    }
                }
            }
                
        }
    }

    // Update is called once per frame
    private float totalTime = 0F;
    private float[] error = new float[numberofGuards];
    void Update()
    {
        updateWind();
        Vector3 newPackagePos = findPackage();
        packageSpeed = newPackagePos - packagePos;
        packagePos = newPackagePos;
        packageObject.transform.position = packagePos;
        packageObject.transform.LookAt(packageObject.transform.position + packageSpeed, new Vector3(0, 0, -1));
        updateCurrentCheckpoint();
        //buildingObjects[4].transform.Rotate(Vector3.forward * Time.deltaTime*2f);
        //buildingObjects[2].transform.Translate(Vector3.down * Time.deltaTime);

        for (int i = 0; i < buildingObjects.Count; i++)
        {
            for (int j = 0; j < numberofGuards; j++)
            {
                if (buildingObjects[i].GetComponent<BoxCollider>().bounds.Contains(point[j].transform.position))
                    Debug.Log("Drone "+j+" is in building "+i);
            }
        }

        //error
        totalTime += Time.deltaTime;

        //Debug.Log("Time Elapsed: " + totalTime);

        for (int i = 0; i < point.Length; i++)
        {
            Vector3 pos = point[i].transform.position;
            Vector3 vClosest = new Vector3(100, 100, 100);
            for (int j = 0; j < buildingObjects.Count; j++)
            {
                buildingObjects[j].transform.rotation = Quaternion.Euler(0, 0, 0);

                Vector3 center = buildingObjects[j].transform.position;
                Vector3 rotatedP = Quaternion.Inverse(buildingRotations[j]) * (pos - center) + center;
                Vector3 rotatedv = buildingObjects[j].GetComponent<BoxCollider>().ClosestPointOnBounds(rotatedP);
                Vector3 v = (buildingRotations[j]) * (rotatedv - center) + center;

                //var v = Mathf.Infinity;
                if (Vector3.Distance(v, pos) < Vector3.Distance(vClosest, pos))
                    vClosest = v;

                // Pause when collision
                point[i].closestBuildingPoint = vClosest;
                if (buildingObjects[j].GetComponent<BoxCollider>().bounds.Contains(pos))
                {
                    Debug.Log("Drone " + i + " is in building " + j);
                    //Debug.Break();
                }
                buildingObjects[j].transform.rotation = buildingRotations[j];


            }
        }
        //moving_camera.transform.position = packagePos - 2*packageSpeed.normalized+new Vector3(0,0-2);
        //moving_camera.transform.LookAt(moving_camera.transform.position + packageSpeed, new Vector3(0, 0, -1));
        Vector3 dir = (packageGoal - moving_camera.transform.position).normalized;
        moving_camera.transform.position = packagePos -formation_sizes[currentCheckpoint] * 2 * dir + new Vector3(0, 0,-formation_sizes[currentCheckpoint]);
        moving_camera.transform.LookAt(moving_camera.transform.position + dir, new Vector3(0, 0, -1));
        if (UnityEditor.SceneView.sceneViews.Count > 0)
        {
            //UnityEditor.SceneView sceneView = (UnityEditor.SceneView)UnityEditor.SceneView.sceneViews[0];
            //sceneView.Focus();
        }
    }
}
