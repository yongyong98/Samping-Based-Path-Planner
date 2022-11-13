using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Linq;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UIElements;


public class SpawnObject : MonoBehaviour
{
    public float speed = 10;
    public GameObject Foodprefab;
    public GameObject StartPoint, GoalPoint; 
    public Vector3 center; 
    public Vector3 size; 
    public BoxCollider[] obstacles; //Obstacle 
    RaycastHit hitData; // RayCast 쏴서 부딪히는게 있는지 확인하는 변수 
    public List<Vector3> NodePosition = new List<Vector3>(); // 랜덤 생성한 Node의 위치를 받는 배열 변수
    List<int> checkNode = new List<int>(); // v 계산할때 이전에 호출 됐으면 안하려고 
    System.Random Rand = new System.Random();
    float INF = float.MaxValue;


    // Start is called before the first frame update
    void Start()
    {
        SpawnNode();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.Q)){
            SpawnNode();
        }
        if (Input.GetKey(KeyCode.S)) {

            CheckObstacle();
        }

        
    }
    public void SpawnNode()
    {   
        Vector3 pos;
        NodePosition.Add(StartPoint.transform.position);
        NodePosition.Add(GoalPoint.transform.position);
        
        while (true)
        {
            bool isTriggered = false;
            pos = center + new Vector3(UnityEngine.Random.Range(-size.x / 2, size.x / 2), UnityEngine.Random.Range(-size.y / 2, size.y / 2), UnityEngine.Random.Range(-size.z / 2, size.z / 2));
            //pos = center + new Vector3(Random.Range(-size.x/2,size.x/2), Random.Range(-size.y /2, size.y / 2), Random.Range(-size.z/2,size.z/2));
            foreach(BoxCollider obstacle in obstacles)
            {
                Vector3 obPos = obstacle.bounds.center;
                Vector3 obSize = obstacle.bounds.extents;

                if(pos.x > obPos.x - obSize.x && pos.x < obPos.x + obSize.x &&
                   pos.y > obPos.y - obSize.y && pos.y < obPos.y + obSize.y &&
                   pos.z > obPos.z - obSize.z && pos.z < obPos.z + obSize.z)
                {
                    isTriggered = true;
                    break;
                }
            }
            
            if(isTriggered == false)
            {
                break;
            }
        }
        NodePosition.Add(pos);
        Instantiate(Foodprefab, pos, Quaternion.identity);
    }

    public void CheckObstacle()
    {
        //NodePosition = NodePosition.OrderBy(v => v.x).ToList();
        //int i = 0; 
        //foreach (Vector3 c in NodePosition)
        //{
        //
        //    int v_index = Rand.Next(0, NodePosition.Count);
        //    //Vector3 v = NodePosition[Rand.Next(0, NodePosition.Count)];
        //    Vector3 v = NodePosition[v_index];
        //  
        //    checkNode.Add(i);
        //    checkNode.Add(v_index);
        //
        //    if (!Physics.Raycast(c, v, out hitData))
        //    {
        //        Debug.Log("Hit point: " + hitData.point + "distance: " + hitData.distance);
        //        Debug.DrawRay(c, v, Color.red, 10f);
        //    }
        //    else
        //    {
        //        //Debug.DrawRay(c, v * 1000f, Color.red);
        //    }
        //   
        //}
        //Debug.Log(NodePosition.Count);


        float[,] Graph = new float[NodePosition.Count, NodePosition.Count]; // Graph 정보 저장할 배열 생성
        Debug.Log(NodePosition.Count);
        for(int i = 0; i<NodePosition.Count;i++)
        {
            for (int j = 0; j<NodePosition.Count; j++)
            {
                if (i == j)
                    Graph[i, j] = 0;
                else
                    Graph[i,j] = INF;
            }
        }
        int x = 0;
        
        foreach (Vector3 c in NodePosition)
        {
            int y = 0;
            foreach (Vector3 v in NodePosition)
            {
                //if (c != v && !(Physics.Raycast(c, v - c, Vector3.Distance(c,v))) && Vector3.Distance(c, v) < 200f)
                if (c != v && !(Physics.Raycast(c, v-c, Vector3.Distance(c, v))) && Vector3.Distance(c, v) < 100f)
                {
                    //Debug.DrawRay(c, v - c, Color.red, 100f);
                    //Debug.DrawRay(c, v, Color.red, 100f);
                       Graph[x, y] = Vector3.Distance(c,v);
                    //Debug.Log(Graph[x, y]);
                }
                y++;
            }
            x++;
        }

        GFG t = new GFG();
        int[] s = new int[100];
        s = t.dijkstra(Graph, 0, NodePosition.Count);        
        for (int i = 0; i < s.Length; i++)
        {
            Debug.Log(s[i]);
        }


        Vector3 st;
        st = StartPoint.transform.position;
        for (int i = 0; i < s.Length; i++)
        {
            Debug.DrawRay(st, NodePosition[s[i]] - st, Color.red, 100f);
            //Vector3 dir = NodePosition[s[i]] - st;            
            //StartPoint.transform.position = Vector3.MoveTowards(st, NodePosition[s[i]], speed * Time.deltaTime); //두번째 파라메터 마지막 +전까지가 vector 방향을 나타냄 (앞에 vector normalization을 더한 이유는 옆으로 돌때 잘 돌라고 보정한거임 )
         
            st = NodePosition[s[i]];
        }
        //StartPoint.transform.position = Vector3.MoveTowards(st, NodePosition[1], speed * Time.deltaTime); //두번째 파라메터 마지막 +전까지가 vector 방향을 나타냄 (앞에 vector normalization을 더한 이유는 옆으로 돌때 잘 돌라고 보정한거임 )
        


        /*
        if (path_idx.Length <= curTargetNode)
        {
            return;
        }

        //Vector3 dir = NodePosition[path_idx[curTargetNode]] - StartPoint.transform.position;
        //StartPoint.transform.position += dir.normalized * Time.deltaTime * speed;

        //for(int i = 1; i < curTargetNode - 1; i++)
        //{
        //    Debug.DrawRay(NodePosition[path_idx[i]], NodePosition[path_idx[i+1]] - NodePosition[path_idx[i]], Color.red, 100f);
        //}

        Vector3 st = StartPoint.transform.position;

        Debug.DrawRay(st, NodePosition[path_idx[curTargetNode]] - st, Color.red, 100f);
        Vector3 dir = NodePosition[path_idx[curTargetNode]] - st;
        //StartPoint.transform.position = (dir.normalized * speed * Time.deltaTime) + StartPoint.transform.position;
        StartPoint.transform.position = Vector3.MoveTowards(st, NodePosition[path_idx[curTargetNode]], speed * Time.deltaTime);
        //StartPoint.transform.position = Vector3.MoveTowards(st, NodePosition[s[i]], speed * Time.deltaTime); //두번째 파라메터 마지막 +전까지가 vector 방향을 나타냄 (앞에 vector normalization을 더한 이유는 옆으로 돌때 잘 돌라고 보정한거임 )
        //st = NodePosition[path_idx[curTargetNode]];

        Debug.Log(Vector3.Distance(st, NodePosition[path_idx[curTargetNode]]));

        if (Vector3.Distance(st, NodePosition[path_idx[curTargetNode]]) < 1)
        {
            curTargetNode++;
        }
        */

    }
    
    void OnDrawGizmosSelected(){
        Gizmos.color = new Color(1,0,0,0.5f);
        Gizmos.DrawCube(center, size);

    }
}

 class GFG
{
    // A utility function to find the
    // vertex with minimum distance
    // value, from the set of vertices
    // not yet included in shortest
    // path tree
    static int V = 10000;
    //static int V = NodePosition.Count;
    int minDistance(float[] dist,
                    bool[] sptSet)
    {
        // Initialize min value
        float min = float.MaxValue;
        int min_index = -1;

        for (int v = 0; v < V; v++)
            if (sptSet[v] == false && dist[v] <= min)
            {
                min = dist[v];
                min_index = v;
            }

        return min_index;
    }

    // A utility function to print
    // the constructed distance array
    void printSolution(float[] dist, int n)
    {
        //Debug.Log("Vertex     Distance " + "from Source\n");
        //for (int i = 0; i < V; i++)
        //    Debug.Log(i + " \t\t " + dist[i] + "\n");
    }
    int[] returnNode(int[] s)
    {
        return s;
    }

    // Function that implements Dijkstra's
    // single source shortest path algorithm
    // for a graph represented using adjacency
    // matrix representation
    public int[] dijkstra(float[,] graph, int src, int _count)
    {
        V = _count;

        float[] dist = new float[V]; // The output array. dist[i]
                                     // will hold the shortest
                                     // distance from src to i
        int[] parent = new int[V];
        for (int i = 0; i< V; i++)
        {
            parent[i] = 0;
        }
        // sptSet[i] will true if vertex
        // i is included in shortest path
        // tree or shortest distance from
        // src to i is finalized
        bool[] sptSet = new bool[V];
        // Initialize all distances as
        // INFINITE and stpSet[] as false
        for (int i = 0; i < V; i++)
        {
            dist[i] = float.MaxValue;
            sptSet[i] = false;
        }

        // Distance of source vertex
        // from itself is always 0
        dist[src] = 0;

        // Find shortest path for all vertices
        for (int count = 0; count < V - 1; count++)
        {
            // Pick the minimum distance vertex
            // from the set of vertices not yet
            // processed. u is always equal to
            // src in first iteration.
            int u = minDistance(dist, sptSet);

            // Mark the picked vertex as processed
            sptSet[u] = true;

            // Update dist value of the adjacent
            // vertices of the picked vertex.
            for (int v = 0; v < V; v++)
            {

                if (!sptSet[v] && graph[u, v] != 0 && dist[u] != float.MaxValue && dist[u] + graph[u, v] < dist[v])
                {
                    dist[v] = dist[u] + graph[u, v];

                    if (v < V)
                    {
                        parent[v] = u;
                    }
                }
            }
            // Update dist[v] only if is not in
            // sptSet, there is an edge from u
            // to v, and total weight of path
            // from src to v through u is smaller
            // than current value of dist[v]


        }
        Stack<int> s = new Stack<int>();
        int z = 1;
  
        while(z != parent[z])
        {
            s.Push(z);
            z = parent[z];
        }
        s.Push(z);
        
        int[] array = new int[s.Count];
        int a;
        int k = 0;
        while(s.Count > 0)
        {
            a = s.Pop();
            array[k] = a;
            k++;
        }
        printSolution(dist, V);
        return array;
        //returnNode(array);
        // print the constructed distance array
        
    }

  
}
