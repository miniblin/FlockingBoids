using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using CielaSpike;
public class FlockMangerWithInsertionSort : MonoBehaviour
{
    public int size;
    Vector3[,,] values;
    int numChanges = 0;
    GameObject[,,] objects;
    public Transform target;
    public Transform flee;
    public int radius;
    public float desiredSeperation;
    public int spawnArea;

    public float neighbourDistance;

    void Start()
    {
        objects = new GameObject[size, size, size];
        values = new Vector3[size, size, size];

        CreateGameObjects(size);
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    objects[i, j, h].GetComponent<Boid>().SetBoidArraySize(size);
                    //consider usng a static.for much faster startup
                }
            }
        }
        GetPositions();
        this.StartCoroutineAsync(SortAll());
    }

    void Update()
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    objects[i, j, h].GetComponent<Boid>().Flock(objects, radius, i, j, h, desiredSeperation, neighbourDistance);
                }
            }
        }
    }

    int a = 0;
    public void MainThreadSort()
    {
        GetPositions();
        SortX(values);
        SortY(values);
        SortZ(values);
        a++;
        Debug.Log("Pass #" + a);
    }

    //used in testing, to see if the sort is calculating the correct neighbours for the boids. (color codes them)
    public void checkNeighbours(int e, int f, int g, int radius)
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    objects[i, j, h].GetComponent<Renderer>().material.color = Color.red;
                }
            }
        }

        for (int i = (-radius); i <= radius; i++)
        {
            for (int j = (-radius); j <= radius; j++)
            {
                for (int k = (-radius); k <= radius; k++)
                {
                    if (e + i < size && f + j < size && g + k < size && e + i >= 0 && f + j >= 0 && g + k >= 0)
                    {
                        objects[e + i, f + j, g + k].GetComponent<Renderer>().material.color = Color.blue;
                    }
                }
            }
        }
        objects[e, f, g].GetComponent<Renderer>().material.color = Color.green;
    }

    bool UpdateFlock = true;
    IEnumerator SortAll()
    {
        while (UpdateFlock)
        {               
            yield return Ninja.JumpToUnity;
            GetPositions();
            yield return Ninja.JumpBack;

            SortX(values);
            SortY(values);
            SortZ(values);
            a++;
            Debug.Log("Pass #" + a);
        }
        yield break;
    }
    
    private void OnApplicationQuit()
    {
        UpdateFlock = false;
    }
    
    public GameObject[,,] CreateGameObjects(int size)
    {
        System.Random random = new System.Random();
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    objects[i, j, h] = (GameObject)Instantiate(Resources.Load("Seagull"), new Vector3(random.Next(-spawnArea, spawnArea), random.Next(-spawnArea, spawnArea), random.Next(-spawnArea, spawnArea)), Quaternion.identity);                    
                }
            }
        }
        return objects;
    }

    public void GetPositions()
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    values[i, j, h] = objects[i, j, h].transform.position;
                }
            }
        }
    }

    public Vector3[,,] CreateRandomVectorArray(int size)
    {
        System.Random random = new System.Random();
        Vector3[,,] values = new Vector3[size, size, size];
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                for (int h = 0; h < size; h++)
                {
                    values[i, j, h] = new Vector3((random.Next(-250, 250)), random.Next(-250, 250), random.Next(-250, 250));
                }
            }
        }
        return values;
    }

    public void SortX(Vector3[,,] values)
    {
        for (int y = 0; y < values.GetLength(1); y++)
        {
            for (int z = 0; z < values.GetLength(2); z++)
            {
                int loopy = y;
                int loopz = z;
                SortRow(loopy, loopz);
            }
        }
    }

    public void SortRow(int y, int z)
    {
        int j;
        for (int i = 1; i < values.GetLength(0); i++)
        {
            Vector3 x = values[i, y, z];
            GameObject gX = objects[i, y, z];
            j = i - 1;
            while (j >= 0 && values[j, y, z].x > x.x)
            {
                numChanges++;
                values[j + 1, y, z] = values[j, y, z];
                objects[j + 1, y, z] = objects[j, y, z];
                j = j - 1;
            }

            values[j + 1, y, z] = x;
            objects[j + 1, y, z] = gX;
        }       
    }

    public void SortY(Vector3[,,] values)
    {
        int j;
        for (int x = 0; x < values.GetLength(0); x++)
        {
            for (int z = 0; z < values.GetLength(2); z++)
            {
                for (int i = 1; i < values.GetLength(0); i++)
                {
                    Vector3 y = values[x, i, z];
                    GameObject gY = objects[x, i, z];
                    j = i - 1;
                    while (j >= 0 && values[x, j, z].y > y.y)
                    {
                        numChanges++;
                        values[x, j + 1, z] = values[x, j, z];
                        objects[x, j + 1, z] = objects[x, j, z];
                        j = j - 1;
                    }

                    values[x, j + 1, z] = y;
                    objects[x, j + 1, z] = gY;
                }
            }
        }
    }

    public void SortZ(Vector3[,,] values)
    {
        int j;
        for (int x = 0; x < values.GetLength(0); x++)
        {
            for (int y = 0; y < values.GetLength(1); y++)
            {
                for (int i = 1; i < values.GetLength(2); i++)
                {
                    Vector3 z = values[x, y, i];
                    GameObject gZ = objects[x, y, i];
                    j = i - 1;
                    while (j >= 0 && values[x, y, j].z > z.z)
                    {
                        numChanges++;
                        values[x, y, j + 1] = values[x, y, j];
                        objects[x, y, j + 1] = objects[x, y, j];
                        j = j - 1;
                    }
                    values[x, y, j + 1] = z;
                    objects[x, y, j + 1] = gZ;
                }
            }
        }
    }
}
