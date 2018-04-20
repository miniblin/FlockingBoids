using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    public float maxSpeed;
    public float maxForce;
    public float arrivalRadius;    
    public Transform target;
    public Transform target;
    public Transform flee;    
    public float pathRadius;
    public List<Transform> pathCheckPoints;
    public Vector3 minBounds;
    public Vector3 maxBounds;
    private Rigidbody rigidbody;

    private int boidArraySize;

    // Use this for initialization
    void Start()
    {
        this.rigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        Wander();
        rigidbody.velocity = Vector3.ClampMagnitude(rigidbody.velocity, maxSpeed);
        transform.LookAt(transform.position + rigidbody.velocity);        
    }

    public void SetBoidArraySize(int size)
    {
        boidArraySize = size;
    }

    public void CheckBounds()
    {
        Vector3 desired = rigidbody.velocity;
        if (transform.position.x < minBounds.x)
        {
            desired = (new Vector3(minBounds.x, transform.position.y, transform.position.z) - transform.position);
        }
        if (transform.position.y < minBounds.y)
        {
            desired = (new Vector3(transform.position.x, minBounds.y, transform.position.z) - transform.position);
        }
        if (transform.position.z < minBounds.z)
        {
            desired = (new Vector3(transform.position.x, transform.position.y, minBounds.z) - transform.position);
        }
        if (transform.position.x > maxBounds.x)
        {
            desired = (new Vector3(maxBounds.x, transform.position.y, transform.position.z) - transform.position);
        }
        if (transform.position.y > maxBounds.y)
        {
            desired = (new Vector3(transform.position.x, maxBounds.y, transform.position.z) - transform.position);
        }

        if (transform.position.z > maxBounds.z)
        {
            desired = (new Vector3(transform.position.x, transform.position.y, maxBounds.z) - transform.position);
        }

        Vector3 steer = 1.3f * Vector3.ClampMagnitude((desired - rigidbody.velocity), maxForce);

        rigidbody.AddForce(steer);       
    }

    public void Flee(Vector3 ThreatPosition)
    {
        Vector3 desired = (transform.position - ThreatPosition);
        desired.Normalize();
        desired *= maxSpeed;
        Vector3 steer = 0.8f * Vector3.ClampMagnitude((desired - rigidbody.velocity), maxForce);
        rigidbody.AddForce(steer);
    }

    public void Seek(Vector3 target)
    {
        Vector3 desired = (target - transform.position);
        float squaredDistance = desired.sqrMagnitude;
        desired.Normalize();

        //slow the boid down as it approaches its destination
        if (squaredDistance < (arrivalRadius * arrivalRadius))
        {
            desired *= (Map(squaredDistance, 0, arrivalRadius, 0, maxForce));
        }
        else
        {
            desired *= maxSpeed;
        }

        Vector3 steer = Vector3.ClampMagnitude((desired - rigidbody.velocity), maxForce);
        rigidbody.AddForce(steer);
    }

    public void FollowPath()
    {
        Vector3 predictedPoint = PredictedLocation(10);
        Vector3 pathTarget = new Vector3(0, 0, 0);
        float distanceFromNormal = 0;
        float closestNormal = 10000;

        for (int i = 0; i < pathCheckPoints.Count - 1; i++)
        {
            Vector3 pathStart = pathCheckPoints[i].position;
            Vector3 pathEnd = pathCheckPoints[i + 1].position;
            Vector3 normal = PointOfNormal(transform.position, pathStart, pathEnd);

            //check normal is on line
            if (DistanceLineSegmentPoint(pathStart, pathEnd, normal) > 0.0000000001)
            {
                Debug.Log(DistanceLineSegmentPoint(normal, pathStart, pathEnd));
                normal = pathEnd;
            }
            distanceFromNormal = Vector3.Distance(predictedPoint, normal);
            Debug.DrawLine(normal, transform.position, Color.red);
            if (distanceFromNormal < closestNormal)
            {
                closestNormal = distanceFromNormal;
                pathTarget = normal;
            }
            Debug.DrawLine(pathEnd, pathStart, Color.blue);
        }

        if (closestNormal > pathRadius)
        {
            Seek(pathTarget);
        }
        Debug.DrawLine(transform.position, pathTarget, Color.green);
    }
       

    public void Wander()
    {
        Vector3 predictedPoint = PredictedLocation(100);
        //TODO this is not the exact behaviour you want. 
        Vector3 randomDirection = predictedPoint + (Random.onUnitSphere.normalized * 30);        
        Seek(new Vector3(randomDirection.x, transform.position.y, randomDirection.z));
    }


    public void Flock(GameObject[,,] neighbours, int radius, int x, int y, int z, float desiredSeperation, float neighbourDistance)
    {    
        Vector3 difference;
        float d;
       
        seperationCount = 0;
        cohesionCount = 0;
        alignmentCount = 0;

        sumOfFleeVectors = Vector3.zero;
        sumOfNeighborPositions = Vector3.zero;
        sumOfNeighborPositions = Vector3.zero;

        for (int i = (-neighbourRadius); i <= neighbourRadius; i++)
        {
            for (int j = (-neighbourRadius); j <= neighbourRadius; j++)
            {
                for (int k = (-neighbourRadius); k <= neighbourRadius; k++)
                {
                    if (x + i < boidArraySize && y + j < boidArraySize && z + k < boidArraySize && x + i >= 0 && y + j >= 0 && z + k >= 0)
                    {
                        //seperation
                        d = Vector3.Distance(transform.position, neighbours[x + i, y + j, z + k].transform.position);
                        if (d < desiredSeperation && (d > 0))
                        {
                            difference = (transform.position - neighbours[x + i, y + j, z + k].transform.position);
                            difference.Normalize();
                            difference /= d;
                            sumOfFleeVectors += difference;
                            seperationCount++;
                        }

                        //cohesion and Alignment
                        if (d < neighbourDistance && (d > 0))
                        {
                            sumOfNeighborPositions += neighbours[x + i, y + j, z + k].transform.position;
                            cohesionCount++;
                            
                            sumOfNeighbourVeleocities += neighbours[x + i, y + j, z + k].GetComponent<Rigidbody>().velocity;
                            alignmentCount++;
                        }
                    }
                }
            }
        }

        if (seperationCount > 0)
        {
            sumOfFleeVectors /= seperationCount;
            sumOfFleeVectors.Normalize();
            sumOfFleeVectors *= (maxSpeed);
            Vector3 seperationSteer = Vector3.ClampMagnitude((sumOfFleeVectors - rigidbody.velocity), maxForce);
            rigidbody.AddForce(seperationSteer);
        }

        if (cohesionCount > 0)
        {
            sumOfNeighborPositions /= cohesionCount;
            Seek(sumOfNeighborPositions);
        }

        if (alignmentCount > 0)
        {
            sumOfNeighbourVeleocities /= alignmentCount;
            sumOfNeighbourVeleocities.Normalize();
            sumOfNeighbourVeleocities *= (maxSpeed);
            Vector3 steer = Vector3.ClampMagnitude((sumOfNeighbourVeleocities - rigidbody.velocity), maxForce);
            rigidbody.AddForce(0.4f * steer);
        }
    }

    private Vector3 PointOfNormal(Vector3 boidLocation, Vector3 pathStart, Vector3 pathEnd)
    {
        Vector3 path = pathEnd - pathStart;
        Vector3 startToBoid = boidLocation - pathStart;
        path.Normalize();
        path *= Vector3.Dot(startToBoid, path);
        return pathStart + path;
    }

    //maps a value from one range to another
    private float Map(float value, float originalMin, float originalMax, float newMin, float newMax)
    {
        return (newMin + (value - originalMin) * (newMax - newMin) / (originalMax - originalMin));
    }

    private Vector3 PredictedLocation(float distance)
    {
        return transform.position + (rigidbody.velocity.normalized * distance);
    }

    private DistanceLineSegmentPoint(Vector3 a, Vector3 b, Vector3 p)
    {
        // If a == b line segment is a point and will cause a divide by zero in the line segment test.
        // Instead return distance from a
        if (a == b)
            return Vector3.Distance(a, p);
        // Line segment to point distance equation
        Vector3 ba = b - a;
        Vector3 pa = a - p;
        return (pa - ba * (Vector3.Dot(pa, ba) / Vector3.Dot(ba, ba))).magnitude;
    }
}