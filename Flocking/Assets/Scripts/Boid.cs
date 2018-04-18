using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    public float maxSpeed;
    public float maxForce;
    public float arrivalRadius;
    private Rigidbody rigidbody;
    public Transform target;
    public Transform target;
    public Transform flee;    
    public float pathRadius;
    public List<Transform> pathCheckPoints;

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

    public void Flock()
    {

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