using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Boid : MonoBehaviour
{
    public float maxSpeed;
    public float maxForce;
    private Rigidbody rigidbody;
    public Transform target;

    // Use this for initialization
    void Start()
    {
        this.rigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        Seek(target.position);
        rigidbody.velocity = Vector3.ClampMagnitude(rigidbody.velocity, maxSpeed);
    }


    public void Flee(Vector3 ThreatPosition)
    {

    }

    public void Seek(Vector3 target)
    {
        Vector3 desired = (target - transform.position);
        desired.Normalize();
        desired *= maxSpeed;
        Vector3 steer = Vector3.ClampMagnitude((desired - rigidbody.velocity), maxForce);
        rigidbody.AddForce(steer);
    }


    public void Wander()
    {

    }

    public void Flock()
    {

    }

}