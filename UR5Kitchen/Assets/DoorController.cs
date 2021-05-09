using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
public class DoorController : MonoBehaviour
{
    public float value = 0f;


    private Vector3 _startPos;
    private ConfigurableJoint _joint;
    //public GameObject Base;
    private int jointIdx = 0;
    void Start()
    {
        _startPos = transform.localPosition;
        _joint = GetComponent<ConfigurableJoint>();
        //Physics.IgnoreCollision(Base.GetComponent<Collider>(), GetComponent<Collider>());
        
    }

    // Update is called once per frame
    void Update()
    {   
        
        value =  (transform.localPosition[jointIdx] - _startPos[jointIdx]) / _joint.linearLimit.limit;
        value =  Mathf.Clamp(value, -1f, 1f);
    }

    public void Reset(float pos) {
        Vector3 localPos = transform.localPosition;
        localPos[jointIdx] = pos* _joint.linearLimit.limit + _startPos[jointIdx];
        transform.localPosition = localPos;
    }
}
