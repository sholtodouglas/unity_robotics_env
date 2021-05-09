using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
public class PhysicsButton : MonoBehaviour
{
    public float value = 0f;
    [SerializeField] private float threshold = 0.1f;
    [SerializeField] private float deadZone = 0.025f;

    private bool _isPressed;
    private Vector3 _startPos;
    private ConfigurableJoint _joint;
    //public GameObject Base;
    
    public UnityEvent onPressed, onReleased;
    // Start is called before the first frame update
    void Start()
    {
        _startPos = transform.localPosition;
        _joint = GetComponent<ConfigurableJoint>();
        //Physics.IgnoreCollision(Base.GetComponent<Collider>(), GetComponent<Collider>());
    }

    // Update is called once per frame
    void Update()
    {
        if (!_isPressed && GetValue() + threshold >= 1)
            Pressed();
        if (_isPressed && GetValue()  - threshold <= 0)
            Released();
    }

    private float GetValue()
    {
        value = Vector3.Distance(_startPos, transform.localPosition) / _joint.linearLimit.limit;
        if (Math.Abs(value) < deadZone)
            value = 0;
        value =  Mathf.Clamp(value, -1f, 1f);
        
        return value;
    }

    private void Pressed()
    {
        _isPressed = true;
        onPressed.Invoke();
        print("Pressed");
    }
    
    private void Released()
    {
        _isPressed = false;
        onReleased.Invoke();
        print("Released");
    }

}
