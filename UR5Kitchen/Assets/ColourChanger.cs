using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ColourChanger : MonoBehaviour
{
    public Material red;
    public Material green;
    public Material blue;
    public Material white;
    // Start is called before the first frame update
    void Start()
    {
       
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    // these are used by the events in the buttons
    public void makeRed() {
         this.GetComponent<Renderer>().material = red;
    }

    
    public void makeBlue() {
         this.GetComponent<Renderer>().material = blue;
    }

    
    public void makeGreen() {
         this.GetComponent<Renderer>().material = green;
    }

    
    public void makeWhite() {
         this.GetComponent<Renderer>().material = white;
    }
}
