using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WallBehaviour : MonoBehaviour {

    private float Deathtimer;
	// Use this for initialization
	void Start () {

        Deathtimer = 1000.0f;
		
	}
	
	// Update is called once per frame
	void Update () {

        Deathtimer -= 1;

        if (Deathtimer == 0)
        {
            Object.Destroy(this);
        }
		
	}
}
