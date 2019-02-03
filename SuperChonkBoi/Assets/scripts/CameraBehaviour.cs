using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraBehaviour : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}


    public GameObject BALL;

    // Update is called once per frame
    void LateUpdate () {

        Vector3 pos = BALL.transform.position;
        pos.z -= 10.0f;
        transform.position = pos;


        /*
        if (Input.GetKey(KeyCode.LeftShift))
            Time.timeScale = 0.5F;
        else
            Time.timeScale = 1.0F;
        */
    }
}
