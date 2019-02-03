using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class PoopmanBehaviour : MonoBehaviour {

    CircleCollider2D col;
    Text gameover;
	// Use this for initialization
	void Start () {
        
	}
    public GameObject Bird;
    // Update is called once per frame
    void Update () {
        transform.position = Vector2.Lerp(transform.position, Bird.transform.position, 0.02f);
        CircleCollider2D bcol = Bird.GetComponent<CircleCollider2D>();
        col = GetComponent<CircleCollider2D>();
        if (col.IsTouching(bcol))
        {
            SceneManager.LoadScene("gameover");

        }
    }
}
