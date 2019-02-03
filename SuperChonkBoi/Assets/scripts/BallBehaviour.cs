using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BallBehaviour : MonoBehaviour {


    private Vector3 origin;
    GameObject brik;
    public GameObject stamina;
    public static float currStamina = 1000;
    public static float maxStamina = 1000;
    public static float maxVelocity = 4.5f;
    public Text ScoreText;
    public int score;
    public float blockrate;
    public int initscore;
    public float force;
	// Use this for initialization
	void Start () {

        origin = transform.position;
        stamina.GetComponent<Image>().color = new Color(0,1,0);
        stamina.GetComponent<RectTransform>().localScale = new Vector3(currStamina / maxStamina, 1, 1);
        brik = Resources.Load("WALL") as GameObject;
        score = 0;
        initscore = 0;
        blockrate = 1.5f;
        force = 0.3f;
        PlayerPrefs.SetInt("CurrScore", 0);
        currStamina = 1000;
	}
	
	// Update is called once per frame
	void Update () {
        Rigidbody2D rb = this.GetComponent<Rigidbody2D>();

        checkInput();

        genWalls();

        stamina.GetComponent<RectTransform>().localScale = new Vector3(currStamina / maxStamina, 1, 1);

        rb.velocity = Vector2.ClampMagnitude(rb.velocity, maxVelocity);

        score++;
        PlayerPrefs.SetInt("CurrScore", score);

        if (score - initscore > 500)
        {
            initscore = score;
            blockrate -= 0.30f*blockrate;
        }

        ScoreText.text = "Score: " + score.ToString();

    }

    void genWalls() {
        if(Vector3.Distance(origin, transform.position) > blockrate)
        {
            GameObject obstacle = Instantiate(brik) as GameObject;
            //GameObject obstacle1 = Instantiate(brik) as GameObject;
            System.Random rnd = new System.Random();
            obstacle.transform.position = transform.position + new Vector3(rnd.Next(-5,5), rnd.Next(-10, -3), 0.0f);
            //obstacle1.transform.position = transform.position + new Vector3(rnd.Next(-5, 5), rnd.Next(-10, -3), 0.0f);
            origin = transform.position;
        }


    }

    void checkInput()
    {
        Rigidbody2D rb = this.GetComponent<Rigidbody2D>();

        //add forces and drain stamina appropriately
        if (Input.GetKey("a"))
        {
            if (currStamina > 0)
            {
                currStamina -= 5;
                rb.AddForce(new Vector2(-force, 0), ForceMode2D.Impulse);
                transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(new Vector3(-0.001f, 0.0f, 0.0f)), 0.15f);

            }
        }
        else if (Input.GetKey("d"))
        {
            if (currStamina > 0)
            {
                currStamina -= 5;
                rb.AddForce(new Vector2(force, 0), ForceMode2D.Impulse);
                transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.LookRotation(new Vector3(0.001f, 0.0f, 0.0f)), 0.15f);

            }
        }
        else
        {
            if (currStamina < maxStamina)
                currStamina += 1.5f;
            
            transform.rotation = new Quaternion(0, 0, 0, 0);
        }


        //drain stamina if slowing time
        if (Input.GetKey(KeyCode.LeftShift))
        {
            if (currStamina > 0)
            {
                currStamina -= 5;
                force = 40.0f;
            }
            }
        else
        {
            force = 0.3f;
        }
        
        
    }

}
