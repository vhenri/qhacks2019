using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class GameOverBehaviour : MonoBehaviour {

    public Text finalscore;
    public Text HighScore;
	// Use this for initialization
	void Start () {

        int finalscorea = PlayerPrefs.GetInt("CurrScore");
        finalscore.text = "Score: " + finalscorea.ToString();
        if (finalscorea > PlayerPrefs.GetInt("HighScore"))
        {
            PlayerPrefs.SetInt("HighScore", finalscorea);
        }
        HighScore.text = "High Score: " + PlayerPrefs.GetInt("HighScore").ToString();

    }
	
	// Update is called once per frame
	void Update () {

        if (Input.GetKey("s"))
        {
            SceneManager.LoadScene("title");
        }
        
	}
}
