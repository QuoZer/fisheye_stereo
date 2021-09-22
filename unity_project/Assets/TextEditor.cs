using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TextEditor : MonoBehaviour
{
    public Slider slider;
    private Text label;
    // Start is called before the first frame update

    private void Awake()
    {
        this.slider.onValueChanged.AddListener(this.OnSliderChanged);
        label = gameObject.GetComponent<Text>();
    }
    
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnSliderChanged(float value)
    {
        label.text = System.Convert.ToString((int)value);
    }
}
