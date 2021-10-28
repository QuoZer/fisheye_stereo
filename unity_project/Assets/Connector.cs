using System.Collections;
using System.Collections.Generic;
using System;
using System.Globalization;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;
using System.Threading.Tasks;

public struct FilterValues
{
    public byte HLow;
    public byte HHigh;
    public byte SLow;
    public byte SHigh;
    public byte VLow;
    public byte VHigh;
};


public class Connector : MonoBehaviour
{

    [Header("Cameras")]
    public Camera camera1;
    public Camera camera2;

    [Header("Bowl")]
    public GameObject bowl;

    [Header("Sliders")]
    public Slider HLow;
    public Slider HHigh;
    public Slider SLow;
    public Slider SHigh;
    public Slider VLow;
    public Slider VHigh;

    [Header("Img parameters")]
    public int width = 1080;
    public int height = 1080;
    public bool showImages = false;
    public bool showPano = false;

    private Texture2D camTex1;
    private Texture2D camTex2;

    private FilterValues filter;        // structure to store HSV filter values

    private Rect readingRect;
    int isOk = 0;
    public int gap = 40;
    //bool isPanoReady = false;

    // Start is called before the first frame update
    void Start()
    {
        unsafe
        {
            //AllocConsole();
            initialize(width, height, 2);
        }

    }

    // Update is called once per frame
    bool pano = false;
    void Update()
    {

        camTex1 = CameraToTexture2D(camera1);
        camTex2 = CameraToTexture2D(camera2);

        TextureToCVMat(camTex1, camTex2);

        if (Input.GetKeyUp("[1]"))
        {
            InitTexture();
            bowl.GetComponent<Renderer>().material.mainTexture = tex;
            MatToTexture2D();
        }

        if (Input.GetKeyUp("[2]"))
        {
            // pano = !pano;        //idk
            Screenshoter(camTex1, camTex2, 0, true);

        }

        if (pano)
        {

            MatToTexture2D();
        }

        if (Input.GetKeyUp("[3]"))
        {
            initialize(width, height, 1);
        }
    }

    void OnApplicationQuit()
    {
        terminate();
        pixelHandle.Free();
    }

    void OnGUI()
    {
        GUI.Label(new Rect(0, 0, 100, 100), (1.0f / (Time.smoothDeltaTime)).ToString());
    }


    private Texture2D CameraToTexture2D(Camera camera)
    {
        Rect rect = new Rect(0, 0, width, height);
        RenderTexture renderTexture = new RenderTexture(width, height, 24);
        Texture2D screenShot = new Texture2D(width, height, TextureFormat.ARGB32, false);

        camera.targetTexture = renderTexture;
        camera.Render();

        RenderTexture.active = renderTexture;
        screenShot.ReadPixels(rect, 0, 0);

        camera.targetTexture = null;
        RenderTexture.active = null;

        Destroy(renderTexture);
        renderTexture = null;
        return screenShot;
    }

    unsafe void TextureToCVMat(Texture2D raw1, Texture2D raw2)
    {
        Color32[] rawColor1 = raw1.GetPixels32();
        Color32[] rawColor2 = raw2.GetPixels32();

        Color32*[] rawColors = new Color32*[2];
        /*
        filter.HLow = (byte)HLow.value;
        filter.HHigh = (byte)HHigh.value;
        filter.SLow = (byte)SLow.value;
        filter.SHigh = (byte)SHigh.value;
        filter.VLow = (byte)VLow.value;
        filter.VHigh = (byte)VHigh.value;
        */

    fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            //getImages((IntPtr)p1, (IntPtr)p2, (IntPtr)p3, (IntPtr)p4, raw1.width, raw1.height, show);
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                getImages((IntPtr)pointer, width, height, 2, showImages, filter);   // OCV gets images
            }
        }

        Destroy(raw1);
        Destroy(raw2);

        raw1 = null;
        raw2 = null;
    }

    unsafe void Screenshoter(Texture2D raw1, Texture2D raw2, int numOfCam, bool show)
    {
        Color32[] rawColor1 = raw1.GetPixels32();
        Color32[] rawColor2 = raw2.GetPixels32();

        Color32*[] rawColors = new Color32*[2];
    fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                //takeScreenshot((IntPtr)pointer, width, height, numOfCam, show);
                takeStereoScreenshot((IntPtr)pointer, width, height, 0, 1, show);
            }
        }

        Destroy(raw1);
        Destroy(raw2);

        raw1 = null;
        raw2 = null;
    }


    // for bowl
    private int polarWidth = 0;
    private int polarHeight = 0;

    private Texture2D tex;
    private Color32[] pixel32;
    private GCHandle pixelHandle;
    private IntPtr pixelPtr;

    public Texture2D getter_tex()
    {
        return tex;
    }

    public Color32[] getter_pix()
    {
        return pixel32;
    }

    void InitTexture()
    {
        tex = new Texture2D(polarWidth, polarHeight, TextureFormat.ARGB32, false);
        pixel32 = tex.GetPixels32();
        //Pin pixel32 array
        pixelHandle = GCHandle.Alloc(pixel32, GCHandleType.Pinned);
        //Get the pinned address
        pixelPtr = pixelHandle.AddrOfPinnedObject();
    }

    unsafe void MatToTexture2D()
    {
        //Convert Mat to Texture2D
        processImage(pixelPtr, polarWidth, polarHeight);
        //Update the Texture2D with array updated in C++
        tex.SetPixels32(pixel32);
        tex.Apply();
    }

    #region dllimport

    [DllImport("unity_plugin", EntryPoint = "initialize")]
    unsafe private static extern int initialize(int width, int height, int num);

    [DllImport("unity_plugin", EntryPoint = "terminate")]
    unsafe private static extern void terminate();

    [DllImport("unity_plugin", EntryPoint = "processImage")]
    unsafe private static extern void processImage(IntPtr data, int width, int height);

    [DllImport("unity_plugin", EntryPoint = "getImages")]
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, FilterValues filter);

    [DllImport("unity_plugin", EntryPoint = "takeScreenshot")]
    unsafe private static extern int takeScreenshot(IntPtr raw, int width, int height, int numOfCam, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "AllocConsole")]
    private static extern bool AllocConsole();
    #endregion
}
