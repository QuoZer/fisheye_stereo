using System.Collections;
using System.Collections.Generic;
using System;
using System.Globalization;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.UI;
using MultiThreading;



public struct SGBMparams
{
    public byte preFilterSize;
    public byte preFilterCap;
    public byte blockSize;
    public byte minDisparity;
    public byte numDisparities;
    public byte textureThreshold;
    public byte uniquenessRatio;
    public byte speckleRange;
    public byte disp12MaxDiff;
    public byte speckleWindowSize;
};

public class Connector : MonoBehaviour
{
    [Header("Cameras")]
    public Camera camera1;
    public int cam1XRot;
    public Camera camera2;
    public int cam2XRot;
    public Camera camera3;
    public Camera camera4;

    [Header("Screens")]
    public GameObject disparityScreen;
    // for bowl
    private int dispScreenWidth = 1080;
    private int dispScreenHeight = 1080;
    private Texture2D dispScreenTexture;
    private Color32[] dispScreenPixels;
    private GCHandle pixelHandle;
    private IntPtr pixelPtr;


    [Header("Sliders")]
    public Slider preFilterSize;
    public Slider preFilterCap;
    public Slider blockSize;
    public Slider minDisparity;
    public Slider numDisparities;
    public Slider textureThreshold;
    public Slider uniquenessRatio;
    public Slider speckleRange;
    public Slider speckleWindowSize;
    public Slider disp12MaxDiff;

    [Header("Img parameters")]
    public static int width = 1080;
    public static int height = 1080;
    public static bool showImages = false;
    public bool showPano = false;
    // Fisheye
    private Texture2D camTex1;
    private Texture2D camTex2;
    // Regular
    private Texture2D camTex3;
    private Texture2D camTex4;

    private SGBMparams sgbm;        // structure to store SGBM parameter values

    private Rect readingRect;
    int isOk = 0;
    public int gap = 40;
    int actionId = 0;

    bool pano = false;
    // Threading stuff
    DisaprityCalculator imageProcessingThread;
    DisaprityCalculator regularCameraThread;
    bool threadStarted = false;
    bool initFlag = false;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start Function");
        InitTexture();
        disparityScreen.GetComponent<Renderer>().material.mainTexture = dispScreenTexture;
        unsafe {
            //AllocConsole();
            sgbm = new SGBMparams();
            imageProcessingThread  = new DisaprityCalculator(1, width, height, showImages, cam1XRot, cam2XRot, pixelPtr);
            regularCameraThread = new DisaprityCalculator(0, width, height, showImages, 0, 0, pixelPtr);
        }

    }
    // Update is called once per frame
    void Update()
    {
        // Fisheye cameras
        camTex1 = CameraToTexture2D(camera1);
        camTex2 = CameraToTexture2D(camera2);
        // Regular cameras
        camTex3 = CameraToTexture2D(camera3);
        camTex4 = CameraToTexture2D(camera4);

        fillStereoParams();        // fills 'sgbm' structure with slider values 
        actionId = 0;               // Default to update action
        // TODO: id's are messed up
        /* Update texture in the world */
        if (Input.GetKeyUp("[1]")) {
            actionId = 3;
            // TODO: World texture renderer
        }
        /* Take screenshot */
        if (Input.GetKeyUp("[2]")) {

           actionId = 1;
        }
        /* Reinitialize */
        if (Input.GetKeyUp("[3]")) {
            actionId = 2;
        }
        /* Start the second thread */
        if (!threadStarted)         // image processing thread hasn't been started before
        {
            Debug.Log("Starting thread");
            regularCameraThread.Start();
            imageProcessingThread.Start();      // start the thread
            threadStarted = true;
        }
        /* Pass the data to the thread */
        imageProcessingThread.Update(camTex1.GetPixels32(), camTex2.GetPixels32(), sgbm, actionId);
        regularCameraThread.Update(camTex3.GetPixels32(), camTex4.GetPixels32(), sgbm, 0);
        
        /* textures are not erased by the GC automatically */
        Destroy(camTex1);
        Destroy(camTex2);
        Destroy(camTex3);
        Destroy(camTex4);
    }


    void OnApplicationQuit()
    {
        pixelHandle.Free();
        imageProcessingThread.Abort();
        regularCameraThread.Abort();
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

        fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                getImages((IntPtr)pointer, width, height, 1, 2, showImages, sgbm);   // OCV gets images
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


    void InitTexture()
    {
        dispScreenTexture = new Texture2D(dispScreenWidth, dispScreenHeight, TextureFormat.ARGB32, false);
        dispScreenPixels = dispScreenTexture.GetPixels32();
        //Pin pixel32 array
        pixelHandle = GCHandle.Alloc(dispScreenPixels, GCHandleType.Pinned);
        //Get the pinned address
        pixelPtr = pixelHandle.AddrOfPinnedObject();
    }

    unsafe void MatToTexture2D()
    {
        //Convert Mat to Texture2D
        processImage(pixelPtr, dispScreenWidth, dispScreenHeight);
        //Update the Texture2D with array updated in C++
        dispScreenTexture.SetPixels32(dispScreenPixels);
        dispScreenTexture.Apply();
    }
    
    void fillStereoParams()
    {     
        sgbm.preFilterCap     = (byte)preFilterCap.value;
        sgbm.preFilterSize    = (byte)preFilterSize.value;
        sgbm.blockSize        = (byte)blockSize.value;
        sgbm.minDisparity     = (byte)minDisparity.value;
        sgbm.numDisparities   = (byte)numDisparities.value;
        sgbm.textureThreshold = (byte)textureThreshold.value;
        sgbm.uniquenessRatio  = (byte)uniquenessRatio.value;
        sgbm.speckleRange     = (byte)speckleRange.value;
        sgbm.disp12MaxDiff    = (byte)disp12MaxDiff.value;
    }

    #region dllimport

    [DllImport("unity_plugin", EntryPoint = "initialize")]
    unsafe private static extern int initialize(int width, int height, int num, int leftRot, int rightRot);

    [DllImport("unity_plugin", EntryPoint = "terminate")]
    unsafe private static extern void terminate();

    [DllImport("unity_plugin", EntryPoint = "processImage")]
    unsafe private static extern void processImage(IntPtr data, int width, int height);

    [DllImport("unity_plugin", EntryPoint = "getImages")]
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, int imageType, bool isShow, SGBMparams sgbm);

    [DllImport("unity_plugin", EntryPoint = "takeScreenshot")]
    unsafe private static extern int takeScreenshot(IntPtr raw, int width, int height, int numOfCam, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "AllocConsole")]
    private static extern bool AllocConsole();
    #endregion
}
