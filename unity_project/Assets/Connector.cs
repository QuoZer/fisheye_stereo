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

public class ThreadInitializer: MultiThreading.ThreadedJob
{
    public int code = -1;  // out error code 
    int width;
    int height;
    int leftYaw;
    int rightYaw;

    public void SetParameters(int w, int h, int leftAngle, int rightAngle)
    {
        width = w;
        height = h;
        leftYaw = leftAngle;
        rightYaw = rightAngle;
    }

    unsafe protected override void ThreadFunction()
    {
        Debug.Log("Initializing...");
        code = initialize(width, height, 2, leftYaw, rightYaw);
        Debug.Log("Closing thread with code:" + code);
    }


    [DllImport("unity_plugin", EntryPoint = "initialize")]
    unsafe private static extern int initialize(int width, int height, int num, int leftRot, int rightRot);
}

public class DisaprityCalculator: MultiThreading.ThreadedJob
{
    public int code;  // out error code 
    Texture2D raw1; 
    Texture2D raw2;
    int width;
    int height;
    bool showImages;
    bool parametersUpdated = false;
    SGBMparams sgbm;
    unsafe Color32** imagePtr;


    public void SetParameters(Texture2D rawImg1, Texture2D rawImg2, int w, int h, bool show, SGBMparams stereoparams)
    {
        raw1 = rawImg1;
        raw2 = rawImg2;
        width = w;
        height = h;
        showImages = show;
        sgbm = stereoparams;
        parametersUpdated = true;

        Debug.Log("Parameters Set");
    }

    unsafe protected override void PreStart()
    {
        //Debug.Log("Pre Start");
        Color32[] rawColor1 = raw1.GetPixels32();
        Color32[] rawColor2 = raw2.GetPixels32();
        
        Color32*[] rawColors = new Color32*[2];
        
        fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                imagePtr = pointer;
            }
            
        }
    }

    public override bool Update()
    {
        if (!parametersUpdated)
        {
            //OnFinished();
            return true;
        }
        return false;
    }

    protected override void OnFinished()
    {
        UnityEngine.Object.Destroy(raw1);
        UnityEngine.Object.Destroy(raw2);
        
        raw1 = null;
        raw2 = null;
    }

    unsafe protected override void ThreadFunction()
    {
        Debug.Log("Sending frame...");
        //code = takeStereoScreenshot((IntPtr)imagePtr, width, height, 0, 1, false);

        while (true)
        {
            if (parametersUpdated)
            {
                code = getImages((IntPtr)imagePtr, width, height, 2, showImages, sgbm);
                parametersUpdated = false;
                Debug.Log("image processed");
            }
            else Thread.Sleep(50);
        }

        //Debug.Log("Leaving thread with code:"+code);
    }

    [DllImport("unity_plugin", EntryPoint = "getImages")]
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);
    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

}

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

    [Header("Bowl")]
    public GameObject bowl;

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
    public int width = 1080;
    public int height = 1080;
    public bool showImages = true;
    public bool showPano = false;

    private Texture2D camTex1;
    private Texture2D camTex2;

    private SGBMparams sgbm;        // structure to store SGBM parameter values

    private Rect readingRect;
    int isOk = 0;
    public int gap = 40;
    //bool isPanoReady = false;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start Function");
        unsafe {
            //AllocConsole();
            sgbm = new SGBMparams();
            StartCoroutine(Initializer());         // HACK: to quickly test initialization
        }

    }

    bool pano = false;
    // Threading stuff
    DisaprityCalculator imageProcessingThread = new DisaprityCalculator();
    bool threadStarted = false;
    bool initFlag = false;
    bool processingFrame = false;

    // Update is called once per frame
    void Update()
    {

        camTex1 = CameraToTexture2D(camera1);
        camTex2 = CameraToTexture2D(camera2);

        fillStereoParams();
        if (initFlag == true)           // once the second thread finishes initialization
        {
            if (!processingFrame)
            {
                processingFrame = true;
                StartCoroutine(Dewarper(camTex1, camTex2));
            }
        }


        if (Input.GetKeyUp("[1]")) {
            InitTexture();
            bowl.GetComponent<Renderer>().material.mainTexture = tex;
            MatToTexture2D();
        }

        if (Input.GetKeyUp("[2]")) {
            // pano = !pano;        //idk
            Screenshoter(camTex1, camTex2, 0, true);
        }

        if (pano) {
            MatToTexture2D();
        }

        if (Input.GetKeyUp("[3]")) {
            initialize(width, height, 1, cam1XRot, cam2XRot);
        }

        System.GC.Collect();        // doesn't seem to fix leaking
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

    
    IEnumerator Dewarper(Texture2D raw1, Texture2D raw2)
    {
        imageProcessingThread.SetParameters(raw1, raw2, width, height, showImages, sgbm);
        if (!threadStarted)
        {
            Debug.Log("Starting thread");
            imageProcessingThread.Start();
            threadStarted = true;
        }

        yield return StartCoroutine(imageProcessingThread.WaitFor());
        processingFrame = imageProcessingThread.code == 0 ? false : true;
    }

    IEnumerator Initializer()
    {
        ThreadInitializer thread = new ThreadInitializer();
        thread.SetParameters(width, height, cam1XRot, cam2XRot);
        Debug.Log("Starting init thread");
        thread.Start();

        yield return StartCoroutine(thread.WaitFor());

        initFlag = thread.code == 0 ? true : false;
        if (initFlag)
        {
            thread.Abort();
            Debug.Log("Abort init thread");
        }
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
                getImages((IntPtr)pointer, width, height, 2, showImages, sgbm);   // OCV gets images
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
    private int polarWidth = 1080;
    private int polarHeight = 1080;

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
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);

    [DllImport("unity_plugin", EntryPoint = "takeScreenshot")]
    unsafe private static extern int takeScreenshot(IntPtr raw, int width, int height, int numOfCam, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

    [DllImport("unity_plugin", EntryPoint = "AllocConsole")]
    private static extern bool AllocConsole();
    #endregion
}
