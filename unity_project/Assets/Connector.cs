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

public class DisaprityCalculator
{
    public int code;  // out error code 
    Color32[] rawColor1; 
    Color32[] rawColor2;
    int width;
    int height;
    int leftYaw;
    int rightYaw;
    bool showImages;
    bool parametersUpdated = false;
    bool processingFrame = false;
    bool threadIsRunning = true;
    int action;
    SGBMparams sgbm;
    unsafe Color32** imagePtr;
    EventWaitHandle ChildThreadWait = new EventWaitHandle(true, EventResetMode.ManualReset);
    EventWaitHandle MainThreadWait = new EventWaitHandle(true, EventResetMode.ManualReset);

    private System.Threading.Thread m_Thread = null;
    private bool m_IsDone = false;
    public bool IsDone
    {
        get
        {
            bool tmp;
            tmp = m_IsDone;
            return tmp;
        }
        set
        {
            m_IsDone = value;
        }
    }

    public DisaprityCalculator(int w, int h, bool show, int cam1XRot, int cam2XRot)
    {
        width = w;
        height = h;
        showImages = show;
        leftYaw = cam1XRot;
        rightYaw = cam2XRot;
        Debug.Log("Parameters Set");
    }

    unsafe protected void TextureToMat()
    {
        Color32*[] rawColors = new Color32*[2];
        
        fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                code = getImages((IntPtr)pointer, width, height, 2, showImages, sgbm);
            }
            
        }

    }

    unsafe void Screenshoter()
    {
        Color32*[] rawColors = new Color32*[2];
        fixed (Color32* p1 = rawColor1, p2 = rawColor2)
        {
            rawColors[0] = p1;
            rawColors[1] = p2;
            fixed (Color32** pointer = rawColors)
            {
                //takeScreenshot((IntPtr)pointer, width, height, numOfCam, show);
                takeStereoScreenshot((IntPtr)pointer, width, height, 0, 1, true);
            }
        }
    }

    private void Run()
    {
        ThreadFunction();
        IsDone = true;
    }

    public void Start()
    {
        m_Thread = new System.Threading.Thread(Run);
        m_Thread.Start();
    }
    public void Abort()
    {
        threadIsRunning = false;
    }

    public void Update(Color32[] rawImg1, Color32[] rawImg2, SGBMparams stereoparams, int inpAction)          // , IntPtr data
    {   
        // Update is called every frame from the main thred Update(). We don't want to mess with the data while the image is processed.
        if (!processingFrame)
        {
            //MainThreadWait.WaitOne();
            MainThreadWait.Reset();

            // copying into this thread memory
            rawColor1 = rawImg1;
            rawColor2 = rawImg2;
            sgbm = stereoparams;
            action = inpAction;
            //processImage(data, width, height);

            ChildThreadWait.Set();
            //WaitHandle.SignalAndWait(ChildThreadWait, MainThreadWait);
        }
    }

    unsafe protected void ThreadFunction()
    {
        ChildThreadWait.Reset();
        ChildThreadWait.WaitOne();

        //processingFrame = true;
        //initialize(width, height, 2, leftYaw, rightYaw);

        while (threadIsRunning)
        {
            ChildThreadWait.Reset();
            Debug.Log("Current action="+action);
            processingFrame = true;
            switch (action)
            {
                case 0:
                    TextureToMat();
                    break;
                case 1:
                    Screenshoter();
                    break;
                case 2:
                    initialize(width, height, 2, leftYaw, rightYaw);
                    break;
            }
            action = 0;
            processingFrame = false;

            WaitHandle.SignalAndWait(MainThreadWait, ChildThreadWait);
        }
        MainThreadWait.Set();
        //m_Thread.Abort();           // TODO: seems like it aborts the wrong thread (everything just suddenly closes)
    }

    [DllImport("unity_plugin", EntryPoint = "getImages")]
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);
    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);
    [DllImport("unity_plugin", EntryPoint = "processImage")]
    unsafe private static extern void processImage(IntPtr data, int width, int height);
    [DllImport("unity_plugin", EntryPoint = "initialize")]
    unsafe private static extern int initialize(int width, int height, int num, int leftRot, int rightRot);

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
    public static int width = 1080;
    public static int height = 1080;
    public static bool showImages = true;
    public bool showPano = false;

    private Texture2D camTex1;
    private Texture2D camTex2;

    private SGBMparams sgbm;        // structure to store SGBM parameter values

    private Rect readingRect;
    int isOk = 0;
    public int gap = 40;
    int actionId = 0;
    //bool isPanoReady = false;


    bool pano = false;
    // Threading stuff
    DisaprityCalculator imageProcessingThread;
    bool threadStarted = false;
    bool initFlag = false;

    // Start is called before the first frame update
    void Start()
    {
        Debug.Log("Start Function");
        unsafe {
            //AllocConsole();
            sgbm = new SGBMparams();
            imageProcessingThread  = new DisaprityCalculator(width, height, showImages, cam1XRot, cam2XRot);
            initialize(width, height, 2, cam1XRot, cam2XRot);
        }

    }
    // Update is called once per frame
    void Update()
    {
        camTex1 = CameraToTexture2D(camera1);
        camTex2 = CameraToTexture2D(camera2);

        fillStereoParams();        // fills 'sgbm' structure with slider values 
        actionId = 0;               // Default to update action

        /* Update texture in the world */
        if (Input.GetKeyUp("[1]")) {
            /*InitTexture();
            bowl.GetComponent<Renderer>().material.mainTexture = tex;
            MatToTexture2D();*/
            // TODO: World texture renderer
        }
        if (pano) {
            MatToTexture2D();
        }
        /* Take screenshot */
        if (Input.GetKeyUp("[2]")) {
            Debug.Log("Two pressed");
            // pano = !pano;        //idk
            // Screenshoter(camTex1, camTex2, 0, true);
            actionId = 1;
        }
        /* Reinitialize */
        if (Input.GetKeyUp("[3]")) {
            actionId = 2;
            // initialize(width, height, 1, cam1XRot, cam2XRot);
        }
        /* Start the second thread */
        if (!threadStarted)         // image processing thread hasn't been started before
        {
            Debug.Log("Starting thread");
            imageProcessingThread.Start();      // start the thread
            threadStarted = true;
        }
        /* Pass the data to the thread */
        imageProcessingThread.Update(camTex1.GetPixels32(), camTex2.GetPixels32(), sgbm, actionId);
        /* textures are not erased by the GC automatically */
        Destroy(camTex1);
        Destroy(camTex2);
    }


    void OnApplicationQuit()
    {
        terminate();
        pixelHandle.Free();
        imageProcessingThread.Abort();
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
