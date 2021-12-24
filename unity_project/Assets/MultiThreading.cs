using UnityEngine;
using System.Collections;
using System.Threading;
using System;
using System.Runtime.InteropServices;

namespace MultiThreading
{
    public class DisaprityCalculator
    {
        public int code;  // out error code 
        Color32[] rawColor1;
        Color32[] rawColor2;
        IntPtr pixelPointer;
        int width;
        int height;
        int dispScreenWidth;
        int dispScreenHeight;
        int cameraType;

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

        public DisaprityCalculator(int camType, int w, int h, bool show, int cam1XRot, int cam2XRot, IntPtr pixelPtr)
        {
            cameraType = camType;
            width = w;
            height = h;
            dispScreenWidth = w;
            dispScreenHeight = h;
            showImages = show;
            leftYaw = cam1XRot;
            rightYaw = cam2XRot;
            action = 0;
            pixelPointer = pixelPtr;
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
                    takeStereoScreenshot((IntPtr)pointer, width, height, 0, 1, false);
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
            if (!processingFrame || true)       // hmmm, data is not actually messed this way //HACK: ??
            {
                //MainThreadWait.WaitOne();
                MainThreadWait.Reset();

                // copying into this thread memory
                rawColor1 = rawImg1;
                rawColor2 = rawImg2;
                sgbm = stereoparams;
                if (inpAction != 0) action = inpAction;

                ChildThreadWait.Set();
                //WaitHandle.SignalAndWait(ChildThreadWait, MainThreadWait);
            }
        }

        unsafe protected void ThreadFunction()
        {
            ChildThreadWait.Reset();
            ChildThreadWait.WaitOne();

            Debug.Log("Building LUTs with: w=" + width + ", h=" + height + ", lYaw=" + leftYaw + ", rYaw=" + rightYaw);
            code = initialize(width, height, 2, cameraType, leftYaw, rightYaw);
            if (code == 0) Debug.Log("LUTs ready, proceeding...");

            while (threadIsRunning)
            {
                ChildThreadWait.Reset();
                processingFrame = true;
                switch (action)
                {
                    case 0:
                        TextureToMat();
                        break;
                    case 1:
                        Screenshoter();
                        action = 0;
                        break;
                    case 2:
                        initialize(width, height, 2, cameraType, leftYaw, rightYaw);
                        action = 0;
                        break;
                    case 3:
                        processImage(pixelPointer, dispScreenWidth, dispScreenHeight);
                        action = 0;
                        break;

                }
                
                processingFrame = false;

                WaitHandle.SignalAndWait(MainThreadWait, ChildThreadWait);
            }

            terminate();
            //MainThreadWait.Set();
            //m_Thread.Abort();           // TODO: seems like it aborts the wrong thread (everything just suddenly closes)
        }

        [DllImport("unity_plugin", EntryPoint = "terminate")]
        unsafe private static extern void terminate();
        [DllImport("unity_plugin", EntryPoint = "getImages")]
        unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);
        [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
        unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);
        [DllImport("unity_plugin", EntryPoint = "processImage")]
        unsafe private static extern void processImage(IntPtr data, int width, int height);
        [DllImport("unity_plugin", EntryPoint = "initialize")]
        unsafe private static extern int initialize(int width, int height, int num, int camType, int leftRot, int rightRot);


    }
}

/* Old threading impementation. Curious ideas, didn't work unfortunately
 * 
 * 
 public class DisaprityCalculator: MultiThreading.ThreadedJob
{
    public int code;  // out error code 
    Texture2D raw1; 
    Texture2D raw2;
    int width;
    int height;
    bool showImages;
    bool parametersUpdated = false;
    bool processingFrame = false;
    SGBMparams sgbm;
    unsafe Color32** imagePtr;
    EventWaitHandle ChildThreadWait = new EventWaitHandle(true, EventResetMode.ManualReset);
    EventWaitHandle MainThreadWait = new EventWaitHandle(true, EventResetMode.ManualReset);

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
        if (!processingFrame)
            ChildThreadWait.Set();

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
        ChildThreadWait.Reset();
        ChildThreadWait.WaitOne();

        while (true)
        {
            ChildThreadWait.Reset();

            if (parametersUpdated)
            {
                processingFrame = true;
                code = getImages((IntPtr)imagePtr, width, height, 2, showImages, sgbm);
                parametersUpdated = false;
                Debug.Log("image processed");
            }
            else Thread.Sleep(50);

            WaitHandle.SignalAndWait(MainThreadWait, ChildThreadWait);
        }
        //Debug.Log("Leaving thread with code:"+code);
    }

    [DllImport("unity_plugin", EntryPoint = "getImages")]
    unsafe private static extern int getImages(IntPtr raw, int width, int height, int numOfImg, bool isShow, SGBMparams sgbm);
    [DllImport("unity_plugin", EntryPoint = "takeStereoScreenshot")]
    unsafe private static extern int takeStereoScreenshot(IntPtr raw, int width, int height, int numOfCam1, int numOfCam2, bool isShow);

}
///////////// CALLED LIKE THIS \\\\\\\\\\\\\\\\\\\


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

*/