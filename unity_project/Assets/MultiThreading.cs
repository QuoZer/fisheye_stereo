using UnityEngine;
using System.Collections;

namespace MultiThreading
{
    public class ThreadedJob
    {
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
        public virtual void SetAffinity()
        {
            m_Thread.Priority = System.Threading.ThreadPriority.Highest;
            m_Thread.IsBackground = true;
        }
        public virtual void Start()
        {
            PreStart();
            m_Thread = new System.Threading.Thread(Run);
            m_Thread.Start();
        }
        public virtual void Abort()
        {
            m_Thread.Abort();
        }

        protected virtual void PreStart() { }

        protected virtual void ThreadFunction() { }

        protected virtual void OnFinished() { }

        public virtual bool Update()
        {
            if (IsDone)
            {
                OnFinished();
                return true;
            }
            return false;
        }
        public IEnumerator WaitFor()
        {
            while (!Update())
            {
                yield return null;
            }
        }
        private void Run()
        {
            ThreadFunction();
            IsDone = true;
        }
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