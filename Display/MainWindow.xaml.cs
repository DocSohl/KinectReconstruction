using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using Wpf3DTools;

namespace Display {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        public MainWindow() {
            InitializeComponent();
            this.sensor = KinectSensor.GetDefault();
            this.mapper = this.sensor.CoordinateMapper;
            this.sensor.Open();
            this.reader = this.sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color);
            FrameDescription depthFrameDescription = this.sensor.DepthFrameSource.FrameDescription;
            this.depthWidth = depthFrameDescription.Width;
            this.depthHeight = depthFrameDescription.Height;
            this.depthPixelCount = this.depthWidth * this.depthHeight;
            FrameDescription colorFrameDescription = this.sensor.ColorFrameSource.FrameDescription;
            this.colorWidth = colorFrameDescription.Width;
            this.colorHeight = colorFrameDescription.Height;
            this.colorPixelCount = this.colorWidth * this.colorHeight;
            this.depthVisibilityTestMapWidth = this.colorWidth / ColorDownsampleFactor;
            this.depthVisibilityTestMapHeight = this.colorHeight / ColorDownsampleFactor;
            this.depthVisibilityTestMap = new ushort[this.depthVisibilityTestMapWidth * this.depthVisibilityTestMapHeight];
            this.virtualCameraStartTranslation = new Point3D(0, 0, this.voxelsZ / this.voxelsPerMeter);
            this.virtualCamera = new GraphicsCamera(this.virtualCameraStartTranslation, this.virtualCameraStartRotation, (float)Width / (float)Height);
            this.ReconstructionViewport.Camera = this.virtualCamera.Camera;
            this.StartWorkerThread();
            this.fpsTimer = new DispatcherTimer(DispatcherPriority.Send);
            this.fpsTimer.Interval = new TimeSpan(0, 0, FpsInterval);
            this.fpsTimer.Tick += this.FpsTimerTick;
            this.fpsTimer.Start();
            this.lastFPSTimestamp = DateTime.UtcNow;
            this.statusBarTimer = new DispatcherTimer(DispatcherPriority.Send);
            this.statusBarTimer.Interval = new TimeSpan(0, 0, StatusBarInterval);
            this.statusBarTimer.Start();
            this.lastStatusTimestamp = DateTime.Now;
            this.reader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            this.AllocateKinectFusionResources();
            this.virtualCamera.CreateFrustum3DGraphics(this.ReconstructionViewport, this.depthWidth, this.depthHeight);
            this.recreateReconstruction = true;
            Closing += onClose;
        }

        private const ReconstructionProcessor ProcessorType = ReconstructionProcessor.Amp;
        private const int DeviceToUse = -1;
        private const bool AutoResetReconstructionWhenLost = false;
        private const int MaxTrackingErrors = 100;
        private const int ResetOnTimeStampSkippedMillisecondsGPU = 2000;
        private const int ResetOnTimeStampSkippedMillisecondsCPU = 6000;
        private const int RawDepthWidth = 512;
        private const int RawDepthHeight = 424;
        private const int RawColorWidth = 1920;
        private const int RawColorHeight = 1080;
        private const int ColorDownsampleFactor = 4;
        private const int RawDepthHeightWithSpecialRatio = 384;
        private const int FpsInterval = 5;
        private const int StatusBarInterval = 1;
        private const int RenderIntervalMilliseconds = 100;
        private const int ColorIntegrationInterval = 1;
        private const int DeltaFrameCalculationInterval = 2;
        private const int LineThickness = 2;
        private const float OriginCoordinateCrossAxisSize = 0.1f;
        private const int CameraPoseFinderProcessFrameCalculationInterval = 5;
        private const int MinSuccessfulTrackingFramesForCameraPoseFinder = 45;
        private const int MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure = 200;
        private const float MaxAlignToReconstructionEnergyForSuccess = 0.27f;
        private const float MinAlignToReconstructionEnergyForSuccess = 0.005f;
        private const float MaxAlignPointCloudsEnergyForSuccess = 0.006f;
        private const float MinAlignPointCloudsEnergyForSuccess = 0.0f;
        private const int MaxCameraPoseFinderPoseTests = 5;
        private const float CameraPoseFinderDistanceThresholdReject = 1.0f;
        private const float CameraPoseFinderDistanceThresholdAccept = 0.1f;
        private const int SmoothingKernelWidth = 1;
        private const float SmoothingDistanceThreshold = 0.04f;
        private const float MaxTranslationDeltaAlignPointClouds = 0.3f;
        private const float MaxRotationDeltaAlignPointClouds = 20.0f;
        private const int DownsampleFactor = 2;
        private const ushort DepthVisibilityTestThreshold = 50;

        private static System.Windows.Media.Color volumeCubeLineColor = System.Windows.Media.Color.FromArgb(200, 0, 200, 0);
        private bool autoResetReconstructionOnTimeSkip = false;
        private bool disposed;
        private bool savingMesh;
        private bool displayNormals;
        private bool captureColor;
        private bool pauseIntegration;
        private bool mirrorDepth;
        private bool kinectView = true;
        private bool volumeGraphics;
        private int depthWidth = 0;
        private int depthHeight = 0;
        private int depthPixelCount = 0;
        private int colorWidth = 0;
        private int colorHeight = 0;
        private int colorPixelCount = 0;
        private int downsampledWidth;
        private int downsampledHeight;
        private int depthVisibilityTestMapWidth = 0;
        private int depthVisibilityTestMapHeight = 0;
        private int trackingErrorCount = 0;
        private bool trackingFailed;
        private bool trackingHasFailedPreviously;
        private bool cameraPoseFinderAvailable;
        private int successfulFrameCount;
        private int processedFrameCount = 0;
        private TimeSpan lastFrameTimestamp;
        private DispatcherTimer fpsTimer;
        private DateTime lastFPSTimestamp = DateTime.UtcNow;
        private DateTime lastRenderTimestamp = DateTime.UtcNow;
        private DispatcherTimer statusBarTimer;
        private DateTime lastStatusTimestamp;
        private Queue<string> statusMessageQueue = new Queue<string>();
        private KinectSensor sensor = null;
        private MultiSourceFrameReader reader;
        private ushort[] depthImagePixels;
        private ushort[] depthVisibilityTestMap;
        private byte[] colorImagePixels;
        private int[] resampledColorImagePixels;
        private int[] downsampledDeltaFromReferenceColorPixels;
        private ColorReconstruction volume;
        private FusionFloatImageFrame depthFloatFrame;
        private FusionFloatImageFrame smoothDepthFloatFrame;
        private FusionColorImageFrame resampledColorFrame;
        private FusionColorImageFrame resampledColorFrameDepthAligned;
        private FusionFloatImageFrame deltaFromReferenceFrame;
        private FusionColorImageFrame shadedSurfaceFrame;
        private FusionColorImageFrame shadedSurfaceNormalsFrame;
        private FusionPointCloudImageFrame raycastPointCloudFrame;
        private FusionPointCloudImageFrame depthPointCloudFrame;
        private FusionFloatImageFrame downsampledDepthFloatFrame;
        private FusionFloatImageFrame downsampledSmoothDepthFloatFrame;
        private FusionPointCloudImageFrame downsampledRaycastPointCloudFrame;
        private FusionPointCloudImageFrame downsampledDepthPointCloudFrame;
        private FusionColorImageFrame downsampledDeltaFromReferenceFrameColorFrame;
        private WriteableBitmap depthFloatFrameBitmap;
        private WriteableBitmap deltaFromReferenceFrameBitmap;
        private WriteableBitmap shadedSurfaceFrameBitmap;
        private float[] depthFloatFrameDepthPixels;
        private float[] deltaFromReferenceFrameFloatPixels;
        private int[] depthFloatFramePixelsArgb;
        private int[] deltaFromReferenceFramePixelsArgb;
        private int[] shadedSurfaceFramePixelsArgb;
        private ColorSpacePoint[] colorCoordinates;
        private int[] resampledColorImagePixelsAlignedToDepth;
        private float[] downsampledDepthImagePixels;
        private CoordinateMapper mapper;
        private float alignmentEnergy;
        private Thread workerThread = null;
        private ManualResetEvent workerThreadStopEvent;
        private ManualResetEvent depthReadyEvent;
        private ManualResetEvent colorReadyEvent;
        private object rawDataLock = new object();
        private object volumeLock = new object();
        private ScreenSpaceLines3D volumeCube;
        private ScreenSpaceLines3D volumeCubeAxisX;
        private ScreenSpaceLines3D volumeCubeAxisY;
        private ScreenSpaceLines3D volumeCubeAxisZ;
        private ScreenSpaceLines3D axisX;
        private ScreenSpaceLines3D axisY;
        private ScreenSpaceLines3D axisZ;
        private bool haveAddedVolumeCube = false;
        private bool haveAddedCoordinateCross = false;
        private bool viewChanged = true;
        private GraphicsCamera virtualCamera;
        private Quaternion virtualCameraStartRotation = Quaternion.Identity;
        private Point3D virtualCameraStartTranslation = new Point3D();
        private bool resetReconstruction = false;
        private bool recreateReconstruction = false;
        private Matrix4 worldToCameraTransform;
        private Matrix4 defaultWorldToVolumeTransform;
        private float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;
        private float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;
        private short integrationWeight = FusionDepthProcessor.DefaultIntegrationWeight;
        private float voxelsPerMeter = 256.0f;
        private int voxelsX = 384;
        private int voxelsY = 384;
        private int voxelsZ = 384;
        private bool translateResetPoseByMinDepthThreshold = true;
        private Matrix4 worldToBGRTransform = new Matrix4();
        private Matrix4 virtualCameraWorldToCameraMatrix4 = new Matrix4();
        private bool colorCaptured;
        private CameraPoseFinder cameraPoseFinder;
        private bool autoFindCameraPoseWhenLost = true;


        private void onClose(object sender, System.ComponentModel.CancelEventArgs e) {
            this.Dispose();
            // Stop timer
            if (null != this.fpsTimer) {
                this.fpsTimer.Stop();
                this.fpsTimer.Tick -= this.FpsTimerTick;
            }

            if (null != this.statusBarTimer) {
                this.statusBarTimer.Stop();
            }

            if (this.reader != null) {
                this.reader.Dispose();
                this.reader = null;
            }

            if (null != this.sensor) {
                this.sensor.Close();
                this.sensor = null;
            }
            this.virtualCamera.DisposeFrustum3DGraphics();
            this.StopWorkerThread();
        }


        #region Properties
        public event PropertyChangedEventHandler PropertyChanged;
        public TimeSpan RelativeTime { get; set; }
        public bool PauseIntegration {
            get {
                return this.pauseIntegration;
            }

            set {
                this.pauseIntegration = value;
                if (null != this.PropertyChanged) {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("PauseIntegration"));
                }
            }
        }
        public bool MirrorDepth {
            get {
                return this.mirrorDepth;
            }

            set {
                this.mirrorDepth = value;
                if (null != this.PropertyChanged) {
                    this.PropertyChanged.Invoke(this, new PropertyChangedEventArgs("MirrorDepth"));
                }

                this.resetReconstruction = true;
            }
        }
        public bool KinectView {
            get {
                return this.kinectView;
            }

            set {}
        }
        public bool IsRenderOverdue {
            get {
                return (DateTime.UtcNow - this.lastRenderTimestamp).TotalMilliseconds >= RenderIntervalMilliseconds;
            }
        }

        #endregion

        public void Dispose() {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing) {
            if (!this.disposed) {
                if (disposing) {
                    if (null != this.depthReadyEvent) {
                        this.depthReadyEvent.Dispose();
                    }

                    if (null != this.colorReadyEvent) {
                        this.colorReadyEvent.Dispose();
                    }

                    if (null != this.workerThreadStopEvent) {
                        this.workerThreadStopEvent.Dispose();
                    }

                    this.RemoveVolumeCube3DGraphics();
                    this.DisposeVolumeCube3DGraphics();


                    if (null != this.virtualCamera) {
                        this.virtualCamera.CameraTransformationChanged -= this.OnVirtualCameraTransformationChanged;
                        this.virtualCamera.Dispose();
                    }

                    this.SafeDisposeFusionResources();

                    if (null != this.volume) {
                        this.volume.Dispose();
                    }
                }
            }

            this.disposed = true;
        }

        private static void RenderColorImage(FusionColorImageFrame colorFrame, ref int[] colorPixels, ref WriteableBitmap bitmap, System.Windows.Controls.Image image) {
            if (null == image || null == colorFrame) {
                return;
            }

            if (null == colorPixels || colorFrame.PixelDataLength != colorPixels.Length) {
                colorPixels = new int[colorFrame.PixelDataLength];
            }

            if (null == bitmap || colorFrame.Width != bitmap.Width || colorFrame.Height != bitmap.Height) {
                bitmap = new WriteableBitmap(colorFrame.Width, colorFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                image.Source = bitmap;
            }
            colorFrame.CopyPixelDataTo(colorPixels);
            bitmap.WritePixels(
                        new Int32Rect(0, 0, colorFrame.Width, colorFrame.Height),
                        colorPixels,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }


        private void FpsTimerTick(object sender, EventArgs e) {
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.UtcNow;
        }

        private void ResetFps() {
            if (null != this.fpsTimer) {
                this.fpsTimer.Stop();
                this.fpsTimer.Start();
            }
            this.processedFrameCount = 0;
            this.lastFPSTimestamp = DateTime.UtcNow;
        }


        private void StartWorkerThread() {
            if (null == this.workerThread) {
                this.depthReadyEvent = new ManualResetEvent(false);
                this.colorReadyEvent = new ManualResetEvent(false);
                this.workerThreadStopEvent = new ManualResetEvent(false);
                this.workerThread = new Thread(this.WorkerThreadProc);
                this.workerThread.Start();
            }
        }


        private void StopWorkerThread() {
            if (null != this.workerThread) {
                this.workerThreadStopEvent.Set();
                this.workerThread.Join();
            }
        }

        private void WorkerThreadProc() {
            WaitHandle[] events = new WaitHandle[2] { this.workerThreadStopEvent, this.depthReadyEvent };
            while (true) {
                int index = WaitHandle.WaitAny(events);
                if (0 == index) {
                    break;
                }
                this.depthReadyEvent.Reset();
                this.Process();
            }
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e) {
            bool validDepth = false;
            bool validColor = false;
            MultiSourceFrameReference frameReference = e.FrameReference;
            MultiSourceFrame multiSourceFrame = null;
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;

            try {
                multiSourceFrame = frameReference.AcquireFrame();
                if (multiSourceFrame != null) {
                    lock (this.rawDataLock) {
                        ColorFrameReference colorFrameReference = multiSourceFrame.ColorFrameReference;
                        DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;

                        colorFrame = colorFrameReference.AcquireFrame();
                        depthFrame = depthFrameReference.AcquireFrame();

                        if ((depthFrame != null) && (colorFrame != null)) {
                            this.RelativeTime = depthFrame.RelativeTime;

                            FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                            int colorWidth = colorFrameDescription.Width;
                            int colorHeight = colorFrameDescription.Height;

                            if ((colorWidth * colorHeight * sizeof(int)) == this.colorImagePixels.Length) {
                                colorFrame.CopyConvertedFrameDataToArray(this.colorImagePixels, ColorImageFormat.Bgra);

                                validColor = true;
                            }

                            FrameDescription depthFrameDescription = depthFrame.FrameDescription;
                            int depthWidth = depthFrameDescription.Width;
                            int depthHeight = depthFrameDescription.Height;

                            if ((depthWidth * depthHeight) == this.depthImagePixels.Length) {
                                depthFrame.CopyFrameDataToArray(this.depthImagePixels);

                                validDepth = true;
                            }
                        }
                    }
                }
            }
            catch (Exception) {}
            finally {
                if (depthFrame != null) {
                    depthFrame.Dispose();
                    depthFrame = null;
                }

                if (colorFrame != null) {
                    colorFrame.Dispose();
                    colorFrame = null;
                }

                if (multiSourceFrame != null) {
                    multiSourceFrame = null;
                }
            }

            if (validDepth) {
                this.depthReadyEvent.Set();
            }

            if (validColor) {
                this.colorReadyEvent.Set();
            }
        }

        private void Process() {
            if (this.recreateReconstruction) {
                lock (this.volumeLock) {
                    this.recreateReconstruction = false;
                    this.RecreateReconstruction();
                }
            }

            if (this.resetReconstruction) {
                this.resetReconstruction = false;
                this.ResetReconstruction();
            }

            if (null != this.volume && !this.savingMesh) {
                try {
                    this.cameraPoseFinderAvailable = this.IsCameraPoseFinderAvailable();
                    this.ProcessDepthData();
                    this.TrackCamera();
                    if (0 == this.trackingErrorCount) {
                        bool colorAvailable = this.IntegrateData();
                        if (!this.depthReadyEvent.WaitOne(0) || this.IsRenderOverdue) {
                            this.RenderReconstruction();
                        }
                        if (this.autoFindCameraPoseWhenLost && !this.trackingHasFailedPreviously
                            && this.successfulFrameCount > MinSuccessfulTrackingFramesForCameraPoseFinder
                            && this.processedFrameCount % CameraPoseFinderProcessFrameCalculationInterval == 0
                            && colorAvailable) {
                            this.UpdateCameraPoseFinder();
                        }
                    }
                }
                catch (InvalidOperationException ex) {
                    this.ShowStatusMessage(ex.Message);
                }
            }
        }

        private bool IsCameraPoseFinderAvailable() {
            return this.autoFindCameraPoseWhenLost
                && null != this.cameraPoseFinder
                && this.cameraPoseFinder.GetStoredPoseCount() > 0;
        }

        private void ProcessDepthData() {
            if (this.autoResetReconstructionOnTimeSkip) {
                this.CheckResetTimeStamp(this.RelativeTime);
            }

            lock (this.rawDataLock) {
                this.volume.DepthToDepthFloatFrame(
                    this.depthImagePixels,
                    this.depthFloatFrame,
                    this.minDepthClip,
                    this.maxDepthClip,
                    this.MirrorDepth);
            }
            this.Dispatcher.BeginInvoke(
                (Action)
                (() =>
                    this.RenderDepthFloatImage(
                        ref this.depthFloatFrameBitmap,
                        this.depthImage)));
        }

        private bool TrackCameraAlignDepthFloatToReconstruction(bool calculateDeltaFrame, ref Matrix4 calculatedCameraPos) {
            bool trackingSucceeded = false;
            if (calculateDeltaFrame) {
                trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    this.depthFloatFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.deltaFromReferenceFrame,
                    out this.alignmentEnergy,
                    this.worldToCameraTransform);
            }
            else {
                trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    this.depthFloatFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    null,
                    out this.alignmentEnergy,
                    this.worldToCameraTransform);
            }

            if (!trackingSucceeded || this.alignmentEnergy > MaxAlignToReconstructionEnergyForSuccess || (this.alignmentEnergy <= MinAlignToReconstructionEnergyForSuccess && this.successfulFrameCount > 0)) {
                trackingSucceeded = false;
            }
            else {
                calculatedCameraPos = this.volume.GetCurrentWorldToCameraTransform();
            }

            return trackingSucceeded;
        }

        private bool TrackCameraAlignPointClouds(ref bool calculateDeltaFrame, ref Matrix4 calculatedCameraPose) {
            bool trackingSucceeded = false;
            this.DownsampleDepthFrameNearestNeighbor(this.downsampledDepthFloatFrame, DownsampleFactor);
            this.volume.SmoothDepthFloatFrame(this.downsampledDepthFloatFrame, this.downsampledSmoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.downsampledSmoothDepthFloatFrame, this.downsampledDepthPointCloudFrame);
            this.volume.CalculatePointCloud(this.downsampledRaycastPointCloudFrame, calculatedCameraPose);
            Matrix4 initialPose = calculatedCameraPose;
            if (calculateDeltaFrame) {
                trackingSucceeded = FusionDepthProcessor.AlignPointClouds(
                    this.downsampledRaycastPointCloudFrame,
                    this.downsampledDepthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.downsampledDeltaFromReferenceFrameColorFrame,
                    ref calculatedCameraPose);
                this.UpsampleColorDeltasFrameNearestNeighbor(DownsampleFactor);
                calculateDeltaFrame = false;
            }
            else {
                trackingSucceeded = FusionDepthProcessor.AlignPointClouds(
                    this.downsampledRaycastPointCloudFrame,
                    this.downsampledDepthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    null,
                    ref calculatedCameraPose);
            }

            if (trackingSucceeded) {
                bool failed = KinectFusionHelper.CameraTransformFailed(
                    initialPose,
                    calculatedCameraPose,
                    MaxTranslationDeltaAlignPointClouds,
                    MaxRotationDeltaAlignPointClouds);

                if (failed) {
                    trackingSucceeded = false;
                }
            }

            return trackingSucceeded;
        }

        private void TrackCamera() {
            bool calculateDeltaFrame = this.processedFrameCount % DeltaFrameCalculationInterval == 0;
            bool trackingSucceeded = false;
            Matrix4 calculatedCameraPos = this.worldToCameraTransform;
            if (this.autoFindCameraPoseWhenLost) {
                trackingSucceeded = this.TrackCameraAlignPointClouds(ref calculateDeltaFrame, ref calculatedCameraPos);
            }
            else {
                trackingSucceeded = this.TrackCameraAlignDepthFloatToReconstruction(calculateDeltaFrame, ref calculatedCameraPos);
            }

            if (!trackingSucceeded && 0 != this.successfulFrameCount) {
                this.SetTrackingFailed();
                if (!this.cameraPoseFinderAvailable) {
                    Console.WriteLine("Camera Tracking Failed");
                }
                else {
                    trackingSucceeded = this.FindCameraPoseAlignPointClouds();
                    if (!trackingSucceeded) {
                        Console.WriteLine("Camera Tracking Failed 2");
                    }
                }
            }
            else {
                if (this.trackingHasFailedPreviously) {
                    Console.WriteLine("Tracking Recovered");
                }
                this.SetTrackingSucceeded();
                this.worldToCameraTransform = calculatedCameraPos;
            }

            if (AutoResetReconstructionWhenLost && !trackingSucceeded
                && this.trackingErrorCount >= MaxTrackingErrors) {
                this.ShowStatusMessage("Reset Volume Automatically");
                this.ResetReconstruction();
            }

            if (trackingSucceeded) {
                if (this.kinectView) {
                    Dispatcher.BeginInvoke((Action)(() => this.UpdateVirtualCameraTransform()));
                }
                else {
                    Dispatcher.BeginInvoke((Action)(() => this.virtualCamera.UpdateFrustumTransformMatrix4(this.worldToCameraTransform)));
                }
                this.processedFrameCount++;
            }
        }

        private void SetTrackingFailed() {
            this.trackingFailed = true;
            this.trackingHasFailedPreviously = true;
            this.trackingErrorCount++;
            this.successfulFrameCount = 0;
        }

        private void SetTrackingSucceeded() {
            this.trackingFailed = false;
            this.trackingErrorCount = 0;
            this.successfulFrameCount++;
        }

        private void ResetTracking() {
            this.trackingFailed = false;
            this.trackingHasFailedPreviously = false;
            this.trackingErrorCount = 0;
            this.successfulFrameCount = 0;
            this.PauseIntegration = false;

            if (null != this.cameraPoseFinder) {
                this.cameraPoseFinder.ResetCameraPoseFinder();
            }
        }

        private unsafe void ProcessColorForCameraPoseFinder() {
            if (null == this.resampledColorFrame || null == this.downsampledDepthImagePixels) {
                throw new ArgumentException("inputs null");
            }
            if (this.depthWidth != RawDepthWidth || this.depthHeight != RawDepthHeight
                || this.colorWidth != RawColorWidth || this.colorHeight != RawColorHeight) {
                return;
            }
            float factor = RawColorWidth / RawDepthHeightWithSpecialRatio;
            const int FilledZeroMargin = (RawDepthHeight - RawDepthHeightWithSpecialRatio) / 2;
            fixed (byte* ptrColorPixels = this.colorImagePixels) {
                int* rawColorPixels = (int*)ptrColorPixels;
                Parallel.For(
                    FilledZeroMargin,
                    this.depthHeight - FilledZeroMargin,
                    y => {
                        int destIndex = y * this.depthWidth;

                        for (int x = 0; x < this.depthWidth; ++x, ++destIndex) {
                            int srcX = (int)(x * factor);
                            int srcY = (int)(y * factor);
                            int sourceColorIndex = (srcY * this.colorWidth) + srcX;

                            this.resampledColorImagePixels[destIndex] = rawColorPixels[sourceColorIndex];
                        }
                    });
            }
            this.resampledColorFrame.CopyPixelDataFrom(this.resampledColorImagePixels);
        }

        private void SetReferenceFrame(Matrix4 pose) {
            this.volume.CalculatePointCloudAndDepth(this.raycastPointCloudFrame, this.smoothDepthFloatFrame, null, pose);
            this.volume.SetAlignDepthFloatToReconstructionReferenceFrame(this.smoothDepthFloatFrame);
        }

        private bool FindCameraPoseAlignPointClouds() {
            if (!this.cameraPoseFinderAvailable) {
                return false;
            }
            this.ProcessColorForCameraPoseFinder();
            MatchCandidates matchCandidates = this.cameraPoseFinder.FindCameraPose(
                this.depthFloatFrame,
                this.resampledColorFrame);
            if (null == matchCandidates) {
                return false;
            }
            int poseCount = matchCandidates.GetPoseCount();
            float minDistance = matchCandidates.CalculateMinimumDistance();
            if (0 == poseCount || minDistance >= CameraPoseFinderDistanceThresholdReject) {
                this.ShowStatusMessage("Pose finder could not find enough matches");
                return false;
            }
            this.volume.SmoothDepthFloatFrame(this.depthFloatFrame, this.smoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.smoothDepthFloatFrame, this.depthPointCloudFrame);
            double smallestEnergy = double.MaxValue;
            int smallestEnergyNeighborIndex = -1;
            int bestNeighborIndex = -1;
            Matrix4 bestNeighborCameraPose = Matrix4.Identity;
            double bestNeighborAlignmentEnergy = MaxAlignPointCloudsEnergyForSuccess;
            int maxTests = Math.Min(MaxCameraPoseFinderPoseTests, poseCount);
            var neighbors = matchCandidates.GetMatchPoses();
            for (int n = 0; n < maxTests; n++) {
                Matrix4 poseProposal = neighbors[n];
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, poseProposal);
                bool success = this.volume.AlignPointClouds(
                    this.raycastPointCloudFrame,
                    this.depthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.resampledColorFrame,
                    out this.alignmentEnergy,
                    ref poseProposal);
                bool relocSuccess = success && this.alignmentEnergy < bestNeighborAlignmentEnergy && this.alignmentEnergy > MinAlignPointCloudsEnergyForSuccess;
                if (relocSuccess) {
                    bestNeighborAlignmentEnergy = this.alignmentEnergy;
                    bestNeighborIndex = n;
                    bestNeighborCameraPose = poseProposal;
                    this.resampledColorFrame.CopyPixelDataTo(this.deltaFromReferenceFramePixelsArgb);
                }
                if (this.alignmentEnergy < smallestEnergy) {
                    smallestEnergy = this.alignmentEnergy;
                    smallestEnergyNeighborIndex = n;
                }
            }
            matchCandidates.Dispose();
            if (bestNeighborIndex > -1) {
                this.worldToCameraTransform = bestNeighborCameraPose;
                this.SetReferenceFrame(this.worldToCameraTransform);
                this.SetTrackingSucceeded();
                Console.WriteLine("Pose Finding sucesses");
                return true;
            }
            else {
                this.worldToCameraTransform = neighbors[smallestEnergyNeighborIndex];
                this.SetReferenceFrame(this.worldToCameraTransform);
                this.SetTrackingFailed();
                Console.WriteLine("Pose Finding failure");
                return false;
            }
        }

        
        private unsafe void MapColorToDepth() {
            this.mapper.MapDepthFrameToColorSpace(this.depthImagePixels, this.colorCoordinates);
            lock (this.rawDataLock) {
                Array.Clear(this.depthVisibilityTestMap, 0, this.depthVisibilityTestMap.Length);
                fixed (ushort* ptrDepthVisibilityPixels = this.depthVisibilityTestMap, ptrDepthPixels = this.depthImagePixels) {
                    for (int index = 0; index < this.depthImagePixels.Length; ++index) {
                        if (!float.IsInfinity(this.colorCoordinates[index].X) && !float.IsInfinity(this.colorCoordinates[index].Y)) {
                            int x = (int)(Math.Floor(this.colorCoordinates[index].X + 0.5f) / ColorDownsampleFactor);
                            int y = (int)(Math.Floor(this.colorCoordinates[index].Y + 0.5f) / ColorDownsampleFactor);
                            if ((x >= 0) && (x < this.depthVisibilityTestMapWidth) &&
                                (y >= 0) && (y < this.depthVisibilityTestMapHeight)) {
                                int depthVisibilityTestIndex = (y * this.depthVisibilityTestMapWidth) + x;
                                if ((ptrDepthVisibilityPixels[depthVisibilityTestIndex] == 0) ||
                                    (ptrDepthVisibilityPixels[depthVisibilityTestIndex] > ptrDepthPixels[index])) {
                                    ptrDepthVisibilityPixels[depthVisibilityTestIndex] = ptrDepthPixels[index];
                                }
                            }
                        }
                    }
                }

                if (this.mirrorDepth) {
                    fixed (byte* ptrColorPixels = this.colorImagePixels) {
                        int* rawColorPixels = (int*)ptrColorPixels;
                        Parallel.For(
                            0,
                            this.depthHeight,
                            y => {
                                int destIndex = y * this.depthWidth;

                                for (int x = 0; x < this.depthWidth; ++x, ++destIndex) {
                                    int colorInDepthX = (int)Math.Floor(colorCoordinates[destIndex].X + 0.5);
                                    int colorInDepthY = (int)Math.Floor(colorCoordinates[destIndex].Y + 0.5);
                                    int depthVisibilityTestX = (int)(colorInDepthX / ColorDownsampleFactor);
                                    int depthVisibilityTestY = (int)(colorInDepthY / ColorDownsampleFactor);
                                    int depthVisibilityTestIndex = (depthVisibilityTestY * this.depthVisibilityTestMapWidth) + depthVisibilityTestX;
                                    if (colorInDepthX >= 0 && colorInDepthX < this.colorWidth && colorInDepthY >= 0
                                        && colorInDepthY < this.colorHeight && this.depthImagePixels[destIndex] != 0) {
                                        ushort depthTestValue = this.depthVisibilityTestMap[depthVisibilityTestIndex];
                                        if ((this.depthImagePixels[destIndex] - depthTestValue) < DepthVisibilityTestThreshold) {
                                            int sourceColorIndex = colorInDepthX + (colorInDepthY * this.colorWidth);
                                            this.resampledColorImagePixelsAlignedToDepth[destIndex] = rawColorPixels[sourceColorIndex];
                                        }
                                        else {
                                            this.resampledColorImagePixelsAlignedToDepth[destIndex] = 0;
                                        }
                                    }
                                    else {
                                        this.resampledColorImagePixelsAlignedToDepth[destIndex] = 0;
                                    }
                                }
                            });
                    }
                }
                else {
                    fixed (byte* ptrColorPixels = this.colorImagePixels) {
                        int* rawColorPixels = (int*)ptrColorPixels;
                        Parallel.For(
                            0,
                            this.depthHeight,
                            y => {
                                int destIndex = y * this.depthWidth;
                                int flippedDestIndex = destIndex + (this.depthWidth - 1);
                                for (int x = 0; x < this.depthWidth; ++x, ++destIndex, --flippedDestIndex) {
                                    int colorInDepthX = (int)Math.Floor(colorCoordinates[destIndex].X + 0.5);
                                    int colorInDepthY = (int)Math.Floor(colorCoordinates[destIndex].Y + 0.5);
                                    int depthVisibilityTestX = (int)(colorInDepthX / ColorDownsampleFactor);
                                    int depthVisibilityTestY = (int)(colorInDepthY / ColorDownsampleFactor);
                                    int depthVisibilityTestIndex = (depthVisibilityTestY * this.depthVisibilityTestMapWidth) + depthVisibilityTestX;
                                    if (colorInDepthX >= 0 && colorInDepthX < this.colorWidth && colorInDepthY >= 0
                                        && colorInDepthY < this.colorHeight && this.depthImagePixels[destIndex] != 0) {
                                        ushort depthTestValue = this.depthVisibilityTestMap[depthVisibilityTestIndex];
                                        if ((this.depthImagePixels[destIndex] - depthTestValue) < DepthVisibilityTestThreshold) {
                                            int sourceColorIndex = colorInDepthX + (colorInDepthY * this.colorWidth);
                                            this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = rawColorPixels[sourceColorIndex];
                                        }
                                        else {
                                            this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = 0;
                                        }
                                    }
                                    else {
                                        this.resampledColorImagePixelsAlignedToDepth[flippedDestIndex] = 0;
                                    }
                                }
                            });
                    }
                }
            }

            this.resampledColorFrameDepthAligned.CopyPixelDataFrom(this.resampledColorImagePixelsAlignedToDepth);
        }

        private bool IntegrateData() {
            bool colorAvailable = this.colorReadyEvent.WaitOne(0);
            bool integrateData = !this.trackingFailed && !this.PauseIntegration &&
                (!this.cameraPoseFinderAvailable || (this.cameraPoseFinderAvailable && !(this.trackingHasFailedPreviously && this.successfulFrameCount < MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure)));

            if (integrateData) {
                bool integrateColor = this.processedFrameCount % ColorIntegrationInterval == 0 && colorAvailable;
                this.trackingHasFailedPreviously = false;
                if (this.captureColor && integrateColor) {
                    this.MapColorToDepth();
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.resampledColorFrameDepthAligned,
                        this.integrationWeight,
                        FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
                        this.worldToCameraTransform);
                    this.colorCaptured = true;
                }
                else {
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.integrationWeight,
                        this.worldToCameraTransform);
                }
                this.colorReadyEvent.Reset();
            }

            return colorAvailable;
        }

        private void RenderReconstruction() {
            if (null == this.volume || this.savingMesh || null == this.raycastPointCloudFrame
                || null == this.shadedSurfaceFrame || null == this.shadedSurfaceNormalsFrame) {
                return;
            }
            Matrix4 cameraView = this.KinectView ? this.worldToCameraTransform : this.virtualCameraWorldToCameraMatrix4;
            if (this.captureColor) {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, this.shadedSurfaceFrame, cameraView);
            }
            else {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, cameraView);
                FusionDepthProcessor.ShadePointCloud(
                    this.raycastPointCloudFrame,
                    cameraView,
                    this.worldToBGRTransform,
                    this.displayNormals ? null : this.shadedSurfaceFrame,
                    this.displayNormals ? this.shadedSurfaceNormalsFrame : null);
            }
            Dispatcher.BeginInvoke((Action)(() => this.ReconstructFrameComplete()));
            this.lastRenderTimestamp = DateTime.UtcNow;
        }

        private void UpdateCameraPoseFinder() {
            if (null == this.depthFloatFrame || null == this.resampledColorFrame || null == this.cameraPoseFinder) {
                return;
            }
            this.ProcessColorForCameraPoseFinder();
            bool poseHistoryTrimmed = false;
            bool addedPose = false;
            this.cameraPoseFinder.ProcessFrame(
                this.depthFloatFrame,
                this.resampledColorFrame,
                this.worldToCameraTransform,
                CameraPoseFinderDistanceThresholdAccept,
                out addedPose,
                out poseHistoryTrimmed);
            if (true == addedPose) {
                Console.WriteLine("Pose finder added a frame");
            }
            if (true == poseHistoryTrimmed) {
                this.ShowStatusMessage("Pose finder history is full");
            }
        }


        private void RenderAlignDeltasFloatImage(FusionFloatImageFrame alignDeltasFloatFrame, ref WriteableBitmap bitmap, System.Windows.Controls.Image image) {
            if (null == alignDeltasFloatFrame) {
                return;
            }

            if (null == bitmap || alignDeltasFloatFrame.Width != bitmap.Width || alignDeltasFloatFrame.Height != bitmap.Height) {
                bitmap = new WriteableBitmap(alignDeltasFloatFrame.Width, alignDeltasFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                image.Source = bitmap;
            }
            alignDeltasFloatFrame.CopyPixelDataTo(this.deltaFromReferenceFrameFloatPixels);
            Parallel.For(
            0,
            alignDeltasFloatFrame.Height,
            y => {
                int index = y * alignDeltasFloatFrame.Width;
                for (int x = 0; x < alignDeltasFloatFrame.Width; ++x, ++index) {
                    float residue = this.deltaFromReferenceFrameFloatPixels[index];

                    if (residue < 1.0f) {
                        this.deltaFromReferenceFramePixelsArgb[index] = (byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f - residue, 0.0f, 1.0f)); // blue
                        this.deltaFromReferenceFramePixelsArgb[index] |= ((byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f - Math.Abs(residue), 0.0f, 1.0f))) << 8; // green
                        this.deltaFromReferenceFramePixelsArgb[index] |= ((byte)(255.0f * KinectFusionHelper.ClampFloatingPoint(1.0f + residue, 0.0f, 1.0f))) << 16; // red
                    }
                    else {
                        this.deltaFromReferenceFramePixelsArgb[index] = 0;
                    }
                }
            });
            bitmap.WritePixels(
                        new Int32Rect(0, 0, alignDeltasFloatFrame.Width, alignDeltasFloatFrame.Height),
                        this.deltaFromReferenceFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }


        private void RenderDepthFloatImage(ref WriteableBitmap bitmap, System.Windows.Controls.Image image) {
            if (null == this.depthFloatFrame) {
                return;
            }

            if (null == bitmap || this.depthFloatFrame.Width != bitmap.Width || this.depthFloatFrame.Height != bitmap.Height) {
                bitmap = new WriteableBitmap(this.depthFloatFrame.Width, this.depthFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                image.Source = bitmap;
            }
            this.depthFloatFrame.CopyPixelDataTo(this.depthFloatFrameDepthPixels);
            float range = 4.0f;
            float oneOverRange = (1.0f / range) * 256.0f;
            float minRange = 0.0f;
            Parallel.For(
            0,
            this.depthFloatFrame.Height,
            y => {
                int index = y * this.depthFloatFrame.Width;
                for (int x = 0; x < this.depthFloatFrame.Width; ++x, ++index) {
                    float depth = this.depthFloatFrameDepthPixels[index];
                    int intensity = (depth >= minRange) ? ((byte)((depth - minRange) * oneOverRange)) : 0;

                    this.depthFloatFramePixelsArgb[index] = (255 << 24) | (intensity << 16) | (intensity << 8) | intensity; // set blue, green, red
                }
            });
            bitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthFloatFrame.Width, this.depthFloatFrame.Height),
                        this.depthFloatFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        private void OnVirtualCameraTransformationChanged(object sender, EventArgs e) {
            this.virtualCameraWorldToCameraMatrix4 = this.virtualCamera.WorldToCameraMatrix4;
            this.viewChanged = true;
        }

        private void CheckResetTimeStamp(TimeSpan frameTimestamp) {
            if (!this.lastFrameTimestamp.Equals(TimeSpan.Zero)) {
                long timeThreshold = (ReconstructionProcessor.Amp == ProcessorType) ? ResetOnTimeStampSkippedMillisecondsGPU : ResetOnTimeStampSkippedMillisecondsCPU;
                long skippedMilliseconds = (long)frameTimestamp.Subtract(this.lastFrameTimestamp).Duration().TotalMilliseconds;
                if (skippedMilliseconds >= timeThreshold) {
                    this.ShowStatusMessage("Reset Volume");
                    this.resetReconstruction = true;
                }
            }
            this.lastFrameTimestamp = frameTimestamp;
        }

        private void ResetReconstruction() {
            if (null == this.sensor) {
                return;
            }
            this.trackingErrorCount = 0;
            this.worldToCameraTransform = Matrix4.Identity;
            if (null != this.volume) {
                try {
                    if (this.translateResetPoseByMinDepthThreshold) {
                        Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;
                        float minDist = (this.minDepthClip < this.maxDepthClip) ? this.minDepthClip : this.maxDepthClip;
                        worldToVolumeTransform.M43 -= minDist * this.voxelsPerMeter;
                        this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                    }
                    else {
                        this.volume.ResetReconstruction(this.worldToCameraTransform);
                    }

                    this.ResetTracking();
                    this.ResetColorImage();
                }
                catch (InvalidOperationException) {
                    this.ShowStatusMessage("Reset Failed");
                }
            }
            this.ShowStatusMessage("Volume Reset");
        }

        private bool RecreateReconstruction() {
            if (null == this.sensor) {
                return false;
            }
            if (null != this.volume) {
                this.volume.Dispose();
                this.volume = null;
            }
            try {
                ReconstructionParameters volParam = new ReconstructionParameters(this.voxelsPerMeter, this.voxelsX, this.voxelsY, this.voxelsZ);
                this.worldToCameraTransform = Matrix4.Identity;
                this.volume = ColorReconstruction.FusionCreateReconstruction(volParam, ProcessorType, DeviceToUse, this.worldToCameraTransform);
                this.defaultWorldToVolumeTransform = this.volume.GetCurrentWorldToVolumeTransform();
                if (this.translateResetPoseByMinDepthThreshold) {
                    this.ResetReconstruction();
                }
                else {
                    this.ResetTracking();
                    this.ResetColorImage();
                }
                this.worldToBGRTransform = Matrix4.Identity;
                this.worldToBGRTransform.M11 = this.voxelsPerMeter / this.voxelsX;
                this.worldToBGRTransform.M22 = this.voxelsPerMeter / this.voxelsY;
                this.worldToBGRTransform.M33 = this.voxelsPerMeter / this.voxelsZ;
                this.worldToBGRTransform.M41 = 0.5f;
                this.worldToBGRTransform.M42 = 0.5f;
                this.worldToBGRTransform.M44 = 1.0f;
                if (this.volumeGraphics) {
                    Dispatcher.BeginInvoke(
                        (Action)(() => {
                                this.RemoveVolumeCube3DGraphics();
                                this.DisposeVolumeCube3DGraphics();
                                this.CreateCube3DGraphics(volumeCubeLineColor, LineThickness, new Vector3D(0, 0, 0));
                                this.AddVolumeCube3DGraphics();
                            }));
                }
                this.viewChanged = true;

                return true;
            }
            catch (ArgumentException) {
                this.volume = null;
                this.ShowStatusMessage("Volume Resolution invalid");
            }
            catch (InvalidOperationException ex) {
                this.volume = null;
                this.ShowStatusMessage(ex.Message);
            }
            catch (DllNotFoundException) {
                this.volume = null;
                this.ShowStatusMessage("Missing requisite DLL");
            }
            catch (OutOfMemoryException) {
                this.volume = null;
                this.ShowStatusMessage("Out of memory");
            }
            return false;
        }

        private void ResetColorImage() {
            if (null != this.resampledColorFrameDepthAligned && null != this.resampledColorImagePixelsAlignedToDepth) {
                Array.Clear(this.resampledColorImagePixelsAlignedToDepth, 0, this.resampledColorImagePixelsAlignedToDepth.Length);
                this.resampledColorFrameDepthAligned.CopyPixelDataFrom(this.resampledColorImagePixelsAlignedToDepth);
            }

            this.colorCaptured = false;
        }

        private void UpdateVirtualCameraTransform() {
            this.virtualCamera.WorldToCameraMatrix4 = this.worldToCameraTransform;
        }

        
        private void CreateCube3DGraphics(System.Windows.Media.Color color, int thickness, Vector3D translation) {
            float cubeSizeScaler = 1.0f;
            float oneOverVpm = 1.0f / this.voxelsPerMeter;
            float cubeSideX = this.voxelsX * oneOverVpm * cubeSizeScaler;
            float halfSideX = cubeSideX * 0.5f;
            float cubeSideY = this.voxelsY * oneOverVpm * cubeSizeScaler;
            float halfSideY = cubeSideY * 0.5f;
            float cubeSideZ = this.voxelsZ * oneOverVpm * cubeSizeScaler;
            float halfSideZ = cubeSideZ * 0.5f;
            translation.Z -= halfSideZ / cubeSizeScaler;
            this.volumeCube = new ScreenSpaceLines3D();
            this.volumeCube.Points = new Point3DCollection();
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, -halfSideZ + translation.Z));
            this.volumeCube.Points.Add(new Point3D(-halfSideX + translation.X, -halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCube.Thickness = thickness;
            this.volumeCube.Color = color;
            this.volumeCubeAxisX = new ScreenSpaceLines3D();
            this.volumeCubeAxisX.Points = new Point3DCollection();
            this.volumeCubeAxisX.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisX.Points.Add(new Point3D(-halfSideX + 0.1f + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisX.Thickness = thickness + 2;
            this.volumeCubeAxisX.Color = System.Windows.Media.Color.FromArgb(200, 255, 0, 0);
            this.volumeCubeAxisY = new ScreenSpaceLines3D();
            this.volumeCubeAxisY.Points = new Point3DCollection();
            this.volumeCubeAxisY.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisY.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY - 0.1f + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisY.Thickness = thickness + 2;
            this.volumeCubeAxisY.Color = System.Windows.Media.Color.FromArgb(200, 0, 255, 0);
            this.volumeCubeAxisZ = new ScreenSpaceLines3D();
            this.volumeCubeAxisZ.Points = new Point3DCollection();
            this.volumeCubeAxisZ.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ + translation.Z));
            this.volumeCubeAxisZ.Points.Add(new Point3D(-halfSideX + translation.X, halfSideY + translation.Y, halfSideZ - 0.1f + translation.Z));

            this.volumeCubeAxisZ.Thickness = thickness + 2;
            this.volumeCubeAxisZ.Color = System.Windows.Media.Color.FromArgb(200, 0, 0, 255);
        }

        private void AddVolumeCube3DGraphics() {
            if (this.haveAddedVolumeCube) {
                return;
            }

            if (null != this.volumeCube) {
                this.ReconstructionViewport.Children.Add(this.volumeCube);

                this.haveAddedVolumeCube = true;
            }

            if (null != this.volumeCubeAxisX) {
                this.ReconstructionViewport.Children.Add(this.volumeCubeAxisX);
            }

            if (null != this.volumeCubeAxisY) {
                this.ReconstructionViewport.Children.Add(this.volumeCubeAxisY);
            }

            if (null != this.volumeCubeAxisZ) {
                this.ReconstructionViewport.Children.Add(this.volumeCubeAxisZ);
            }
        }

        private void RemoveVolumeCube3DGraphics() {
            if (null != this.volumeCube) {
                this.ReconstructionViewport.Children.Remove(this.volumeCube);
            }

            if (null != this.volumeCubeAxisX) {
                this.ReconstructionViewport.Children.Remove(this.volumeCubeAxisX);
            }

            if (null != this.volumeCubeAxisY) {
                this.ReconstructionViewport.Children.Remove(this.volumeCubeAxisY);
            }

            if (null != this.volumeCubeAxisZ) {
                this.ReconstructionViewport.Children.Remove(this.volumeCubeAxisZ);
            }

            this.haveAddedVolumeCube = false;
        }

        private void DisposeVolumeCube3DGraphics() {
            if (this.haveAddedVolumeCube) {
                this.RemoveVolumeCube3DGraphics();
            }

            if (null != this.volumeCube) {
                this.volumeCube.Dispose();
                this.volumeCube = null;
            }

            if (null != this.volumeCubeAxisX) {
                this.volumeCubeAxisX.Dispose();
                this.volumeCubeAxisX = null;
            }

            if (null != this.volumeCubeAxisY) {
                this.volumeCubeAxisY.Dispose();
                this.volumeCubeAxisY = null;
            }

            if (null != this.volumeCubeAxisZ) {
                this.volumeCubeAxisZ.Dispose();
                this.volumeCubeAxisZ = null;
            }
        }

        private void ShowStatusMessage(string message) {
            Console.WriteLine(message);
        }
        
        private void AllocateKinectFusionResources() {
            this.SafeDisposeFusionResources();
            this.depthFloatFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);
            this.resampledColorFrameDepthAligned = new FusionColorImageFrame(this.depthWidth, this.depthHeight);
            this.deltaFromReferenceFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);
            this.raycastPointCloudFrame = new FusionPointCloudImageFrame(this.depthWidth, this.depthHeight);
            this.shadedSurfaceFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);
            this.shadedSurfaceNormalsFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);
            this.resampledColorFrame = new FusionColorImageFrame(this.depthWidth, this.depthHeight);
            this.depthPointCloudFrame = new FusionPointCloudImageFrame(this.depthWidth, this.depthHeight);
            this.smoothDepthFloatFrame = new FusionFloatImageFrame(this.depthWidth, this.depthHeight);
            this.downsampledWidth = this.depthWidth / DownsampleFactor;
            this.downsampledHeight = this.depthHeight / DownsampleFactor;
            this.downsampledDepthFloatFrame = new FusionFloatImageFrame(this.downsampledWidth, this.downsampledHeight);
            this.downsampledSmoothDepthFloatFrame = new FusionFloatImageFrame(this.downsampledWidth, this.downsampledHeight);
            this.downsampledRaycastPointCloudFrame = new FusionPointCloudImageFrame(this.downsampledWidth, this.downsampledHeight);
            this.downsampledDepthPointCloudFrame = new FusionPointCloudImageFrame(this.downsampledWidth, this.downsampledHeight);
            this.downsampledDeltaFromReferenceFrameColorFrame = new FusionColorImageFrame(this.downsampledWidth, this.downsampledHeight);
            int depthImageSize = this.depthWidth * this.depthHeight;
            int colorImageByteSize = this.colorWidth * this.colorHeight * sizeof(int);
            int downsampledDepthImageSize = this.downsampledWidth * this.downsampledHeight;
            this.depthImagePixels = new ushort[depthImageSize];
            this.colorImagePixels = new byte[colorImageByteSize];
            this.resampledColorImagePixels = new int[depthImageSize];
            this.depthFloatFrameDepthPixels = new float[depthImageSize];
            this.deltaFromReferenceFrameFloatPixels = new float[depthImageSize];
            this.depthFloatFramePixelsArgb = new int[depthImageSize];
            this.deltaFromReferenceFramePixelsArgb = new int[depthImageSize];
            this.colorCoordinates = new ColorSpacePoint[depthImageSize];
            this.resampledColorImagePixelsAlignedToDepth = new int[depthImageSize];
            this.downsampledDepthImagePixels = new float[downsampledDepthImageSize];
            this.downsampledDeltaFromReferenceColorPixels = new int[downsampledDepthImageSize];
            CameraPoseFinderParameters cameraPoseFinderParams = CameraPoseFinderParameters.Defaults;
            this.cameraPoseFinder = CameraPoseFinder.FusionCreateCameraPoseFinder(cameraPoseFinderParams);
        }

        private void SafeDisposeFusionResources() {
            if (null != this.depthFloatFrame) {
                this.depthFloatFrame.Dispose();
            }

            if (null != this.resampledColorFrameDepthAligned) {
                this.resampledColorFrameDepthAligned.Dispose();
            }

            if (null != this.deltaFromReferenceFrame) {
                this.deltaFromReferenceFrame.Dispose();
            }

            if (null != this.raycastPointCloudFrame) {
                this.raycastPointCloudFrame.Dispose();
            }

            if (null != this.shadedSurfaceFrame) {
                this.shadedSurfaceFrame.Dispose();
            }

            if (null != this.shadedSurfaceNormalsFrame) {
                this.shadedSurfaceNormalsFrame.Dispose();
            }

            if (null != this.resampledColorFrame) {
                this.resampledColorFrame.Dispose();
            }

            if (null != this.depthPointCloudFrame) {
                this.depthPointCloudFrame.Dispose();
            }

            if (null != this.smoothDepthFloatFrame) {
                this.smoothDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledDepthFloatFrame) {
                this.downsampledDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledSmoothDepthFloatFrame) {
                this.downsampledSmoothDepthFloatFrame.Dispose();
            }

            if (null != this.downsampledRaycastPointCloudFrame) {
                this.downsampledRaycastPointCloudFrame.Dispose();
            }

            if (null != this.downsampledDepthPointCloudFrame) {
                this.downsampledDepthPointCloudFrame.Dispose();
            }

            if (null != this.downsampledDeltaFromReferenceFrameColorFrame) {
                this.downsampledDeltaFromReferenceFrameColorFrame.Dispose();
            }

            if (null != this.cameraPoseFinder) {
                this.cameraPoseFinder.Dispose();
            }
        }

        
        private unsafe void DownsampleDepthFrameNearestNeighbor(FusionFloatImageFrame dest, int factor) {
            if (null == dest || null == this.downsampledDepthImagePixels) {
                throw new ArgumentException("inputs null");
            }

            if (false == (2 == factor || 4 == factor || 8 == factor || 16 == factor)) {
                throw new ArgumentException("factor != 2, 4, 8 or 16");
            }

            int downsampleWidth = this.depthWidth / factor;
            int downsampleHeight = this.depthHeight / factor;
            if (dest.Width != downsampleWidth || dest.Height != downsampleHeight) {
                throw new ArgumentException("dest != downsampled image size");
            }

            if (this.mirrorDepth) {
                fixed (ushort* rawDepthPixelPtr = this.depthImagePixels) {
                    ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;
                    Parallel.For(
                        0,
                        downsampleHeight,
                        y => {
                            int destIndex = y * downsampleWidth;
                            int sourceIndex = y * this.depthWidth * factor;

                            for (int x = 0; x < downsampleWidth; ++x, ++destIndex, sourceIndex += factor) {
                                this.downsampledDepthImagePixels[destIndex] = (float)rawDepthPixels[sourceIndex] * 0.001f;
                            }
                        });
                }
            }
            else {
                fixed (ushort* rawDepthPixelPtr = this.depthImagePixels) {
                    ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;
                    Parallel.For(
                        0,
                        downsampleHeight,
                        y => {
                            int flippedDestIndex = (y * downsampleWidth) + (downsampleWidth - 1);
                            int sourceIndex = y * this.depthWidth * factor;

                            for (int x = 0; x < downsampleWidth; ++x, --flippedDestIndex, sourceIndex += factor) {
                                this.downsampledDepthImagePixels[flippedDestIndex] = (float)rawDepthPixels[sourceIndex] * 0.001f;
                            }
                        });
                }
            }

            dest.CopyPixelDataFrom(this.downsampledDepthImagePixels);
        }

        
        private unsafe void UpsampleColorDeltasFrameNearestNeighbor(int factor) {
            if (null == this.downsampledDeltaFromReferenceFrameColorFrame || null == this.downsampledDeltaFromReferenceColorPixels || null == this.deltaFromReferenceFramePixelsArgb) {
                throw new ArgumentException("inputs null");
            }

            if (false == (2 == factor || 4 == factor || 8 == factor || 16 == factor)) {
                throw new ArgumentException("factor != 2, 4, 8 or 16");
            }
            int upsampleWidth = this.downsampledWidth * factor;
            int upsampleHeight = this.downsampledHeight * factor;
            if (this.depthWidth != upsampleWidth || this.depthHeight != upsampleHeight) {
                throw new ArgumentException("upsampled image size != depth size");
            }
            this.downsampledDeltaFromReferenceFrameColorFrame.CopyPixelDataTo(this.downsampledDeltaFromReferenceColorPixels);
            fixed (int* rawColorPixelPtr = this.downsampledDeltaFromReferenceColorPixels) {
                int* rawColorPixels = (int*)rawColorPixelPtr;
                Parallel.For(
                    0,
                    this.downsampledHeight,
                    y => {
                        int destIndex = y * upsampleWidth * factor;
                        int sourceColorIndex = y * this.downsampledWidth;
                        for (int x = 0; x < this.downsampledWidth; ++x, ++sourceColorIndex) {
                            int color = rawColorPixels[sourceColorIndex];
                            for (int colFactorIndex = 0; colFactorIndex < factor; ++colFactorIndex, ++destIndex) {
                                for (int rowFactorIndex = 0; rowFactorIndex < factor; ++rowFactorIndex) {
                                    this.deltaFromReferenceFramePixelsArgb[destIndex + (rowFactorIndex * upsampleWidth)] = color;
                                }
                            }
                        }
                    });
            }

            int sizeOfInt = sizeof(int);
            int rowByteSize = this.downsampledHeight * sizeOfInt;
            for (int y = 0; y < this.downsampledHeight; ++y) {
                int srcRowIndex = upsampleWidth * factor * y;
                for (int r = 1; r < factor; ++r) {
                    int index = upsampleWidth * ((y * factor) + r);
                    System.Buffer.BlockCopy(
                        this.deltaFromReferenceFramePixelsArgb, srcRowIndex * sizeOfInt, this.deltaFromReferenceFramePixelsArgb, index * sizeOfInt, rowByteSize);
                }
            }
        }

        private void ReconstructFrameComplete() {
            RenderColorImage(
                this.captureColor ? this.shadedSurfaceFrame : (this.displayNormals ? this.shadedSurfaceNormalsFrame : this.shadedSurfaceFrame),
                ref this.shadedSurfaceFramePixelsArgb,
                ref this.shadedSurfaceFrameBitmap,
                this.reconstructionImage);
        }

        
    }
}
