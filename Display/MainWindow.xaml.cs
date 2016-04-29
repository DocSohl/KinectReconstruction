using Microsoft.Kinect;
using Microsoft.Kinect.Fusion;
using System;
using System.Collections.Generic;
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

namespace Display {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        KinectSensor sensor = null;
        CoordinateMapper mapper = null;
        MultiSourceFrameReader reader = null;
        int depthWidth;
        int depthHeight;
        int depthPixelCount;
        int colorWidth;
        int colorHeight;
        int colorPixelCount;
        int depthVisibilityTestMapWidth;
        int depthVisibilityTestMapHeight;
        ushort[] depthVisibilityTestMap;

        Point3D virtualCameraStartTranslation;
        Quaternion virtualCameraStartRotation;
        int voxelsX = 512;
        int voxelsY = 512;
        int voxelsZ = 512;
        float voxelsPerMeter = 256.0f;
        GraphicsCamera virtualCamera;

        
        ManualResetEvent depthReadyEvent;
        ManualResetEvent colorReadyEvent;
        ManualResetEvent workerThreadStopEvent;
        Thread workerThread;
        object rawDataLock;
        object volumeLock;
        bool recreateReconstruction = false;
        bool resetReconstruction = false;
        bool savingMesh = false;
        bool cameraPoseFinderAvailable = false;
        bool IsRenderOverdue = false;
        bool autoFindCameraPoseWhenLost = true;
        bool trackingHasFailedPreviously = false;
        bool colorAvailable = false;
        bool trackingFailed = false;
        bool kinectView = true;
        bool AutoResetReconstructionWhenLost = true;


        byte[] colorImagePixels;
        ushort[] depthImagePixels;
        ColorReconstruction volume;
        int trackingErrorCount = 0;
        int successfulFrameCount = 0;
        int processedFrameCount = 0;

        FusionFloatImageFrame depthFloatFrame;
        float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;
        float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;
        short integrationWeight = FusionDepthProcessor.DefaultIntegrationWeight;
        Matrix4 worldToCameraTransform;
        CameraPoseFinder cameraPoseFinder;
        FusionFloatImageFrame downsampledDepthFloatFrame;
        FusionFloatImageFrame downsampledSmoothDepthFloatFrame;
        FusionPointCloudImageFrame downsampledDepthPointCloudFrame;
        FusionPointCloudImageFrame downsampledRaycastPointCloudFrame;
        FusionColorImageFrame downsampledDeltaFromReferenceFrameColorFrame;
        FusionFloatImageFrame deltaFromReferenceFrame;

        WriteableBitmap bitmap;
        float[] depthFloatFrameDepthPixels;
        int[] depthFloatFramePixelsArgb;


        

        const int ColorDownsampleFactor = 4;
        const int MinSuccessfulTrackingFramesForCameraPoseFinder = 45;
        const int MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure = 200;
        const int CameraPoseFinderProcessFrameCalculationInterval = 5;
        const int DeltaFrameCalculationInterval = 2;
        const int MaxTrackingErrors = 100;
        const int ResetOnTimeStampSkippedMillisecondsGPU = 2000;
        const int RawDepthWidth = 512;
        const int RawDepthHeight = 424;
        const int RawColorWidth = 1920;
        const int RawColorHeight = 1080;
        const int RawDepthHeightWithSpecialRatio = 384;
        const int RenderIntervalMilliseconds = 100;
        const int LineThickness = 2;
        const float OriginCoordinateCrossAxisSize = 0.1f;
        const float MaxAlignToReconstructionEnergyForSuccess = 0.27f;
        const float MinAlignToReconstructionEnergyForSuccess = 0.005f;
        const float MaxAlignPointCloudsEnergyForSuccess = 0.006f;
        const float MinAlignPointCloudsEnergyForSuccess = 0.0f;
        const int MaxCameraPoseFinderPoseTests = 5;
        const float CameraPoseFinderDistanceThresholdReject = 1.0f;
        const float CameraPoseFinderDistanceThresholdAccept = 0.1f;
        const int SmoothingKernelWidth = 1;
        const float SmoothingDistanceThreshold = 0.04f;
        const float MaxTranslationDeltaAlignPointClouds = 0.3f;
        const float MaxRotationDeltaAlignPointClouds = 20.0f;
        const int DownsampleFactor = 2;
        const ushort DepthVisibilityTestThreshold = 50;
        static System.Windows.Media.Color volumeCubeLineColor = System.Windows.Media.Color.FromArgb(200, 0, 200, 0);
        bool autoResetReconstructionOnTimeSkip = false; 

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
            this.virtualCameraStartRotation = Quaternion.Identity;
            this.virtualCamera = new GraphicsCamera(this.virtualCameraStartTranslation, this.virtualCameraStartRotation, (float)Width / (float)Height);
            this.ReconstructionViewport.Camera = this.virtualCamera.Camera;
            this.StartWorkerThread();
            this.recreateReconstruction = true;

            this.reader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;
            this.AllocateKinectFusionResources();
            this.virtualCamera.CreateFrustum3DGraphics(this.ReconstructionViewport, this.depthWidth, this.depthHeight);

            Closing += onClose;
        }

        private void AllocateKinectFusionResources() {
            throw new NotImplementedException();
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
                    // MultiSourceFrame is IDisposable
                    lock (this.rawDataLock) {
                        ColorFrameReference colorFrameReference = multiSourceFrame.ColorFrameReference;
                        DepthFrameReference depthFrameReference = multiSourceFrame.DepthFrameReference;

                        colorFrame = colorFrameReference.AcquireFrame();
                        depthFrame = depthFrameReference.AcquireFrame();

                        if ((depthFrame != null) && (colorFrame != null)) {

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
            catch (Exception) {
                // ignore if the frame is no longer available
            }
            finally {
                // MultiSourceFrame, DepthFrame, ColorFrame, BodyIndexFrame are IDispoable
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
                // Signal worker thread to process
                this.depthReadyEvent.Set();
            }

            if (validColor) {
                // Signal worker thread to process
                this.colorReadyEvent.Set();
            }
        }

        private void StartWorkerThread() {
            this.depthReadyEvent = new ManualResetEvent(false);
            this.colorReadyEvent = new ManualResetEvent(false);
            this.workerThreadStopEvent = new ManualResetEvent(false);
            this.workerThread = new Thread(this.WorkerThreadProc);
            this.workerThread.Start();
            
        }

        private void WorkerThreadProc() {
            WaitHandle[] events = new WaitHandle[2] { this.workerThreadStopEvent, this.depthReadyEvent };
            while (true) {
                int index = WaitHandle.WaitAny(events);
                if (index == 0) {
                    return;
                }
                this.depthReadyEvent.Reset();
                this.Process();
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
                    // Check if camera pose finder is available
                    this.cameraPoseFinderAvailable = this.autoFindCameraPoseWhenLost
                        && null != this.cameraPoseFinder
                        && this.cameraPoseFinder.GetStoredPoseCount() > 0;

                    // Convert depth to float and render depth frame
                    this.ProcessDepthData();

                    // Track camera pose
                    this.TrackCamera();

                    // Only continue if we do not have tracking errors
                    if (0 == this.trackingErrorCount) {
                        // Integrate depth
                        bool colorAvailable = this.IntegrateData();

                        // Check to see if another depth frame is already available. 
                        // If not we have time to calculate a point cloud and render, 
                        // but if so we make sure we force a render at least every 
                        // RenderIntervalMilliseconds.
                        if (!this.depthReadyEvent.WaitOne(0) || this.IsRenderOverdue) {
                            // Raycast and render
                            this.RenderReconstruction();
                        }

                        // Update camera pose finder, adding key frames to the database
                        if (this.autoFindCameraPoseWhenLost && !this.trackingHasFailedPreviously
                            && this.successfulFrameCount > MinSuccessfulTrackingFramesForCameraPoseFinder
                            && this.processedFrameCount % CameraPoseFinderProcessFrameCalculationInterval == 0
                            && colorAvailable) {
                            this.UpdateCameraPoseFinder();
                        }
                    }
                }
                catch (InvalidOperationException ex) {
                    Console.WriteLine(ex.Message);
                }
            }
        }

        private void ProcessDepthData() {
            // Lock the depth operations
            lock (this.rawDataLock) {
                this.volume.DepthToDepthFloatFrame(
                    this.depthImagePixels,
                    this.depthFloatFrame,
                    this.minDepthClip,
                    this.maxDepthClip,
                    false);
            }

            // Render depth float frame
            this.Dispatcher.BeginInvoke(
                (Action)
                (() =>
                    this.RenderDepthFloatImage()));
        }

        private void RenderDepthFloatImage() {
            if (null == this.depthFloatFrame) {
                return;
            }

            if (null == bitmap || this.depthFloatFrame.Width != bitmap.Width || this.depthFloatFrame.Height != bitmap.Height) {
                // Create bitmap of correct format
                bitmap = new WriteableBitmap(this.depthFloatFrame.Width, this.depthFloatFrame.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // Set bitmap as source to UI image object
                this.depthImage.Source = bitmap;
            }

            this.depthFloatFrame.CopyPixelDataTo(this.depthFloatFrameDepthPixels);

            // Calculate color of pixels based on depth of each pixel
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

            // Copy colored pixels to bitmap
            bitmap.WritePixels(
                        new Int32Rect(0, 0, this.depthFloatFrame.Width, this.depthFloatFrame.Height),
                        this.depthFloatFramePixelsArgb,
                        bitmap.PixelWidth * sizeof(int),
                        0);
        }

        private void TrackCamera() {
            bool calculateDeltaFrame = this.processedFrameCount % DeltaFrameCalculationInterval == 0;
            bool trackingSucceeded = false;

            // Get updated camera transform from image alignment
            Matrix4 calculatedCameraPos = this.worldToCameraTransform;

            // Here we can either call TrackCameraAlignDepthFloatToReconstruction or TrackCameraAlignPointClouds
            // The TrackCameraAlignPointClouds function typically has higher performance with the camera pose finder 
            // due to its wider basin of convergence, enabling it to more robustly regain tracking from nearby poses
            // suggested by the camera pose finder after tracking is lost.
            if (this.autoFindCameraPoseWhenLost) {
                // Track using AlignPointClouds
                trackingSucceeded = this.TrackCameraAlignPointClouds(ref calculateDeltaFrame, ref calculatedCameraPos);
            }
            else {
                // Track using AlignDepthFloatToReconstruction
                trackingSucceeded = this.TrackCameraAlignDepthFloatToReconstruction(calculateDeltaFrame, ref calculatedCameraPos);
            }

            if (!trackingSucceeded && 0 != this.successfulFrameCount) {
                this.trackingFailed = true;
                this.trackingHasFailedPreviously = true;
                this.trackingErrorCount++;
                this.successfulFrameCount = 0;

                if (!this.cameraPoseFinderAvailable) {
                    // Show tracking error on status bar
                    Console.WriteLine("Camera Pose Finder not Available");
                }
                else {
                    // Here we try to find the correct camera pose, to re-localize camera tracking.
                    // We can call either the version using AlignDepthFloatToReconstruction or the
                    // version using AlignPointClouds, which typically has a higher success rate.
                    // trackingSucceeded = this.FindCameraPoseAlignDepthFloatToReconstruction();
                    trackingSucceeded = this.FindCameraPoseAlignPointClouds();

                    if (!trackingSucceeded) {
                        // Show tracking error on status bar
                        Console.WriteLine("Camera Tracking Failed");
                    }
                }
            }
            else {

                this.UpdateAlignDeltas();

                this.SetTrackingSucceeded();

                this.worldToCameraTransform = calculatedCameraPos;
            }

            if (AutoResetReconstructionWhenLost && !trackingSucceeded
                && this.trackingErrorCount >= MaxTrackingErrors) {
                // Bad tracking
                    Console.WriteLine("Reseting Auto Volume");

                // Automatically Clear Volume and reset tracking if tracking fails
                this.ResetReconstruction();
            }

            if (trackingSucceeded) {
                if (this.kinectView) {
                    Dispatcher.BeginInvoke((Action)(() => this.virtualCamera.WorldToCameraMatrix4 = this.worldToCameraTransform));
                }
                else {
                    // Just update the frustum
                    Dispatcher.BeginInvoke((Action)(() => this.virtualCamera.UpdateFrustumTransformMatrix4(this.worldToCameraTransform)));
                }

                // Increase processed frame counter
                this.processedFrameCount++;
            }
        }

        private bool TrackCameraAlignPointClouds(ref bool calculateDeltaFrame, ref Matrix4 calculatedCameraPose) {
            bool trackingSucceeded = false;

            this.DownsampleDepthFrameNearestNeighbor(this.downsampledDepthFloatFrame, DownsampleFactor);

            // Smooth the depth frame
            this.volume.SmoothDepthFloatFrame(this.downsampledDepthFloatFrame, this.downsampledSmoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);

            // Calculate point cloud from the smoothed frame
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.downsampledSmoothDepthFloatFrame, this.downsampledDepthPointCloudFrame);

            // Get the saved pose view by raycasting the volume from the current camera pose
            this.volume.CalculatePointCloud(this.downsampledRaycastPointCloudFrame, calculatedCameraPose);

            Matrix4 initialPose = calculatedCameraPose;

            // Note that here we only calculate the deltaFromReferenceFrame every 
            // DeltaFrameCalculationInterval frames to reduce computation time
            if (calculateDeltaFrame) {
                trackingSucceeded = FusionDepthProcessor.AlignPointClouds(
                    this.downsampledRaycastPointCloudFrame,
                    this.downsampledDepthPointCloudFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.downsampledDeltaFromReferenceFrameColorFrame,
                    ref calculatedCameraPose);

                this.UpsampleColorDeltasFrameNearestNeighbor(DownsampleFactor);

                this.UpdateAlignDeltas();

                // Set calculateDeltaFrame to false as we are rendering it here
                calculateDeltaFrame = false;
            }
            else {
                // Don't bother getting the residual delta from reference frame to cut computation time
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

        private bool TrackCameraAlignDepthFloatToReconstruction(bool calculateDeltaFrame, ref Matrix4 calculatedCameraPos) {
            bool trackingSucceeded = false;

            // Note that here we only calculate the deltaFromReferenceFrame every 
            // DeltaFrameCalculationInterval frames to reduce computation time
            if (calculateDeltaFrame) {
                trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    this.depthFloatFrame,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    this.deltaFromReferenceFrame,
                    out this.alignmentEnergy,
                    this.worldToCameraTransform);
            }
            else {
                // Don't bother getting the residual delta from reference frame to cut computation time
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
                // Tracking succeeded, get the updated camera pose
                calculatedCameraPos = this.volume.GetCurrentWorldToCameraTransform();
            }

            return trackingSucceeded;
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
                this.ShowStatusMessage(Properties.Resources.PoseFinderNotEnoughMatches);
                return false;
            }

            // Smooth the depth frame
            this.volume.SmoothDepthFloatFrame(this.depthFloatFrame, this.smoothDepthFloatFrame, SmoothingKernelWidth, SmoothingDistanceThreshold);

            // Calculate point cloud from the smoothed frame
            FusionDepthProcessor.DepthFloatFrameToPointCloud(this.smoothDepthFloatFrame, this.depthPointCloudFrame);

            double smallestEnergy = double.MaxValue;
            int smallestEnergyNeighborIndex = -1;

            int bestNeighborIndex = -1;
            Matrix4 bestNeighborCameraPose = Matrix4.Identity;

            double bestNeighborAlignmentEnergy = MaxAlignPointCloudsEnergyForSuccess;

            // Run alignment with best matched poseCount (i.e. k nearest neighbors (kNN))
            int maxTests = Math.Min(MaxCameraPoseFinderPoseTests, poseCount);

            var neighbors = matchCandidates.GetMatchPoses();

            for (int n = 0; n < maxTests; n++) {
                // Run the camera tracking algorithm with the volume
                // this uses the raycast frame and pose to find a valid camera pose by matching the raycast against the input point cloud
                Matrix4 poseProposal = neighbors[n];

                // Get the saved pose view by raycasting the volume
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

                    // This is after tracking succeeds, so should be a more accurate pose to store...
                    bestNeighborCameraPose = poseProposal;

                    // Update the delta image
                    this.resampledColorFrame.CopyPixelDataTo(this.deltaFromReferenceFramePixelsArgb);
                }

                // Find smallest energy neighbor independent of tracking success
                if (this.alignmentEnergy < smallestEnergy) {
                    smallestEnergy = this.alignmentEnergy;
                    smallestEnergyNeighborIndex = n;
                }
            }

            matchCandidates.Dispose();

            // Use the neighbor with the smallest residual alignment energy
            // At the cost of additional processing we could also use kNN+Mean camera pose finding here
            // by calculating the mean pose of the best n matched poses and also testing this to see if the 
            // residual alignment energy is less than with kNN.
            if (bestNeighborIndex > -1) {
                this.worldToCameraTransform = bestNeighborCameraPose;
                this.SetReferenceFrame(this.worldToCameraTransform);

                // Tracking succeeded!
                this.SetTrackingSucceeded();

                this.UpdateAlignDeltas();

                this.ShowStatusMessageLowPriority("Camera Pose Finder SUCCESS! Residual energy= " + string.Format(CultureInfo.InvariantCulture, "{0:0.00000}", bestNeighborAlignmentEnergy) + ", " + poseCount + " frames stored, minimum distance=" + minDistance + ", best match index=" + bestNeighborIndex);

                return true;
            }
            else {
                this.worldToCameraTransform = neighbors[smallestEnergyNeighborIndex];
                this.SetReferenceFrame(this.worldToCameraTransform);

                // Camera pose finding failed - return the tracking failed error code
                this.SetTrackingFailed();

                // Tracking Failed will be set again on the next iteration in ProcessDepth
                this.ShowStatusMessageLowPriority("Camera Pose Finder FAILED! Residual energy=" + string.Format(CultureInfo.InvariantCulture, "{0:0.00000}", smallestEnergy) + ", " + poseCount + " frames stored, minimum distance=" + minDistance + ", best match index=" + smallestEnergyNeighborIndex);

                return false;
            }
        }

        private bool RecreateReconstruction() {
            // Check if sensor has been initialized
            if (null == this.sensor) {
                return false;
            }

            if (null != this.volume) {
                this.volume.Dispose();
                this.volume = null;
            }

            try {
                ReconstructionParameters volParam = new ReconstructionParameters(this.voxelsPerMeter, this.voxelsX, this.voxelsY, this.voxelsZ);

                // Set the world-view transform to identity, so the world origin is the initial camera location.
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

                // Map world X axis to blue channel, Y axis to green channel and Z axis to red channel,
                // normalizing each to the range [0, 1]. We also add a shift of 0.5 to both X,Y channels
                // as the world origin starts located at the center of the front face of the volume,
                // hence we need to map negative x,y world vertex locations to positive color values.
                this.worldToBGRTransform = Matrix4.Identity;
                this.worldToBGRTransform.M11 = this.voxelsPerMeter / this.voxelsX;
                this.worldToBGRTransform.M22 = this.voxelsPerMeter / this.voxelsY;
                this.worldToBGRTransform.M33 = this.voxelsPerMeter / this.voxelsZ;
                this.worldToBGRTransform.M41 = 0.5f;
                this.worldToBGRTransform.M42 = 0.5f;
                this.worldToBGRTransform.M44 = 1.0f;

                // Update the graphics volume cube rendering
                if (this.volumeGraphics) {
                    Dispatcher.BeginInvoke(
                        (Action)(() => {
                                // re-create the volume cube display with the new size
                                this.RemoveVolumeCube3DGraphics();
                                this.DisposeVolumeCube3DGraphics();
                                this.CreateCube3DGraphics(volumeCubeLineColor, LineThickness, new Vector3D(0, 0, 0));
                                this.AddVolumeCube3DGraphics();
                            }));
                }

                // Signal that a render is required
                this.viewChanged = true;

                return true;
            }
            catch (ArgumentException) {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.VolumeResolution);
            }
            catch (InvalidOperationException ex) {
                this.volume = null;
                this.ShowStatusMessage(ex.Message);
            }
            catch (DllNotFoundException) {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.MissingPrerequisite);
            }
            catch (OutOfMemoryException) {
                this.volume = null;
                this.ShowStatusMessage(Properties.Resources.OutOfMemory);
            }

            return false;
        }

        private void ResetReconstruction() {
            if (null == this.sensor) {
                return;
            }

            // Reset tracking error counter
            this.trackingErrorCount = 0;

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;

            // Reset volume
            if (null != this.volume) {
                try {
                    // Translate the reconstruction volume location away from the world origin by an amount equal
                    // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
                    // If set false, the default world origin is set to the center of the front face of the 
                    // volume, which has the effect of locating the volume directly in front of the initial camera
                    // position with the +Z axis into the volume along the initial camera direction of view.
                    if (this.translateResetPoseByMinDepthThreshold) {
                        Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;

                        // Translate the volume in the Z axis by the minDepthClip distance
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
                    this.ShowStatusMessage(Properties.Resources.ResetFailed);
                }
            }

            // Update manual reset information to status bar
            this.ShowStatusMessage(Properties.Resources.ResetVolume);
        }

        private bool IntegrateData() {
            // Color may opportunistically be available here - check
            bool colorAvailable = this.colorReadyEvent.WaitOne(0);

            // Don't integrate depth data into the volume if:
            // 1) tracking failed
            // 2) camera pose finder is off and we have paused capture
            // 3) camera pose finder is on and we are still under the m_cMinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure
            //    number of successful frames count.
            bool integrateData = !this.trackingFailed && !this.PauseIntegration &&
                (!this.cameraPoseFinderAvailable || (this.cameraPoseFinderAvailable && !(this.trackingHasFailedPreviously && this.successfulFrameCount < MinSuccessfulTrackingFramesForCameraPoseFinderAfterFailure)));

            // Integrate the frame to volume
            if (integrateData) {
                bool integrateColor = this.processedFrameCount % ColorIntegrationInterval == 0 && colorAvailable;

                // Reset this flag as we are now integrating data again
                this.trackingHasFailedPreviously = false;

                if (this.captureColor && integrateColor) {
                    // Pre-process color
                    this.MapColorToDepth();

                    // Integrate color and depth
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.resampledColorFrameDepthAligned,
                        this.integrationWeight,
                        FusionDepthProcessor.DefaultColorIntegrationOfAllAngles,
                        this.worldToCameraTransform);

                    // Flag that we have captured color
                    this.colorCaptured = true;
                }
                else {
                    // Just integrate depth
                    this.volume.IntegrateFrame(
                        this.depthFloatFrame,
                        this.integrationWeight,
                        this.worldToCameraTransform);
                }

                // Reset color ready event
                this.colorReadyEvent.Reset();
            }

            return colorAvailable;
        }

        private void RenderReconstruction() {
            if (null == this.volume || this.savingMesh || null == this.raycastPointCloudFrame
                || null == this.shadedSurfaceFrame || null == this.shadedSurfaceNormalsFrame) {
                return;
            }

            // If KinectView option has been set, use the worldToCameraTransform, else use the virtualCamera transform
            Matrix4 cameraView = this.KinectView ? this.worldToCameraTransform : this.virtualCameraWorldToCameraMatrix4;

            if (this.captureColor) {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, this.shadedSurfaceFrame, cameraView);
            }
            else {
                this.volume.CalculatePointCloud(this.raycastPointCloudFrame, cameraView);

                // Shade point cloud frame for rendering
                FusionDepthProcessor.ShadePointCloud(
                    this.raycastPointCloudFrame,
                    cameraView,
                    this.worldToBGRTransform,
                    this.displayNormals ? null : this.shadedSurfaceFrame,
                    this.displayNormals ? this.shadedSurfaceNormalsFrame : null);
            }

            // Update the rendered UI image
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

            // This function will add the pose to the camera pose finding database when the input frame's minimum
            // distance to the existing database is equal to or above CameraPoseFinderDistanceThresholdAccept 
            // (i.e. indicating that the input has become dis-similar to the existing database and a new frame 
            // should be captured). Note that the color and depth frames must be the same size, however, the 
            // horizontal mirroring setting does not have to be consistent between depth and color. It does have
            // to be consistent between camera pose finder database creation and calling FindCameraPose though,
            // hence we always reset both the reconstruction and database when changing the mirror depth setting.
            this.cameraPoseFinder.ProcessFrame(
                this.depthFloatFrame,
                this.resampledColorFrame,
                this.worldToCameraTransform,
                CameraPoseFinderDistanceThresholdAccept,
                out addedPose,
                out poseHistoryTrimmed);

            if (true == addedPose) {
                this.ShowStatusMessageLowPriority("Camera Pose Finder Added Frame! " + this.cameraPoseFinder.GetStoredPoseCount() + " frames stored, minimum distance >= " + CameraPoseFinderDistanceThresholdAccept);
            }

            if (true == poseHistoryTrimmed) {
                this.ShowStatusMessage(Properties.Resources.PoseFinderPoseHistoryFull);
            }
        }

        private void UpdateAlignDeltas() {
            if (this.autoFindCameraPoseWhenLost) {
                Dispatcher.BeginInvoke(
                    (Action)
                    (() =>
                        this.RenderAlignDeltasColorImage(
                            ref this.deltaFromReferenceFrameBitmap,
                            this.deltaFromReferenceImage)));
            }
            else {
                Dispatcher.BeginInvoke(
                    (Action)
                    (() =>
                        this.RenderAlignDeltasFloatImage(
                            this.deltaFromReferenceFrame,
                            ref this.deltaFromReferenceFrameBitmap,
                            this.deltaFromReferenceImage)));
            }
        }

        private void SetTrackingSucceeded() {
            // Clear track error count and increment the successful frame count
            this.trackingFailed = false;
            this.trackingErrorCount = 0;
            this.successfulFrameCount++;
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
                                // Copy depth value
                                this.downsampledDepthImagePixels[destIndex] = (float)rawDepthPixels[sourceIndex] * 0.001f;
                            }
                        });
                }
            }
            else {
                fixed (ushort* rawDepthPixelPtr = this.depthImagePixels) {
                    ushort* rawDepthPixels = (ushort*)rawDepthPixelPtr;

                    // Horizontal flip the color image as the standard depth image is flipped internally in Kinect Fusion
                    // to give a viewpoint as though from behind the Kinect looking forward by default.
                    Parallel.For(
                        0,
                        downsampleHeight,
                        y => {
                            int flippedDestIndex = (y * downsampleWidth) + (downsampleWidth - 1);
                            int sourceIndex = y * this.depthWidth * factor;

                            for (int x = 0; x < downsampleWidth; ++x, --flippedDestIndex, sourceIndex += factor) {
                                // Copy depth value
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

            // Here we make use of unsafe code to just copy the whole pixel as an int for performance reasons, as we do
            // not need access to the individual rgba components.
            fixed (int* rawColorPixelPtr = this.downsampledDeltaFromReferenceColorPixels) {
                int* rawColorPixels = (int*)rawColorPixelPtr;

                // Note we run this only for the source image height pixels to sparsely fill the destination with rows
                Parallel.For(
                    0,
                    this.downsampledHeight,
                    y => {
                        int destIndex = y * upsampleWidth * factor;
                        int sourceColorIndex = y * this.downsampledWidth;

                        for (int x = 0; x < this.downsampledWidth; ++x, ++sourceColorIndex) {
                            int color = rawColorPixels[sourceColorIndex];

                            // Replicate pixels horizontally
                            for (int colFactorIndex = 0; colFactorIndex < factor; ++colFactorIndex, ++destIndex) {
                                // Replicate pixels vertically
                                for (int rowFactorIndex = 0; rowFactorIndex < factor; ++rowFactorIndex) {
                                    // Copy color pixel
                                    this.deltaFromReferenceFramePixelsArgb[destIndex + (rowFactorIndex * upsampleWidth)] = color;
                                }
                            }
                        }
                    });
            }

            int sizeOfInt = sizeof(int);
            int rowByteSize = this.downsampledHeight * sizeOfInt;

            // Duplicate the remaining rows with memcpy
            for (int y = 0; y < this.downsampledHeight; ++y) {
                // iterate only for the smaller number of rows
                int srcRowIndex = upsampleWidth * factor * y;

                // Duplicate lines
                for (int r = 1; r < factor; ++r) {
                    int index = upsampleWidth * ((y * factor) + r);

                    System.Buffer.BlockCopy(
                        this.deltaFromReferenceFramePixelsArgb, srcRowIndex * sizeOfInt, this.deltaFromReferenceFramePixelsArgb, index * sizeOfInt, rowByteSize);
                }
            }
        }

        private void onClose(object sender, System.ComponentModel.CancelEventArgs e) {
            this.workerThreadStopEvent.Set();
            this.workerThread.Join();
        }


        
    }
}
