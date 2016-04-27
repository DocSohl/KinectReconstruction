using Microsoft.Kinect;
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
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Display {
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window {
        Thread t2;
        private KinectSensor kinectSensor = null;
        private DepthFrameReader depthFrameReader = null;
        private FrameDescription depthFrameDescription = null;
        private byte[] depthPixels = null;
        private float[] depthFloatPixels = null;
        private int[] depthFrameRgb = null;
        private long updateKinectTicks = 0;
        private WriteableBitmap bmp = null;

        Thread t1;


        public MainWindow() {
            InitializeComponent();
            t2 = new Thread(new ThreadStart(KinectLoop));
            t2.Start();
            t1 = new Thread(new ThreadStart(UpdateLoop));
            t1.Start();
            Closing += onClose;
        }

        private void onClose(object sender, System.ComponentModel.CancelEventArgs e) {
            t1.Abort();
            t2.Abort();
        }


        private void UpdateLoop() {
            while (true) {
                Thread.Sleep(10);
                this.Dispatcher.Invoke((Action)(() => { renderDepthImage(); }));
            }
        }


        private void renderDepthImage() {
            if (bmp == null) {
                bmp = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                this.depthImage.Source = bmp;
            }
            float range = 4.0f;
            float oneOverRange = (1.0f / range) * 256.0f;
            float minRange = 0.0f;
            Parallel.For(
            0,
            this.depthFrameDescription.Height,
            y => {
                int index = y * this.depthFrameDescription.Width;
                for (int x = 0; x < this.depthFrameDescription.Width; ++x, ++index) {
                    float depth = this.depthFloatPixels[index];
                    int intensity = (depth >= minRange) ? ((byte)((depth - minRange) * oneOverRange)) : 0;

                    this.depthFrameRgb[index] = (255 << 24) | (intensity << 16) | (intensity << 8) | intensity; // set blue, green, red
                }
            });
            bmp.WritePixels(new Int32Rect(0, 0, this.depthFrameDescription.Width, this.depthFrameDescription.Height), this.depthFrameRgb, bmp.PixelWidth * sizeof(int), 0);
        }


        public void KinectLoop() {
            this.kinectSensor = KinectSensor.GetDefault();
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();
            this.depthFrameReader.FrameArrived += this.readerFrameArrived;
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.depthFloatPixels = new float[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.depthFrameRgb = new int[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.kinectSensor.IsAvailableChanged += this.sensorIsAvailableChanged;
            this.kinectSensor.Open();
            if (!this.kinectSensor.IsAvailable) {
                Console.WriteLine("Kinect not available");
            }
        }


        private void readerFrameArrived(object sender, DepthFrameArrivedEventArgs e) {
            double interval = (double)Stopwatch.Frequency / 10;
            long newticks = Stopwatch.GetTimestamp();
            if (newticks > this.updateKinectTicks + interval) {
                this.updateKinectTicks = newticks;
                using (DepthFrame depthFrame = e.FrameReference.AcquireFrame()) {
                    if (depthFrame != null) {
                        using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer()) {
                            if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel))) {
                                this.processData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance);
                            }
                        }
                    }
                }
            }
        }

        private unsafe void processData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth) {
            ushort* frameData = (ushort*)depthFrameData;
            int pixels = (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel);
            for (int i = 0; i < pixels; ++i) {
                ushort depth = frameData[i];
                this.depthFloatPixels[i] = (float)depth * 0.001f;
            }
        }

        private void sensorIsAvailableChanged(object sender, IsAvailableChangedEventArgs e) {
            if (!this.kinectSensor.IsAvailable) {
                Console.WriteLine("Kinect disconnected");
            }
            else {
                Console.WriteLine("Kinect connected");
            }
        }
    }
}
