var pipeG = new BlobEdgePipelineV2
 {
 MinAreaLR = 10000,
 LeftRightThreshold = 165,
 MinAreaTB = 150,
 TopBottomThreshold = 150,
 dilateIterationsLR = 0,
 TrimPercent = 15,
 BandScale = 0.98,
 MinSamplesForFit = 10
 };
 var pipeC = new BlobEdgePipelineV2
 {
 MinAreaLR = 10000,
 LeftRightThreshold = 165,
 MinAreaTB = 150,
 TopBottomThreshold = 150,
 dilateIterationsLR = 0,
 TrimPercent = 15,
 BandScale = 0.98,
 MinSamplesForFit = 10
 };
 
 double X, Z;
 Image<Bgr, byte> visG, visC;
 PointF avgG, avgC;
 Image<Bgr, byte> gold = new Image<Bgr, byte>("2D(x=0,z=107).png");
 Image<Bgr, byte> current = new Image<Bgr, byte>("2D(x=0,z=117).png");
 //Image<Gray, byte> gold_gray = new Image<Gray, byte>(gold.Width, gold.Height);
 //Image<Gray, byte> current_gray = new Image<Gray, byte>(gold.Width, gold.Height);
 
 Image<Gray, byte> gold_gray = gold.Convert<Gray, byte>();
 Image<Gray, byte> current_gray = current.Convert<Gray, byte>();
 bool ok = BlobEdgePipelineV2.CompareAverageCornersAndDraw(
 pipeG, gold_gray,
 pipeC, current_gray,
 out X, out Z,
 out visG, out visC,
 out avgG, out avgC);
 
 tb_msg.Text += ok ? $"ΔX={X:F2}, ΔY={Z:F2} (gold avg=({avgG.X:F1},{avgG.Y:F1}), curr avg=({avgC.X:F1},{avgC.Y:F1}))\r\n"
 : "Could not compute average corners.\r\n";
 
 // Show CURRENT with arrow
 if (visC != null)
 {
 //var old = pictureBox1.Image;
 pictureBox1.Image = visC.Mat.ToBitmap();
 //old?.Dispose();
 }
 if (visG != null)
 {
 //var old = pictureBox1.Image;
 pictureBox2.Image = visG.Mat.ToBitmap();
 //old?.Dispose();
 }
