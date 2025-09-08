internal class vision_1
{
     //public PictureBox pbox = new PictureBox();
     public vision_1() { }
    // public vision_1(PictureBox pb) { pbox = pb; }

     public int batteryframelocationcompensation(string golden_path_name, string current_image_path_name, ref double X, ref double Z)
     {
         Image<Bgr, byte> color_golden = new Image<Bgr, byte>(golden_path_name);
         Image<Gray, byte>[] gray_golden = color_golden.Split();
         Image<Bgr, byte> color_current = new Image<Bgr, byte>(current_image_path_name);
         Image<Gray, byte>[] gray_current = color_current.Split();



         int thr1 = 150;
         int thr2 = 120;
         int thr3 = 150;
         int area = 350;
         PointF[] point_golden = new PointF[4];
         PointF[] point_current = new PointF[4];

         findoutbatteryframecorner(gray_golden[2], thr1, thr2, thr3, area, ref point_golden);
         findoutbatteryframecorner(gray_current[2], thr1, thr2, thr3, area, ref point_current);

         double x_golden_sum = 0;
         double y_golden_sum = 0;
         double x_golden_average = 0;
         double y_golden_average = 0;
         double x_current_sum = 0;
         double y_current_sum = 0;
         double x_current_average = 0;
         double y_current_average = 0;

         for (int i = 0; i < 4; i++)
         {
             x_golden_sum += point_golden[i].X;
             y_golden_sum += point_golden[i].Y;
             x_current_sum += point_current[i].X;
             y_current_sum += point_current[i].Y;
         }

         x_golden_average = (x_golden_sum / 4.0);
         y_golden_average = (y_golden_sum / 4.0);

         x_current_average = (x_current_sum / 4.0);
         y_current_average = (y_current_sum / 4.0);

         X = x_golden_average - x_current_average;
         Z = y_golden_average - y_current_average;


         return 0;
     }
     public static List<BlobInfo> FindBlobs(Image<Gray, byte> binary)
     {
         // Outputs
         Mat labels = new Mat();
         Mat stats = new Mat();
         Mat centroids = new Mat();

         // Run connected components
         int nLabels = CvInvoke.ConnectedComponentsWithStats(
             binary, labels, stats, centroids, LineType.EightConnected, DepthType.Cv32S);

         // Parse results
         int[,] statsArr = (int[,])stats.GetData();       // nLabels x 5
         double[,] centArr = (double[,])centroids.GetData(); // nLabels x 2

         var blobs = new List<BlobInfo>();

         // Index 0 is background, skip it
         for (int i = 1; i < nLabels; i++)
         {
             int x = statsArr[i, 0];   // CC_STAT_LEFT
             int y = statsArr[i, 1];   // CC_STAT_TOP
             int w = statsArr[i, 2];   // CC_STAT_WIDTH
             int h = statsArr[i, 3];   // CC_STAT_HEIGHT
             int area = statsArr[i, 4];// CC_STAT_AREA

             float cx = (float)centArr[i, 0];
             float cy = (float)centArr[i, 1];

             // Create mask for this blob: mask = (labels == i)
             Mat mask = new Mat(labels.Size, DepthType.Cv8U, 1);
             CvInvoke.Compare(labels, new ScalarArray(i), mask, CmpType.Equal);

             blobs.Add(new BlobInfo
             {
                 Label = i,
                 Area = area,
                 BBox = new Rectangle(x, y, w, h),
                 Centroid = new PointF(cx, cy),
                 Mask = mask
             });
         }

         return blobs;
     }
     private double Median(IEnumerable<double> vals)
     {
         var arr = vals.OrderBy(v => v).ToArray();
         if (arr.Length == 0) return 0;
         int mid = arr.Length / 2;
         return (arr.Length % 2 == 1) ? arr[mid] : 0.5 * (arr[mid - 1] + arr[mid]);
     }

     private static Point[] LineAcrossImage(float vx, float vy, float x0, float y0, int W, int H)
     {
         var pts = new List<Point>();
         const double EPS = 1e-9;

         if (System.Math.Abs(vx) > EPS)
         {
             double t = (0 - x0) / vx;            // x=0
             double y = y0 + t * vy;
             if (y >= 0 && y <= H - 1) pts.Add(new Point(0, (int)System.Math.Round(y)));

             t = ((W - 1) - x0) / vx;             // x=W-1
             y = y0 + t * vy;
             if (y >= 0 && y <= H - 1) pts.Add(new Point(W - 1, (int)System.Math.Round(y)));
         }
         if (System.Math.Abs(vy) > EPS)
         {
             double t = (0 - y0) / vy;            // y=0
             double x = x0 + t * vx;
             if (x >= 0 && x <= W - 1) pts.Add(new Point((int)System.Math.Round(x), 0));

             t = ((H - 1) - y0) / vy;             // y=H-1
             x = x0 + t * vx;
             if (x >= 0 && x <= W - 1) pts.Add(new Point((int)System.Math.Round(x), H - 1));
         }
         if (pts.Count < 2)
         {
             var p1 = new Point((int)System.Math.Round(x0 - 100 * vx), (int)System.Math.Round(y0 - 100 * vy));
             var p2 = new Point((int)System.Math.Round(x0 + 100 * vx), (int)System.Math.Round(y0 + 100 * vy));
             return new Point[] { Clamp(p1, W, H), Clamp(p2, W, H) };
         }

         // choose farthest pair
         double maxd = -1; Point a = pts[0], b = pts[1];
         for (int i = 0; i < pts.Count; i++)
             for (int j = i + 1; j < pts.Count; j++)
             {
                 double dx = pts[i].X - pts[j].X, dy = pts[i].Y - pts[j].Y;
                 double d2 = dx * dx + dy * dy;
                 if (d2 > maxd) { maxd = d2; a = pts[i]; b = pts[j]; }
             }
         return new Point[] { a, b };
     }

     private static Point Clamp(Point p, int W, int H)
     {
         return new Point(
             System.Math.Max(0, System.Math.Min(W - 1, p.X)),
             System.Math.Max(0, System.Math.Min(H - 1, p.Y))
         );
     }

     void OrderCorners(Point[] pts, out Point topLeft, out Point topRight, out Point bottomLeft, out Point bottomRight)
     {
         if (pts == null || pts.Length != 4)
             throw new ArgumentException("pts must contain exactly 4 points");

         // 1) Sort by Y (ascending). First two are 'top', last two are 'bottom'
         var sortedByY = pts.OrderBy(p => p.Y).ToArray();

         // 2) Split into top and bottom pairs
         var topPair = new[] { sortedByY[0], sortedByY[1] };
         var bottomPair = new[] { sortedByY[2], sortedByY[3] };

         // 3) Within each pair, sort by X to get left/right
         var topSortedByX = topPair.OrderBy(p => p.X).ToArray();
         var bottomSortedByX = bottomPair.OrderBy(p => p.X).ToArray();

         topLeft = topSortedByX[0];
         topRight = topSortedByX[1];
         bottomLeft = bottomSortedByX[0];
         bottomRight = bottomSortedByX[1];
     }

     bool LineLineIntersection(PointF p1, PointF p2, PointF p3, PointF p4, out PointF inter)
     {
         const float EPS = 1e-9f;
         inter = new PointF(float.NaN, float.NaN);

         float r_x = p2.X - p1.X, r_y = p2.Y - p1.Y;   // r = p2 - p1
         float s_x = p4.X - p3.X, s_y = p4.Y - p3.Y;   // s = p4 - p3
         float qp_x = p3.X - p1.X, qp_y = p3.Y - p1.Y; // q - p

         float rxs = Cross(r_x, r_y, s_x, s_y);
         float qpxs = Cross(qp_x, qp_y, s_x, s_y);

         if (Math.Abs(rxs) < EPS)
         {
             // Lines are parallel (and possibly collinear). No unique intersection.
             return false;
         }

         float t = qpxs / rxs;
         inter = new PointF(p1.X + t * r_x, p1.Y + t * r_y);
         return true;
     }

     float Cross(float ax, float ay, float bx, float by) => ax * by - ay * bx;



     int findoutbatteryframecorner(Image<Gray, byte> img, int thr1, int thr2, int thr3, int area, ref PointF[] point_cornors)
     {
         if (img == null) return -1;
         Image<Gray, byte> img_binary1 = new Image<Gray, byte>(img.Width, img.Height);
         Image<Gray, byte> img_binary2 = new Image<Gray, byte>(img.Width, img.Height);
         Image<Gray, byte> img_binary3 = new Image<Gray, byte>(img.Width, img.Height);
         img_binary1 = img.ThresholdBinary(new Gray(thr1), new Gray(255));
         img_binary2 = img.ThresholdBinary(new Gray(thr2), new Gray(255));
         img_binary3 = img.ThresholdBinary(new Gray(thr3), new Gray(255));

         img_binary1 = img_binary1.Erode(8);
         var blobs1 = FindBlobs(img_binary1);


         int count1 = 0;
         int max_index1 = -1;
         int max_value1 = 0;

         int second_max_index1 = -1;
         int second_max_value1 = 0;


         // 3. Visualize results
         //Image<Bgr, byte> vis = srcGray.Convert<Bgr, byte>();
         int index = 0;
         foreach (var b in blobs1)
         {
             if (b.Area > 10000)
             {
                 if (b.Area > max_value1)
                 {
                     max_value1 = b.Area;
                     max_index1 = index;

                 }
                 else
                 {
                     if (b.Area > second_max_value1)
                     {
                         second_max_value1 = b.Area;
                         second_max_index1 = index;
                     }
                 }

             }
             index++;
         }

         Image<Gray, byte> blol_mask1 = new Image<Gray, byte>(img_binary1.Width, img_binary1.Height);
         blol_mask1 = blobs1[max_index1].Mask.ToImage<Gray, byte>();
         blol_mask1 = blol_mask1.Dilate(5);


         VectorOfVectorOfPoint contours1 = new VectorOfVectorOfPoint();
         Mat hierarchy1 = new Mat();
         CvInvoke.FindContours(
             blol_mask1,                      // 8U, 0/255
             contours1,                    // output
             hierarchy1,                   // output
             RetrType.External,           // or RetrType.Tree
             ChainApproxMethod.ChainApproxSimple);



         //// 1) Find contours
         //var contours = new VectorOfVectorOfPoint();
         //using var hierarchy = new Mat();
         //CvInvoke.FindContours(binary, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxNone);
         if (contours1.Size == 0) return -2;

         // Pick largest contour (or choose by your logic)
         int bestIdx1 = 0;
         double bestArea1 = -1;
         for (int ii = 0; ii < contours1.Size; ii++)
         {
             double a = CvInvoke.ContourArea(contours1[ii], false);
             if (a > bestArea1) { bestArea1 = a; bestIdx1 = ii; }
         }
         var contour1 = contours1[bestIdx1];

         // 2) Keep only right-edge points
         var rect1 = CvInvoke.BoundingRectangle(contour1);
         int rightX1 = rect1.Right; // x of the right bbox edge
                                    // Keep points whose X is within the rightmost band (e.g., rightmost 12% of width or at least 6 px)
         int band1 = System.Math.Max(6, (int)System.Math.Round(rect1.Width * 0.12));
         int thresholdX1 = rightX1 - band1;

         var rightPts1 = new List<Point>();
         for (int iiii = 0; iiii < contour1.Size; iiii++)
         {
             var pincontour1 = contour1[iiii];
             if (pincontour1.X >= thresholdX1) rightPts1.Add(pincontour1);
         }
         if (rightPts1.Count < 5) // fallback: take rightmost N points
         {
             var allPts1 = Enumerable.Range(0, contour1.Size).Select(j => contour1[j]);
             rightPts1 = allPts1.OrderByDescending(p => p.X).Take(50).ToList();
         }
         var rightVec1 = new VectorOfPoint(rightPts1.ToArray());

         // 3) First robust fit (FitLine)
         var lineVec1 = new Mat(4, 1, DepthType.Cv32F, 1);
         // DistType.Fair or DistType.Huber are robust; L1 is also OK
         CvInvoke.FitLine(rightVec1, lineVec1, DistType.Fair, 0, 0.01, 0.01);
         float[] d1 = new float[4]; lineVec1.CopyTo(d1);
         float vx1 = d1[0], vy1 = d1[1], x0_1 = d1[2], y0_1 = d1[3];

         // 4) Outlier rejection by distance-to-line (MAD)
         // distance = |v_y*(x-x0) - v_x*(y-y0)| (vx,vy is unit from FitLine)
         var distances1 = new double[rightPts1.Count];
         for (int iii = 0; iii < rightPts1.Count; iii++)
         {
             var p_right1 = rightPts1[iii];
             distances1[iii] = System.Math.Abs(vy1 * (p_right1.X - x0_1) - vx1 * (p_right1.Y - y0_1));
         }
         // Compute median and MAD
         double median1 = Median(distances1);
         double[] absDev1 = distances1.Select(t => System.Math.Abs(t - median1)).ToArray();
         double mad1 = Median(absDev1) + 1e-6;

         // Keep inliers: |d - median| <= k * MAD (k ~ 2.5)
         double k1 = 2.5;
         var inlierPts1 = new List<Point>();
         for (int iiiii = 0; iiiii < rightPts1.Count; iiiii++)
         {
             if (System.Math.Abs(distances1[iiiii] - median1) <= k1 * mad1)
                 inlierPts1.Add(rightPts1[iiiii]);
         }
         if (inlierPts1.Count >= 5)
         {
             // 5) Refit with inliers
             var inliersVec1 = new VectorOfPoint(inlierPts1.ToArray());
             CvInvoke.FitLine(inliersVec1, lineVec1, DistType.Fair, 0, 0.01, 0.01);
             lineVec1.CopyTo(d1);
             vx1 = d1[0]; vy1 = d1[1]; x0_1 = d1[2]; y0_1 = d1[3];
             rightVec1 = inliersVec1; // for return/visualization
         }

         // Build a long drawable line across the image
         //Point[] p = LineAcrossImage(vx, vy, x0, y0, binary.Width, binary.Height);
         Point[] points1 = LineAcrossImage(vx1, vy1, x0_1, y0_1, blol_mask1.Width, blol_mask1.Height);
         var lineSeg1 = new LineSegment2D(points1[0], points1[1]);
         double angleDeg1 = System.Math.Atan2(vy1, vx1) * 180.0 / System.Math.PI;

         //Image<Bgr, byte> vis = blol_mask1.Convert<Bgr, byte>();
         // Optional visualization
         //if (vis != null)
         //{
         //    // mark inliers
         //    foreach (var p_rightVec1 in rightVec1.ToArray())
         //        CvInvoke.Circle(vis, p_rightVec1, 1, new MCvScalar(0, 0, 255), -1);
         //    // draw line
         //    CvInvoke.Line(vis, points1[0], points1[1], new MCvScalar(0, 0, 255), 10);
         //    // bbox for reference
         //    CvInvoke.Rectangle(vis, rect1, new MCvScalar(255, 0, 0), 1);

         //    vis.Save("debug_line1.png");
         //}


         Point left_line_1 = new Point(points1[0].X + 20, points1[0].Y);
         Point left_line_2 = new Point(points1[1].X + 20, points1[1].Y);


         ///////////////////////////////////////////////////////////////////////////////



         img_binary2 = img_binary2.Erode(8);
         var blobs2 = FindBlobs(img_binary2);

         //tb_msg.Text += blobs.Count.ToString() + Environment.NewLine;
         int count2 = 0;
         int max_index2 = -1;
         int max_value2 = 0;

         int second_max_index2 = -1;
         int second_max_value2 = 0;

         // 3. Visualize results
         //Image<Bgr, byte> vis = srcGray.Convert<Bgr, byte>();
         int index2 = 0;
         foreach (var b in blobs2)
         {

             if (b.Area > 10000)
             {
                 if (b.Area > max_value2)
                 {

                     max_value2 = b.Area;
                     max_index2 = index2;

                     //tb_msg.Text += b.Area.ToString() + Environment.NewLine;
                 }
                 else
                 {
                     if (b.Area > second_max_value2)
                     {
                         second_max_value2 = b.Area;
                         second_max_index2 = index2;
                     }
                 }

                 //CvInvoke.Rectangle(vis, b.BBox, new MCvScalar(0, 255, 0), 2);
                 //CvInvoke.Circle(vis, Point.Round(b.Centroid), 3, new MCvScalar(0, 0, 255), -1);
                 //// count++;

             }
             index2++;
         }
         //tb_msg.Text += "max_index = " + max_index.ToString() + Environment.NewLine;
         //tb_msg.Text += "max_value = " + max_value.ToString() + Environment.NewLine;
         //tb_msg.Text += (++count).ToString() + Environment.NewLine;

         Image<Gray, byte> blol_mask2 = new Image<Gray, byte>(img_binary2.Width, img_binary2.Height);
         blol_mask2 = blobs2[second_max_index2].Mask.ToImage<Gray, byte>();
         blol_mask2 = blol_mask2.Dilate(5);
         //pictureBox2.Image = temp.Mat.ToBitmap();


         VectorOfVectorOfPoint contours2 = new VectorOfVectorOfPoint();
         Mat hierarchy2 = new Mat();
         CvInvoke.FindContours(
             blol_mask2,                      // 8U, 0/255
             contours2,                    // output
             hierarchy2,                   // output
             RetrType.External,           // or RetrType.Tree
             ChainApproxMethod.ChainApproxSimple);



         //// 1) Find contours
         //var contours = new VectorOfVectorOfPoint();
         //using var hierarchy = new Mat();
         //CvInvoke.FindContours(binary, contours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxNone);
         if (contours2.Size == 0) return -3;

         // Pick largest contour (or choose by your logic)
         int bestIdx2 = 0;
         double bestArea2 = -1;
         for (int j = 0; j < contours2.Size; j++)
         {
             double a = CvInvoke.ContourArea(contours2[j], false);
             if (a > bestArea2) { bestArea2 = a; bestIdx2 = j; }
         }
         var contour = contours2[bestIdx2];

         // 2) Keep only LEFT-edge points
         var rect = CvInvoke.BoundingRectangle(contour);
         int leftX = rect.Left;                       // x of the left bbox edge
         int band = System.Math.Max(6, (int)System.Math.Round(rect.Width * 0.12));
         int thresholdX = leftX + band;               // left band: [leftX, leftX+band]

         var leftPts = new List<Point>();
         for (int jj = 0; jj < contour.Size; jj++)
         {
             var p_2 = contour[jj];
             if (p_2.X <= thresholdX) leftPts.Add(p_2);   // <= for left band
         }
         if (leftPts.Count < 5) // fallback: take leftmost N points
         {
             var allPts = Enumerable.Range(0, contour.Size).Select(j => contour[j]);
             leftPts = allPts.OrderBy(p => p.X).Take(50).ToList();
         }
         var leftVec = new VectorOfPoint(leftPts.ToArray());

         // 3) First robust fit (FitLine)
         var lineVec = new Mat(4, 1, DepthType.Cv32F, 1);
         CvInvoke.FitLine(leftVec, lineVec, DistType.Fair, 0, 0.01, 0.01);
         float[] d = new float[4]; lineVec.CopyTo(d);
         float vx = d[0], vy = d[1], x0 = d[2], y0 = d[3];

         // 4) Outlier rejection by distance-to-line (MAD)
         var distances = new double[leftPts.Count];
         for (int jjj = 0; jjj < leftPts.Count; jjj++)
         {
             var p22 = leftPts[jjj];
             distances[jjj] = System.Math.Abs(vy * (p22.X - x0) - vx * (p22.Y - y0));
         }
         double median = Median(distances);
         double[] absDev = distances.Select(t => System.Math.Abs(t - median)).ToArray();
         double mad = Median(absDev) + 1e-6;
         double k = 2.5;

         var inlierPts = new List<Point>();
         for (int jjjj = 0; jjjj < leftPts.Count; jjjj++)
         {
             if (System.Math.Abs(distances[jjjj] - median) <= k * mad)
                 inlierPts.Add(leftPts[jjjj]);
         }
         if (inlierPts.Count >= 5)
         {
             var inliersVec = new VectorOfPoint(inlierPts.ToArray());
             CvInvoke.FitLine(inliersVec, lineVec, DistType.Fair, 0, 0.01, 0.01);
             lineVec.CopyTo(d);
             vx = d[0]; vy = d[1]; x0 = d[2]; y0 = d[3];
             leftVec = inliersVec; // keep for visualization
         }

         // Build long drawable line
         Point[] points = LineAcrossImage(vx, vy, x0, y0, blol_mask2.Width, blol_mask2.Height);
         double angleDeg = System.Math.Atan2(vy, vx) * 180.0 / System.Math.PI;

         // Visualization
         //Image<Bgr, byte> vis = m_image_processed2.Convert<Bgr, byte>();

        // vis = vis + blol_mask2.Mat.ToImage<Bgr, byte>();
         //foreach (var p__2 in leftVec.ToArray())
         //    CvInvoke.Circle(vis, p__2, 1, new MCvScalar(0, 0, 255), -1);
         //CvInvoke.Line(vis, points[0], points[1], new MCvScalar(0, 255, 0), 10);
         //CvInvoke.Rectangle(vis, rect, new MCvScalar(255, 0, 0), 1);


         Point right_line_1 = new Point(points[0].X - 20, points[0].Y);
         Point right_line_2 = new Point(points[1].X - 20, points[1].Y);

         /////////////////////////////////////////////////////////////////////////////


         Mat mask = MaskBetweenTwoLines(new Size(img.Width, img.Height), left_line_1, left_line_2, right_line_1, right_line_2);
         //// m_image_processed3 =
         //Image<Bgr, byte> result = m_image_processed3.Copy(mask.ToImage<Gray, byte>());
         //CvInvoke.BitwiseAnd(m_image_processed3, m_image_processed3, m_image_processed3, mask);
         Image<Gray, byte> temp = img_binary3.Copy(mask.ToImage<Gray, byte>());
         //pictureBox5.Image = mask.ToBitmap();

         var blobs = FindBlobs(temp);
         int i = 0;
         int p1_i = 0;
         int p2_i = 0;
         //Image<Bgr, byte> vis = temp.Convert<Bgr, byte>();
         Point[] point1 = new Point[64];
         Point[] point2 = new Point[64];
         foreach (var b in blobs)
         {

             if (b.Area > 350)
             {

                 //CvInvoke.Circle(vis, Point.Round(b.Centroid), 20, new MCvScalar(0, 0, 255), -1);
                 //CvInvoke.Circle(temp, new Point( (int)b.Centroid.X, (int)b.Centroid.Y), 50 , new MCvScalar(0, 255, 0), 1-1);

                 point1[p1_i] = Point.Round(b.Centroid);
                 p1_i++;
             }

         }

         Point[] four_points = new Point[4];
         four_points[0] = point1[0];
         four_points[1] = point1[1];
         four_points[2] = point1[2];
         four_points[3] = point1[3];

         OrderCorners(four_points,
                        out Point topLeft, out Point topRight,
                        out Point bottomLeft, out Point bottomRight);


         Point bottom_line_1 = bottomLeft;
         Point bottom_line_2 = bottomRight;

         Point top_line_1 = topLeft;
         Point top_line_2 = topRight;

         ////////////////////////////////////////////////////////////////////////////////
         PointF p1 = new Point();
         PointF p2 = new Point();
         PointF p3 = new Point();
         PointF p4 = new Point();

         LineLineIntersection(left_line_1, left_line_2, top_line_1, top_line_2, out p1); //top left
         LineLineIntersection(left_line_1, left_line_2, bottom_line_1, bottom_line_2, out p2); //bottom left
         LineLineIntersection(right_line_1, right_line_2, top_line_1, top_line_2, out p3); //top right
         LineLineIntersection(right_line_1, right_line_2, bottom_line_1, bottom_line_2, out p4); //bottom right


         PointF intersetion_left_top = p1;
         PointF intersetion_right_top = p3;
         PointF intersetion_left_bottom = p2;
         PointF intersetion_right_bottom = p4;

         //Image<Bgr, byte> vis = m_image;
         //vis = vis + img.Mat.ToImage<Bgr, byte>();
         //CvInvoke.Line(vis, Point.Round(p1), Point.Round(p2), new MCvScalar(0, 255, 0), 5);
         //CvInvoke.Line(vis, Point.Round(p1), Point.Round(p3), new MCvScalar(0, 255, 0), 5);
         //CvInvoke.Line(vis, Point.Round(p2), Point.Round(p4), new MCvScalar(0, 255, 0), 5);
         //CvInvoke.Line(vis, Point.Round(p3), Point.Round(p4), new MCvScalar(0, 255, 0), 5);


         double L1 = Math.Sqrt((p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y));
         double L2 = Math.Sqrt((p1.X - p3.X) * (p1.X - p3.X) + (p1.Y - p3.Y) * (p1.Y - p3.Y));
         double L3 = Math.Sqrt((p2.X - p4.X) * (p2.X - p4.X) + (p2.Y - p4.Y) * (p2.Y - p4.Y));
         double L4 = Math.Sqrt((p3.X - p4.X) * (p3.X - p4.X) + (p3.Y - p4.Y) * (p3.Y - p4.Y));

         ////tb_msg.Text += L1.ToString + Environment.NewLine;
         //tb_msg.Text += "L1 =" + L1.ToString() + Environment.NewLine;
         //tb_msg.Text += "L2 =" + L2.ToString() + Environment.NewLine;
         //tb_msg.Text += "L3 =" + L3.ToString() + Environment.NewLine;
         //tb_msg.Text += "L4 =" + L4.ToString() + Environment.NewLine;

         //pictureBox6.Image = vis.Mat.ToBitmap();
         //vis.Save("X=" + p1.X.ToString("F2") + "-Y=" + p1.Y.ToString("F2") + ".png");

         point_cornors[0] = intersetion_left_top;
         point_cornors[1] = intersetion_left_bottom;
         point_cornors[2] = intersetion_right_top;
         point_cornors[3] = intersetion_right_bottom;

         //pbox.Image = vis.Mat.ToBitmap();

         return 0;
     }




}

