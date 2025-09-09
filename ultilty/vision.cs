// BlobEdgePipelineV2.cs
// Self-contained pipeline for Emgu CV 4.x
// - Left/Right (LR): threshold ~150, MinArea 10000
// - Top/Bottom (TB): threshold ~120, MinArea 350
// - Trims branches by centroid band, fits inner vertical lines, builds band mask
// - Finds top/bottom 2 inside band, fits horizontal lines, computes intersections
// - Returns a ready-to-show visualization (no extra FindBlobs in your UI)
//
// USAGE (example at bottom).

using System;
using System.Collections.Generic;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace WinFormsApp2
{
    public class BlobInfo
    {
        public int Area { get; set; }
        public Rectangle BBox { get; set; }
        public PointF Centroid { get; set; }
        public int Label { get; set; } = -1;      // connected component id (if used)
        public Mat Mask { get; set; }             // full-frame 0/255 mask
    }

    public class BlobEdgePipelineV2
    {
        // ---------- Tunables ----------
        // Left/Right stage
        public int MinAreaLR { get; set; } = 10000;
        public int LeftRightThreshold { get; set; } = 150;  // around 150
        public int dilateIterationsLR { get; set; } = 3;

        // Top/Bottom stage
        public int MinAreaTB { get; set; } = 300;
        public int TopBottomThreshold { get; set; } = 150;  // around 120
        public int ErodeIterationsTB { get; set; } = 0;

        // Shared
        public int TrimPercent { get; set; } = 15;
        public double BandScale { get; set; } = 0.98;
        public int? SplitXOverride { get; set; } = null;
        public int MinSamplesForFit { get; set; } = 10;

        // ---------- Unified line result ----------
        public struct LineResult
        {
            public bool IsValid;
            public Point P1, P2;

            public bool HasFit;
            public float Vx, Vy, X0, Y0;
            public double Rmse;
            public int N;
            public double AngleDegFromVertical;

            public static LineResult FromFit(float vx, float vy, float x0, float y0, Rectangle clip, List<PointF> samples, double rmse)
            {
                var (p1, p2) = LineEndpointsFromFit(vx, vy, x0, y0, clip);
                return new LineResult
                {
                    IsValid = true, P1 = p1, P2 = p2,
                    HasFit = true, Vx = vx, Vy = vy, X0 = x0, Y0 = y0,
                    Rmse = rmse, N = samples?.Count ?? 0,
                    AngleDegFromVertical = Math.Atan2(vx, vy) * 180.0 / Math.PI
                };
            }

            public static LineResult FromSegment(Point a, Point b)
            {
                float vx = b.X - a.X;
                float vy = b.Y - a.Y;
                return new LineResult
                {
                    IsValid = true, P1 = a, P2 = b,
                    HasFit = false, Vx = vx, Vy = vy, X0 = a.X, Y0 = a.Y,
                    Rmse = double.NaN, N = 0,
                    AngleDegFromVertical = Math.Atan2(vx, vy) * 180.0 / Math.PI
                };
            }

            public bool TryXatY(int y, out int x)
            {
                if (Math.Abs(Vy) > 1e-6)
                {
                    double xx = X0 + (Vx / Vy) * (y - Y0);
                    x = (int)Math.Round(xx);
                    return true;
                }
                if (!IsValid) { x = 0; return false; }
                x = (P1.X + P2.X) / 2;
                return true;
            }

            private static (Point p1, Point p2) LineEndpointsFromFit(float vx, float vy, float x0, float y0, Rectangle clip)
            {
                if (Math.Abs(vy) >= Math.Abs(vx) && Math.Abs(vy) > 1e-6)
                {
                    int y1 = clip.Top, y2 = clip.Bottom - 1;
                    double t1 = (y1 - y0) / vy, t2 = (y2 - y0) / vy;
                    int x1 = (int)Math.Round(x0 + vx * t1);
                    int x2 = (int)Math.Round(x0 + vx * t2);
                    return (new Point(Clamp(x1, clip.Left, clip.Right - 1), y1),
                            new Point(Clamp(x2, clip.Left, clip.Right - 1), y2));
                }
                else
                {
                    int x1 = clip.Left, x2 = clip.Right - 1;
                    double t1 = (x1 - x0) / vx, t2 = (x2 - x0) / vx;
                    int y1 = (int)Math.Round(y0 + vy * t1);
                    int y2 = (int)Math.Round(y0 + vy * t2);
                    return (new Point(x1, Clamp(y1, clip.Top, clip.Bottom - 1)),
                            new Point(x2, Clamp(y2, clip.Top, clip.Bottom - 1)));
                }
            }
        }

        public static bool CompareAverageCornersAndDraw(
        BlobEdgePipelineV2 pipeGolden, string strgoldenGray,
        BlobEdgePipelineV2 pipeCurrent, string strcurrentGray,
        out double X, out double Z,
        out Image<Bgr, byte> visGolden, out Image<Bgr, byte> visCurrent,
        out PointF avgGolden, out PointF avgCurrent)
        {
            Image<Gray, byte> goldenGray = new Image<Gray, byte>(strgoldenGray);
            Image<Gray, byte> currentGray = new Image<Gray, byte>(strcurrentGray);
            return CompareAverageCornersAndDraw(
        pipeGolden, goldenGray,
        pipeCurrent, currentGray,
        out X, out Z,
        out visGolden, out visCurrent,
        out avgGolden, out avgCurrent);
        }

        public static bool CompareAverageCornersAndDraw(
        BlobEdgePipelineV2 pipeGolden, Image<Gray, byte> goldenGray,
        BlobEdgePipelineV2 pipeCurrent, Image<Gray, byte> currentGray,
        out double X, out double Z,
        out Image<Bgr, byte> visGolden, out Image<Bgr, byte> visCurrent,
        out PointF avgGolden, out PointF avgCurrent)
        {
            X = Z = 0;
            avgGolden = avgCurrent = PointF.Empty;
            visGolden = null; visCurrent = null;

            if (goldenGray == null || currentGray == null) return false;

            // ---------- Run on GOLDEN ----------
            RunOnce(pipeGolden, goldenGray,
                out visGolden, out var gCornersOk, out var gLT, out var gLB, out var gRT, out var gRB);

            var goldPts = CollectValid(gCornersOk, gLT, gLB, gRT, gRB);
            if (goldPts.Count == 0) return false;
            avgGolden = Average(goldPts);

            // ---------- Run on CURRENT ----------
            RunOnce(pipeCurrent, currentGray,
                out visCurrent, out var cCornersOk, out var cLT, out var cLB, out var cRT, out var cRB);

            var currPts = CollectValid(cCornersOk, cLT, cLB, cRT, cRB);
            if (currPts.Count == 0) return false;
            avgCurrent = Average(currPts);

            // ---------- Delta ----------
            X = avgGolden.X - avgCurrent.X;
            Z = avgGolden.Y - avgCurrent.Y;

            // ---------- Draw overlays ----------
            // Colors
            MCvScalar colCorner = new MCvScalar(255, 0, 255); // magenta
            MCvScalar colAvgG = new MCvScalar(255, 255, 0); // cyan-like (BGR: 0,255,255)
            MCvScalar colAvgC = new MCvScalar(0, 0, 255);   // red
            MCvScalar colText = new MCvScalar(255, 255, 255);

            // Golden: draw valid corners + avg
            DrawCorners(visGolden, gCornersOk, gLT, gLB, gRT, gRB, colCorner);
            DrawAvgDot(visGolden, avgGolden, colAvgG, "G");

            // Current: draw valid corners + avg
            DrawCorners(visCurrent, cCornersOk, cLT, cLB, cRT, cRB, colCorner);
            DrawAvgDot(visCurrent, avgCurrent, colAvgC, "C");

            // Arrow Current → Golden on CURRENT image (so you see how far to move)
            DrawArrowAndDelta(visCurrent, avgCurrent, avgGolden, X, Z, colAvgC, colAvgG, colText);

            return true;
        }

        // ---- helpers (local to this utility) ----
        private static void RunOnce(
            BlobEdgePipelineV2 pipe, Image<Gray, byte> gray,
            out Image<Bgr, byte> vis,
            out (bool LT, bool LB, bool RT, bool RB) ok,
            out Point LT, out Point LB, out Point RT, out Point RB)
        {
            // many outs we don't need here
            pipe.Run(
                srcGray: gray,
                out vis,
                out Mat binLR,
                out System.Collections.Generic.List<BlobInfo> blobsLR,
                out int leftIdx,
                out int rightIdx,
                out Mat cleanLeft,
                out Mat cleanRight,
                out var leftLine,
                out var rightLine,
                out Mat bandMask,
                out Mat binBand,
                out System.Collections.Generic.List<BlobInfo> top2,
                out System.Collections.Generic.List<BlobInfo> bottom2,
                out Mat topUnion,
                out Mat bottomUnion,
                out var topLine,
                out var bottomLine,
                out bool okLT, out bool okLB, out bool okRT, out bool okRB,
                out LT, out LB, out RT, out RB
            );
            ok = (okLT, okLB, okRT, okRB);
            // dispose extra mats you don't use further here
            binLR?.Dispose(); bandMask?.Dispose(); binBand?.Dispose();
            topUnion?.Dispose(); bottomUnion?.Dispose();
            cleanLeft?.Dispose(); cleanRight?.Dispose();
        }

        private static List<Point> CollectValid((bool LT, bool LB, bool RT, bool RB) ok,
                                                Point LT, Point LB, Point RT, Point RB)
        {
            var pts = new List<Point>(4);
            if (ok.LT) pts.Add(LT);
            if (ok.LB) pts.Add(LB);
            if (ok.RT) pts.Add(RT);
            if (ok.RB) pts.Add(RB);
            return pts;
        }

        private static PointF Average(IList<Point> pts)
        {
            double sx = 0, sy = 0;
            for (int i = 0; i < pts.Count; i++) { sx += pts[i].X; sy += pts[i].Y; }
            return new PointF((float)(sx / pts.Count), (float)(sy / pts.Count));
        }

        private static void DrawCorners(Image<Bgr, byte> vis,
                                        (bool LT, bool LB, bool RT, bool RB) ok,
                                        Point LT, Point LB, Point RT, Point RB,
                                        MCvScalar col)
        {
            if (vis == null) return;
            void Dot(Point p, string label)
            {
                CvInvoke.Circle(vis, p, 7, col, -1);
                CvInvoke.PutText(vis, label, new Point(p.X + 6, p.Y - 6),
                    FontFace.HersheySimplex, 0.7, col, 2);
            }
            if (ok.LT) Dot(LT, "LT");
            if (ok.LB) Dot(LB, "LB");
            if (ok.RT) Dot(RT, "RT");
            if (ok.RB) Dot(RB, "RB");
        }

        private static void DrawAvgDot(Image<Bgr, byte> vis, PointF avg, MCvScalar col, string tag)
        {
            if (vis == null) return;
            var p = new Point((int)Math.Round(avg.X), (int)Math.Round(avg.Y));
            CvInvoke.Circle(vis, p, 9, col, -1);
            CvInvoke.Circle(vis, p, 12, col, 2);
            CvInvoke.PutText(vis, tag, new Point(p.X + 8, p.Y - 8),
                FontFace.HersheySimplex, 0.8, col, 2);
        }

        private static void DrawArrowAndDelta(Image<Bgr, byte> vis,
                                              PointF fromAvg, PointF toAvg,
                                              double dX, double dY,
                                              MCvScalar colFrom, MCvScalar colTo, MCvScalar colText)
        {
            if (vis == null) return;

            var pFrom = new Point((int)Math.Round(fromAvg.X), (int)Math.Round(fromAvg.Y));
            var pTo = new Point((int)Math.Round(toAvg.X), (int)Math.Round(toAvg.Y));

            // arrow from current → golden
            try { CvInvoke.ArrowedLine(vis, pFrom, pTo, new MCvScalar(0, 255, 255), 3, LineType.AntiAlias, 0, 0.2); }
            catch { CvInvoke.Line(vis, pFrom, pTo, new MCvScalar(0, 255, 255), 3); }

            // small info box near the mid-point
            int mx = (pFrom.X + pTo.X) / 2;
            int my = (pFrom.Y + pTo.Y) / 2;
            string txt = $"ΔX={dX:F2}, ΔY={dY:F2}";
            CvInvoke.PutText(vis, txt, new Point(mx + 10, my - 10),
                FontFace.HersheySimplex, 0.8, colText, 2);
        }

        // ---------- Public Run ----------
        public void Run(
            Image<Gray, byte> srcGray,
            out Image<Bgr, byte> vis,
            out Mat binLR,
            out List<BlobInfo> blobsLR,
            out int leftIdx,
            out int rightIdx,
            out Mat cleanLeft,
            out Mat cleanRight,
            out LineResult leftLine,
            out LineResult rightLine,
            out Mat bandMask,
            out Mat binBand,
            out List<BlobInfo> top2,
            out List<BlobInfo> bottom2,
            out Mat topUnion,
            out Mat bottomUnion,
            out LineResult topLine,
            out LineResult bottomLine,
            out bool okLT, out bool okLB, out bool okRT, out bool okRB,
            out Point LT, out Point LB, out Point RT, out Point RB)
        {
            if (srcGray == null) throw new ArgumentNullException(nameof(srcGray));
            vis = srcGray.Convert<Bgr, byte>();

            // ---------- Stage A: Left/Right threshold ~150 ----------
            binLR = new Mat(srcGray.Size, DepthType.Cv8U, 1);
            CvInvoke.Threshold(srcGray, binLR, LeftRightThreshold, 255, ThresholdType.Binary);
            if (dilateIterationsLR > 0)
                CvInvoke.Dilate(binLR, binLR, null, new Point(-1, -1), dilateIterationsLR, BorderType.Default, new MCvScalar(0));

            blobsLR = FindBlobsFromBinary(binLR, MinAreaLR, out _labelsCacheLR);
            leftIdx = rightIdx = -1;
            cleanLeft = new Mat(srcGray.Size, DepthType.Cv8U, 1); cleanLeft.SetTo(new MCvScalar(0));
            cleanRight = new Mat(srcGray.Size, DepthType.Cv8U, 1); cleanRight.SetTo(new MCvScalar(0));
            leftLine = new LineResult();
            rightLine = new LineResult();

            if (blobsLR.Count > 0)
            {
                int midX = SplitXOverride ?? (srcGray.Width / 2);
                int leftArea = 0, rightArea = 0;
                for (int i = 0; i < blobsLR.Count; i++)
                {
                    var b = blobsLR[i];
                    int cx = (int)Math.Round(b.Centroid.X);
                    if (cx < midX)
                    {
                        if (b.Area > leftArea) { leftArea = b.Area; leftIdx = i; }
                    }
                    else
                    {
                        if (b.Area > rightArea) { rightArea = b.Area; rightIdx = i; }
                    }
                }

                if (leftIdx >= 0)
                {
                    cleanLeft = CleanBlobByCentroidBand(binLR, blobsLR[leftIdx], TrimPercent, BandScale, _labelsCacheLR);
                    leftLine = FitBlobInnerEdge(cleanLeft, blobsLR[leftIdx].BBox, takeRightEdge: true);
                }
                if (rightIdx >= 0)
                {
                    cleanRight = CleanBlobByCentroidBand(binLR, blobsLR[rightIdx], TrimPercent, BandScale, _labelsCacheLR);
                    rightLine = FitBlobInnerEdge(cleanRight, blobsLR[rightIdx].BBox, takeRightEdge: false);
                }
            }

            // ---------- Stage B: Band between lines ----------
            bandMask = BuildBandMaskBetweenLines(leftLine, rightLine, srcGray.Width, srcGray.Height, 30, 80, 10, 100);

            // ---------- Stage C: Top/Bottom threshold ~120 inside band ----------
            Mat binTB = new Mat(srcGray.Size, DepthType.Cv8U, 1);
            CvInvoke.Threshold(srcGray, binTB, TopBottomThreshold, 255, ThresholdType.Binary);
            if (ErodeIterationsTB > 0)
                CvInvoke.Erode(binTB, binTB, null, new Point(-1, -1), ErodeIterationsTB, BorderType.Default, new MCvScalar(0));

            binBand = new Mat(srcGray.Size, DepthType.Cv8U, 1);
            CvInvoke.BitwiseAnd(binTB, bandMask, binBand);

            var blobsBand = FindBlobsFromBinary(binBand, MinAreaTB, out _labelsCacheTB);

            int midY = srcGray.Height / 2;
            var topAll = new List<BlobInfo>();
            var botAll = new List<BlobInfo>();
            foreach (var bb in blobsBand)
            {
                if (bb.Centroid.Y < midY) topAll.Add(bb);
                else                      botAll.Add(bb);
            }
            topAll.Sort((a, b) => b.Area.CompareTo(a.Area));
            botAll.Sort((a, b) => b.Area.CompareTo(a.Area));

            top2 = new List<BlobInfo>(2);
            bottom2 = new List<BlobInfo>(2);
            if (topAll.Count > 0) top2.Add(topAll[0]);
            if (topAll.Count > 1) top2.Add(topAll[1]);
            if (botAll.Count > 0) bottom2.Add(botAll[0]);
            if (botAll.Count > 1) bottom2.Add(botAll[1]);

            topUnion = new Mat(srcGray.Size, DepthType.Cv8U, 1); topUnion.SetTo(new MCvScalar(0));
            bottomUnion = new Mat(srcGray.Size, DepthType.Cv8U, 1); bottomUnion.SetTo(new MCvScalar(0));
            foreach (var t in top2)   CvInvoke.BitwiseOr(topUnion, t.Mask, topUnion);
            foreach (var btm in bottom2) CvInvoke.BitwiseOr(bottomUnion, btm.Mask, bottomUnion);

            // Fit horizontal lines
            //topLine    = FitHorizontalLineFromMask(topUnion,   preferTop: true,  fullSize: srcGray.Size);
            //bottomLine = FitHorizontalLineFromMask(bottomUnion, preferTop: false, fullSize: srcGray.Size);
            topLine = FitHorizontalLineFromBlobs(top2, useBottomEdge: true, fullSize: srcGray.Size, minSamplesForFit: MinSamplesForFit);
            bottomLine = FitHorizontalLineFromBlobs(bottom2, useBottomEdge: false, fullSize: srcGray.Size, minSamplesForFit: MinSamplesForFit);
            // ---------- Intersections ----------
            okLT = okLB = okRT = okRB = false;
            LT = LB = RT = RB = Point.Empty;
            if (leftLine.IsValid && topLine.IsValid)    okLT = LineIntersection(leftLine.P1,  leftLine.P2,  topLine.P1,    topLine.P2,    out LT);
            if (leftLine.IsValid && bottomLine.IsValid) okLB = LineIntersection(leftLine.P1,  leftLine.P2,  bottomLine.P1, bottomLine.P2, out LB);
            if (rightLine.IsValid && topLine.IsValid)   okRT = LineIntersection(rightLine.P1, rightLine.P2, topLine.P1,    topLine.P2,    out RT);
            if (rightLine.IsValid && bottomLine.IsValid)okRB = LineIntersection(rightLine.P1, rightLine.P2, bottomLine.P1, bottomLine.P2, out RB);

            // ---------- Visualization ----------
            DrawVisualization(
                vis,
                cleanLeft, cleanRight,
                leftIdx, rightIdx, blobsLR,
                top2, bottom2, topUnion, bottomUnion,
                leftLine, rightLine, topLine, bottomLine,
                okLT, okLB, okRT, okRB, LT, LB, RT, RB);
        }

        // Label caches for last CC calls (LR and TB separately)
        Mat _labelsCacheLR;
        Mat _labelsCacheTB;

        List<BlobInfo> FindBlobsFromBinary(Mat bin, int minArea, out Mat labelsOut)
        {
            // Ensure binary is 0/255
            using (Mat tmp = new Mat())
            {
                CvInvoke.Threshold(bin, tmp, 0, 255, ThresholdType.Binary);
                tmp.CopyTo(bin);
            }

            Mat labels = new Mat();
            Mat stats = new Mat();
            Mat centroids = new Mat();

            int nLabels = CvInvoke.ConnectedComponentsWithStats(
                bin, labels, stats, centroids, LineType.EightConnected, DepthType.Cv32S);

            // Read stats / centroids into typed arrays for easy indexing
            var statsArr = (int[,])stats.GetData();
            var centArr  = (double[,])centroids.GetData();

            var list = new List<BlobInfo>(Math.Max(0, nLabels - 1));
            for (int lbl = 1; lbl < nLabels; lbl++) // skip background 0
            {
                int x    = statsArr[lbl, (int)ConnectedComponentsTypes.Left];
                int y    = statsArr[lbl, (int)ConnectedComponentsTypes.Top];
                int w    = statsArr[lbl, (int)ConnectedComponentsTypes.Width];
                int h    = statsArr[lbl, (int)ConnectedComponentsTypes.Height];
                int area = statsArr[lbl, (int)ConnectedComponentsTypes.Area];
                if (area < minArea) continue;

                double cx = centArr[lbl, 0];
                double cy = centArr[lbl, 1];

                // Full-frame mask for this label
                Mat mask = BuildMaskFromLabelImage(lbl, labels, bin.Size);

                list.Add(new BlobInfo
                {
                    Area = area,
                    BBox = new Rectangle(x, y, w, h),
                    Centroid = new PointF((float)cx, (float)cy),
                    Label = lbl,
                    Mask = mask
                });
            }

            // sort desc by area (optional)
            list.Sort((a, b) => b.Area.CompareTo(a.Area));
            labelsOut = labels; // return labels so caller can cache
            return list;
        }

        Mat BuildMaskFromLabelImage(int labelId, Mat labels, Size size)
        {
            Mat mask = new Mat(size, DepthType.Cv8U, 1);
            CvInvoke.Compare(labels, new ScalarArray(labelId), mask, CmpType.Equal);
            CvInvoke.Threshold(mask, mask, 0, 255, ThresholdType.Binary);
            return mask;
        }

        // ---- Blob trimming by centroid band ----
        Mat CleanBlobByCentroidBand(Mat binary, BlobInfo blob, int trimPercent, double bandScale, Mat labelsForThisBlobSet)
        {
            Mat blobMask = blob.Mask ?? BuildMaskFromLabelImage(blob.Label, labelsForThisBlobSet, binary.Size);

            Rectangle bbox = blob.BBox;
            int cx = (int)Math.Round(blob.Centroid.X);
            cx = Math.Max(0, Math.Min(cx, binary.Cols - 1));
            List<int> halfWidths = new List<int>(Math.Max(1, bbox.Height));
            using (Mat rowVec = new Mat(1, binary.Cols, DepthType.Cv8U, 1))
            using (VectorOfPoint nz = new VectorOfPoint())
            {
                Rectangle B = Rectangle.Intersect(bbox, new Rectangle(0, 0, blobMask.Cols, blobMask.Rows));
                for (int y = B.Top; y < B.Bottom; y++)
                {
                    using (Mat row = new Mat(blobMask, new Rectangle(0, y, blobMask.Cols, 1)))
                    {
                        row.CopyTo(rowVec);
                        CvInvoke.FindNonZero(rowVec, nz);
                        if (nz.Size == 0) continue;

                        int minX = int.MaxValue, maxX = int.MinValue;
                        for (int k = 0; k < nz.Size; k++)
                        {
                            int x = nz[k].X;
                            if (x < B.Left || x >= B.Right) continue;
                            if (x < minX) minX = x;
                            if (x > maxX) maxX = x;
                        }
                        if (minX == int.MaxValue || maxX == int.MinValue) continue;

                        int leftGap  = cx - minX;
                        int rightGap = maxX - cx;
                        if (leftGap > 0 && rightGap > 0)
                            halfWidths.Add(Math.Min(leftGap, rightGap));
                    }
                }
            }

            if (halfWidths.Count == 0) return blobMask;

            int H = RobustHalfWidth(halfWidths, trimPercent);
            H = Math.Max(1, (int)Math.Round(H * bandScale));

            Mat bandMask = new Mat(binary.Size, DepthType.Cv8U, 1); bandMask.SetTo(new MCvScalar(0));
            int x1 = Math.Max(0, cx - H);
            int x2 = Math.Min(binary.Cols, cx + H + 1);
            Rectangle bandRect = Rectangle.FromLTRB(x1, bbox.Top, x2, bbox.Bottom);
            CvInvoke.Rectangle(bandMask, bandRect, new MCvScalar(255), -1);

            Mat cleaned = new Mat();
            CvInvoke.BitwiseAnd(blobMask, bandMask, cleaned);
            return cleaned;
        }

        int RobustHalfWidth(List<int> halfWidths, int trimPercent)
        {
            halfWidths.Sort();
            int n = halfWidths.Count;
            int drop = (int)Math.Round(n * (trimPercent / 100.0));
            int lo = Math.Min(n - 1, drop);
            int hi = Math.Max(lo, n - 1 - drop);
            long sum = 0; int cnt = 0;
            for (int i = lo; i <= hi; i++) { sum += halfWidths[i]; cnt++; }
            if (cnt == 0) return Math.Max(1, halfWidths[n / 2]);
            return (int)Math.Round(sum / (double)cnt);
        }

        // ---- Fit left/right inner edges ----
        LineResult FitBlobInnerEdge(Mat cleanedMask, Rectangle bbox, bool takeRightEdge)
        {
            var pts = ExtractEdgeSamples(cleanedMask, bbox, takeRightEdge);
            if (pts.Count < MinSamplesForFit) return FitByMinAreaRect(cleanedMask, bbox, takeRightEdge);

            using (var vpf = new VectorOfPointF(pts.ToArray()))
            using (var line = new Mat(4, 1, DepthType.Cv32F, 1))
            {
                CvInvoke.FitLine(vpf, line, DistType.Huber, 0, 0.01, 0.01);

                float[] p = new float[4]; line.CopyTo(p);
                float vx = p[0], vy = p[1], x0 = p[2], y0 = p[3];

                double denom = Math.Sqrt(vx * vx + vy * vy);
                if (denom < 1e-6) return FitByMinAreaRect(cleanedMask, bbox, takeRightEdge);

                double rmse = ComputeRMSE(pts, vx, vy, x0, y0);
                return LineResult.FromFit(vx, vy, x0, y0, bbox, pts, rmse);
            }
        }

        LineResult FitByMinAreaRect(Mat cleanedMask, Rectangle bbox, bool takeRightEdge)
        {
            Rectangle B = Rectangle.Intersect(bbox, new Rectangle(0, 0, cleanedMask.Cols, cleanedMask.Rows));
            using var roi = new Mat(cleanedMask, B);
            using var bin = new Mat();
            CvInvoke.Threshold(roi, bin, 1, 255, ThresholdType.Binary);

            using var contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(bin, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return new LineResult { IsValid = false };

            int best = 0; double bestArea = 0;
            for (int i = 0; i < contours.Size; i++)
            {
                double a = CvInvoke.ContourArea(contours[i]);
                if (a > bestArea) { bestArea = a; best = i; }
            }

            var rect = CvInvoke.MinAreaRect(contours[best]);
            PointF[] box = rect.GetVertices();
            for (int i = 0; i < box.Length; i++)
                box[i] = new PointF(box[i].X + B.Left, box[i].Y + B.Top);

            var edges = new (PointF A, PointF B)[]
            {
                (box[0], box[1]), (box[1], box[2]), (box[2], box[3]), (box[3], box[0])
            };

            var verticalIdx = new List<int>(2);
            for (int i = 0; i < edges.Length; i++)
            {
                var e = edges[i];
                double dx = e.B.X - e.A.X, dy = e.B.Y - e.A.Y;
                if (Math.Abs(dy) >= Math.Abs(dx)) verticalIdx.Add(i);
            }
            if (verticalIdx.Count == 0)
            {
                Array.Sort(edges, (u, v) => Math.Abs(v.B.Y - v.A.Y).CompareTo(Math.Abs(u.B.Y - u.A.Y)));
                verticalIdx.Add(0); verticalIdx.Add(1);
            }

            int chosen = -1;
            double bestX = takeRightEdge ? double.NegativeInfinity : double.PositiveInfinity;
            foreach (int idx in verticalIdx)
            {
                var e = edges[idx];
                double mx = (e.A.X + e.B.X) * 0.5;
                if (takeRightEdge) { if (mx > bestX) { bestX = mx; chosen = idx; } }
                else               { if (mx < bestX) { bestX = mx; chosen = idx; } }
            }
            if (chosen < 0) return new LineResult { IsValid = false };

            var E = edges[chosen];
            var p1 = new Point((int)Math.Round(E.A.X), (int)Math.Round(E.A.Y));
            var p2 = new Point((int)Math.Round(E.B.X), (int)Math.Round(E.B.Y));
            p1 = new Point(Clamp(p1.X, bbox.Left, bbox.Right - 1), Clamp(p1.Y, bbox.Top, bbox.Bottom - 1));
            p2 = new Point(Clamp(p2.X, bbox.Left, bbox.Right - 1), Clamp(p2.Y, bbox.Top, bbox.Bottom - 1));

            return LineResult.FromSegment(p1, p2);
        }

        List<PointF> ExtractEdgeSamples(Mat mask, Rectangle bbox, bool takeRightEdge)
        {
            var pts = new List<PointF>(bbox.Height);
            Rectangle B = Rectangle.Intersect(bbox, new Rectangle(0, 0, mask.Cols, mask.Rows));
            if (B.Width <= 0 || B.Height <= 0) return pts;

            using Mat rowBuf = new Mat(1, B.Width, DepthType.Cv8U, 1);
            using VectorOfPoint nz = new VectorOfPoint();

            for (int y = B.Top; y < B.Bottom; y++)
            {
                using Mat row = new Mat(mask, new Rectangle(B.Left, y, B.Width, 1));
                row.CopyTo(rowBuf);
                CvInvoke.FindNonZero(rowBuf, nz);
                if (nz.Size == 0) continue;

                int bestLocalX = takeRightEdge ? int.MinValue : int.MaxValue;
                for (int i = 0; i < nz.Size; i++)
                {
                    int xLocal = nz[i].X;
                    if (takeRightEdge) { if (xLocal > bestLocalX) bestLocalX = xLocal; }
                    else               { if (xLocal < bestLocalX) bestLocalX = xLocal; }
                }
                if (bestLocalX == int.MinValue || bestLocalX == int.MaxValue) continue;

                int xAbs = B.Left + bestLocalX;
                pts.Add(new PointF(xAbs, y));
            }
            return pts;
        }

        double ComputeRMSE(List<PointF> pts, float vx, float vy, float x0, float y0)
        {
            double denom = Math.Sqrt(vx * vx + vy * vy);
            if (denom < 1e-9) return double.NaN;

            double sum2 = 0;
            for (int i = 0; i < pts.Count; i++)
            {
                double dx = pts[i].X - x0;
                double dy = pts[i].Y - y0;
                double cross = Math.Abs(dx * vy - dy * vx);
                double d = cross / denom;
                sum2 += d * d;
            }
            return Math.Sqrt(sum2 / Math.Max(1, pts.Count));
        }
        LineResult FitHorizontalLineFromBlobs(
    IList<BlobInfo> blobs, bool useBottomEdge, Size fullSize, int minSamplesForFit = 10)
        {
            if (blobs == null || blobs.Count == 0) return new LineResult { IsValid = false };

            // Build union mask from provided blobs (0/255)
            using Mat union = new Mat(fullSize, DepthType.Cv8U, 1);
            union.SetTo(new MCvScalar(0));
            Rectangle urect = Rectangle.Empty;

            foreach (var b in blobs)
            {
                if (b?.Mask == null || b.Mask.IsEmpty) continue;
                CvInvoke.BitwiseOr(union, b.Mask, union);
                urect = urect == Rectangle.Empty ? b.BBox : Rectangle.Union(urect, b.BBox);
            }
            if (urect == Rectangle.Empty) return new LineResult { IsValid = false };

            urect = Rectangle.Intersect(urect, new Rectangle(0, 0, fullSize.Width, fullSize.Height));
            if (urect.Width <= 1 || urect.Height <= 1) return new LineResult { IsValid = false };

            // Collect edge samples: one (x,y) per column where union has pixels
            var samples = new List<PointF>(urect.Width);

            using Mat colBuf = new Mat(urect.Height, 1, DepthType.Cv8U, 1);
            using VectorOfPoint nz = new VectorOfPoint();

            for (int x = urect.Left; x < urect.Right; x++)
            {
                using Mat col = new Mat(union, new Rectangle(x, urect.Top, 1, urect.Height));
                col.CopyTo(colBuf);
                CvInvoke.FindNonZero(colBuf, nz);
                if (nz.Size == 0) continue;

                int y; // y in absolute image coords
                if (useBottomEdge)
                {
                    // max Y (lowest pixel in this column)
                    int maxY = int.MinValue;
                    for (int i = 0; i < nz.Size; i++)
                        if (nz[i].Y > maxY) maxY = nz[i].Y;
                    y = urect.Top + maxY;
                }
                else
                {
                    // min Y (highest pixel in this column)
                    int minY = int.MaxValue;
                    for (int i = 0; i < nz.Size; i++)
                        if (nz[i].Y < minY) minY = nz[i].Y;
                    y = urect.Top + minY;
                }
                samples.Add(new PointF(x, y));
            }

            if (samples.Count < minSamplesForFit)
            {
                // Fallback: reuse your previous mask-based method favoring top/bottom
                // preferTop == !useBottomEdge  (top edge => preferTop=true; bottom edge => preferTop=false)
                return FitHorizontalLineFromMask(union, preferTop: !useBottomEdge, fullSize: fullSize);
            }

            // Robust line fit (Huber) on the edge samples
            using var vpf = new Emgu.CV.Util.VectorOfPointF(samples.ToArray());
            using var line = new Mat(4, 1, DepthType.Cv32F, 1);
            CvInvoke.FitLine(vpf, line, DistType.Huber, 0, 0.01, 0.01);

            float[] p = new float[4];
            line.CopyTo(p);
            float vx = p[0], vy = p[1], x0 = p[2], y0 = p[3];

            // Build result clipped to the full image
            return LineResult.FromFit(vx, vy, x0, y0, new Rectangle(0, 0, fullSize.Width, fullSize.Height), samples, rmse: double.NaN);
        }
        LineResult FitHorizontalLineFromMask(Mat mask, bool preferTop, Size fullSize)
        {
            if (mask == null || mask.IsEmpty) return new LineResult { IsValid = false };

            using var contours = new VectorOfVectorOfPoint();
            using var tmp = new Mat();
            CvInvoke.Threshold(mask, tmp, 1, 255, ThresholdType.Binary);
            CvInvoke.FindContours(tmp, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0) return new LineResult { IsValid = false };

            // pick largest contour
            int best = 0; double bestArea = 0;
            for (int i = 0; i < contours.Size; i++)
            {
                double a = CvInvoke.ContourArea(contours[i]);
                if (a > bestArea) { bestArea = a; best = i; }
            }

            var rect = CvInvoke.MinAreaRect(contours[best]);
            PointF[] box = rect.GetVertices();

            // choose the more horizontal edge (or prefer top/bottom by Y)
            int idx1 = 0, idx2 = 1, idx3 = 2, idx4 = 3;
            var e1 = (A: box[0], B: box[1]);
            var e2 = (A: box[1], B: box[2]);
            var e3 = (A: box[2], B: box[3]);
            var e4 = (A: box[3], B: box[0]);
            var edges = new[] { e1, e2, e3, e4 };

            int chosen = -1;
            double bestScore = double.NegativeInfinity;
            for (int i = 0; i < edges.Length; i++)
            {
                var e = edges[i];
                double dx = e.B.X - e.A.X, dy = e.B.Y - e.A.Y;
                double horizScore = Math.Abs(dx) - Math.Abs(dy); // prefer horizontal
                // refine by top/bottom preference
                double meanY = (e.A.Y + e.B.Y) * 0.5;
                double pref = preferTop ? -meanY : meanY;
                double score = horizScore * 1000.0 + pref;
                if (score > bestScore) { bestScore = score; chosen = i; }
            }

            var E = edges[chosen];
            Point p1 = new Point(Clamp((int)Math.Round(E.A.X), 0, fullSize.Width - 1),
                                  Clamp((int)Math.Round(E.A.Y), 0, fullSize.Height - 1));
            Point p2 = new Point(Clamp((int)Math.Round(E.B.X), 0, fullSize.Width - 1),
                                  Clamp((int)Math.Round(E.B.Y), 0, fullSize.Height - 1));

            // extend to full width for nicer drawing
            if (Math.Abs(p2.X - p1.X) < 1)
            {
                int y = Clamp((int)Math.Round(0.5 * (p1.Y + p2.Y)), 0, fullSize.Height - 1);
                return LineResult.FromSegment(new Point(0, y), new Point(fullSize.Width - 1, y));
            }
            else
            {
                double m = (p2.Y - p1.Y) / (double)(p2.X - p1.X);
                int y0 = Clamp((int)Math.Round(p1.Y + m * (0 - p1.X)), 0, fullSize.Height - 1);
                int yW = Clamp((int)Math.Round(p1.Y + m * (fullSize.Width - 1 - p1.X)), 0, fullSize.Height - 1);
                return LineResult.FromSegment(new Point(0, y0), new Point(fullSize.Width - 1, yW));
            }
        }

        Mat BuildBandMaskBetweenLines(BlobEdgePipelineV2.LineResult left,
                              BlobEdgePipelineV2.LineResult right,
                              int W, int H,
                              int insetPx = 0,
                              int blockCenterHalfHeight = 0,
                              int blockTopPx = 0,
                              int blockBottomPx = 0)
        {
            Mat mask = new Mat(new Size(W, H), DepthType.Cv8U, 1);
            mask.SetTo(new MCvScalar(0));
            if (!left.IsValid || !right.IsValid) return mask;

            // Sample line x at top/bottom
            if (!left.TryXatY(0, out int xL0)) return mask;
            if (!left.TryXatY(H - 1, out int xL1)) return mask;
            if (!right.TryXatY(0, out int xR0)) return mask;
            if (!right.TryXatY(H - 1, out int xR1)) return mask;

            // Safe insets (avoid crossing)
            int gapTop = xR0 - xL0;
            int gapBot = xR1 - xL1;
            if (gapTop <= 0 && gapBot <= 0) return mask;

            int insetTop = Math.Min(insetPx, Math.Max(0, (gapTop - 1) / 2));
            int insetBot = Math.Min(insetPx, Math.Max(0, (gapBot - 1) / 2));

            xL0 += insetTop; xR0 -= insetTop;
            xL1 += insetBot; xR1 -= insetBot;

            // Clamp & validate
            xL0 = Clamp(xL0, 0, W - 1);
            xL1 = Clamp(xL1, 0, W - 1);
            xR0 = Clamp(xR0, 0, W - 1);
            xR1 = Clamp(xR1, 0, W - 1);
            if (xL0 >= xR0 || xL1 >= xR1) return mask;

            // Fill band quad
            var poly = new Point[]
            {
        new Point(xL0, 0),
        new Point(xR0, 0),
        new Point(xR1, H - 1),
        new Point(xL1, H - 1)
            };
            using (var vpoly = new Emgu.CV.Util.VectorOfPoint(poly))
                CvInvoke.FillConvexPoly(mask, vpoly, new MCvScalar(255), LineType.AntiAlias, 0);

            // Build a single block mask that unions all "no-pass" stripes, then subtract it
            bool needBlocks = blockCenterHalfHeight > 0 || blockTopPx > 0 || blockBottomPx > 0;
            if (needBlocks)
            {
                using Mat block = new Mat(new Size(W, H), DepthType.Cv8U, 1);
                block.SetTo(new MCvScalar(0));

                // Center stripe
                if (blockCenterHalfHeight > 0)
                {
                    int cy = H / 2;
                    int y0 = Math.Max(0, cy - blockCenterHalfHeight);
                    int y1 = Math.Min(H - 1, cy + blockCenterHalfHeight);
                    if (y1 >= y0)
                        CvInvoke.Rectangle(block, Rectangle.FromLTRB(0, y0, W, y1 + 1), new MCvScalar(255), -1);
                }

                // Top margin
                if (blockTopPx > 0)
                {
                    int y0 = 0;
                    int y1 = Math.Min(H - 1, blockTopPx - 1);
                    if (y1 >= y0)
                        CvInvoke.Rectangle(block, Rectangle.FromLTRB(0, y0, W, y1 + 1), new MCvScalar(255), -1);
                }

                // Bottom margin
                if (blockBottomPx > 0)
                {
                    int y0 = Math.Max(0, H - blockBottomPx);
                    int y1 = H - 1;
                    if (y1 >= y0)
                        CvInvoke.Rectangle(block, Rectangle.FromLTRB(0, y0, W, y1 + 1), new MCvScalar(255), -1);
                }

                using Mat keep = new Mat();
                CvInvoke.BitwiseNot(block, block);     // block := NOT(block)
                CvInvoke.BitwiseAnd(mask, block, keep); // keep = mask & ~block
                keep.CopyTo(mask);
            }

            return mask;
        }

        public static bool LineIntersection(Point p1, Point p2, Point q1, Point q2, out Point inter)
        {
            inter = Point.Empty;
            double x1 = p1.X, y1 = p1.Y, x2 = p2.X, y2 = p2.Y;
            double x3 = q1.X, y3 = q1.Y, x4 = q2.X, y4 = q2.Y;

            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.Abs(denom) < 1e-6) return false;

            double px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
            double py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

            inter = new Point((int)Math.Round(px), (int)Math.Round(py));
            return true;
        }

        // ---------- Visualization ----------
        void DrawVisualization(
            Image<Bgr, byte> vis,
            Mat cleanLeft, Mat cleanRight,
            int leftIdx, int rightIdx, List<BlobInfo> blobsLR,
            List<BlobInfo> top2, List<BlobInfo> bottom2,
            Mat topUnion, Mat bottomUnion,
            LineResult leftLine, LineResult rightLine,
            LineResult topLine, LineResult bottomLine,
            bool okLT, bool okLB, bool okRT, bool okRB,
            Point LT, Point LB, Point RT, Point RB)
        {
            // Fill left/right
            using (Mat overlay = vis.Mat.Clone())
            using (Mat color = new Mat(vis.Size, DepthType.Cv8U, 3))
            {
                color.SetTo(new MCvScalar( 80,  80, 220)); if (cleanLeft  != null && !cleanLeft.IsEmpty)  color.CopyTo(overlay, cleanLeft);
                color.SetTo(new MCvScalar(220, 220,  80)); if (cleanRight != null && !cleanRight.IsEmpty) color.CopyTo(overlay, cleanRight);

                // Fill top/bottom unions
                color.SetTo(new MCvScalar( 80, 200, 255)); if (topUnion    != null && !topUnion.IsEmpty)    color.CopyTo(overlay, topUnion);
                color.SetTo(new MCvScalar(200,  80, 255)); if (bottomUnion != null && !bottomUnion.IsEmpty) color.CopyTo(overlay, bottomUnion);

                CvInvoke.AddWeighted(vis, 0.65, overlay, 0.35, 0, vis);
            }

            // Optional: distinct colors for #1/#2 per side
            using (Mat overlay2 = vis.Mat.Clone())
            using (Mat color2 = new Mat(vis.Size, DepthType.Cv8U, 3))
            {
                if (top2 != null)
                {
                    if (top2.Count > 0 && top2[0].Mask != null) { color2.SetTo(new MCvScalar( 80,220,255)); color2.CopyTo(overlay2, top2[0].Mask); }
                    if (top2.Count > 1 && top2[1].Mask != null) { color2.SetTo(new MCvScalar(160,240,255)); color2.CopyTo(overlay2, top2[1].Mask); }
                }
                if (bottom2 != null)
                {
                    if (bottom2.Count > 0 && bottom2[0].Mask != null) { color2.SetTo(new MCvScalar(220,100,255)); color2.CopyTo(overlay2, bottom2[0].Mask); }
                    if (bottom2.Count > 1 && bottom2[1].Mask != null) { color2.SetTo(new MCvScalar(240,160,255)); color2.CopyTo(overlay2, bottom2[1].Mask); }
                }
                CvInvoke.AddWeighted(vis, 0.75, overlay2, 0.25, 0, vis);
            }

            // Lines
            int thickness = 5;
            if (leftLine.IsValid)    CvInvoke.Line(vis, leftLine.P1,    leftLine.P2,    new MCvScalar(0,255,0), thickness);
            if (rightLine.IsValid)   CvInvoke.Line(vis, rightLine.P1,   rightLine.P2,   new MCvScalar(0,0,255), thickness);
            if (topLine.IsValid)     CvInvoke.Line(vis, topLine.P1,     topLine.P2,     new MCvScalar(0,160,0), thickness);
            if (bottomLine.IsValid)  CvInvoke.Line(vis, bottomLine.P1,  bottomLine.P2,  new MCvScalar(0,0,160), thickness);

            // Boxes + labels for LR winners
            if (leftIdx >= 0 && blobsLR != null && leftIdx < blobsLR.Count)
            {
                var lb = blobsLR[leftIdx];
                CvInvoke.Rectangle(vis, lb.BBox, new MCvScalar(255,0,0), 2);
                CvInvoke.PutText(vis, "LEFT", new Point(lb.BBox.X, Math.Max(0, lb.BBox.Y - 6)),
                    FontFace.HersheySimplex, 0.7, new MCvScalar(255,0,0), 2);
            }
            if (rightIdx >= 0 && blobsLR != null && rightIdx < blobsLR.Count)
            {
                var rb = blobsLR[rightIdx];
                CvInvoke.Rectangle(vis, rb.BBox, new MCvScalar(0,255,255), 2);
                CvInvoke.PutText(vis, "RIGHT", new Point(rb.BBox.X, Math.Max(0, rb.BBox.Y - 6)),
                    FontFace.HersheySimplex, 0.7, new MCvScalar(0,255,255), 2);
            }

            // Intersections
            void Dot(Point p, string label)
            {
                CvInvoke.Circle(vis, p, 7, new MCvScalar(255,0,255), -1);
                CvInvoke.PutText(vis, label, new Point(p.X + 6, p.Y - 6),
                    FontFace.HersheySimplex, 0.7, new MCvScalar(255,0,255), 2);
            }
            if (okLT) Dot(LT, "LT");
            if (okLB) Dot(LB, "LB");
            if (okRT) Dot(RT, "RT");
            if (okRB) Dot(RB, "RB");
        }

        // ---------- Utils ----------
        static int Clamp(int v, int lo, int hi) => Math.Min(hi, Math.Max(lo, v));
    }

    /*
    ======================= USAGE EXAMPLE =======================

    var pipe = new BlobEdgePipelineV2
    {
        MinAreaLR = 10000,
        LeftRightThreshold = 150,
        MinAreaTB = 350,
        TopBottomThreshold = 120,
        ErodeIterationsLR = 8,
        TrimPercent = 15,
        BandScale = 0.98,
        MinSamplesForFit = 10
    };

    pipe.Run(
        srcGray: m_image_black[2],
        out Image<Bgr, byte> vis,
        out Mat binLR,
        out List<BlobInfo> blobsLR,
        out int leftIdx,
        out int rightIdx,
        out Mat cleanLeft,
        out Mat cleanRight,
        out var leftLine,
        out var rightLine,
        out Mat bandMask,
        out Mat binBand,          // (threshold @120) & bandMask
        out List<BlobInfo> top2,
        out List<BlobInfo> bottom2,
        out Mat topUnion,
        out Mat bottomUnion,
        out var topLine,
        out var bottomLine,
        out bool okLT, out bool okLB, out bool okRT, out bool okRB,
        out System.Drawing.Point LT, out System.Drawing.Point LB,
        out System.Drawing.Point RT, out System.Drawing.Point RB
    );

    var old = pictureBox1.Image;
    pictureBox1.Image = vis.Mat.ToBitmap();
    old?.Dispose();

    ============================================================
    */
}


