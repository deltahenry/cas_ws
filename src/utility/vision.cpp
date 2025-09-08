//#include "battery_frame_comp.hpp"
#include "vision.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <numeric>
#include <limits>
#include <array>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

namespace bfc {

    // ------------------------- small utilities (internal) -------------------------

    template<typename T>
    static inline T clampT(T v, T lo, T hi) { return std::max(lo, std::min(hi, v)); }

    static inline Point clampPoint(const Point& p, int W, int H) {
        return Point(clampT(p.x, 0, W - 1), clampT(p.y, 0, H - 1));
    }

    static double medianOf(vector<double>& vals) {
        if (vals.empty()) return 0.0;
        size_t n = vals.size();
        nth_element(vals.begin(), vals.begin() + n / 2, vals.end());
        double hi = vals[n / 2];
        if (n % 2 == 1) return hi;
        nth_element(vals.begin(), vals.begin() + n / 2 - 1, vals.end());
        double lo = vals[n / 2 - 1];
        return 0.5 * (lo + hi);
    }

    static float cross2d(float ax, float ay, float bx, float by) {
        return ax * by - ay * bx;
    }

    // Infinite-line intersection from two segments; false if parallel/collinear.
    static bool lineLineIntersection(Point2f p1, Point2f p2, Point2f p3, Point2f p4, Point2f& inter) {
        const float EPS = 1e-9f;
        inter = Point2f(std::numeric_limits<float>::quiet_NaN(),
            std::numeric_limits<float>::quiet_NaN());
        Point2f r = p2 - p1;
        Point2f s = p4 - p3;
        Point2f qp = p3 - p1;
        float rxs = cross2d(r.x, r.y, s.x, s.y);
        float qpxs = cross2d(qp.x, qp.y, s.x, s.y);
        if (std::abs(rxs) < EPS) return false;
        float t = qpxs / rxs;
        inter = p1 + t * r;
        return true;
    }

    // Return two endpoints where (vx,vy,x0,y0) crosses the image border
    static array<Point, 2> lineAcrossImage(float vx, float vy, float x0, float y0, int W, int H) {
        vector<Point> pts;
        const double EPS = 1e-9;
        if (std::abs(vx) > EPS) {
            double t = (0 - x0) / vx;          // x=0
            double y = y0 + t * vy;
            if (y >= 0 && y <= H - 1) pts.emplace_back(0, (int)std::lround(y));
            t = ((W - 1) - x0) / vx;           // x=W-1
            y = y0 + t * vy;
            if (y >= 0 && y <= H - 1) pts.emplace_back(W - 1, (int)std::lround(y));
        }
        if (std::abs(vy) > EPS) {
            double t = (0 - y0) / vy;          // y=0
            double x = x0 + t * vx;
            if (x >= 0 && x <= W - 1) pts.emplace_back((int)std::lround(x), 0);
            t = ((H - 1) - y0) / vy;           // y=H-1
            x = x0 + t * vx;
            if (x >= 0 && x <= W - 1) pts.emplace_back((int)std::lround(x), H - 1);
        }
        if (pts.size() < 2) {
            Point p1((int)std::lround(x0 - 100 * vx), (int)std::lround(y0 - 100 * vy));
            Point p2((int)std::lround(x0 + 100 * vx), (int)std::lround(y0 + 100 * vy));
            return { clampPoint(p1, W, H), clampPoint(p2, W, H) };
        }
        double maxd = -1.0; Point a = pts[0], b = pts[1];
        for (size_t i = 0; i < pts.size(); ++i)
            for (size_t j = i + 1; j < pts.size(); ++j) {
                double dx = pts[i].x - pts[j].x, dy = pts[i].y - pts[j].y;
                double d2 = dx * dx + dy * dy;
                if (d2 > maxd) { maxd = d2; a = pts[i]; b = pts[j]; }
            }
        return { a, b };
    }

    static void orderCorners(const vector<Point>& pts4,
        Point& topLeft, Point& topRight,
        Point& bottomLeft, Point& bottomRight)
    {
        if (pts4.size() != 4) throw runtime_error("orderCorners: need exactly 4 points");
        vector<Point> sorted = pts4;
        sort(sorted.begin(), sorted.end(), [](const Point& a, const Point& b) { return a.y < b.y; });
        vector<Point> topPair = { sorted[0], sorted[1] };
        vector<Point> bottomPair = { sorted[2], sorted[3] };
        sort(topPair.begin(), topPair.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
        sort(bottomPair.begin(), bottomPair.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
        topLeft = topPair[0];
        topRight = topPair[1];
        bottomLeft = bottomPair[0];
        bottomRight = bottomPair[1];
    }

    // Quad mask between two (infinite) lines described by two points each
    static Mat maskBetweenTwoLines(Size sizeImg, Point L1a, Point L1b, Point R1a, Point R1b) {
        auto buildParam = [](Point a, Point b, float& vx, float& vy, float& x0, float& y0) {
            vx = (float)(b.x - a.x); vy = (float)(b.y - a.y);
            float n = std::sqrt(vx * vx + vy * vy);
            if (n < 1e-6f) { vx = 1.0f; vy = 0.0f; }
            else { vx /= n; vy /= n; }
            x0 = (float)a.x; y0 = (float)a.y;
            };
        float lvx, lvy, lx0, ly0, rvx, rvy, rx0, ry0;
        buildParam(L1a, L1b, lvx, lvy, lx0, ly0);
        buildParam(R1a, R1b, rvx, rvy, rx0, ry0);

        auto Lends = lineAcrossImage(lvx, lvy, lx0, ly0, sizeImg.width, sizeImg.height);
        auto Rends = lineAcrossImage(rvx, rvy, rx0, ry0, sizeImg.width, sizeImg.height);

        vector<Point> poly = { Lends[0], Lends[1], Rends[1], Rends[0] };
        Mat mask(sizeImg, CV_8U, Scalar(0));
        fillConvexPoly(mask, poly, Scalar(255), LINE_AA);
        return mask;
    }

    // ------------------------- blobs (internal) -------------------------

    struct BlobInfo {
        int label = -1;
        int area = 0;
        Rect bbox;
        Point2f centroid;
        Mat mask; // CV_8U
    };

    static vector<BlobInfo> findBlobs(const Mat& binary /* 0/255 CV_8U */) {
        CV_Assert(binary.type() == CV_8U);
        Mat labels, stats, centroids;
        int nLabels = connectedComponentsWithStats(binary, labels, stats, centroids, 8, CV_32S);

        vector<BlobInfo> blobs;
        blobs.reserve(std::max(0, nLabels - 1));
        for (int i = 1; i < nLabels; ++i) {
            int x = stats.at<int>(i, CC_STAT_LEFT);
            int y = stats.at<int>(i, CC_STAT_TOP);
            int w = stats.at<int>(i, CC_STAT_WIDTH);
            int h = stats.at<int>(i, CC_STAT_HEIGHT);
            int area = stats.at<int>(i, CC_STAT_AREA);
            float cx = (float)centroids.at<double>(i, 0);
            float cy = (float)centroids.at<double>(i, 1);
            Mat mask = (labels == i);
            mask.convertTo(mask, CV_8U, 255);

            blobs.push_back(BlobInfo{ i, area, Rect(x,y,w,h), Point2f(cx,cy), mask });
        }
        return blobs;
    }

    // robust edge fit on LEFT or RIGHT band of a contour (fitLine + MAD)
    enum class EdgeSide { LEFT, RIGHT };

    static bool robustEdgeFitFromContour(const vector<Point>& contour,
        EdgeSide side,
        float& vx, float& vy, float& x0, float& y0,
        Rect& usedRect,
        vector<Point>* usedInliers = nullptr)
    {
        if (contour.size() < 5) return false;

        Rect rect = boundingRect(contour);
        usedRect = rect;

        int band = std::max(6, (int)std::lround(rect.width * 0.12));
        int leftX = rect.x;
        int rightX = rect.x + rect.width;

        vector<Point> sel;
        sel.reserve(contour.size());

        if (side == EdgeSide::RIGHT) {
            int thresholdX = rightX - band;
            for (const auto& p : contour) if (p.x >= thresholdX) sel.push_back(p);
            if (sel.size() < 5) {
                sel = contour;
                sort(sel.begin(), sel.end(), [](const Point& a, const Point& b) { return a.x > b.x; });
                if (sel.size() > 50) sel.resize(50);
            }
        }
        else { // LEFT
            int thresholdX = leftX + band;
            for (const auto& p : contour) if (p.x <= thresholdX) sel.push_back(p);
            if (sel.size() < 5) {
                sel = contour;
                sort(sel.begin(), sel.end(), [](const Point& a, const Point& b) { return a.x < b.x; });
                if (sel.size() > 50) sel.resize(50);
            }
        }

        if (sel.size() < 5) return false;

        Vec4f line; // (vx,vy,x0,y0)
        fitLine(sel, line, DIST_FAIR, 0, 0.01, 0.01);
        vx = line[0]; vy = line[1]; x0 = line[2]; y0 = line[3];

        // distances
        vector<double> dists; dists.reserve(sel.size());
        for (const auto& p : sel) dists.push_back(std::abs(vy * (p.x - x0) - vx * (p.y - y0)));

        vector<double> tmp = dists;
        double med = medianOf(tmp);
        for (auto& v : tmp) v = std::abs(v - med);
        double mad = medianOf(tmp) + 1e-6;
        double k = 2.5;

        vector<Point> inliers; inliers.reserve(sel.size());
        for (size_t i = 0; i < sel.size(); ++i)
            if (std::abs(dists[i] - med) <= k * mad) inliers.push_back(sel[i]);

        if (inliers.size() >= 5) {
            fitLine(inliers, line, DIST_FAIR, 0, 0.01, 0.01);
            vx = line[0]; vy = line[1]; x0 = line[2]; y0 = line[3];
            if (usedInliers) *usedInliers = std::move(inliers);
        }
        else {
            if (usedInliers) *usedInliers = std::move(sel);
        }
        return true;
    }

    // ------------------------- corner finder (internal) -------------------------

    // Fills corners as: [LT, LB, RT, RB]
    static int findOutBatteryFrameCorner(const Mat& imgGray /*8U*/,
        int thr1, int thr2, int thr3, int areaMin,
        array<Point2f, 4>& point_corners)
    {
        if (imgGray.empty() || imgGray.type() != CV_8U) return -1;

        Mat bin1, bin2, bin3;
        threshold(imgGray, bin1, thr1, 255, THRESH_BINARY);
        threshold(imgGray, bin2, thr2, 255, THRESH_BINARY);
        threshold(imgGray, bin3, thr3, 255, THRESH_BINARY);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        erode(bin1, bin1, kernel, Point(-1, -1), 8);
        erode(bin2, bin2, kernel, Point(-1, -1), 8);

        // Right side from bin1 (largest blob)
        auto blobs1 = findBlobs(bin1);
        int maxIdx1 = -1, secondIdx1 = -1;
        int maxArea1 = 0, secondArea1 = 0;
        for (int i = 0; i < (int)blobs1.size(); ++i) {
            if (blobs1[i].area > 10000) {
                if (blobs1[i].area > maxArea1) {
                    secondArea1 = maxArea1; secondIdx1 = maxIdx1;
                    maxArea1 = blobs1[i].area; maxIdx1 = i;
                }
                else if (blobs1[i].area > secondArea1) {
                    secondArea1 = blobs1[i].area; secondIdx1 = i;
                }
            }
        }
        if (maxIdx1 < 0) return kErrRightSideDetect;
        std::cout << "Right side from bin1 \n";
        

        Mat mask1 = blobs1[maxIdx1].mask.clone();
        dilate(mask1, mask1, kernel, Point(-1, -1), 5);
        
	/////////////////////////////////////////////////////////////////////////////////////
	bool ok = cv::imwrite("mask1.png", mask1);
	std::cout << "write mask1 to mask1.png" << endl;
        
        vector<vector<Point>> contours1;
        findContours(mask1, contours1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours1.empty()) return kErrRightSideDetect;
        std::cout << "Right side detect \n";
        
        
        

        int best1 = 0; double bestArea1d = -1;
        for (int i = 0; i < (int)contours1.size(); ++i) {
            double a = contourArea(contours1[i], false);
            if (a > bestArea1d) { bestArea1d = a; best1 = i; }
        }
        Rect rect1;
        float vx1, vy1, x01, y01;
        if (!robustEdgeFitFromContour(contours1[best1], EdgeSide::RIGHT, vx1, vy1, x01, y01, rect1))
            return kErrRightSideDetect;
            
	std::cout << "Right side detect" << "v x1 = "<< vx1 << " vy1 = " << vy1 << " x01 = " << x01 << " y01 = " << y01 <<  std::endl;
	
        auto lineRight = lineAcrossImage(vx1, vy1, x01, y01, imgGray.cols, imgGray.rows);
        Point left_line_1(lineRight[0].x + 20, lineRight[0].y);
        Point left_line_2(lineRight[1].x + 20, lineRight[1].y);

        // Left side from bin2 (second-largest blob)
        auto blobs2 = findBlobs(bin2);
        int maxIdx2 = -1, secondIdx2 = -1;
        int maxArea2 = 0, secondArea2 = 0;
        for (int i = 0; i < (int)blobs2.size(); ++i) {
            if (blobs2[i].area > 10000) {
                if (blobs2[i].area > maxArea2) {
                    secondArea2 = maxArea2; secondIdx2 = maxIdx2;
                    maxArea2 = blobs2[i].area; maxIdx2 = i;
                }
                else if (blobs2[i].area > secondArea2) {
                    secondArea2 = blobs2[i].area; secondIdx2 = i;
                }
            }
        }
        if (secondIdx2 < 0) return kErrLeftSideDetect;
        std::cout << "Left side from bin2 (second-largest blob)"  <<  std::endl;
        

        Mat mask2 = blobs2[secondIdx2].mask.clone();
        dilate(mask2, mask2, kernel, Point(-1, -1), 5);
        
         ok = cv::imwrite("mask2.png", mask2);
        std::cout << "write mask2 as mask2.png" <<  std::endl;

        vector<vector<Point>> contours2;
        findContours(mask2, contours2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours2.empty()) return kErrLeftSideDetect;
        std::cout << "Left side detect"  <<  std::endl;

        int best2 = 0; double bestArea2d = -1;
        for (int i = 0; i < (int)contours2.size(); ++i) {
            double a = contourArea(contours2[i], false);
            if (a > bestArea2d) { bestArea2d = a; best2 = i; }
        }

        Rect rect2;
        float vx2, vy2, x02, y02;
        if (!robustEdgeFitFromContour(contours2[best2], EdgeSide::LEFT, vx2, vy2, x02, y02, rect2))
            return kErrLeftSideDetect;
            
        std::cout << "Right side detect" << "v x1 = "<< vx2 << " vy1 = " << vy2 << " x01 = " << x02 << " y01 = " << y02 <<  std::endl;

        auto lineLeft = lineAcrossImage(vx2, vy2, x02, y02, imgGray.cols, imgGray.rows);
        Point right_line_1(lineLeft[0].x - 20, lineLeft[0].y);
        Point right_line_2(lineLeft[1].x - 20, lineLeft[1].y);

        // Mask between two lines, apply to bin3, then pick small feature blobs (area>areaMin)
        Mat bandMask = maskBetweenTwoLines(imgGray.size(), left_line_1, left_line_2, right_line_1, right_line_2);
        Mat temp;
        bitwise_and(bin3, bandMask, temp);
        
        ok = cv::imwrite("bandMask.png", bandMask);
        std::cout << "write bandmask as bandmask.png" <<  std::endl;

        auto blobs3 = findBlobs(temp);
        vector<Point> points; points.reserve(8);
        for (const auto& b : blobs3) {
            if (b.area > areaMin) {
                points.emplace_back((int)std::lround(b.centroid.x),
                    (int)std::lround(b.centroid.y));
                if (points.size() == 4) break;
            }
        }
        if (points.size() < 4) return kErrFourPointsNotFound;
        
        

        Point tl, tr, bl, br;
        orderCorners(points, tl, tr, bl, br);

        Point2f pLT, pLB, pRT, pRB;
        if (!lineLineIntersection(left_line_1, left_line_2, tl, tr, pLT)) return kErrLineIntersection;
        if (!lineLineIntersection(left_line_1, left_line_2, bl, br, pLB)) return kErrLineIntersection;
        if (!lineLineIntersection(right_line_1, right_line_2, tl, tr, pRT)) return kErrLineIntersection;
        if (!lineLineIntersection(right_line_1, right_line_2, bl, br, pRB)) return kErrLineIntersection;

        point_corners = { pLT, pLB, pRT, pRB };
        return kOk;
    }

    // ------------------------- public API -------------------------

    int batteryFrameLocationCompensation(const std::string& golden_path_name,
        const std::string& current_image_path,
        double& X_out, double& Z_out)
    {
        Mat color_golden = imread(golden_path_name, IMREAD_COLOR);
        Mat color_current = imread(current_image_path, IMREAD_COLOR);
        if (color_golden.empty() || color_current.empty()) return kErrReadImage;

        vector<Mat> ch_g, ch_c;
        split(color_golden, ch_g);
        split(color_current, ch_c);
        if (ch_g.size() < 3 || ch_c.size() < 3) return kErrChannelMissing;

        // Use red channel (index 2) as in your C#
        const int thr1 = 150, thr2 = 110, thr3 = 150;
        const int areaMin = 350;

        //cout<< "findOutBatteryFrameCorner"<< "\n";
        std::cout << "[log here---------------------------------------]" << std::endl;
        array<Point2f, 4> pts_golden, pts_current;
        int rc1 = findOutBatteryFrameCorner(ch_g[2], thr1, thr2, thr3, areaMin, pts_golden);
        if (rc1 != 0) return rc1;
        int rc2 = findOutBatteryFrameCorner(ch_c[2], thr1, thr2, thr3, areaMin, pts_current);
        if (rc2 != 0) return rc2;

        auto avgXY = [](const array<Point2f, 4>& arr) {
            double sx = 0, sy = 0;
            for (const auto& p : arr) { sx += p.x; sy += p.y; }
            return pair<double, double>{ sx / 4.0, sy / 4.0 };
            };
        auto [xg, yg] = avgXY(pts_golden);
        auto [xc, yc] = avgXY(pts_current);

        X_out = (xg - xc) * 0.53;
        Z_out = (yg - yc) * 0.18;
        return kOk;
    }

    int batteryFrameLocationCompensation(const std::string& golden_path_name,
        Mat current,
        double& X_out, double& Z_out)
    {
        Mat color_golden = imread(golden_path_name, IMREAD_COLOR);
        Mat color_current = current;
        if (color_golden.empty() || color_current.empty()) return kErrReadImage;

        vector<Mat> ch_g, ch_c;
        split(color_golden, ch_g);
        split(color_current, ch_c);
        if (ch_g.size() < 3 || ch_c.size() < 3) return kErrChannelMissing;

        // Use red channel (index 2) as in your C#
        const int thr1 = 150, thr2 = 120, thr3 = 150;
        const int areaMin = 350;

        array<Point2f, 4> pts_golden, pts_current;
        int rc1 = findOutBatteryFrameCorner(ch_g[2], thr1, thr2, thr3, areaMin, pts_golden);
        if (rc1 != 0) return rc1;
        int rc2 = findOutBatteryFrameCorner(ch_c[2], thr1, thr2, thr3, areaMin, pts_current);
        if (rc2 != 0) return rc2;

        auto avgXY = [](const array<Point2f, 4>& arr) {
            double sx = 0, sy = 0;
            for (const auto& p : arr) { sx += p.x; sy += p.y; }
            return pair<double, double>{ sx / 4.0, sy / 4.0 };
            };
        auto [xg, yg] = avgXY(pts_golden);
        auto [xc, yc] = avgXY(pts_current);

        X_out = (xg - xc) * 0.53;
        Z_out = (yg - yc) * 0.18;
        return kOk;
    }


} // namespace bfc

