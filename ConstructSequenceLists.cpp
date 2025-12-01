#include "ConstructSequenceLists.h"
#include "sqlist.h"
#include <cstring>
#include <iostream>

double ERROR = 0.00001;// 全局误差阈值：浮点比较的容忍度
double CUR_LINE_H = 0;// 全局当前扫描线高度：用于扫描线算法

// === 数学函数实现 ===
double dot(Point a, Point b) { return a.x * b.x + a.y * b.y; }// 计算两个点的点积
double cross(Point a, Point b) { return a.x * b.y - a.y * b.x; }// 计算两个点的叉积
double arg(Point p) { return atan2(p.y, p.x); }// 计算点p的辐角（与x轴正方向的夹角，弧度）
Point polar(double a, double r) { return Point(cos(r) * a, sin(r) * a); }// 极坐标转直角坐标：a=模长，r=辐角，返回对应点

// 判断两个圆弧/圆是否相交（基于圆心距和半径和）
bool intersect(Edge c1, Edge c2) {
    // 圆心距 ≤ 半径和 → 相交
    if ((c1.center - c2.center).abs() <= c1.radius + c2.radius) return true;
    else return false;
}

// 计算两个圆/圆弧的交点（返回两个交点）
pair<Point, Point> getCrossPoints(Edge c1, Edge c2) {
    // 计算两圆心距离
    double d = (c1.center - c2.center).abs();
    // 余弦定理计算夹角
    double a = acos((c1.radius * c1.radius + d * d - c2.radius * c2.radius) / (2 * c1.radius * d));
    // 计算从c1到c2的辐角
    double t = arg(c2.center - c1.center);
    // 计算两个交点并返回
    return make_pair(c1.center + polar(c1.radius, t + a), c1.center + polar(c1.radius, t - a));
}

// 计算直线段（bot→top）与圆（center, radius）的交点
// res：返回两个交点是否有效（true=有效）
pair<Point, Point> abc(const Point& bot, const Point& top, const Point& center, double radius, pair<bool, bool>& res) {
    // 直线段参数方程：bot + t*(top-bot)，t∈[0,1]
    // 展开圆方程并整理为二次方程：a*t² + b*t + c = 0
    double a = (top.x - bot.x) * (top.x - bot.x) + (top.y - bot.y) * (top.y - bot.y);
    double b = 2 * ((top.x - bot.x) * (bot.x - center.x) + (top.y - bot.y) * (bot.y - center.y));
    double m = center.x * center.x + center.y * center.y + bot.x * bot.x + bot.y * bot.y;
    double l = 2 * (center.x * bot.x + center.y * bot.y) + radius * radius;
    double c = m - l;
    // 计算判别式（判断是否有实根）
    double delta = b * b - 4 * a * c;
    pair<Point, Point> p;
    // 有两个不同实根（相交）
    if (delta > 0) {
        double u1 = sqrt(delta) - b; double u2 = -sqrt(delta) - b;
        // 计算t值（直线参数）
        double utrue = u1 / (2 * a); double ufalse = u2 / (2 * a);
        // 第一个交点：t∈[0,1]则有效
        if (utrue < 1 && utrue>0) { 
            res.first = true; 
            p.first.x = bot.x + utrue * (top.x - bot.x); 
            p.first.y = bot.y + utrue * (top.y - bot.y); 
        }
        else res.first = false;
        // 第二个交点：t∈[0,1]则有效
        if (ufalse < 1 && ufalse>0) { 
            res.second = true; 
            p.second.x = bot.x + ufalse * (top.x - bot.x); 
            p.second.y = bot.y + ufalse * (top.y - bot.y); 
        }
        else res.second = false;
    }
    return p;
}

// 判断角度pTheta是否在圆弧的角度区间[thetaB, thetaT]内（逆时针圆弧）
bool res(double thetaB, double thetaT, double pTheta) {
    // 角度区间未跨0度（thetaB < thetaT）
    if (thetaB < thetaT) { 
        if (pTheta <= thetaB || pTheta >= thetaT) 
        return true;
    }
    // 角度区间跨0度（thetaB > thetaT）
    else { 
        if (thetaT < pTheta && pTheta < thetaB) 
            return true;
        else if (thetaB == pTheta || thetaT == pTheta) 
            return true; 
    }
    return false;
}

// 判断角度pTheta是否在圆弧的角度区间[thetaB, thetaT]内（顺时针圆弧）
bool resCw(double thetaB, double thetaT, double pTheta) {
    // 角度区间跨0度（thetaB > thetaT）
    if (thetaB > thetaT) { 
        if (pTheta >= thetaB || pTheta <= thetaT) 
            return true; 
    }
    // 角度区间未跨0度（thetaB < thetaT）
    else { 
        if (thetaB < pTheta && pTheta < thetaT)
            return true; 
        else if (thetaB == pTheta || thetaT == pTheta) 
            return true; 
    }
    return false;
}

// 计算圆弧边rEdge的起止角度（转换为[0, 2π]范围）
void thetaArc(Edge rEdge, double& thetaB, double& thetaT) {
    // 计算起点相对于圆心的角度
    thetaB = atan2(rEdge.Bot.data.y - rEdge.center.y, rEdge.Bot.data.x - rEdge.center.x);
    // 计算终点相对于圆心的角度
    thetaT = atan2(rEdge.Top.data.y - rEdge.center.y, rEdge.Top.data.x - rEdge.center.x);
    // 转换为[0, 2π]范围（负角度加2π）
    if (thetaB < 0) 
        thetaB += 2 * M_PI; 
    if (thetaT < 0) 
        thetaT += 2 * M_PI;
}

// 判断点point是否在圆弧边rEdge上（结合角度区间和圆弧方向）
bool isPointArc(const Point& point, Edge rEdge, const double& thetaB, const double& thetaT) {
    // 计算点相对于圆弧圆心的角度
    double theta = atan2(point.y - rEdge.center.y, point.x - rEdge.center.x);
    // 转换为[0, 2π]范围
    if (theta < 0) theta += 2 * M_PI;
    // 根据圆弧方向判断角度是否在区间内
    if (rEdge.cwFlag == true) return res(thetaB, thetaT, theta);
    else return resCw(thetaB, thetaT, theta);
}

// 计算直线边l1与圆弧边c1的交点（过滤不在圆弧上的交点）
// res：返回两个交点是否有效
pair<Point, Point> intersecArcandLine(Edge& l1, Edge& c1, pair<bool, bool>& res) {
    pair<Point, Point> ptemp, p; pair<bool, bool> restemp;
    // 先计算直线与圆的交点（不考虑圆弧范围）
    ptemp = abc(l1.Bot.data, l1.Top.data, c1.center, c1.radius, restemp);
    // 计算圆弧的起止角度
    double thetaB, thetaT; thetaArc(c1, thetaB, thetaT);
    // 第一个交点：若在圆上且在圆弧范围内则有效
    if (restemp.first == true) { 
        if (isPointArc(ptemp.first, c1, thetaB, thetaT)) { 
            p.first = ptemp.first; 
            res.first = true; 
        } 
        else res.first = false; 
    }
    // 第二个交点：若在圆上且在圆弧范围内则有效
    if (restemp.second == true) { 
        if (isPointArc(ptemp.second, c1, thetaB, thetaT)) { 
            p.second = ptemp.second; 
            res.second = true;
        } 
        else res.second = false; 
    }
    return p;
}

// 计算两个圆弧边c1和c2的交点（过滤不在圆弧上的交点）
// res：返回两个交点是否有效
pair<Point, Point> intersecArcandArc(const Edge& c1, const Edge& c2, pair<bool, bool>& res) {
    pair<Point, Point> p;
    // 先判断两圆是否相交
    if (intersect(c1, c2)) {
        // 计算两圆交点
        p = getCrossPoints(c1, c2);
        // 检查第一个交点是否在c1圆弧上
        double thetaB, thetaT; thetaArc(c1, thetaB, thetaT);
        bool res1 = isPointArc(p.first, c1, thetaB, thetaT); 
        bool res2 = isPointArc(p.second, c1, thetaB, thetaT);
        // 第一个交点：若在c1上，再检查是否在c2上
        if (res1) { thetaArc(c2, thetaB, thetaT); 
        if (isPointArc(p.first, c2, thetaB, thetaT)) res.first = true; }
        // 第二个交点：若在c1上，再检查是否在c2上
        if (res2) { thetaArc(c2, thetaB, thetaT); 
        if (isPointArc(p.second, c2, thetaB, thetaT)) res.second = true; }
    }
    return p;
}

// 判断两个线段的包围盒是否相交（快速排斥实验）
// p1-p2为第一个线段，q1-q2为第二个线段
bool IsRectCross(const Point& p1, const Point& p2, const Point& q1, const Point& q2) {
    return min(p1.x, p2.x) <= max(q1.x, q2.x) && min(q1.x, q2.x) <= max(p1.x, p2.x) &&
        min(p1.y, p2.y) <= max(q1.y, q2.y) && min(q1.y, q2.y) <= max(p1.y, p2.y);
}

// 计算向量(p0-p1)和(p0-p2)的叉积（用于判断点的位置）
double mult(Point p0, Point p1, Point p2) { 
    return (p0.x - p1.x) * (p0.y - p2.y) - (p0.y - p1.y) * (p0.x - p2.x);
}

// 判断两个线段是否相交（跨立实验）
// pFirst1-pFirst2为第一个线段，pSecond1-pSecond2为第二个线段
bool IsLineSegmentCross(const Point& pFirst1, const Point& pFirst2, const Point& pSecond1, const Point& pSecond2) {
    // 检查第一个线段的两端是否在第二个线段的同侧
    if (mult(pFirst1, pSecond1, pFirst2) * mult(pFirst1, pFirst2, pSecond2) < 0) return false;
    // 检查第二个线段的两端是否在第一个线段的同侧
    if (mult(pSecond1, pFirst1, pSecond2) * mult(pSecond1, pSecond2, pFirst2) < 0) return false;
    return true;
}

// 计算两条直线段的交点，并存入interNodes
// 返回值：true=相交，false=不相交
bool intersecLineandLine(const Point& p1, const Point& p2, const Point& q1, const Point& q2, vector<IntersectNode>& interNodes) {
    IntersectNode intersectN;
    // 快速排斥实验：包围盒不相交则直接返回
    if (IsRectCross(p1, p2, q1, q2)) {
        // 跨立实验：判断线段是否相交
        if (IsLineSegmentCross(p1, p2, q1, q2)) {
            // 计算面积（用于参数t的计算）
            double area_abc = (p1.x - q1.x) * (p2.y - q1.y) - (p1.y - q1.y) * (p2.x - q1.x);
            double area_abd = (p1.x - q2.x) * (p2.y - q2.y) - (p1.y - q2.y) * (p2.x - q2.x);
            double area_cda = (q1.x - p1.x) * (q2.y - p1.y) - (q1.y - p1.y) * (q2.x - p1.x);
            // 计算参数t（线段p1-p2上的交点位置）
            double t = area_cda / (area_abd - area_abc);
            // 计算交点坐标
            double dx = t * (p2.x - p1.x); double dy = t * (p2.y - p1.y);
            intersectN.Pt.data.x = p1.x + dx; intersectN.Pt.data.y = p1.y + dy;
            // 标记交点类型：2=线段相交
            intersectN.Pt.crossing = 2; interNodes.push_back(intersectN);
            return true;
        }
    }
    return false;
}

// 计算两个向量(x1,y1)和(x2,y2)的旋转角度（单位：度）
double getRotateAngle(double x1, double y1, double x2, double y2) {
    const double epsilon = 1.0e-6; const double nyPI = acos(-1.0);
    double dist, dot, degree, angle;
    // 归一化第一个向量
    dist = sqrt(x1 * x1 + y1 * y1); x1 /= dist; y1 /= dist;
    // 归一化第二个向量
    dist = sqrt(x2 * x2 + y2 * y2); x2 /= dist; y2 /= dist;
    // 计算点积
    dot = x1 * x2 + y1 * y2;
    // 处理特殊情况：点积=1（夹角0度），点积=-1（夹角180度）
    if (fabs(dot - 1.0) <= epsilon) angle = 0.0; else if (fabs(dot + 1.0) <= epsilon) angle = nyPI;
    else { 
        // 计算夹角（弧度）
        double cross; angle = acos(dot); 
        // 叉积判断旋转方向（顺时针/逆时针）
        cross = x1 * y2 - x2 * y1; if (cross < 0) angle = 2 * nyPI - angle; 
    }
    // 转换为角度
    degree = angle * 180.0 / nyPI; return degree;
}

// === Edge 成员函数 ===
// 计算边与当前扫描线（CUR_LINE_H）的交点x坐标
double Edge::Int_Line() const {
    // 水平线特殊处理：返回最大x坐标（乘以缩放因子mults）
    if (fabs(Bot.data.y - Top.data.y) * mults < 0.01) return max(Bot.data.x, Top.data.x) * mults;
    // 直线边
    if (isArc == false) {
        // 直线参数方程求解交点x坐标（考虑缩放因子mults）
        double molecule = (Top.data.x - Bot.data.x) * mults * mults * CUR_LINE_H + Bot.data.x * Top.data.y * mults * mults - Top.data.x * Bot.data.y * mults * mults;
        double denominator = (Top.data.y - Bot.data.y) * mults;
        return molecule / denominator;
    }
    // 圆弧边
    else {
        // 圆方程求解交点x坐标（y=CUR_LINE_H）
        double m = radius * radius * mults * mults - (CUR_LINE_H - center.y) * mults * (CUR_LINE_H - center.y) * mults;
        double x1 = sqrt(fabs(m)) + center.x * mults; 
        double x2 = center.x * mults - sqrt(fabs(m));
        // 根据圆弧起止点位置选择有效交点
        if ((Bot.data.x + Top.data.x) * mults > 2 * center.x * mults) { 
            if (x1 >= center.x * mults) 
                return x1; 
            else 
                return x2;
        }
        else if ((Bot.data.x + Top.data.x) * mults < 2 * center.x * mults) { 
            if (x1 <= center.x * mults) 
                return x1; 
            else 
                return x2; 
        }
        else { 
            if (Bot.data.y > Top.data.y) { 
                if (cwFlag == true) return max(x1, x2); 
                else return min(x1, x2);
            } else { 
                if (cwFlag == true) 
                    return min(x1, x2);
                else 
                    return max(x1, x2); 
            } 
        }
    }
}

// 计算边与偏移扫描线（CUR_LINE_H-0.03）的交点x坐标（用于排序）
double Edge::Int_Line2() const {
    // 直线边
    if (isArc == false) {
        // 直线参数方程求解交点x坐标（考虑缩放因子和偏移）
        double molecule = (Top.data.x - Bot.data.x) * mults * (CUR_LINE_H * mults - 0.03) + Bot.data.x * Top.data.y * mults * mults - Top.data.x * Bot.data.y * mults * mults;
        double denominator = (Top.data.y - Bot.data.y) * mults;
        return molecule / denominator;
    }
    // 圆弧边
    else {
        // 圆方程求解交点x坐标（y=CUR_LINE_H-0.03）
        double m = radius * radius * mults * mults - (CUR_LINE_H * mults - 0.03 - center.y * mults) * (CUR_LINE_H * mults - 0.03 - center.y * mults);
        double x1 = sqrt(fabs(m)) + center.x * mults; 
        double x2 = center.x * mults - sqrt(fabs(m));
        // 根据圆弧起止点位置选择有效交点
        if ((Bot.data.x + Top.data.x) * mults > 2 * center.x * mults) { 
            if (x1 >= center.x * mults) 
                return x1; 
            else 
                return x2; }
        else if ((Bot.data.x + Top.data.x) * mults < 2 * center.x * mults) { 
            if (x1 <= center.x * mults) 
                return x1; 
            else 
                return x2;
        }
        else { if (Bot.data.y > Top.data.y) { 
            if (cwFlag == true) 
                return max(x1, x2);
            else 
                return min(x1, x2); 
        } else { 
            if (cwFlag == true) 
                return min(x1, x2); 
            else 
                return max(x1, x2); 
        } 
        }
    }
}

// Edge的比较器：用于扫描线算法中的边排序
bool Edge::comp::operator()(const Edge& L1, const Edge& L2) const {
    // 同一编号的边不排序
    if (L1.nums == L2.nums) return false;
    // 先按Int_Line排序，误差范围内则按Int_Line2排序
    if (fabs(L1.Int_Line() - L2.Int_Line()) < 0.0001) return L1.Int_Line2() < L2.Int_Line2();
    return L1.Int_Line() < L2.Int_Line();
}

// 获取交点的对侧点（若交点是边的端点，返回另一端点）
Point IntersectNode::Int_inter() const {
    // 交点是belong1的起点 → 返回终点
    if (fabs(belong1.Bot.data.x - Pt.data.x) < ERROR && fabs(belong1.Bot.data.y - Pt.data.y) < ERROR) 
        return belong1.Top.data;
    // 交点是belong1的终点 → 返回起点
    else if (fabs(belong1.Top.data.x - Pt.data.x) < ERROR && fabs(belong1.Top.data.y - Pt.data.y) < ERROR) 
        return belong1.Bot.data;
    // 非端点 → 返回空点
    return Point();
}

// 获取交点对侧点的x坐标（辅助排序）
double IntersectNode::Int_interfirst() const {
    if (fabs(belong1.Bot.data.x - Pt.data.x) < ERROR && fabs(belong1.Bot.data.y - Pt.data.y) < ERROR) 
        return belong1.Top.data.x;
    else if (fabs(belong1.Top.data.x - Pt.data.x) < ERROR && fabs(belong1.Top.data.y - Pt.data.y) < ERROR) 
        return belong1.Bot.data.x;
    return 0;
}

// 获取交点对侧点的y坐标（辅助排序）
double IntersectNode::Int_intersecond() const {
    if (fabs(belong1.Bot.data.x - Pt.data.x) < ERROR && fabs(belong1.Bot.data.y - Pt.data.y) < ERROR) 
        return belong1.Top.data.y;
    else if (fabs(belong1.Top.data.x - Pt.data.x) < ERROR && fabs(belong1.Top.data.y - Pt.data.y) < ERROR) 
        return belong1.Bot.data.y;
    return 0;
}

// IntersectNode的比较器：用于交点排序
bool IntersectNode::operator()(IntersectNode P1, IntersectNode P2) {
    // y坐标相近时
    if (fabs(P1.Pt.data.y - P2.Pt.data.y) < 0.00001) {
        // x坐标也相近时
        if (fabs(P1.Pt.data.x - P2.Pt.data.x) < 0.00001) {
            // 交点类型相同
            if (P1.Pt.crossing == P2.Pt.crossing) {
                // 计算对侧点并判断是否共线
                Point p = P1.Int_inter(); 
                Point p1 = P2.belong1.Bot.data;
                Point p2 = P2.belong1.Top.data;
                double tmpx = (p1.x - p2.x) / (p1.y - p2.y) * (p.y - p2.y) + p2.x;
                if (fabs(tmpx - p.x) < ERROR) return true;
                return false;
            }
            // 按交点类型排序
            return P1.Pt.crossing < P2.Pt.crossing;
        }
        // 按x坐标升序
        else return P1.Pt.data.x < P2.Pt.data.x;
    }
    // 按y坐标升序
    else return P1.Pt.data.y < P2.Pt.data.y;
}

// === Cutline Implementation ===
// 扫描线类构造函数
Cutline::Cutline() {}
// 扫描线类析构函数
Cutline::~Cutline() {}

// 查找当前边it的左侧边（迭代器前移）
set<Edge, Edge::comp>::iterator Cutline::findleft(set<Edge, Edge::comp>::iterator it) {
    if (it == CurCutLine.begin()) return CurCutLine.end();
    return --it;
}

// 查找当前边it的右侧边（迭代器后移）
set<Edge, Edge::comp>::iterator Cutline::findright(set<Edge, Edge::comp>::iterator it) { return ++it; }

// 计算两条边L1和L2的交点，并存入interNodes
// 返回值：true=有交点，false=无交点
bool Cutline::intersec(Edge L1, Edge L2, vector<IntersectNode>& interNodes) {
    Point p1 = L1.Bot.data; Point p2 = L1.Top.data; Point q1 = L2.Bot.data; Point q2 = L2.Top.data;
    pair<Point, Point> p; pair<bool, bool> res;
    // 两条直线边
    if (L1.isArc == false && L2.isArc == false) return intersecLineandLine(p1, p2, q1, q2, interNodes);
    // 两条圆弧边
    else if (L1.isArc == true && L2.isArc == true) p = intersecArcandArc(L1, L2, res);
    // L1直线，L2圆弧
    else if (L1.isArc == false && L2.isArc == true) p = intersecArcandLine(L1, L2, res);
    // L1圆弧，L2直线
    else if (L1.isArc == true && L2.isArc == false) p = intersecArcandLine(L2, L1, res);

    // 有有效交点
    if (res.first == true || res.second == true) {
        IntersectNode temp; temp.Pt.crossing = 2;// 标记为线段/圆弧交点
        // 两个有效交点
        if (res.first == true && res.second == true) { 
            temp.Pt.data = p.first; interNodes.push_back(temp); 
            temp.Pt.data = p.second; interNodes.push_back(temp); 
        }
        // 第一个交点有效
        else if (res.first == true && res.second == false) { 
            temp.Pt.data = p.first; interNodes.push_back(temp); 
        }
        // 第二个交点有效
        else { 
            temp.Pt.data = p.second; interNodes.push_back(temp); 
        }
        return true;
    }
    return false;
}

// 读取边集S1和S2，初始化扫描线事件队列Pm
void Cutline::read_line(const vector<DataType>& S1, const vector<DataType>& S2) {
    int i = 0;
    // 处理S1中的边
    for (auto temp : S1) {
        IntersectNode p1, p2; p1.Pt = temp.rEdge.Bot; p2.Pt = temp.rEdge.Top;
        // 标记边的起止点类型（0=起点，1=终点）
        if (p1.Pt.data.y - p2.Pt.data.y > ERROR) { 
            p1.Pt.crossing = 0; 
            p2.Pt.crossing = 1; 
        }
        else if (fabs(p1.Pt.data.y - p2.Pt.data.y) < ERROR) {
            if (p1.Pt.data.x - p2.Pt.data.x > ERROR) { 
                p1.Pt.crossing = 0; 
                p2.Pt.crossing = 1; 
            }
            else { p1.Pt.crossing = 1; 
            p2.Pt.crossing = 0; 
            }
        }
        else { p1.Pt.crossing = 1; p2.Pt.crossing = 0; }
        // 给边分配唯一编号
        temp.rEdge.nums = i; i++;
        // 关联交点与所属边
        p1.belong1 = temp.rEdge; p2.belong1 = temp.rEdge;
        // 存储边和事件
        Line.push_back(temp.rEdge); Pm.push(p1); Pm.push(p2);
    }
    // 处理S2中的边（逻辑同S1）
    for (auto temp : S2) {
        IntersectNode p1, p2; p1.Pt = temp.rEdge.Bot; p2.Pt = temp.rEdge.Top;
        if (p1.Pt.data.y - p2.Pt.data.y > ERROR) { 
            p1.Pt.crossing = 0; 
            p2.Pt.crossing = 1; 
        }
        else if (fabs(p1.Pt.data.y - p2.Pt.data.y) < ERROR) {
            if (p1.Pt.data.x - p2.Pt.data.x > ERROR) { 
                p1.Pt.crossing = 0; 
                p2.Pt.crossing = 1; 
            }
            else { 
                p1.Pt.crossing = 1;
            p2.Pt.crossing = 0; }
        }
        else {
            p1.Pt.crossing = 1; 
            p2.Pt.crossing = 0; }
        temp.rEdge.nums = i; 
        i++; 
        p1.belong1 = temp.rEdge; 
        p2.belong1 = temp.rEdge;
        Line.push_back(temp.rEdge); 
        Pm.push(p1); 
        Pm.push(p2);
    }
}

// 处理扫描线事件（交点/起点/终点）
void Cutline::HandleEvent(IntersectNode event, vector<DataType>& S1, vector<DataType>& S2) {
    // 交点事件（crossing=2）
    if (event.Pt.crossing == 2) {
        // 去重：避免重复处理同一交点
        if (!intersectpoint.empty() && (event.Pt.data) == (intersectpoint.back().data)) return;
        // 记录交点
        intersectpoint.push_back(event.Pt);
        // 查找相交的两条边
        auto it1 = CurCutLine.find(event.belong1); 
        auto it2 = CurCutLine.find(event.belong2);
        // 查找两条边的左右邻居
        auto left = findleft(it1); 
        auto right = findright(it2);
        bool flag = true;
        // 避免重复处理（邻居交换的情况）
        if (left != CurCutLine.end() && right != CurCutLine.end()) { 
            if ((*left).nums == (*it2).nums && (*right).nums == (*it1).nums) 
                flag = false; 
        }
        // 从当前扫描线中移除相交边
        CurCutLine.erase(it1); CurCutLine.erase(it2);
        // 重新插入边（更新排序）
        auto newline1 = event.belong1; 
        auto newline2 = event.belong2;
        // 更新扫描线高度（取当前交点和队列顶事件的中点）
        CUR_LINE_H = (event.Pt.data.y + Pm.top().Pt.data.y) / 2;
        CurCutLine.insert(newline1); 
        CurCutLine.insert(newline2);
        // 检查邻居边与新边的交点
        if (flag) {
            // 检查左侧邻居与newline2的交点
            if (left != CurCutLine.end()) {
                if ((left)->lb1 != newline2.lb1) {
                    vector<IntersectNode> interNodes; 
                    bool flag = intersec(*left, newline2, interNodes);
                    for (int i = 0; i < interNodes.size(); i++) {
                        // 只处理当前扫描线下方的交点
                        if (flag && interNodes[i].Pt.data.y < event.Pt.data.y) {
                            interNodes[i].belong1 = *left; interNodes[i].belong2 = newline2;
                            // 去重：避免重复入队
                            if (repeater.count(interNodes[i].Pt.data)) return;
                            repeater.insert(interNodes[i].Pt.data); 
                            Pm.push(interNodes[i]);
                            // 记录交点到对应边的交点集
                            if ((left)->lb1 == true) {
                                S1[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                                S2[(newline2).lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            }
                            else { 
                                S1[(newline2).lb2].iPoints.push_back(interNodes[i].Pt.data); 
                                S2[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); }
                            // 更新扫描线高度
                            CUR_LINE_H = max(CUR_LINE_H, (CUR_LINE_H + interNodes[i].Pt.data.y) / 2);
                        }
                    }
                }
            }
            // 检查右侧邻居与newline1的交点（逻辑同左侧）
            if (right != CurCutLine.end()) {
                if ((right)->lb1 != newline1.lb1) {
                    vector<IntersectNode> interNodes; 
                    bool flag = intersec(newline1, *right, interNodes);
                    for (int i = 0; i < interNodes.size(); i++) {
                        if (flag && interNodes[i].Pt.data.y < event.Pt.data.y) {
                            interNodes[i].belong1 = newline1; 
                            interNodes[i].belong2 = *right;
                            if (repeater.count(interNodes[i].Pt.data)) return;
                            repeater.insert(interNodes[i].Pt.data);
                            Pm.push(interNodes[i]);
                            if ((right)->lb1 == true) { 
                                S1[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                                S2[(newline1).lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            }
                            else { 
                                S1[(newline1).lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            }
                            CUR_LINE_H = max(CUR_LINE_H, (CUR_LINE_H + interNodes[i].Pt.data.y) / 2);
                        }
                    }
                }
            }
        }
    }
    // 边起点事件（crossing=0）
    else if (event.Pt.crossing == 0) {
        // 更新扫描线高度为起点y坐标
        CUR_LINE_H = event.Pt.data.y;
        // 水平线特殊处理：检查与所有异侧边的交点
        if (fabs((event.belong1).Bot.data.y - (event.belong1).Top.data.y) * mults < 0.01) {
            for (auto line = CurCutLine.begin(); line != CurCutLine.end(); line++) {
                if ((line)->lb1 != (event.belong1).lb1) {
                    vector<IntersectNode> interNodes; 
                    bool flag = intersec((*line), event.belong1, interNodes);
                    if (flag) {
                        for (int i = 0; i < interNodes.size(); i++) {
                            intersectpoint.push_back(interNodes[i].Pt);
                            // 记录交点到对应边的交点集
                            if (event.belong1.lb1 == true) { 
                                S1[event.belong1.lb2].iPoints.push_back(interNodes[i].Pt.data);
                                S2[(line)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            }
                            else { 
                                S1[(line)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[event.belong1.lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            }
                        }
                    }
                }
            }
            return;
        }
        // 将边插入当前扫描线
        CurCutLine.insert(event.belong1);
        // 查找插入位置的左右邻居
        auto it = CurCutLine.find(event.belong1); 
        auto left = findleft(it); 
        auto right = findright(it);
        // 检查左侧邻居与当前边的交点
        if (left != CurCutLine.end()) {
            if ((left)->lb1 != (it)->lb1) {
                vector<IntersectNode> interNodes; 
                bool flag = intersec(*left, *it, interNodes);
                if (flag) {
                    for (int i = 0; i < interNodes.size(); i++) {
                        interNodes[i].belong1 = *left;
                        interNodes[i].belong2 = *it;
                        // 去重
                        if (repeater.count(interNodes[i].Pt.data)) return;
                        repeater.insert(interNodes[i].Pt.data); 
                        Pm.push(interNodes[i]);
                        // 记录交点
                        if ((it)->lb1 == true) { 
                            S1[(it)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        }
                        else { 
                            S1[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[(it)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        }
                    }
                }
            }
        }
        // 检查右侧邻居与当前边的交点（逻辑同左侧）
        if (right != CurCutLine.end()) {
            if ((right)->lb1 != (it)->lb1) {
                vector<IntersectNode> interNodes; bool flag = intersec(*it, *right, interNodes);
                if (flag) {
                    for (int i = 0; i < interNodes.size(); i++) {
                        interNodes[i].belong1 = *it; 
                        interNodes[i].belong2 = *right;
                        if (repeater.count(interNodes[i].Pt.data)) return;
                        repeater.insert(interNodes[i].Pt.data); Pm.push(interNodes[i]);
                        if ((it)->lb1 == true) { 
                            S1[(it)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        }
                        else { 
                            S1[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                            S2[(it)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        }
                    }
                }
            }
        }
    }
    // 边终点事件（crossing=1）
    else {
        // 更新扫描线高度（终点y+小偏移，避免重复处理）
        CUR_LINE_H = event.Pt.data.y + 0.0001;
        // 查找当前边
        auto it = CurCutLine.find(event.belong1);
        if (it == CurCutLine.end()) return;
        // 查找左右邻居
        auto left = findleft(it); auto right = findright(it);
        // 从扫描线中移除当前边
        CurCutLine.erase(it);
        // 无邻居则返回
        if (left == CurCutLine.end() || right == CurCutLine.end()) return;
        // 检查左右邻居的交点（异侧边）
        if ((left)->lb1 != (right)->lb1) {
            vector<IntersectNode> interNodes; 
            bool flag = intersec(*left, *right, interNodes);
            for (int i = 0; i < interNodes.size(); i++) {
                // 只处理当前扫描线下方的交点
                if (flag && interNodes[i].Pt.data.y < CUR_LINE_H) {
                    interNodes[i].belong1 = *left; interNodes[i].belong2 = *right;
                    // 去重
                    if (repeater.count(interNodes[i].Pt.data)) return;
                    repeater.insert(interNodes[i].Pt.data); Pm.push(interNodes[i]);
                    // 记录交点
                    if ((left)->lb1 == true) { 
                        S1[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        S2[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data);
                    }
                    else { 
                        S1[(right)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                        S2[(left)->lb2].iPoints.push_back(interNodes[i].Pt.data); 
                    }
                }
            }
        }
    }
}

// 处理所有扫描线事件，查找所有交点
void Cutline::FindIntersection(vector<DataType>& S1, vector<DataType>& S2) {
    while (!Pm.empty()) {
        // 取出队首事件
        IntersectNode temp = Pm.top(); 
        // 处理事件
        HandleEvent(temp, S1, S2); 
        // 出队
        Pm.pop();
    }
}

// === Helpers ===
// 初始化边e的参数：关联前后边、设置起点、圆弧属性等
void InitEdge(Edge* e, Edge* eNext, Edge* ePrev, const vector<Point>& pg, const edgeInfo& Info) {
    // 内存清零
    std::memset(e, 0, sizeof(Edge)); 
    // 关联前后边
    e->next = eNext; e->prev = ePrev;
    // 设置起点
    e->Curr.data = pg[Info.startIndex];
    // 圆弧边：初始化圆心、半径、圆弧标志
    if (Info.bCircle == true) { 
        e->center = Info.center; 
        e->radius = Info.radius; 
        e->isArc = true; 
        e->cwFlag = Info.cw;
    }
}

// 初始化边e的起止点：Bot=当前点，Top=下一个点，设置所属标识
void InitEdge2(Edge& e, bool Pt) { 
    e.Bot = e.Curr; 
    e.Top = e.next->Curr; 
    e.lb1 = Pt; 
}

// 构建边链表：将带圆弧的多边形转换为边链表
// pg：顶点集，Info：边信息集，type：所属标识（1=外轮廓，0=内孔）
Edge* addPath(const vector<Point>& pg, const vector<edgeInfo>& Info, bool type) {
    int highl = pg.size() - 1; 
    // 分配边数组
    Edge* edges = new Edge[highl + 1];
    // 初始化第二个边的起点
    edges[1].Curr.data = pg[Info[1].startIndex];
    // 初始化第0条边（首尾相连）
    InitEdge(&edges[0], &edges[1], &edges[highl], pg, Info[0]); 
    // 初始化最后一条边（首尾相连）
    InitEdge(&edges[highl], &edges[0], &edges[highl - 1], pg, Info[highl]);
    // 初始化中间边
    for (unsigned int i = highl - 1; i >= 1; --i) 
        InitEdge(&edges[i], &edges[i + 1], &edges[i - 1], pg, Info[i]);
    // 链表起点
    Edge* eStart = &edges[0]; Edge* E = eStart;
    // 循环初始化所有边的起止点
    do { InitEdge2(*E, type); E = E->next; } while (E != eStart);
    return E;
}

// 计算边链表的包围盒（MBR：最小边界矩形）
Mbr lineMBR(Edge* head) {
    // 初始化包围盒左/下边界为第一个点的坐标
    double lx = head->Bot.data.x, ly = head->Bot.data.y; 
    // 初始化右/上边界为极小值
    double rx = -DBL_MAX, ry = -DBL_MAX; Mbr mbr; 
    // 从下一条边开始遍历
    Edge* line = head->next;
    // 遍历所有边（回到起点则停止）
    while (!(head->Bot.data == line->Bot.data && head->Top.data == line->Top.data)) {
        // 圆弧边：包围盒包含圆心±半径
        if (line->isArc == true) { 
            lx = min({ line->center.x - line->radius,lx }); 
            ly = min({ line->center.y - line->radius,ly }); 
            rx = max({ line->center.x + line->radius,rx }); 
            ry = max({ line->center.y + line->radius,ry }); 
        }
        // 直线边：包围盒包含起止点
        else { 
            lx = min({ line->Bot.data.x,lx }); 
            ly = min({ line->Bot.data.y,ly });
            rx = max({ line->Bot.data.x,rx }); 
            ry = max({ line->Bot.data.y,ry });
        }
        line = line->next;
    }
    // 设置包围盒参数
    mbr.lx = lx;
    mbr.ly = ly; 
    mbr.rx = rx;
    mbr.ry = ry; 
    
    return mbr;
}

// 初始化边的起止点类型（0=起点，1=终点）
void InitializePoint(Edge* line) {
    // 起点y > 终点y → 起点=0，终点=1
    if ((line->Bot.data.y - line->Top.data.y) > ERROR) { 
        line->Bot.crossing = 0; line->Top.crossing = 1; 
    }
    // 水平边
    else if (fabs(line->Bot.data.y - line->Top.data.y) < ERROR) {
        // 起点x > 终点x → 起点=0，终点=1
        if ((line->Bot.data.x - line->Top.data.x) > ERROR) { 
            line->Bot.crossing = 0; line->Top.crossing = 1; 
        }
        // 否则反转
        else { 
            line->Bot.crossing = 1; line->Top.crossing = 0; 
        }
    }
    // 起点y < 终点y → 起点=1，终点=0
    else { 
        line->Bot.crossing = 1; line->Top.crossing = 0; 
    }
}

// 判断线段a1-a2是否与水平线y相交（跨立实验）
bool SegLineIntY(Point a1, Point a2, double y) { 
    double fa1 = (a1.y - y) * (0 - 1); 
    double fa2 = (a2.y - y) * (0 - 1); 
    if (fa1 * fa2 > 0) 
        return false; 
    return true; 
}
// 判断线段a1-a2是否与垂直线x相交（跨立实验）
bool SegLineIntX(Point a1, Point a2, double x) { 
    double fa1 = -(a1.x - x) * (0 - 1); 
    double fa2 = -(a2.x - x) * (0 - 1); 
    if (fa1 * fa2 > 0) 
        return false; 
    return true; 
}
// 判断线段a1-b1是否完全在y范围[bot, top]内
bool inLinesY(Point a1, Point b1, double top, double bot) { 
    double y1 = max(a1.y, b1.y); 
    double y2 = min(a1.y, b1.y); 
    if (y1 <= top && y2 >= bot) 
        return true; 
    return false;
}
// 判断线段a1-b1是否完全在x范围[bot, top]内
bool inLinesX(Point a1, Point b1, double top, double bot) { 
    double x1 = max(a1.x, b1.x); 
    double x2 = min(a1.x, b1.x); 
    if (x1 <= top && x2 >= bot) 
        return true; 
    return false; 
}

// 遍历边链表，筛选出与指定范围相交的边
// top/bot：范围上下界，flag=0=按y轴筛选，1=按x轴筛选
// data：边链表起点，Re：输出筛选后的边集合
vector<Edge*> traverseEdge(double top, double bot, bool flag, Edge* data, vector<Edge>& Re) {
    Edge* cur = data->next; vector<Edge*> R;
    // 按y轴筛选
    if (flag == 0) {
        // 判断边是否与y范围相交/包含
        bool r1 = SegLineIntY(data->Bot.data, data->Top.data, top); 
        bool r2 = SegLineIntY(data->Bot.data, data->Top.data, bot); 
        bool r3 = inLinesY(data->Bot.data, data->Top.data, top, bot); 
        bool r4 = false; bool flg = r1 || r2 || r3;
        // 圆弧边额外判断：圆心与起止点是否跨范围
        if (!(flg) && data->isArc == true) {
            bool c1 = SegLineIntY(data->Bot.data, data->center, top);
            bool c2 = SegLineIntY(data->Top.data, data->center, bot);
            if (c1 || c2) 
                r4 = true; 
        }
        // 符合条件则加入结果
        if (flg || r4) { 
            InitializePoint(data); 
            R.push_back(data); 
            Re.push_back(*data); 
        }
        // 遍历所有边
        while (!(cur->Bot.data == data->Bot.data && cur->Top.data == data->Top.data)) {
            // 逻辑同上
            bool res1 = SegLineIntY(cur->Bot.data, cur->Top.data, top); 
            bool res2 = SegLineIntY(cur->Bot.data, cur->Top.data, bot); 
            bool res3 = inLinesY(cur->Bot.data, cur->Top.data, top, bot); 
            bool res4 = false;
            bool f = res1 || res2 || res3;
            if (!(f) && cur->isArc == true) { 
                bool c1 = SegLineIntY(cur->Bot.data, cur->center, top); 
                bool c2 = SegLineIntY(cur->Top.data, cur->center, bot); 
                if (c1 || c2) res4 = true; 
            }
            if (f || res4) { 
                InitializePoint(cur); 
                R.push_back(cur); 
                Re.push_back(*cur);
            }
            cur = cur->next;
        }
    }
    // 按x轴筛选（逻辑同y轴）
    else {
        bool r1 = SegLineIntX(data->Bot.data, data->Top.data, top); 
        bool r2 = SegLineIntX(data->Bot.data, data->Top.data, bot); 
        bool r3 = inLinesX(data->Bot.data, data->Top.data, top, bot); 
        bool r4 = false; bool flg = r1 || r2 || r3;
        if (!(flg) && data->isArc == true) { 
            bool c1 = SegLineIntX(data->Bot.data, data->center, top); 
            bool c2 = SegLineIntX(data->Top.data, data->center, bot); 
            if (c1 || c2) r4 = true; 
        }
        if (flg || r4) { 
            InitializePoint(data); 
            R.push_back(data); 
            Re.push_back(*data); 
        }
        while (!(cur->Bot.data == data->Bot.data && cur->Top.data == data->Top.data)) {
            bool res1 = SegLineIntX(cur->Bot.data, cur->Top.data, top); 
            bool res2 = SegLineIntX(cur->Bot.data, cur->Top.data, bot); 
            bool res3 = inLinesX(cur->Bot.data, cur->Top.data, top, bot);
            bool res4 = false; bool f = res1 || res2 || res3;
            if (!(f) && cur->isArc == true) { 
                bool c1 = SegLineIntX(cur->Bot.data, cur->center, top); 
                bool c2 = SegLineIntX(cur->Top.data, cur->center, bot); 
                if (c1 || c2) res4 = true; 
            }
            if (f || res4) { 
                InitializePoint(cur); 
                R.push_back(cur); 
                Re.push_back(*cur); 
            }
            cur = cur->next;
        }
    }
    return R;
}

// 关联两条边链表P1和P2，筛选出相交范围内的边
// R1/R2：输出P1/P2的筛选边，ReFirst/ReSecond：输出筛选后的边集合
void relatedEdge(Edge* P1, Edge* P2, vector<Edge*>& R1, vector<Edge*>& R2, vector<Edge>& ReFirst, vector<Edge>& ReSecond) {
    // 计算两条边链表的包围盒
    Mbr mbr1 = lineMBR(P1); 
    Mbr mbr2 = lineMBR(P2); 
    Mbr mbrEffective;
    // 计算有效交集包围盒（两个包围盒的交集）
    mbrEffective.lx = max(mbr1.lx, mbr2.lx); 
    mbrEffective.ly = max(mbr1.ly, mbr2.ly);
    mbrEffective.rx = min(mbr1.rx, mbr2.rx); 
    mbrEffective.ry = min(mbr1.ry, mbr2.ry);
    // 按较长的轴筛选（x轴长则按x，否则按y）
    if ((mbrEffective.rx - mbrEffective.lx) >= (mbrEffective.ry - mbrEffective.ly)) { 
        R1 = traverseEdge(mbrEffective.rx, mbrEffective.lx, 1, P1, ReFirst); 
        R2 = traverseEdge(mbrEffective.rx, mbrEffective.lx, 1, P2, ReSecond); 
    }
    else { 
        R1 = traverseEdge(mbrEffective.ry, mbrEffective.ly, 0, P1, ReFirst); 
        R2 = traverseEdge(mbrEffective.ry, mbrEffective.ly, 0, P2, ReSecond); 
    }
}

// 判断圆弧边rEdge是否经过y轴极值点（π/2或3π/2方向）
pair<bool, bool> isPointofArcfirst(Edge rEdge) {
    // 计算圆弧起止角度
    double thetaB = atan2(rEdge.Bot.data.y - rEdge.center.y, rEdge.Bot.data.x - rEdge.center.x); 
    double thetaT = atan2(rEdge.Top.data.y - rEdge.center.y, rEdge.Top.data.x - rEdge.center.x);
    pair<bool, bool> result;
    // 转换为[0, 2π]范围
    if (thetaB < 0) thetaB += 2 * M_PI; 
    if (thetaT < 0) thetaT += 2 * M_PI;
    // 起止角度为y轴极值点 → 无交点
    if (thetaB == M_PI / 2 || thetaB == 3 * M_PI / 2 || thetaT == M_PI / 2 || thetaT == 3 * M_PI / 2) { 
        result.first = false; result.second = false; 
    }
    // 判断圆弧是否经过y轴极值点
    else { if (rEdge.cwFlag == true) { 
        result.first = res(thetaB, thetaT, M_PI / 2); 
        result.second = res(thetaB, thetaT, 3 * M_PI / 2); 
    }
    else { 
        result.first = resCw(thetaB, thetaT, M_PI / 2);
    result.second = resCw(thetaB, thetaT, 3 * M_PI / 2); 
    } 
    }
    return result;
}

// 初始化序列表：将筛选后的边转换为DataType序列
void InitializeSequenceList(vector<Edge*> R, vector<DataType>& S) {
    int temp = 1; int lb2 = 0;
    for (auto rEdge : R) {
        DataType data;
        // 圆弧边
        if (rEdge->isArc == true) {
            // 判断是否经过y轴极值点
            pair<bool, bool> deCom = isPointofArcfirst(*rEdge);
            // 不经过极值点 → 直接加入
            if (deCom.first == 0 && deCom.second == 0) {
                rEdge->lb2 = lb2; 
                data.rEdge = *rEdge; 
                lb2++; data.type = 0; 
                S.push_back(data); 
            }
            // 经过极值点 → 拆分圆弧
            else {
                // 计算y轴极值点坐标
                double y1 = rEdge->center.y + rEdge->radius; 
                double y2 = rEdge->center.y - rEdge->radius;
                // 经过两个极值点 → 拆分为三段
                if (deCom.first == 1 && deCom.second == 1) {
                    double yfirst, ysecond; 
                    Edge line1, line2, line3;
                    // 确定拆分顺序
                    if (rEdge->Bot.data.y < rEdge->Top.data.y) { 
                        yfirst = y2; ysecond = y1; 
                    }
                    else { 
                        yfirst = y1; 
                    ysecond = y2; 
                    }
                    // 第一段
                    line1 = *rEdge; 
                    int m1 = (rEdge)->Bot.crossing; 
                    int m2 = (rEdge)->Top.crossing;
                    line1.Top = ListData(rEdge->center.x, yfirst); 
                    line1.Top.crossing = m2; 
                    line1.lb2 = lb2; 
                    lb2++; 
                    data.rEdge = line1; 
                    data.type = temp; 
                    S.push_back(data);
                    // 第二段
                    line2 = *rEdge; 
                    line2.Bot = ListData(rEdge->center.x, yfirst);
                    line2.Bot.crossing = m1; 
                    line2.Top = ListData(rEdge->center.x, ysecond); 
                    line2.Top.crossing = m2; 
                    line2.lb2 = lb2; 
                    lb2++; 
                    data.rEdge = line2; 
                    data.type = temp; 
                    S.push_back(data);
                    // 第三段
                    line3 = *rEdge; 
                    line3.Bot = ListData(rEdge->center.x, ysecond); 
                    line3.Bot.crossing = m1; 
                    line3.lb2 = lb2; 
                    lb2++; 
                    data.rEdge = line3; 
                    data.type = temp; 
                    S.push_back(data);
                }
                // 经过一个极值点 → 拆分为两段
                else {
                    double realY; 
                    if (deCom.first == 1) 
                        realY = y1; 
                    else realY = y2;
                    Edge line1, line2;
                    // 第一段
                    line1 = *rEdge; 
                    int m1 = (rEdge)->Bot.crossing; 
                    int m2 = (rEdge)->Top.crossing;
                    line1.Top = ListData(rEdge->center.x, realY); 
                    line1.Top.crossing = m2; 
                    line1.lb2 = lb2; 
                    lb2++; 
                    data.rEdge = line1;
                    data.type = temp; 
                    S.push_back(data);
                    // 第二段
                    line2 = *rEdge; 
                    line2.Bot = ListData(rEdge->center.x, realY); 
                    line2.Bot.crossing = m1; 
                    line2.lb2 = lb2; 
                    lb2++; 
                    data.rEdge = line2;
                    data.type = temp; 
                    S.push_back(data);
                }
            }
            // 交替type值（1/2）
            if (temp == 1) temp = 2; 
            else temp = 1;
        }
        // 直线边 → 直接加入
        else { 
            data.type = 0; 
            rEdge->lb2 = lb2; 
            data.rEdge = *rEdge; 
            S.push_back(data); 
            lb2++; }
    }
}

// 点排序比较器：按d值升序（d为到参考点的距离平方）
bool sortStruct(Point& p1, Point& p2) { 
    return p1.d < p2.d; 
}

// 对点集p按到bot点的距离排序
void sortIpoints(vector<Point>& p, const Point& bot) {
    // 计算每个点到bot的距离平方，存入d
    for (int i = 0; i < p.size(); i++) { 
        double dis = (p[i].x - bot.x) * (p[i].x - bot.x) + (p[i].y - bot.y) * (p[i].y - bot.y); p[i].d = dis; 
    }
    // 排序
    sort(p.begin(), p.end(), sortStruct);
}

// 对S1和S2中的交点集按到边起点的距离排序
void sortS(vector<DataType>& S1, vector<DataType>& S2) {
    for (int i = 0; i < S1.size(); i++) { 
        if (S1[i].iPoints.size() > 1) { 
            vector<Point> temp = S1[i].iPoints; 
            sortIpoints(temp, S1[i].rEdge.Bot.data); 
            S1[i].iPoints = temp; 
        }
    }
    for (int i = 0; i < S2.size(); i++) { 
        if (S2[i].iPoints.size() > 1) { 
            vector<Point> temp = S2[i].iPoints; 
            sortIpoints(temp, S2[i].rEdge.Bot.data); 
            S2[i].iPoints = temp; 
        } 
    }
}

// 带圆弧的布尔运算主函数
// s1/s2：输入形状，res：输出结果，choice：0=并集，1=交集，2=差集
void booleanWithArc(ShapeWithArc& s1, ShapeWithArc& s2, vector<ShapeWithArc>& res, int choice) {
    // 构建s1和s2的边链表
    Edge* path1 = addPath(s1.outer.shape.polyline, s1.outer.shape.edgeData, 1);
    Edge* path2 = addPath(s2.outer.shape.polyline, s2.outer.shape.edgeData, 0);
    
    // 2. 筛选两条轮廓在交集包围盒内的边（减少计算量）
    vector<Edge*> R1, R2;// 存储s1/s2筛选后的边指针 
    vector<Edge> ReFirst, ReSecond;// 存储s1/s2筛选后的边拷贝
    relatedEdge(path1, path2, R1, R2, ReFirst, ReSecond);

    // 3. 初始化扫描线算法所需的变量
    Cutline C; // 扫描线对象
    vector<DataType> S1, S2;// 存储s1/s2的边序列（含交点信息）

    // 4. 将筛选后的边转换为带标记的序列表（处理圆弧拆分）
    InitializeSequenceList(R1, S1); 
    InitializeSequenceList(R2, S2);

    // 5. 读取边到扫描线事件队列，并执行扫描线算法查找所有交点
    C.read_line(S1, S2); // 初始化边的起止点事件
    C.FindIntersection(S1, S2);// 处理所有事件，计算交点

    // 6. 若存在交点，执行布尔运算；否则结果为空
    if (C.intersectpoint.size() != 0) {
        // 6.1 对每条边的交点按到起点的距离排序（保证顺序正确）
        sortS(S1, S2);

        // 6.2 将边链表转换为自定义链表结构（便于遍历）
        List* P1; vector_list(path1, P1); // path1 → 链表P1
        List* P2; vector_list(path2, P2);// path2 → 链表P2

        // 6.3 根据交点拆分原链表，生成新的边链表
        List* P_1a = BuildNewLinkedLists(P1, S1, ReFirst); // 拆分s1的边
        List* P_2a = BuildNewLinkedLists(P2, S2, ReSecond);// 拆分s2的边

        // 6.4 合并拆分后的链表（处理连续边）
        List* P_1 = addList(P_1a); 
        List* P_2 = addList(P_2a);

        // 6.5 存储布尔运算结果的临时变量
        vector<edgeInfo> edge; // 结果边信息
        vector<Point> polygon; // 结果顶点集
        vector<List*> P_3;// 布尔运算后的链表集合

        // 6.6 根据choice执行对应的布尔运算
        if (choice == 0) 
            P_3 = TraverseLinkedLists_union(P_1, P_2, s1, s2);// 并集
        else if (choice == 1)
            P_3 = TraverseLinkedLists_intersect(P_1, P_2, s1, s2);// 交集
        else if (choice == 2) 
            P_3 = TraverseLinkedLists_difference(P_1, P_2, s1, s2);// 差集（s1-s2）


        // 7. 将布尔运算结果的链表转换为ShapeWithArc结构
        int i = 0; // 结果形状索引
        int lable = 0;// 外轮廓标记（1表示已存在外轮廓）
        for (auto ans : P_3) {// 遍历每个结果链表
            ShapeWithArc temp;// 临时形状对象
            Ring hole;// 临时孔对象

            // 7.1 将链表转换为顶点集+边信息集
            ListtoVector(ans, edge, polygon);

            // 7.2 判断多边形是否为逆时针（外轮廓为逆时针，内孔为顺时针）
            if (checkcounter_isCCW(polygon, edge)) {
                // 外轮廓：添加到结果的outer字段
                if (res.empty()) lable = 1;// 第一个外轮廓标记
                res.push_back(temp); 
                res[i].outer.shape.polyline = polygon; 
                res[i].outer.shape.edgeData = edge; 
                i++;
            }
            else {
                // 内孔：添加到最近外轮廓的holes字段
                if (res.size() == 0 || res.size() < i) 
                    res.push_back(temp);// 无外轮廓时先创建空形状
                hole.shape.polyline = polygon;
                hole.shape.edgeData = edge;
                // 关联到对应外轮廓的孔集合
                if (lable == 1) 
                    res[i - 1].holes.push_back(hole); 
                else 
                    res[i].holes.push_back(hole);
                // 清空临时孔对象（避免重复）
                hole.shape.edgeData.clear(); 
                hole.shape.polyline.clear();
            }
            // 清空临时变量，准备处理下一个链表
            polygon.clear(); 
            edge.clear();
        }
        // 8. 结果修正：处理空外轮廓但有孔的情况
        // 若最后一个形状无外轮廓但有孔，将孔合并到前一个形状
        if (res.back().outer.shape.polyline.empty() && !res.back().holes.empty()) { 
            for (auto hole : res.back().holes) 
                res[res.size() - 2].holes.push_back(hole); 
        }
        // 移除空的外轮廓形状
        if (res[res.size() - 1].outer.shape.polyline.empty()) 
            res.pop_back();
    }
    else res.clear();// 无交点时结果为空（布尔运算无重叠）
}
