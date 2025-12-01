#pragma once
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 定义为空，移除DLL导出需求
#define COPPER_EXPORT 

// 定义copper命名空间，封装几何相关类和结构体
namespace copper {

    // 点类：表示二维平面中的点，包含坐标及几何运算方法
    class Point {
    public:
        // 默认构造函数：初始化点坐标为(0,0)，d为辅助变量（距离/角度等）
        Point() {
            this->x = 0;
            this->y = 0;
            this->d = 0;
        }

        // 带参数构造函数：初始化点坐标为(x,y)，d初始化为0
        Point(double x, double y) {
            this->x = x;
            this->y = y;
            this->d = 0;
        }

    public:
        // 获取x坐标（const保证不修改对象）
        double getX() const { return x; }
        // 设置x坐标
        void setX(double x_) { x = x_; }
        // 获取y坐标（const保证不修改对象）
        double getY() const { return y; }
        // 设置y坐标
        void setY(double y_) { y = y_; }

        // 计算当前点到目标点p的欧氏距离
        double distance(const Point& p) const {
            return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
        }

        // 点加法：返回当前点与p点坐标相加后的新点
        Point add(const Point& p) const {
            return Point(x + p.x, y + p.y);
        }

        // 点减法：返回当前点与p点坐标相减后的新点
        Point sub(const Point& p) const {
            return Point(x - p.x, y - p.y);
        }

        // 点缩放：返回当前点坐标乘以缩放因子s后的新点
        Point scale(const float s) const {
            return Point(s * x, s * y);
        }

        // 点旋转：将点绕原点逆时针旋转90度，返回新点（(x,y)→(-y,x)）
        Point turn() const {
            return Point(-y, x);
        }
        // 公有成员变量：x/y坐标，d为辅助变量（存储距离、角度等临时值）
        double x, y, d;

        // 计算点的模长（到原点的距离）
        double abs() { return sqrt(norm()); }
        // 计算点的模长平方（避免开方，提升性能）
        double norm() { return x * x + y * y; }
        // 计算两个点的点积（内积）
        double Dot(Point& a, Point& b) { return a.x * b.x + a.y * b.y; }
        // 静态方法：计算两个点的叉积（外积）
        static double Cross(Point& a, Point& b) { return a.x * b.y - a.y * b.x; }

        // 静态方法：计算从a1到a2的向量与x轴正方向的夹角（弧度）
        static double angle(Point a1, Point a2) {
            // 垂直向上/向下的特殊情况
            if (a1.x == a2.x) {
                if (a2.y - a1.y > 0) return M_PI * 0.5; // 向上：90度
                else return -M_PI * 0.5; // 向下：-90度
            }
            // 通用情况：atan2计算反正切（范围[-π, π]）
            else return atan2((a2.y - a1.y), (a2.x - a1.x));
        }

        Point operator +(const Point& b)const { return Point(x + b.x, y + b.y); }
        Point operator -(const Point& b)const { return Point(x - b.x, y - b.y); }
        Point operator / (double a) { return Point(x / a, y / a); }
        double operator *(const Point& b)const { return x * b.x + y * b.y; }
        bool operator<(const Point& rhs) const { return (x < rhs.x) || (x == rhs.x && y < rhs.y); }
        bool operator>(const Point& rhs) const { return rhs < *this; }
        bool operator<=(const Point& rhs) const { return !(rhs < *this); }
        bool operator>=(const Point& rhs) const { return !(*this < rhs); }

        // 重载==运算符：浮点精度范围内判断两点是否相等（误差1e-6）
        bool operator==(const Point& P1) const {
            if (fabs(x - P1.x) < 0.000001 && fabs(y - P1.y) < 0.000001) return true;
            return false;
        }

        // 严格相等判断（无浮点误差容忍）
        bool equal(const Point& p) const { return x == p.x && y == p.y; }
    };

    // 圆类：表示二维平面中的圆，包含圆心和半径
    class Circle {
    public:
        // 默认构造函数
        Circle() {}
        // 带参数构造函数：初始化圆心和半径
        Circle(const Point& center, double r) : center_(center), radius_(r) {};
        // 获取圆心（const保证不修改）
        const Point& getCenter() const { return center_; }
        // 设置圆心
        void setCenter(const Point& center__) { Circle::center_ = center__; }
        // 获取半径（const保证不修改）
        double getRadius() const { return radius_; }
        // 设置半径
        void setRadius(double radius__) { Circle::radius_ = radius__; }
        Point center_;// 公有成员：圆心
        double radius_;// 公有成员：半径
    };

    // 边信息结构体：存储边的索引、圆弧参数（圆心/半径/顺时针）等
    struct edgeInfo {
        int startIndex;// 边起点在点集中的索引
        int endIndex;// 边终点在点集中的索引
        Point center; // 圆弧圆心（仅圆弧边有效）
        double radius;// 圆弧半径（仅圆弧边有效）
        bool cw;// 圆弧方向：true=顺时针，false=逆时针
        bool bCircle; // 是否为圆弧边：true=圆弧，false=直线

        edgeInfo() {}// 默认构造函数
        // 直线边构造函数：初始化起止索引，默认非圆弧
        edgeInfo(int s, int e) {
            this->startIndex = s; this->endIndex = e; this->bCircle = false;
        }
        // 圆弧边构造函数：初始化所有参数
        edgeInfo(int s, int e, Point cen, double r, bool cw) {
            this->startIndex = s; this->endIndex = e; this->center = cen; this->radius = r; this->bCircle = true; this->cw = cw;
        }
    };

    // 带圆弧的多边形结构体：包含顶点集和边信息集
    struct PolyWithArc {
        std::vector<Point> polyline;// 多边形顶点/圆弧端点集合
        std::vector<edgeInfo> edgeData; // 边信息集合（直线/圆弧属性）
    };

    // 环结构体：表示带圆弧的闭合环（外轮廓/内孔）
    struct Ring {      
        PolyWithArc shape;// 环的几何形状（带圆弧的多边形）      
        Circle c;// 若为完整圆形环，存储圆参数        
        bool bCircle;// 是否为完整圆形：true=圆形，false=带圆弧的多边形
        Ring() { bCircle = false; }// 构造函数：默认非圆形环
    };

    // 带圆弧的形状结构体：包含外轮廓和内孔集合
    struct ShapeWithArc {
        Ring outer;// 外轮廓环
        std::vector<Ring> holes;// 内孔环集合
        bool flag = true;// 形状有效性标志：true=有效，false=无效
    };
}
