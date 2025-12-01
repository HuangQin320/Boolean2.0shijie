#pragma once
#pragma once
#include "BasicStructures.h"
#include <vector>
#include <queue>
#include <set>
#include <algorithm>
#include <float.h>

using namespace std;
using namespace copper;

extern double ERROR;// 浮点精度阈值（如1e-5）
extern double CUR_LINE_H;// 扫描线高度相关常量
#define mults 100  // 倍率常量（可能用于坐标放大避免浮点误差）

// 最小包围盒（MBR）结构体：存储矩形的左右上下边界
class Mbr {
public:
    double lx, ly, rx, ry;// lx=左边界x, ly=下边界y, rx=右边界x, ry=上边界y
};

// 链表节点数据结构体：存储点坐标+交点标记
struct ListData {
    Point data;// 点坐标
    int crossing;// 交点标记（0=普通点，1/2=交点）
    // 构造函数：初始化点坐标
    ListData(double x, double y) { data.x = x; data.y = y; }
    ListData() {} // 空构造函数
};

// 边结构体：描述直线/圆弧边的完整信息
struct Edge {
    int nums;// 边编号（用于标识）
    ListData Bot, Curr, Top;// 边的起点(Bot)/当前点(Curr)/终点(Top)
    bool isArc = false;// 是否为圆弧边（false=直线）
    bool cwFlag;// 圆弧方向（true=顺时针，false=逆时针）
    Point center;// 圆弧圆心（直线边无意义）
    double radius;// 圆弧半径（直线边无意义）
    bool lb1; // 内部标记1（未使用/预留）
    int lb2;// 内部标记2（未使用/预留）
    Edge* prev; // 双向链表前驱指针
    Edge* next; // 双向链表后继指针

    Edge() { prev = nullptr; next = nullptr; }// 构造函数：初始化指针为空
    double Int_Line() const;// 计算边与当前扫描线的交点x坐标（直线）
    double Int_Line2() const;// 计算边与当前扫描线的交点x坐标（圆弧）
    // 边相等判断：起点+终点坐标相同则视为同一条边
    bool operator==(const Edge& L1) const { return L1.Bot.data == Bot.data && L1.Top.data == Top.data; }
    // 边比较器（用于set排序）：按扫描线交点x坐标排序
    struct comp { bool operator()(const Edge& L1, const Edge& L2) const; };
};

// 交点节点结构体：存储交点+所属两条边
struct IntersectNode {
    ListData Pt;// 交点坐标
    Edge belong1, belong2;// 交点所属的两条边
    Point Int_inter() const;// 计算交点坐标（通用）
    double Int_interfirst() const;// 计算第一条边在交点处的参数值
    double Int_intersecond() const;// 计算第二条边在交点处的参数值
    // 优先队列比较器：按交点y坐标升序，y相同按x升序
    bool operator()(IntersectNode P1, IntersectNode P2);
};

// 边序列数据结构体：存储边+交点+类型
typedef struct {
    Edge rEdge;// 原始边
    vector<Point> iPoints;// 该边上的所有交点（按顺序）
    int type; // 边类型（0=普通边，>0=拆分后的子边）
} DataType;

// 扫描线类：核心求交逻辑
class Cutline {
public:
    Cutline();// 构造函数：初始化扫描线
    ~Cutline();// 析构函数：释放资源
    // 核心方法：查找所有边的交点
    void FindIntersection(vector<DataType>& S1, vector<DataType>& S2);
    // 处理扫描线事件（起点/终点/交点）
    void HandleEvent(IntersectNode event, vector<DataType>& S1, vector<DataType>& S2);
    // 判断两条边是否相交，返回交点列表
    bool intersec(Edge L1, Edge L2, vector<IntersectNode>& interNodes);
    // 读取边序列，初始化扫描线事件队列
    void read_line(const vector<DataType>& S1, const vector<DataType>& S2);

    // 成员变量
    vector<ListData> intersectpoint;// 所有交点集合（去重后）
    set<Point> repeater;// 重复交点去重集合
    // 扫描线事件优先队列（按y升序处理）
    priority_queue<IntersectNode, vector<IntersectNode>, IntersectNode> Pm;
    // 当前扫描线与哪些边相交（有序集合，按x升序）
    set<Edge, Edge::comp> CurCutLine;
    vector<Edge> Line;// 所有边的缓存

    // 辅助方法：查找当前边的左/右邻居（用于交点检测）
    set<Edge, Edge::comp>::iterator findleft(set<Edge, Edge::comp>::iterator it);
    set<Edge, Edge::comp>::iterator findright(set<Edge, Edge::comp>::iterator it);
};

// 对外导出的核心函数：带圆弧的布尔运算（并/交/差）
COPPER_EXPORT void booleanWithArc(ShapeWithArc& s1, ShapeWithArc& s2, vector<ShapeWithArc>& res, int choice);
// 辅助函数声明
Edge* addPath(const vector<Point>& pg, const vector<edgeInfo>& Info, bool type);// 构建边链表
void InitEdge(Edge* e, Edge* eNext, Edge* ePrev, const vector<Point>& pg, const edgeInfo& Info);// 初始化边属性
void InitEdge2(Edge& e, bool Pt);// 初始化边的点标记
Mbr lineMBR(Edge* head);// 计算边链表的最小包围盒
// 筛选交集包围盒内的边
void relatedEdge(Edge* P1, Edge* P2, vector<Edge*>& R1, vector<Edge*>& R2, vector<Edge>& ReFirst, vector<Edge>& ReSecond);
// 将边转换为带交点的序列表
void InitializeSequenceList(vector<Edge*> R, vector<DataType>& S);
// 对交点按到起点的距离排序
void sortS(vector<DataType>& S1, vector<DataType>& S2);

// 几何函数声明，供其他文件使用
void thetaArc(Edge rEdge, double& thetaB, double& thetaT);// 计算圆弧的起止角度
bool isPointArc(const Point& point, Edge rEdge, const double& thetaB, const double& thetaT);// 判断点是否在圆弧上
pair<Point, Point> intersecArcandLine(Edge& l1, Edge& c1, pair<bool, bool>& res);// 圆弧与直线求交
pair<Point, Point> intersecArcandArc(const Edge& c1, const Edge& c2, pair<bool, bool>& res);// 圆弧与圆弧求交
bool intersecLineandLine(const Point& p1, const Point& p2, const Point& q1, const Point& q2, vector<IntersectNode>& interNodes);// 直线与直线求交
bool IsRectCross(const Point& p1, const Point& p2, const Point& q1, const Point& q2);// 快速排斥实验（判断矩形是否相交）
bool IsLineSegmentCross(const Point& pFirst1, const Point& pFirst2, const Point& pSecond1, const Point& pSecond2);// 判断线段是否相交
double getRotateAngle(double x1, double y1, double x2, double y2);// 计算两点间的旋转角度
