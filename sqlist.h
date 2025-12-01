#pragma once
#pragma once
#include "ConstructSequenceLists.h"
#include <unordered_set>

// 布尔运算结果链表节点结构体
struct List {
    Point data;// 点坐标
    pair<double, Point> tag;// 圆弧标记（first=半径，second=圆心；直线为(0, (0,0))）
    bool cwFlag;// 圆弧方向（true=顺时针）
    bool crossing; // 是否为交点（true=是）
    int io;// 入出标记（1=入，2=出）
    struct List* next;// 后继指针
    struct List* prev;// 前驱指针
};

// Point的哈希函数（用于unordered_set）
struct Point_hash {
    size_t operator()(const Point& r1) const {
        return hash<int>()(r1.x) ^ hash<int>()(r1.y);
    }
};

// 链表基础操作声明
List* ListInit(); // 初始化循环链表（头节点）
// 创建带属性的链表节点
List* BuyNode(Point x, bool crossing, pair<double, Point> tag, bool cwFlag);
List* BuyNode(Point x);// 创建仅带坐标的节点
// 尾插法添加带属性的节点
void ListPushBack(List* phead, Point x, bool crossing, pair<double, Point> tag, bool cwFlag);
void ListPushBack(List* phead, Point x);// 尾插法添加仅带坐标的节点
void ListPopBack(List* phead);// 尾删节点
List* ListFind(List* phead, Point x);// 查找指定坐标的节点
List* addList(List* P);// 合并链表（移除重复尾节点）

// 链表转换/构建声明
void vector_list(Edge* p, List*& l);// 边链表→结果链表
void ListtoVector(List* P, vector<edgeInfo>& edge, vector<Point>& polygon);// 结果链表→顶点+边信息
// 根据交点拆分边链表，生成新链表
List* BuildNewLinkedLists(List* P1, vector<DataType>S1, vector<Edge>R1);

// 布尔运算遍历逻辑声明
vector<List*> TraverseLinkedLists_union(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2);
vector<List*> TraverseLinkedLists_intersect(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2);
vector<List*> TraverseLinkedLists_difference(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2);

// 辅助几何函数声明
bool checkcounter_isCCW(vector<Point> polygon, vector<edgeInfo> edgeData);// 判断多边形是否逆时针
Point currPoint(Point pt1, Point pt2, Point center, double radius, bool cw);// 计算圆弧上的中间点
