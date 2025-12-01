#include "sqlist.h"
#include <cassert>

#define EPSILON 0.00000001// 浮点精度阈值（比ERROR更严格）
#define PIS 3.14159// π的近似值（角度转换）

// === 基础链表操作 ===
// 初始化循环链表（创建头节点，前驱/后继指向自身）
List* ListInit() {
    // 创建头节点（坐标(0,0)，无属性）
    List* phead = BuyNode(Point(0, 0), 0, make_pair(0, Point(0, 0)), 0);
    phead->io = 0; // 入出标记初始化为0
    phead->next = phead;// 循环链表：后继指向自身
    phead->prev = phead;// 循环链表：前驱指向自身
    return phead;
}

// 创建带完整属性的链表节点
List* BuyNode(Point x, bool crossing, pair<double, Point> tag, bool cwFlag) {
    List* newnode = (List*)malloc(sizeof(List));// 动态分配内存
    newnode->data = x; // 点坐标
    newnode->crossing = crossing; // 交点标记
    newnode->tag = tag;// 圆弧属性（半径+圆心）
    newnode->cwFlag = cwFlag; // 圆弧方向
    newnode->io = 0; // 入出标记初始化为0
    newnode->next = NULL; // 后继初始化为空
    newnode->prev = NULL;// 前驱初始化为空
    return newnode;
}

// 创建仅带坐标的链表节点（默认属性）
List* BuyNode(Point x) {
    List* newnode = (List*)malloc(sizeof(List));
    newnode->data = x; // 点坐标
    newnode->tag.first = 0;  // 半径默认0（直线）
    newnode->tag.second = Point(0, 0);// 圆心默认(0,0)
    newnode->crossing = 0; // 非交点
    newnode->io = 0; // 入出标记0
    newnode->next = NULL;// 后继空
    newnode->prev = NULL;// 前驱空
    return newnode;
}

// 尾插法：添加带完整属性的节点到循环链表
void ListPushBack(List* phead, Point x, bool crossing, pair<double, Point> tag, bool cwFlag) {
    assert(phead);// 断言：头节点不能为空（调试时触发）
    List* tail = phead->prev; // 找到尾节点（头节点的前驱）
    List* newnode = BuyNode(x, crossing, tag, cwFlag);// 创建新节点
    newnode->next = phead; // 新节点后继指向头（保持循环）
    phead->prev = newnode; // 头节点前驱指向新节点
    tail->next = newnode; // 原尾节点后继指向新节点
    newnode->prev = tail;// 新节点前驱指向原尾节点
}

// 尾插法：添加仅带坐标的节点到循环链表
void ListPushBack(List* phead, Point x) {
    assert(phead);// 断言头节点有效
    List* tail = phead->prev; // 找到尾节点
    List* newnode = BuyNode(x);// 创建仅带坐标的节点
    newnode->next = phead;// 新节点后继指向头
    phead->prev = newnode;// 头节点前驱指向新节点
    tail->next = newnode;// 原尾节点后继指向新节点
    newnode->prev = tail;// 新节点前驱指向原尾节点
}

// 辅助函数：删除指定节点的前驱节点（内部用）
void ListErase(List* pos) {
    assert(pos);// 断言pos有效
    List* prev = pos->prev->prev;// 找到待删节点的前驱
    free(pos->prev);// 释放待删节点内存
    prev->next = pos; // 前驱节点后继指向pos
    pos->prev = prev;// pos前驱指向prev
}

// 尾删节点：删除链表最后一个节点（头节点除外）
void ListPopBack(List* phead) {
    assert(phead); // 断言头节点有效
    ListErase(phead);// 调用删除函数（删除头节点的前驱=尾节点）
}

// 查找指定坐标的节点（遍历循环链表）
List* ListFind(List* phead, Point x) {
    assert(phead);// 断言头节点有效
    List* cur = phead->next;// 从第一个有效节点开始遍历
    while (cur != phead) { // 遍历到表头结束
        if (cur->data == x) // 坐标匹配（Point的==已重载，带精度）
            return cur;      // 找到则返回节点
        cur = cur->next;// 下一个节点
    }
    return NULL;// 未找到返回空
}

// 边链表（Edge*）转换为结果链表（List*）
void vector_list(Edge* p, List*& l) {
    l = ListInit();// 初始化结果链表
    Edge* cur = p;// 边链表当前节点
    // 先添加边链表最后一个节点的终点（闭合轮廓）
    ListPushBack(l, p->prev->Top.data, (p->prev->Top.crossing == 2 ? 1 : 0), make_pair(p->prev->radius, p->prev->center), p->prev->cwFlag);
    // 遍历边链表（直到回到起点）
    while (cur->next != p) {
        // 添加当前边的终点，转换交点标记（2→1），携带圆弧属性
        ListPushBack(l, cur->Top.data, (cur->Top.crossing == 2 ? 1 : 0), make_pair(cur->radius, cur->center), cur->cwFlag);
        cur = cur->next;// 下一条边
    }
    // 重复添加最后一个点（保证轮廓闭合）
    ListPushBack(l, p->prev->Top.data, (p->prev->Top.crossing == 2 ? 1 : 0), make_pair(p->prev->radius, p->prev->center), p->prev->cwFlag);
}

// === 辅助几何逻辑 ===
// 检查边序列是否所有边都无交点（返回true=无交点）
bool traverseList(vector<DataType>& L) {
    for (int i = 0; i < L.size(); i++) {
        if (L[i].iPoints.empty()) 
            continue;  // 无交点则跳过
        else 
            return false;// 有交点则返回false
    }
    return true;// 所有边都无交点
}

// 判断当前链表节点的边是否在R1中（匹配起点+终点）
bool findEdge(List* cur, vector<Edge>R1) {
    for (auto const edge : R1) {// 遍历R1中的边
        // 匹配：当前节点→下节点 与 边的起点→终点/终点→起点（双向匹配）
        if (cur->data.equal(edge.Bot.data) && cur->next->data.equal(edge.Top.data) || 
            cur->data.equal(edge.Top.data) && cur->next->data.equal(edge.Bot.data)) 
            return true; // 匹配成功
    }
    return false;// 未匹配
}

// 计算圆弧上的中间点（用于判断方向和插值）
Point currPoint(Point pt1, Point pt2, Point center, double radius, bool cw) {
    Point A = pt1; // 圆弧起点
    Point B = pt2;  // 圆弧终点
    Point O = center; // 圆心
    double r = radius;// 半径
    // 向量：终点-圆心，起点-圆心
    Point pt1_vec = B.sub(O);
    Point pt2_vec = A.sub(O);
    Point P;// 输出中间点

    if (cw == true) {// 顺时针圆弧
        // 起点的极角（弧度）
        double altr = Point::angle(O, A); 
        // 旋转角度（两点间的角度/2，转换为弧度）
        double beta = (getRotateAngle(pt1_vec.x, pt1_vec.y, pt2_vec.x, pt2_vec.y)) * PIS / (180 * 2);
        altr -= beta;// 顺时针旋转：极角减小
        // 极坐标转直角坐标
        P.x = O.x + r * cos(altr); 
        P.y = O.y + r * sin(altr);
    }
    else {// 逆时针圆弧
        double altr = Point::angle(O, A); // 起点极角
        double beta = (getRotateAngle(pt2_vec.x, pt2_vec.y, pt1_vec.x, pt1_vec.y)) * PIS / (180 * 2);
        altr += beta; // 逆时针旋转：极角增大
        P.x = O.x + r * cos(altr);
        P.y = O.y + r * sin(altr);
    }
    return P;
}

// 根据交点拆分边链表，生成新的结果链表
List* BuildNewLinkedLists(List* P1, vector<DataType>S1, vector<Edge>R1) {
    List* M_P1 = ListInit(); // 新链表头节点
    List* cur = P1->next;// 原链表当前节点
    // 先添加第一个节点（初始化新链表）
    ListPushBack(M_P1, cur->data, (cur->crossing == 2 ? 1 : 0), cur->tag, cur->cwFlag);
    int j = 0;// S1的索引
    // 遍历原链表（循环链表，直到回到头节点）
    while (cur != P1) {
        if (!findEdge(cur, R1)) { // 当前边不在筛选后的R1中
            if (cur->next != P1)// 不是最后一个节点
                // 直接添加下一个节点（无交点）
                ListPushBack(M_P1, cur->next->data, 0, cur->next->tag, cur->next->cwFlag); }
        else { // 当前边在R1中（需要处理交点）
            if (S1[j].type == 0) {// 普通边（未拆分）
                if (S1[j].iPoints.empty()) {// 无交点
                    j++;// 下一个边
                    if (cur->next != P1) ListPushBack(M_P1, cur->next->data, 0, cur->next->tag, cur->next->cwFlag); }
                else {// 有交点
                    if (S1[j].rEdge.isArc == true) {// 圆弧边
                        // 添加所有交点（标记为交点）
                        for (auto point : S1[j].iPoints) 
                            ListPushBack(M_P1, point, 1, make_pair(S1[j].rEdge.radius, S1[j].rEdge.center), S1[j].rEdge.cwFlag);
                        // 添加边的终点
                        if (cur->next != P1) 
                            ListPushBack(M_P1, cur->next->data, 0, make_pair(S1[j].rEdge.radius, S1[j].rEdge.center), S1[j].rEdge.cwFlag);
                    }
                    else {// 直线边
                        // 添加所有交点
                        for (auto point : S1[j].iPoints) 
                            ListPushBack(M_P1, point, 1, make_pair(0, Point(0, 0)), 0);
                        // 添加边的终点
                        if (cur->next != P1) 
                            ListPushBack(M_P1, cur->next->data, (cur->next->crossing == 2 ? 1 : 0), cur->next->tag, cur->next->cwFlag);
                    }
                    j++;// 下一个边
                }
            }
            else {// 拆分后的子边（type>0）
                int tri = S1[j].type;// 子边类型标记 
                vector<DataType> temp;// 暂存同类型子边
                // 收集所有同类型子边
                while (j < S1.size() && S1[j].type == tri) { 
                    temp.push_back(S1[j]); 
                j++; 
                }
                if (traverseList(temp)) {// 子边无交点
                    // 计算圆弧中间点
                    Point curr = currPoint(cur->data, cur->next->data, cur->next->tag.second, cur->next->tag.first, cur->next->cwFlag);
                    // 添加中间点和终点
                    ListPushBack(M_P1, curr, 0, cur->next->tag, cur->next->cwFlag);
                    ListPushBack(M_P1, cur->next->data, (cur->next->crossing == 2 ? 1 : 0), cur->next->tag, cur->next->cwFlag);
                }
                else { // 子边有交点
                    // 遍历子边，添加交点和中间点
                    for (int k = 0; k < temp.size(); k++) {
                        if (!temp[k].iPoints.empty()) {// 当前子边有交点
                            // 上一个子边也有交点：添加圆弧中间点
                            if (k > 0 && !temp[k - 1].iPoints.empty()) {
                                Point curr = currPoint(temp[k - 1].iPoints.back(), temp[k].iPoints.front(), temp[k].rEdge.center, temp[k].rEdge.radius, temp[k].rEdge.cwFlag);
                                ListPushBack(M_P1, curr, 0, make_pair(temp[k].rEdge.radius, temp[k].rEdge.center), temp[k].rEdge.cwFlag);
                            }
                            // 添加当前子边的所有交点
                            for (int m = 0; m < temp[k].iPoints.size(); m++) 
                                ListPushBack(M_P1, temp[k].iPoints[m], 1, make_pair(temp[k].rEdge.radius, temp[k].rEdge.center), temp[k].rEdge.cwFlag);
                        }
                    }
                    // 添加最后一个子边的终点
                    ListPushBack(M_P1, temp[temp.size() - 1].rEdge.Top.data, (temp[temp.size() - 1].rEdge.Top.crossing == 2 ? 1 : 0), make_pair(temp[temp.size() - 1].rEdge.radius, temp[temp.size() - 1].rEdge.center), temp[temp.size() - 1].rEdge.cwFlag);
                }
            }
        }
        cur = cur->next;// 下一个节点
    }
    return M_P1;// 返回新链表
}

// 合并链表：移除重复尾节点，返回有效节点起始
List* addList(List* P) {
    List* cur = P->next; // 第一个有效节点
    List* P_ans = cur; // 结果链表起始
    ListPopBack(P_ans); // 删除尾节点（重复的闭合点）
    return P_ans;
}

// 判断点是否在线段上（带精度）
bool IsPointOnLine(double px0, double py0, double px1, double py1, double px2, double py2) {
    bool flag = false; 
    // 叉积：判断点是否在直线上（共线）
    double d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
    // 条件1：叉积绝对值<精度（共线）；条件2：点在x/y区间内（在线段上）
    if ((abs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= 0) && ((py0 - py1) * (py0 - py2) <= 0)) 
        flag = true;
    return flag;
}

// 判断点是否在多边形内（射线法，支持圆弧边）
bool Point_In_Polygon(const vector<Point>& POL, vector<edgeInfo>edgeData, Point key) {
    bool isInside = false;// 默认在外部
    int count = 0; // 射线与边的交点数
    double minX = DBL_MAX;// 多边形最小x坐标
    double minR = DBL_MAX;// 最小圆弧半径

    // 计算多边形的最小x坐标和最小圆弧半径
    for (int i = 0; i < edgeData.size(); i++) {
        if (edgeData[i].bCircle == true) minR = std::min(minR, edgeData[i].radius);
        minX = std::min(minX, POL[edgeData[i].startIndex].x); minX = std::min(minX, POL[edgeData[i].endIndex].x);
    }
    // 构造射线：从key向左延伸（x=minX-2*minR，y=key.y）
    double px = key.x; 
    double py = key.y; 
    double linePoint1x = key.x; 
    double linePoint1y = key.y;
    double linePoint2x = minX - abs(2 * minR); 
    double linePoint2y = key.y;

    pair<Point, Point> p;// 存储圆弧与射线的交点
    // 遍历所有边
    for (int i = 0; i < edgeData.size(); i++) {
        Point p1 = POL[edgeData[i].startIndex];// 边起点
        Point p2 = POL[edgeData[i].endIndex];// 边终点
        if (edgeData[i].bCircle == false) {// 直线边
            // 点在直线边上：直接返回在内部
            if (IsPointOnLine(px, py, p1.x, p1.y, p2.x, p2.y)) 
                return true;
            // 边水平：跳过（避免射线重合）
            if (fabs(p2.y - p1.y) < EPSILON) 
                continue;
            // 边的起点/终点在射线上：计数
            if (IsPointOnLine(p1.x, p1.y, linePoint1x, linePoint1y, linePoint2x, linePoint2y)) { 
                if (p1.y > p2.y) count++; 
            }
            else if (IsPointOnLine(p2.x, p2.y, linePoint1x, linePoint1y, linePoint2x, linePoint2y)) { 
                if (p2.y > p1.y) count++;
            }
            // 射线与线段相交：计数
            else if (IsLineSegmentCross(p1, p2, key, Point(linePoint2x, linePoint2y))) 
                count++;
        }
        else {// 圆弧边
            Edge arc, line;
            // 初始化圆弧边
            arc.Bot.data = p1; 
            arc.Top.data = p2; 
            arc.center = edgeData[i].center;
            arc.radius = edgeData[i].radius; 
            arc.cwFlag = edgeData[i].cw;
            // 初始化射线边
            line.Top.data = Point(linePoint1x, linePoint1y); 
            line.Bot.data = Point(linePoint2x, linePoint2y);
         
            // 圆弧与射线求交
            pair<bool, bool> restemp(false, false);
            p = intersecArcandLine(line, arc, restemp);
            // 交点有效则计数
            if (restemp.first == true) count++; 
            if (restemp.second == true) count++;
        }
    }
    // 交点数为奇数：在内部；偶数：在外部
    if (count % 2 == 1) isInside = true; 
    return isInside;
}

// === 入出点判断逻辑 (旧版, 用于 Intersect) ===
// 标记边的入点（io=1）和出点（io=2），用于交集运算
void EntryAndExit(List*& P1, List*& P2, unordered_set<Point, Point_hash>& ipoint, unordered_set<Point, Point_hash>& ipoint_start, List*& temp, ShapeWithArc& s1, ShapeWithArc& s2) {
    List* cur = P1->next; // P1当前节点
    int flag = 0; // 入出标记计数器
    temp = P1->next;// 初始临时节点
    // s2的外轮廓（用于点内判断）
    vector<Point>polygon = s2.outer.shape.polyline; 
    vector<edgeInfo>fans = s2.outer.shape.edgeData;
    // s1的外轮廓
    vector<Point>polygon1 = s1.outer.shape.polyline; 
    vector<edgeInfo>fans1 = s1.outer.shape.edgeData;

    // 处理P1：找到第一个在s2外部的交点（入点）
    while (cur != P1) {
        if (cur->crossing == true) { // 当前是交点
            // 前一个点在s2外部：标记为入点起点
            if (Point_In_Polygon(polygon, fans, cur->prev->data) == false) { 
                temp = cur; 
                break; 
            }
        }
        cur = cur->next;
    }
    // 遍历P1，标记入出点（1=入，2=出）
    while (cur->next != temp) {
        if (cur->crossing == true) {
            cur->io = flag % 2 + 1;// 1/2交替
            if (cur->io == 1) 
                ipoint_start.insert(cur->data);
            flag++;
        }
        cur = cur->next;
    }

    // 处理P2：找到第一个在s1外部的交点
    cur = P2->next; 
    flag = 1;// 初始为出点
    List* temp2 = P2->next;
    while (cur != P2) {
        if (cur->crossing == true) {
            if (Point_In_Polygon(polygon1, fans1, cur->prev->data) == false) { temp2 = cur; break; }
        }
        cur = cur->next;
    }
    // 遍历P2，标记入出点
    while (cur->next != temp2) {
        if (cur->crossing == true) {
            ipoint.insert(cur->data); // 交点存入ipoint
            cur->io = flag % 2 + 1;
            flag++;
        }
        cur = cur->next;
    }
}

// === 入出点判断逻辑 (新版, 用于 Union 和 Difference) ===
void EntryAndExit_d(List*& P1, List*& P2, unordered_set<Point, Point_hash>& ipoint, unordered_set<Point, Point_hash>& vpoint, List*& temp, ShapeWithArc& s1, ShapeWithArc& s2) {
    List* cur = P1; 
    int flag = 1; // 初始为入点
    temp = P1;
    // s2外轮廓
    vector<Point>polygon = s2.outer.shape.polyline; 
    vector<edgeInfo>fans = s2.outer.shape.edgeData;
    // s1外轮廓
    vector<Point>polygon1 = s1.outer.shape.polyline;
    vector<edgeInfo>fans1 = s1.outer.shape.edgeData;

    // 收集P1中在s2外部的非交点
    while (cur->next != P1) {
        if (cur->crossing != true && Point_In_Polygon(polygon, fans, cur->data) != true) 
            vpoint.insert(cur->data);
        cur = cur->next;
    }
    // 处理P1：找到第一个在s2外部的交点（入点）
    cur = P1;
    while (cur->next != P1) {
        if (cur->crossing == true) { 
            if (Point_In_Polygon(polygon, fans, cur->prev->data) == false) { 
                temp = cur; 
                break; 
            } 
        }
        cur = cur->next;
    }
    cur->io = 1; // 标记为入点
    cur = cur->next;
    // 遍历P1，标记入出点
    while (cur != temp) {
        if (cur->crossing == true) { 
            cur->io = flag % 2 + 1; 
            flag++; }
        if (cur->io == 2) 
            vpoint.insert(cur->data);
        cur = cur->next;
    }

    // 处理P2：找到第一个在s1外部的交点
    cur = P2; 
    flag = 1; 
    List* temp2 = P2;
    while (cur->next != P2) {
        if (cur->crossing == true) { 
            if (Point_In_Polygon(polygon1, fans1, cur->prev->data) == false) { 
                temp2 = cur;
                break; 
            } 
        }
        cur = cur->next;
    }
    // 遍历P2，标记入出点
    while (cur->next != temp2) {
        if (cur->crossing == true) { 
            ipoint.insert(cur->data); 
            cur->io = flag % 2 + 1;
            flag++; 
        }
        cur = cur->next;
    }
}

// === 遍历逻辑：并集 (Union) ===
vector<List*> TraverseLinkedLists_union(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2) {
    vector<List*> P3; // 结果链表集合
    List* cur, * Nodei;// 当前节点/起始节点
    int j = 1; // 遍历轮次
    int lable; // 标记当前遍历的链表（1=P1，2=P2）
    // 交点集合（P2的交点）/外部点集合（P1的外部点）
    unordered_set<Point, Point_hash> vpoint; 
    unordered_set<Point, Point_hash> ipoint;
    // 标记入出点
    EntryAndExit_d(P1, P2, ipoint, vpoint, Nodei, s1, s2);

    do {
        if (j == 1) { // 第一轮：从P1的第一个节点开始
            Nodei = P1->next; 
            lable = 1; 
        }
        else {// 后续轮次：从P1的外部点开始
            Nodei = ListFind(P1, *vpoint.begin()); 
            lable = 1; 
        }
        List* list1 = ListInit(); // 结果链表
        cur = Nodei;// 当前节点
        if (j == 1) { // 第一轮：找到第一个入点
            while (cur->next->io != 1) 
                cur = cur->next;
        }
        List* temp = cur;  // 保存起始节点（闭合用）
        ipoint.erase(cur->data);// 移除已处理的交点

        // 遍历闭合轮廓
        do {
            vpoint.erase(cur->data);// 移除已处理的外部点
            if (cur->next->crossing != true) {  // 下一个点不是交点：继续遍历当前链表
                cur = cur->next; 
            }
            else {// 下一个点是交点：切换链表（P1↔P2）
                if (lable == 1) {// 当前在P1，切换到P2
                    List* Noded = ListFind(P2, cur->next->data);// 找到P2中对应的交点
                    // 携带圆弧属性
                    pair<double, Point> p1 = cur->next->tag; 
                    bool temp = cur->next->cwFlag;
                    cur = Noded; 
                    cur->tag = p1; 
                    cur->cwFlag = temp;
                    ipoint.erase(Noded->data);
                    lable = 2;// 标记为P2
                }
                else {// 当前在P2，切换到P1
                    List* Noded = ListFind(P1, cur->next->data);
                    pair<double, Point> p1 = cur->next->tag;
                    bool temp = cur->next->cwFlag;
                    cur = Noded; 
                    cur->tag = p1; 
                    cur->cwFlag = temp; 
                    ipoint.erase(Noded->data);
                    lable = 1;// 标记为P1
                }
            }
            // 将当前节点添加到结果链表（排除表头）
            if (cur != P1 && cur != P2)
                ListPushBack(list1, cur->data, cur->crossing, cur->tag, cur->cwFlag);
        } while (cur != temp);// 回到起始节点，闭合轮廓
        P3.push_back(list1);// 添加到结果集合
        j++;// 下一轮
    } while (!ipoint.empty());// 所有交点处理完毕
    return P3;
}

// === 遍历逻辑：交集 (Intersect) ===
vector<List*> TraverseLinkedLists_intersect(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2) {
    vector<List*> P3; 
    List* cur, * Nodei; 
    int j = 1; 
    int lable;
    // 入点集合（P1）/交点集合（P2）
    unordered_set<Point, Point_hash> ipoint_start; 
    unordered_set<Point, Point_hash> ipoint;
    //// 标记入出点
    EntryAndExit(P1, P2, ipoint, ipoint_start, Nodei, s1, s2);
    do {
        if (j == 1) { // 第一轮：从标记的起始节点开始
            Nodei = Nodei; 
            lable = 1;
        }
        else { // 后续轮次：从P1的入点开始
            Nodei = ListFind(P1, *ipoint_start.begin()); 
            lable = 1; 
        }
        cur = Nodei;
        // 移除已处理的入点/交点
        if (cur->io == 1) 
            ipoint_start.erase(cur->data);
        ipoint.erase(cur->data);

        List* list1 = ListInit();// 结果链表
        // 遍历闭合轮廓
        do {
            if (cur->next->crossing != true) {// 非交点：继续遍历
                cur = cur->next;
            }
            else {// 交点：切换链表
                if (lable == 1) {// P1→P2
                    List* Noded = ListFind(P2, cur->next->data);
                    pair<double, Point> p1 = cur->next->tag;
                    bool temp = cur->next->cwFlag;
                    cur = Noded; 
                    cur->cwFlag = temp; 
                    cur->tag = p1;
                    // 移除已处理的入点/交点
                    if (cur->io == 1) ipoint_start.erase(cur->data);
                    ipoint.erase(Noded->data); 
                    lable = 2;
                }
                else {// P2→P1
                    List* Noded = ListFind(P1, cur->next->data);
                    pair<double, Point> p1 = cur->next->tag; 
                    bool temp = cur->next->cwFlag;
                    cur = Noded; 
                    cur->cwFlag = temp; 
                    cur->tag = p1;
                    if (cur->io == 1)
                        ipoint_start.erase(cur->data);
                    ipoint.erase(Noded->data);
                    lable = 1;
                }
            }
            // 添加到结果链表
            if (cur != P1 && cur != P2)
                ListPushBack(list1, cur->data, cur->crossing, cur->tag, cur->cwFlag);
        } while (cur != Nodei);// 闭合轮廓

        P3.push_back(list1);
        j++;
    } while (!ipoint.empty());// 所有交点处理完毕

    return P3;
}

// === 遍历逻辑：差集 (Difference) ===
vector<List*> TraverseLinkedLists_difference(List* P1, List* P2, ShapeWithArc& s1, ShapeWithArc& s2) {
    vector<List*> P3; 
    List* cur, * Nodei; 
    int j = 1; 
    int lable = 0;// 遍历链表标记
    // 交点集合（P2）/外部点集合（P1）
    unordered_set<Point, Point_hash> ipoint; 
    unordered_set<Point, Point_hash> vpoint;
    // 标记入出点
    EntryAndExit_d(P1, P2, ipoint, vpoint, Nodei, s1, s2);
    do {
        if (j == 1) {// 第一轮：从P1第一个节点开始
            Nodei = P1->next; 
            lable = 1; 
        }
        else {// 后续轮次：从P1外部点开始
            Nodei = ListFind(P1, *vpoint.begin()); 
            lable = 1;
        }
        cur = Nodei; 
        List* list1 = ListInit();// 结果链表

        // 遍历闭合轮廓
        do {
            vpoint.erase(cur->data); // 移除已处理的外部点
            // 非交点 或 标记为2时的非交点
            if ((lable == 1 && cur->next->crossing != true) || (lable == 2 && cur->prev->crossing != true)) {
                if (lable == 1) {// P1：向后遍历
                    List* temp = cur; 
                    cur = cur->next;
                    // 坐标不重复则添加
                    if (!(temp->data.equal(cur->data))) 
                        ListPushBack(list1, cur->data, cur->crossing, cur->tag, cur->cwFlag);
                }
                else {// P2：向前遍历（反向）
                    List* temp = cur;
                    cur = cur->prev;
                    // 圆弧方向取反
                    if (!(temp->data.equal(cur->data))) 
                        ListPushBack(list1, cur->data, cur->crossing, cur->next->tag, !(cur->next->cwFlag));
                }
            }
            else {// 交点：切换链表
                if (lable == 1) {// P1→P2
                    List* Noded = ListFind(P2, cur->next->data);
                    pair<double, Point> p1 = cur->next->tag; 
                    bool m = cur->next->cwFlag;
                    cur = Noded; pair<double, Point>temp = p1; 
                    lable = 2;
                    ipoint.erase(Noded->data);// 移除交点
                    // 添加节点（携带属性）
                    ListPushBack(list1, cur->data, cur->crossing, temp, m);
                }
                else {// P2→P1（反向）
                    List* Noded = ListFind(P1, cur->prev->data);
                    pair<double, Point> p1 = cur->prev->tag;
                    bool m = cur->prev->cwFlag;
                    cur = Noded; 
                    pair<double, Point>temp = p1;
                    // 圆弧方向取反
                    ListPushBack(list1, cur->data, cur->crossing, temp, !m);
                    ipoint.erase(Noded->data);
                    lable = 1;
                }
            }
        } while (cur != Nodei);// 闭合轮廓

        P3.push_back(list1); 
        j++;
    } while (vpoint.size() != 0);// 所有外部点处理完毕

    return P3;
}

// 结果链表转换为顶点+边信息（适配ShapeWithArc结构）
void ListtoVector(List* P, vector<edgeInfo>& edge, vector<Point>& polygon) {
    // 先添加第一个节点（闭合用）
    ListPushBack(P, P->next->data, P->next->crossing, P->next->tag, P->next->cwFlag);
    int flag = 0; // 顶点索引
    List* cur = P->next; 
    polygon.push_back(cur->data);// 添加第一个顶点

    // 遍历链表
    while (cur->next != P) {
        if (cur->next->tag.first != 0) {// 圆弧边（半径≠0）
            polygon.push_back(cur->next->data);// 添加顶点
            // 创建圆弧边信息（起点索引/终点索引/圆心/半径/方向）
            edgeInfo temp(flag, flag + 1, cur->next->tag.second, cur->next->tag.first, cur->next->cwFlag);
            edge.push_back(temp); 
            flag++;// 索引递增
        }
        else {// 直线边
            polygon.push_back(cur->next->data);// 添加顶点
            edgeInfo temp(flag, flag + 1);// 直线边信息（仅起点/终点索引）
            edge.push_back(temp); 
            flag++;
        }
        cur = cur->next;
    }
    polygon.erase(polygon.end() - 1);// 移除重复的闭合顶点
    edge[edge.size() - 1].endIndex = 0; // 最后一条边的终点索引设为0（闭合）
}

// 判断多边形是否为逆时针（外轮廓=逆时针，内孔=顺时针）
bool checkcounter_isCCW(vector<Point>polygon, vector<edgeInfo>edgeData) {
    vector<Point>temp_poly;// 插值后的顶点（含圆弧中间点）
    // 遍历所有边，插值圆弧中间点
    for (auto edge : edgeData) {
        temp_poly.push_back(polygon[edge.startIndex]);// 边起点
        if (edge.bCircle == true) {// 圆弧边：添加中间点
            Point temp = currPoint(polygon[edge.startIndex], polygon[edge.endIndex], edge.center, edge.radius, edge.cw);
            temp_poly.push_back(temp);
        }
        temp_poly.push_back(polygon[edge.endIndex]);// 边终点
    }
    // 计算多边形面积的2倍（叉积求和）
    int n = temp_poly.size(); double S = 0;
    for (int i = 0; i < n; i++) 
        S += temp_poly[i].x * temp_poly[(i + 1) % n].y - temp_poly[i].y * temp_poly[(i + 1) % n].x;
    // 面积>0：逆时针；<0：顺时针
    if (S * 0.5 > 0) return true; else return false;
}
