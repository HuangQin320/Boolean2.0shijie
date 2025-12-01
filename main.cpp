#include "ConstructSequenceLists.h"
#include "FileIO.h" 
#include <iostream>

int main() {
    // 1. 准备数据
    ShapeWithArc fea1, fea2;

    // --- 方式 A: 使用代码硬编码数据 (保持你之前的测试用例) ---
    // 矩形1: 左下(160, 50), 宽70, 高60
    vector<Point> p1;
    p1.push_back(Point(160, 50));  // 0: 左下
    p1.push_back(Point(230, 50));  // 1: 右下
    p1.push_back(Point(230, 110)); // 2: 右上
    p1.push_back(Point(160, 110)); // 3: 左上
    vector<edgeInfo> edgeInfo1;
    edgeInfo1.push_back(edgeInfo(0, 1)); // 下边
    edgeInfo1.push_back(edgeInfo(1, 2)); // 右边
    edgeInfo1.push_back(edgeInfo(2, 3)); // 上边
    edgeInfo1.push_back(edgeInfo(3, 0)); // 左边

    // 矩形2: 左上(230, 110) (与矩形1接触), 宽70, 高60
    vector<Point> p2;
    p2.push_back(Point(200, 110));  // 0: 左下
    p2.push_back(Point(270, 110));  // 1: 右下
    p2.push_back(Point(270, 170)); // 2: 右上
    p2.push_back(Point(200, 170)); // 3: 左上 
    vector<edgeInfo> edgeInfo2;
    edgeInfo2.push_back(edgeInfo(0, 1)); // 下边
    edgeInfo2.push_back(edgeInfo(1, 2)); // 右边
    edgeInfo2.push_back(edgeInfo(2, 3)); // 上边
    edgeInfo2.push_back(edgeInfo(3, 0)); // 左边



    fea1.outer.shape.polyline = p1;
    fea1.outer.shape.edgeData = edgeInfo1;

    fea2.outer.shape.polyline = p2;
    fea2.outer.shape.edgeData = edgeInfo2;

    // --- 方式 B: 从文件读取 (如果你想用文件，取消下面的注释并修改路径) ---
    // string input_path1 = "D:\\test\\input1.txt";
    // read_line_copper(input_path1, fea1);
    // string input_path2 = "D:\\test\\input2.txt";
    // read_line_copper(input_path2, fea2);

    // 2. 执行布尔运算
    vector<ShapeWithArc> res;
    int choice = 0; // Union

    std::cout << "Starting boolean operation..." << std::endl;
    try {
        booleanWithArc(fea1, fea2, res, choice);
    }
    catch (const std::exception& e) {
        std::cout << "Crash: " << e.what() << endl;
    }

    // 3. 输出结果到终端
    std::cout << "Operation done. Result polygons: " << res.size() << std::endl;
    for (int i = 0; i < res.size(); ++i) {
        std::cout << "Polygon " << i << " vertices: " << res[i].outer.shape.polyline.size() << std::endl;
        for (auto p : res[i].outer.shape.polyline) {
            std::cout << "(" << p.x << ", " << p.y << ")" << std::endl;
        }
    }

    // 4. 保存结果到文件
    string output_path = "D:\\xuexi\\graphic processing\\tuxing02\\mixed01\\result\\output000.txt";
    write_export_copper(res, output_path);

    return 0;
}
