#include "FileIO.h"
#include <fstream>
#include <sstream>
#include <iostream>

// 辅助函数：解析文件中的块
vector<vector<string>> read_cut(ifstream& fin, string str) {
    vector<vector<string>> ans;
    vector<string> v1;
    // 假设 str 是类似 "5" 这样的数字字符串，表示后面有几行
    // 但原代码逻辑似乎是getline读入数量
    string countStr;
    // 如果传入的str已经是数量，或者需要新读一行，这里根据原逻辑适配
    // 通常这里是读取行数
    getline(fin, countStr);
    int n = 0;
    try {
        n = stoi(countStr);
    }
    catch (...) {
        n = 0;
    }

    for (int i = 0; i < n; i++) {
        string input;
        getline(fin, input);
        stringstream stringin(input);
        string s;
        while (stringin >> s) {
            v1.push_back(s);
        }
        ans.push_back(v1);
        v1.clear();
    }
    return ans;
}

void read_line_copper(string file_path, ShapeWithArc& shape) {
    ifstream fin(file_path);
    if (!fin) {
        cerr << "Error: Cannot open file " << file_path << endl;
        return; // 或者抛出异常
    }

    string str;
    vector<vector<string>> v1;
    vector<Point> polyline;
    vector<edgeInfo> edges;

    // 简单的状态机解析
    while (getline(fin, str)) {
        // 去除可能的行尾回车符
        if (!str.empty() && str.back() == '\r') str.pop_back();

        if (str == "RingPoints_Begin") {
            v1 = read_cut(fin, str);
            for (auto res : v1) {
                // 假设格式是: index x y
                if (res.size() >= 3) {
                    Point p(stod(res[1]), stod(res[2]));
                    polyline.push_back(p);
                }
            }
        }
        else if (str == "RingEdge_Begin") {
            v1 = read_cut(fin, str);
            for (auto res : v1) {
                // 假设格式: index start end centerX centerY radius cw bCircle
                // 需要根据实际文件格式调整索引，这里按原代码逻辑恢复
                if (res.size() >= 8) {
                    bool isCircle = (stoi(res[7]) != 0);
                    if (isCircle) {
                        edgeInfo edge(stoi(res[1]), stoi(res[2]),
                            Point(stod(res[3]), stod(res[4])),
                            stod(res[5]), stoi(res[6]));
                        edges.push_back(edge);
                    }
                    else {
                        edgeInfo edge(stoi(res[1]), stoi(res[2]));
                        edges.push_back(edge);
                    }
                }
            }
        }
        else if (str == "Holes_Begin") {
            // 原代码这里是空的，暂时保留为空
        }
    }

    fin.close();

    shape.outer.shape.polyline = polyline;
    shape.outer.shape.edgeData = edges;
    // 简单验证
    cout << "Read Shape: " << polyline.size() << " points, " << edges.size() << " edges." << endl;
}

void write_export_copper(const vector<ShapeWithArc>& v1, string output_path) {
    fstream all;
    all.open(output_path, ios::out);
    if (!all) {
        cerr << "Error: Cannot create output file " << output_path << endl;
        return;
    }

    // 1. 先输出外轮廓
    for (auto poly : v1) {
        vector<Point> points = poly.outer.shape.polyline;
        vector<edgeInfo> edges = poly.outer.shape.edgeData;

        // 你的工具似乎依赖这种格式：Polygoniner 或 Polygonarc + ! 分隔
        for (auto t : edges) {
            // 防止索引越界
            if (t.startIndex >= points.size() || t.endIndex >= points.size()) continue;

            if (t.bCircle == false) {
                all << "Polygoniner" << "!";
                all << points[t.startIndex].x << "!";
                all << points[t.startIndex].y << "!";
                all << points[t.endIndex].x << "!";
                all << points[t.endIndex].y << "!";
                all << "\n";
            }
            else {
                all << "Polygonarc" << "!";
                all << t.center.x << "!";
                all << t.center.y << "!";
                all << points[t.startIndex].x << "!";
                all << points[t.startIndex].y << "!";
                all << points[t.endIndex].x << "!";
                all << points[t.endIndex].y << "!";
                all << t.radius << "!";
                all << t.cw << "!";
                all << "\n";
            }
        }
    }

    // 2. 输出孔洞 (Holes)
    for (auto poly : v1) {
        if (!poly.holes.empty()) {
            for (auto m : poly.holes) {
                vector<Point> points = m.shape.polyline;
                vector<edgeInfo> edges = m.shape.edgeData;
                for (auto t : edges) {
                    if (t.startIndex >= points.size() || t.endIndex >= points.size()) continue;

                    if (t.bCircle == false) {
                        all << "Polygoniner" << "!";
                        all << points[t.startIndex].x << "!";
                        all << points[t.startIndex].y << "!";
                        all << points[t.endIndex].x << "!";
                        all << points[t.endIndex].y << "!";
                        all << "\n";
                    }
                    else {
                        all << "Polygonarc" << "!";
                        all << t.center.x << "!";
                        all << t.center.y << "!";
                        all << points[t.startIndex].x << "!";
                        all << points[t.startIndex].y << "!";
                        all << points[t.endIndex].x << "!";
                        all << points[t.endIndex].y << "!";
                        all << t.radius << "!";
                        all << t.cw << "!";
                        all << "\n";
                    }
                }
            }
        }
    }

    all.close();
    cout << "Result saved to: " << output_path << endl;
}
