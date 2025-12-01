#pragma once
#pragma once
#include "BasicStructures.h"
#include <string>
#include <vector>

using namespace std;
using namespace copper;

// 从文件读取图形
void read_line_copper(string file_path, ShapeWithArc& shape);

// 将结果写入文件（用于你的可视化工具）
void write_export_copper(const vector<ShapeWithArc>& v1, string output_path);
