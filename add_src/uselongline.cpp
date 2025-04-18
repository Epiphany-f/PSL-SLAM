#include "uselongline.h"



float PointLineDistance(Eigen::Vector4f line, Eigen::Vector2f point)
{
    float x0 = point(0);
    float y0 = point(1);
    float x1 = line(0);
    float y1 = line(1);
    float x2 = line(2);
    float y2 = line(3);
    float d = (std::fabs((y2 - y1) * x0 + (x1 - x2) * y0 + ((x2 * y1) - (x1 * y2)))) / (std::sqrt(std::pow(y2 - y1, 2) + std::pow(x1 - x2, 2)));
    return d;
}

float AngleDiff(float &angle1, float &angle2)
{
    float d_angle_case1 = std::abs(angle2 - angle1);
    float d_angle_case2 = M_PI + std::min(angle1, angle2) - std::max(angle1, angle2);
    return std::min(d_angle_case1, d_angle_case2);
}

void MergeLines(std::vector<cv::Vec4f> &source_lines, std::vector<cv::Vec4f> &dst_lines,
                float angle_threshold, float distance_threshold, float endpoint_threshold)
{
    // 获取输入线段数量
    size_t source_line_num = source_lines.size();
    // 将所有线段数据拷贝到 Eigen 数组中，并分别提取 x1, y1, x2, y2 四个变量
    Eigen::Array4Xf line_array = Eigen::Map<Eigen::Array4Xf>(source_lines[0].val, 4, source_lines.size());
    Eigen::ArrayXf x1 = line_array.row(0);
    Eigen::ArrayXf y1 = line_array.row(1);
    Eigen::ArrayXf x2 = line_array.row(2);
    Eigen::ArrayXf y2 = line_array.row(3);
    // std::cout << x1 << std::endl;
    // 计算每条线段的长度和角度，并将角度按从小到大排序
    Eigen::ArrayXf dx = x2 - x1;
    Eigen::ArrayXf dy = y2 - y1;
    Eigen::ArrayXf eigen_angles = (dy / dx).atan();
    Eigen::ArrayXf length = (dx * dx + dy * dy).sqrt();
    std::vector<float> angles(&eigen_angles[0], eigen_angles.data() + eigen_angles.cols() * eigen_angles.rows()); // 将Eigen数组转换成std::vector，方便进行排序操作。
    std::vector<size_t> indices(angles.size());                                                                   //  用来存储排序后的索引，其中angles.size()代表需要排序的元素个数
    // Eigen::ArrayXf(indices.begin(), indices.end());
    indices.assign(indices.size(), 0); // 将indices数组的元素全部初始化为0。
    for (size_t a = 0; a < angles.size(); a++)
    {
        indices[a] = a;
    }
    std::sort(indices.begin(), indices.end(), [&angles](size_t i1, size_t i2)
              { return angles[i1] < angles[i2]; }); // 对角度进行排序，使用lambda函数指定排序方式。

    // 设置阈值和常数
    float angle_thr = angle_threshold;
    float distance_thr = distance_threshold;
    float ep_thr = endpoint_threshold * endpoint_threshold;
    float quater_PI = M_PI / 4.0; // Π

    std::vector<std::vector<size_t>> neighbors; // 声明一个名为 neighbors 的二维向量，存储每个源线相邻线的索引
    neighbors.resize(source_line_num);          // 将的大小调整为 source_line_num，即源线数目。
    std::vector<bool> sort_by_x;                // 该向量指示是否按 x 轴排序。
    for (size_t i = 0; i < source_line_num; i++)
    {                             // 遍历所有的源线
        size_t idx1 = indices[i]; // 从索引列表中获取源线的索引
        float x11 = source_lines[idx1](0);
        float y11 = source_lines[idx1](1);
        float x12 = source_lines[idx1](2);
        float y12 = source_lines[idx1](3);
        float angle1 = angles[idx1];
        bool to_sort_x = (std::abs(angle1) < quater_PI); // 判断是否按 x 轴排序
        sort_by_x.push_back(to_sort_x);                  // 如果应该按 x 轴排序并且源线的终点在起点的左边，
        // 或者不应该按 x 轴排序并且源线的终点在起点的上面，则交换起点和终点的坐标。
        if ((to_sort_x && (x12 < x11)) || ((!to_sort_x) && y12 < y11))
        {
            std::swap(x11, x12);
            std::swap(y11, y12);
        }

        for (size_t j = i + 1; j < source_line_num; j++)
        {
            size_t idx2 = indices[j];
            float x21 = source_lines[idx2](0);
            float y21 = source_lines[idx2](1);
            float x22 = source_lines[idx2](2);
            float y22 = source_lines[idx2](3);
            if ((to_sort_x && (x22 < x21)) || ((!to_sort_x) && y22 < y21))
            {
                std::swap(x21, x22);
                std::swap(y21, y22);
            }

            // 角度阈值
            float angle2 = angles[idx2];
            float d_angle = AngleDiff(angle1, angle2);
            if (d_angle > angle_thr)
            {
                if (std::abs(angle1) < (M_PI_2 - angle_threshold))
                {
                    break;
                }
                else
                {
                    continue;
                }
            }

            // 检查线中点与另一条直线的距离
            float mid_x1 = 0.5 * (source_lines[idx1](0) + source_lines[idx1](2));
            float mid_y1 = 0.5 * (source_lines[idx1](1) + source_lines[idx1](3));
            float mid_x2 = 0.5 * (source_lines[idx2](0) + source_lines[idx2](2));
            float mid_y2 = 0.5 * (source_lines[idx2](1) + source_lines[idx2](3));
            Eigen::Vector2f mid1(mid_x1, mid_y1);
            Eigen::Vector2f mid2(mid_x2, mid_y2);
            // Eigen::Vector2f mid1 = 0.5 * (source_lines[idx1](0) + source_lines[idx1](3));
            // Eigen::Vector2f mid2 = 0.5 * (source_lines[idx2](0)+ source_lines[idx2](3));
            std::vector<Eigen::Vector4f> source_idx = cvVecToEigenVec(source_lines);
            // Eigen::Vector4f source_idx2 = Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(source_lines[idx2].val,4);
            float mid1_to_line2 = PointLineDistance(source_idx[idx2], mid1);
            float mid2_to_line1 = PointLineDistance(source_idx[idx1], mid2);
            if (mid1_to_line2 > distance_thr && mid2_to_line1 > distance_thr)
                continue;

            // 最近两端点的距离
            float cx12, cy12, cx21, cy21;
            if ((to_sort_x && x12 > x22) || (!to_sort_x && y12 > y22))
            {
                cx12 = x22;
                cy12 = y22;
                cx21 = x11;
                cy21 = y11;
            }
            else
            {
                cx12 = x12;
                cy12 = y12;
                cx21 = x21;
                cy21 = y21;
            }
            bool to_merge = ((to_sort_x && cx12 >= cx21) || (!to_sort_x && cy12 >= cy21));
            if (!to_merge)
            {
                float d_ep = (cx21 - cx12) * (cx21 - cx12) + (cy21 - cy12) * (cy21 - cy12);
                to_merge = (d_ep < ep_thr);
            }

            if (to_merge)
            {
                neighbors[idx1].push_back(idx2);
                neighbors[idx2].push_back(idx1);
            }
        }
    }

    // 进行聚类，
    std::vector<int> cluster_codes(source_line_num, -1); // 大小，初始值
    std::vector<std::vector<size_t>> cluster_ids;        //
    for (size_t i = 0; i < source_line_num; i++)
    { // 遍历source_line_num个元素
        if (cluster_codes[i] >= 0)
            continue; // 如果cluster_codes[i]大于等于0，则跳过当前循环

        size_t new_code = cluster_ids.size();            // 获取cluster_ids的大小，并将其赋值给new_code
        cluster_codes[i] = new_code;                     // 将new_code的值赋给cluster_codes[i]
        std::vector<size_t> to_check_ids = neighbors[i]; // 获取neighbors[i]的值，并将其赋值给to_check_ids
        std::vector<size_t> cluster;                     // 创建一个空的大小为0的size_t类型向量，命名为cluster
        cluster.push_back(i);                            // 创建一个vector，用于记录新簇中包含的行号，将当前行加入该vector
        while (to_check_ids.size() > 0)
        {                         // 以当前行为中心，向周围扩展，将邻居行加入到当前簇中
            std::set<size_t> tmp; // 用set去重，记录所有需要遍历的邻居行号
            for (auto &j : to_check_ids)
            {
                if (cluster_codes[j] < 0)
                { // 如果该邻居行已经被归类到某个簇中，则跳过
                    cluster_codes[j] = new_code;
                    cluster.push_back(j); // 将该行归类到当前簇中，并将其行号加入到cluster中
                }

                std::vector<size_t> j_neighbor = neighbors[j]; // 获取该邻居行的邻居行
                for (auto &k : j_neighbor)
                {
                    if (cluster_codes[k] < 0)
                    {
                        tmp.insert(k); // 将未归类的邻居行号加入到tmp中
                    }
                }
            }
            to_check_ids.clear(); // 将tmp中的行号赋值给to_check_ids，继续向周围扩展
            to_check_ids.assign(tmp.begin(), tmp.end());
        }
        cluster_ids.push_back(cluster); // 将当前簇的所有行号加入到cluster_ids中
    }

    // search sub-cluster
    std::vector<std::vector<size_t>> new_cluster_ids;
    for (auto &cluster : cluster_ids)
    {
        size_t cluster_size = cluster.size();
        if (cluster_size <= 2)
        {
            new_cluster_ids.push_back(cluster);
            continue;
        }

        std::sort(cluster.begin(), cluster.end(), [&length](size_t i1, size_t i2)
                  { return length(i1) > length(i2); });
        std::unordered_map<size_t, size_t> line_location;
        for (size_t i = 0; i < cluster_size; i++)
        {
            line_location[cluster[i]] = i;
        }

        std::vector<bool> clustered(cluster_size, false);
        for (size_t j = 0; j < cluster_size; j++)
        {
            if (clustered[j])
                continue;

            size_t line_idx = cluster[j];
            std::vector<size_t> sub_cluster;
            sub_cluster.push_back(line_idx);
            std::vector<size_t> line_neighbors = neighbors[line_idx];
            for (size_t k : line_neighbors)
            {
                clustered[line_location[k]] = true;
                sub_cluster.push_back(k);
            }
            new_cluster_ids.push_back(sub_cluster);
        }
    }
    // merge clusters
    dst_lines.clear();
    dst_lines.reserve(new_cluster_ids.size());
    std::vector<Eigen::Vector4f> source_lines_eigen;
    source_lines_eigen.reserve(source_lines.size());
    for (const auto& cvElem : source_lines)
    {
    Eigen::Vector4f tem;
    for(size_t s_d = 0; s_d < 4; s_d++)
    {
        tem(s_d) = cvElem(s_d);
    }
   source_lines_eigen.push_back(tem);
    }
    for (auto &cluster : new_cluster_ids)
    {
        size_t idx0 = cluster[0];
        // Eigen::Vector4f source_idx0 = Eigen::Map<Eigen::Vector4f, Eigen::Unaligned>(source_lines[idx0].val);
        Eigen::Vector4f new_line = source_lines_eigen[idx0];
        // Eigen::Vector4f source_line = Eigen::Map<Eigen::Vector4f>(source_lines[0].val, source_lines.size());
         std::vector<Eigen::Vector4f> old_line;
        //  old_line.push_back(source_line);
        for (size_t i = 0; i < cluster.size(); i++)
        {
            new_line = MergeTwoLines(new_line, source_lines_eigen[cluster[i]]);
        }
        old_line.push_back(new_line);
        std::vector<cv::Vec4f> cv_newline;

        for(const auto& eigenVec: old_line)
        {
            dst_lines.emplace_back(eigenVec[0], eigenVec[1], eigenVec[2], eigenVec[3]);
        }
        // dst_lines.push_back(cv_newline);
    }
}

Eigen::Vector4f MergeTwoLines(const Eigen::Vector4f &line1, const Eigen::Vector4f &line2)
{
    double xg = 0.0, yg = 0.0;
    double delta1x = 0.0, delta1y = 0.0, delta2x = 0.0, delta2y = 0.0;
    float ax = 0, bx = 0, cx = 0, dx = 0;
    float ay = 0, by = 0, cy = 0, dy = 0;
    double li = 0.0, lj = 0.0;
    double thi = 0.0, thj = 0.0, thr = 0.0;
    double axg = 0.0, bxg = 0.0, cxg = 0.0, dxg = 0.0, delta1xg = 0.0, delta2xg = 0.0;

    ax = line1(0);
    ay = line1(1);
    bx = line1(2);
    by = line1(3);

    cx = line2(0);
    cy = line2(1);
    dx = line2(2);
    dy = line2(3);

    float dlix = (bx - ax);
    float dliy = (by - ay);
    float dljx = (dx - cx);
    float dljy = (dy - cy);
    // 线段长度
    li = sqrt((double)(dlix * dlix) + (double)(dliy * dliy));
    lj = sqrt((double)(dljx * dljx) + (double)(dljy * dljy));
    // 运用比重选取端点
    xg = (li * (double)(ax + bx) + lj * (double)(cx + dx)) / (double)(2.0 * (li + lj));
    yg = (li * (double)(ay + by) + lj * (double)(cy + dy)) / (double)(2.0 * (li + lj));
    // 角度
    if (dlix == 0.0f)
        thi = CV_PI / 2.0;
    else
        thi = atan(dliy / dlix);

    if (dljx == 0.0f)
        thj = CV_PI / 2.0;
    else
        thj = atan(dljy / dljx);
    // 选取合并的角度
    if (fabs(thi - thj) <= CV_PI / 2.0)
    {
        thr = (li * thi + lj * thj) / (li + lj);
    }
    else
    {
        double tmp = thj - CV_PI * (thj / fabs(thj));
        thr = li * thi + lj * tmp;
        thr /= (li + lj);
    }

    axg = ((double)ay - yg) * sin(thr) + ((double)ax - xg) * cos(thr);
    bxg = ((double)by - yg) * sin(thr) + ((double)bx - xg) * cos(thr);
    cxg = ((double)cy - yg) * sin(thr) + ((double)cx - xg) * cos(thr);
    dxg = ((double)dy - yg) * sin(thr) + ((double)dx - xg) * cos(thr);

    delta1xg = std::min(axg, std::min(bxg, std::min(cxg, dxg)));
    delta2xg = std::max(axg, std::max(bxg, std::max(cxg, dxg)));

    delta1x = delta1xg * std::cos(thr) + xg;
    delta1y = delta1xg * std::sin(thr) + yg;
    delta2x = delta2xg * std::cos(thr) + xg;
    delta2y = delta2xg * std::sin(thr) + yg;

    Eigen::Vector4f new_line;
    new_line << (float)delta1x, (float)delta1y, (float)delta2x, (float)delta2y;
    return new_line;
}



void FilterShortLines(std::vector<Eigen::Vector4f>& lines, float length_thr){
  Eigen::Array4Xf line_array = Eigen::Map<Eigen::Array4Xf, Eigen::Unaligned>(lines[0].data(), 4, lines.size());
  Eigen::ArrayXf length_square = (line_array.row(2) - line_array.row(0)).square() + (line_array.row(3) - line_array.row(1)).square();
  float thr_square = length_thr * length_thr;

  size_t long_line_num = 0;
  for(size_t i = 0; i < lines.size(); i++){
    if(length_square(i) > thr_square){
      lines[long_line_num] = lines[i];
      long_line_num++;
    }
  }
  lines.resize(long_line_num);
}



std::vector<Eigen::Vector4d> convertVec4fToVec4d(const std::vector<Eigen::Vector4f>& vec_f) 
{
std::vector<Eigen::Vector4d> vec_d;
vec_d.reserve(vec_f.size());
for (const auto& v : vec_f) {
vec_d.emplace_back(v.cast<double>());
}
return vec_d;
}

std::vector<cv::Vec4f> convertEigenToCV(std::vector<Eigen::Vector4f>& eigenVec) 
{
    std::vector<cv::Vec4f> cvVec;
    for (const auto& vec : eigenVec) {
    cvVec.emplace_back(vec(0), vec(1), vec(2), vec(3));
    }
    return cvVec;
}

std::vector<Eigen::Vector4f> cvVecToEigenVec(const std::vector<cv::Vec<float, 4>>& cvVec) {
std::vector<Eigen::Vector4f> eigenVec;
eigenVec.reserve(cvVec.size()); // 预分配空间

for (const auto& cvElem : cvVec) {
Eigen::Vector4f eigenElem(cvElem[0], cvElem[1], cvElem[2], cvElem[3]);
eigenVec.push_back(eigenElem);
}

return eigenVec;
}

std::vector<Eigen::Vector4f> convertVec4dToVec4f(const std::vector<Eigen::Vector4d>& vec_d) 
{
std::vector<Eigen::Vector4f> vec_f;
vec_f.reserve(vec_d.size());
for (const auto& v : vec_d) {
vec_f.emplace_back(v.cast<float>());
}
return vec_f;
}


std::vector<cv::Vec4f> convertKeyLinesToVec4f(const std::vector<cv::line_descriptor::KeyLine>& keylines)
{
std::vector<cv::Vec4f> lines_vec4f;
lines_vec4f.reserve(keylines.size());

for (const auto& keyline : keylines)
{
cv::Vec4f line_vec4f(keyline.startPointX, keyline.startPointY, keyline.endPointX, keyline.endPointY);
lines_vec4f.push_back(line_vec4f);
}

return lines_vec4f;
}

std::vector<cv::line_descriptor::KeyLine> convertVec4fToKeyLine(const std::vector<cv::Vec4f>& lines , cv::Mat img)
{

        std::vector<cv::line_descriptor::KeyLine> fld_lines;

            for( int i = 0; i < lines.size(); i++ )
            {
                cv::line_descriptor::KeyLine kl;
                double octaveScale = 1.f;
                int    octaveIdx   = 0;

                kl.startPointX     = lines[i][0] * octaveScale;
                kl.startPointY     = lines[i][1] * octaveScale;
                kl.endPointX       = lines[i][2] * octaveScale;
                kl.endPointY       = lines[i][3] * octaveScale;

                kl.sPointInOctaveX = lines[i][0];
                kl.sPointInOctaveY = lines[i][1];
                kl.ePointInOctaveX = lines[i][2];
                kl.ePointInOctaveY = lines[i][3];

                kl.lineLength = (float) sqrt( pow( lines[i][0] - lines[i][2], 2 ) + pow( lines[i][1] - lines[i][3], 2 ) );

                kl.angle    = atan2( ( kl.endPointY - kl.startPointY ), ( kl.endPointX - kl.startPointX ) );
                kl.class_id = i;
                kl.octave   = octaveIdx;
                kl.size     = ( kl.endPointX - kl.startPointX ) * ( kl.endPointY - kl.startPointY );
                kl.pt       = cv::Point2f( ( kl.endPointX + kl.startPointX ) / 2, ( kl.endPointY + kl.startPointY ) / 2 );

                kl.response = kl.lineLength / std::max( img.cols, img.rows );
                cv::LineIterator li( img, cv::Point2f( lines[i][0], lines[i][1] ), cv::Point2f( lines[i][2], lines[i][3] ) );
                kl.numOfPixels = li.count;

                fld_lines.push_back( kl );
            }
        return fld_lines;
}

void optimizeAndMergeLines_lsd(std::vector<cv::line_descriptor::KeyLine>& lines, const cv::Mat& img) {
    std::vector<cv::Vec4f> temp_lines = convertKeyLinesToVec4f(lines);
    // lines.clear();

    std::vector<cv::Vec4f> tmp_lines_c, dst_lines_c;
    std::vector<Eigen::Vector4d> tmp_lines_e_d, dst_lines_e_d;
    std::vector<Eigen::Vector4f> tmp_lines_e_f, dst_lines_e_f;
    //合并长线并过滤
    MergeLines(temp_lines, tmp_lines_c, 0.05, 5, 15);  //5
    tmp_lines_e_f = cvVecToEigenVec(tmp_lines_c);
    // tmp_lines_e_d = convertVec4fToVec4d(tmp_lines_e_f);
    FilterShortLines(tmp_lines_e_f, 30);
    // tmp_lines_e_f = convertVec4dToVec4f(tmp_lines_e_d);
    tmp_lines_c = convertEigenToCV(tmp_lines_e_f);

    MergeLines(tmp_lines_c, dst_lines_c, 0.03, 3, 30);  //3
    dst_lines_e_f = cvVecToEigenVec(dst_lines_c);
    // dst_lines_e_d = convertVec4fToVec4d(dst_lines_e_f);
    FilterShortLines(dst_lines_e_f, 50);
    // dst_lines_e_f = convertVec4dToVec4f(dst_lines_e_f);
    std::vector<cv::Vec4f> dst_lines = convertEigenToCV(dst_lines_e_f);
    lines = convertVec4fToKeyLine(dst_lines,img);
    // std::cout<< "合并长线段" << std::endl;
    //过滤
    // tmp_lines_e_f = cvVecToEigenVec(temp_lines);
    // tmp_lines_e_d = convertVec4fToVec4d(tmp_lines_e_f);
    // FilterShortLines(tmp_lines_e_d, 20);
    // tmp_lines_e_f = convertVec4dToVec4f(tmp_lines_e_d);
    // tmp_lines_c = convertEigenToCV(tmp_lines_e_f);

    // dst_lines_e_f = cvVecToEigenVec(dst_lines_c);
    // dst_lines_e_d = convertVec4fToVec4d(dst_lines_e_f);
    // FilterShortLines(dst_lines_e_d, 50);
    // dst_lines_e_f = convertVec4dToVec4f(dst_lines_e_d);
    // std::vector<cv::Vec4f> dst_lines = convertEigenToCV(dst_lines_e_f);
    // lines = convertVec4fToKeyLine(dst_lines, img);
}

void optimizeAndMergeLines_fld(std::vector<cv::Vec4f>& fld_lines) {
    std::vector<cv::Vec4f> tmp_lines_c, dst_lines_c;
    std::vector<Eigen::Vector4d> tmp_lines_e_d, dst_lines_e_d;
    std::vector<Eigen::Vector4f> tmp_lines_e_f, dst_lines_e_f;

    //合并长线并过滤
    MergeLines(fld_lines, tmp_lines_c, 0.05, 5, 15);
    fld_lines.clear();
    tmp_lines_e_f = cvVecToEigenVec(tmp_lines_c);
    // tmp_lines_e_d = convertVec4fToVec4d(tmp_lines_e_f);
    FilterShortLines(tmp_lines_e_f, 30);
    tmp_lines_e_f = convertVec4dToVec4f(tmp_lines_e_d);
    tmp_lines_c = convertEigenToCV(tmp_lines_e_f);

    MergeLines(tmp_lines_c, dst_lines_c, 0.03, 3, 50);
    dst_lines_e_f = cvVecToEigenVec(dst_lines_c);
    // dst_lines_e_d = convertVec4fToVec4d(dst_lines_e_f);
    FilterShortLines(dst_lines_e_f, 40);
    dst_lines_e_f = convertVec4dToVec4f(dst_lines_e_d);
    fld_lines = convertEigenToCV(dst_lines_e_f);

    //过滤线段
    // fld_lines.clear();
    // tmp_lines_e_f = cvVecToEigenVec(fld_lines);
    // tmp_lines_e_d = convertVec4fToVec4d(tmp_lines_e_f);
    // FilterShortLines(tmp_lines_e_d, 20);
    // tmp_lines_e_f = convertVec4dToVec4f(tmp_lines_e_d);
    // tmp_lines_c = convertEigenToCV(tmp_lines_e_f);

    // dst_lines_e_f = cvVecToEigenVec(tmp_lines_c);
    // dst_lines_e_d = convertVec4fToVec4d(dst_lines_e_f);
    // FilterShortLines(dst_lines_e_d, 50);
    // dst_lines_e_f = convertVec4dToVec4f(dst_lines_e_d);
    // fld_lines = convertEigenToCV(dst_lines_e_f);
}