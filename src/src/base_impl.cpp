#include "internal/base_impl.hpp"
#include <pcl/filters/crop_box.h> // 引入 PCL 官方的 CropBox 濾波器

namespace pc_utils {
namespace internal {

void executeCrop(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                 const Eigen::Vector3f &minBound,
                 const Eigen::Vector3f &maxBound) {
  // 1. 防呆：如果傳進來的點雲指標是空的，或者點雲裡面沒有點，就直接返回
  if (!cloud || cloud->empty())
    return;

  // 2. 實例化 PCL 的 CropBox 濾波器
  pcl::CropBox<pcl::PointXYZI> crop_box;

  // 3. 設定裁剪邊界 (CropBox 的 Min/Max 需要 Vector4f，我們手動補上 1.0f
  // 作為固定參數)
  Eigen::Vector4f min_pt(minBound.x(), minBound.y(), minBound.z(), 1.0f);
  Eigen::Vector4f max_pt(maxBound.x(), maxBound.y(), maxBound.z(), 1.0f);
  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);

  // 4. 設定輸入點雲
  crop_box.setInputCloud(cloud);

  // 5. 執行過濾 (關鍵優化：將輸出物件設為同一個指標，PCL
  // 內部會自動進行極速的原地記憶體覆蓋)
  crop_box.filter(*cloud);

  // 6. 修正 PCL 裁剪後的點雲屬性 (因為原本是 1024*128 的 Organized
  // 結構，剪完後形狀變了)
  cloud->width = cloud->points.size();
  cloud->height = 1; // 降維成一維散亂點雲
  cloud->is_dense =
      true; // 裁剪掉無效點後，標記為緊湊點雲，加速後續降採樣與去噪
}

void executeDenoise(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                    float denoiseThres) {}

void executeDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       float voxelSize) {}

void mergeCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &base_cloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &other_cloud,
                float voxelSize) {}

} // namespace internal
} // namespace pc_utils