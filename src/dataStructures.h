#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

template<typename T> 
class CircBuf {
public:
  CircBuf(int sizeIn) { buf_size = sizeIn; };
  int size() const { return count; }
  bool empty() const { return count == 0; }
  bool full() const { return count == buf_size; }
  void remove() {
      if ( !empty() )
        data.erase(data.begin());
        count--;
  }
  void add(const T& inp) {
      if ( full() )
        remove();
      data.push_back(inp);
      count++;
  }
  typename std::vector<T>::iterator getItem(int n) {
    n = std::min({n, buf_size});
    return (data.end() - n);
  }

private:
  int buf_size = 0;
  int count = 0;
  std::vector<T> data;
};

#endif /* dataStructures_h */
