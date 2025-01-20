#include "mapping/ImageToCost.hpp"

int main() {
    cv::Mat colorImage = cv::imread("/home/slsecret/PreferentialTerrainNavigation/src/mapping/data/bev_image.png");
    if (colorImage.empty()) {
        std::cerr << "Failed to load image!" << std::endl;
        return -1;
    }
    cv::imshow("Color Image", colorImage);
    ImageToCost converter;
    cv::Mat grayImage = converter.convert(colorImage);

    cv::imshow("Converted Grayscale Cost Image", grayImage);
    cv::waitKey(0);

    cv::Mat colorImage2 = converter.convertToColor(grayImage);
    cv::imshow("Converted Color Image", colorImage2);
    cv::waitKey(0);

    return 0;
}
