#include <iostream>
#include "ReadRGBD.h"
#include "Features.h"

using namespace std;

int main()
{
    Features feature;
    //feature.FeaturesTesting();
    feature.descriptorMatchingTesting();
    //feature.find_Homography();
    //feature.project_points();
   // feature.rgbdmap();
    return 0;
}
