#include <pcl/point_types.h>
#include <pcl/visualization/MyCloudLUT.h>

/// Lookup table
static const unsigned char CloudLUT[] =
{
  255 , 255, 255 , // default
  255,  0,   0   , // highted
  0  ,  0,   255   // ground

};

/// Number of colors in Glasbey lookup table
static const unsigned int CloudLUT_SIZE = sizeof (CloudLUT) / (sizeof (CloudLUT[0]) * 3);

pcl::RGB
MyCloudLUT::at (unsigned int color_id)
{
  assert (color_id < CloudLUT_SIZE);
  pcl::RGB color;
  color.r = CloudLUT[color_id * 3 + 0];
  color.g = CloudLUT[color_id * 3 + 1];
  color.b = CloudLUT[color_id * 3 + 2];
  return (color);
}

size_t
MyCloudLUT::size ()
{
  return CloudLUT_SIZE;
}

const unsigned char*
MyCloudLUT::data ()
{
  return CloudLUT;
}
