// RGB -> YUV420 conversion
// Found here: http://code.opencv.org/attachments/1116/rgb_to_yuv420.cpp

#ifndef RGB_TO_YUV420_H
#define RGB_TO_YUV420_H

void RGB_to_YUV420(const unsigned char * rgb, unsigned char * yuv420, int width, int height);

#endif
