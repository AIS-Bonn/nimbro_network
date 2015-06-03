inline void rgb_to_yuv(unsigned char   b, unsigned char   g, unsigned char   r,
                       unsigned char & y, unsigned char & u, unsigned char & v)
{
    float yf, uf, vf;

    //Y = R * 0.299 + G * 0.587 + B * 0.114;
    //U = R * -0.169 + G * -0.332 + B * 0.500 + 128.0;
    //V = R * 0.500 + G * -0.419 + B * -0.0813 + 128.0;

    yf =    0.299f * static_cast<float>(r) +
            0.587f * static_cast<float>(g) +
            0.114f * static_cast<float>(b);
    yf = (yf > 255.0f) ? 255.0f: yf;
    yf = (yf < 0.0f) ? 0.0f: yf;
    y = static_cast<unsigned char>(yf);


    uf =   -0.169f * static_cast<float>(r) -
            0.332f * static_cast<float>(g) +
            0.500f * static_cast<float>(b) + 128.0;
    uf = (uf > 255.0f) ? 255.0f: uf;
    uf = (uf < 0.0f) ? 0.0f: uf;
    u = static_cast<unsigned char>(uf);


    vf =    0.500f * static_cast<float>(r) -
            0.419f * static_cast<float>(g) -
            0.081f * static_cast<float>(b) + 128.0;
    vf = (vf > 255.0f) ? 255.0f: vf;
    vf = (vf < 0.0f) ? 0.0f: vf;
    v = static_cast<unsigned char>(vf);

}

void RGB_to_YUV420(const unsigned char * rgb, unsigned char * yuv420, int width, int height)
{
    unsigned char * y_pixel = yuv420;
    unsigned char * u_pixel = yuv420 + width * height;
    unsigned char * v_pixel = yuv420 + width * height + (width * height / 4);

    unsigned char * U_tmp = new unsigned char [width * height];
    unsigned char * V_tmp = new unsigned char [width * height];

    int index = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            rgb_to_yuv(rgb[3 * (y * width + x) + 0], rgb[3 * (y * width + x) + 1], rgb[3 * (y * width + x) + 2], y_pixel[index], U_tmp[index], V_tmp[index]);
            index++;
        }
    }

    index = 0;
    for (int y = 0; y < height; y+=2)
    {
        for (int x = 0; x < width; x+=2)
        {
            u_pixel[index] = U_tmp[y * width + x];
            v_pixel[index] = V_tmp[y * width + x];
            index++;
        }
    }

    delete [] U_tmp;
    delete [] V_tmp;
}
