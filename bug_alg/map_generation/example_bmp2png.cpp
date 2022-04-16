#include "lodepng.h"
#include "lodepng.cpp"
#include "bitmap_image.hpp"

#include <iostream>
#include <cstdlib>
#include <ctime>
//returns 0 if all went ok, non-0 if error
//output image is always given in RGBA (with alpha channel), even if it's a BMP without alpha channel
unsigned decodeBMP(std::vector<unsigned char>& image, unsigned& w, unsigned& h, const std::vector<unsigned char>& bmp) {
  static const unsigned MINHEADER = 54; //minimum BMP header size

  if(bmp.size() < MINHEADER) return -1;
  if(bmp[0] != 'B' || bmp[1] != 'M') return 1; //It's not a BMP file if it doesn't start with marker 'BM'
  unsigned pixeloffset = bmp[10] + 256 * bmp[11]; //where the pixel data starts
  //read width and height from BMP header
  w = bmp[18] + bmp[19] * 256;
  h = bmp[22] + bmp[23] * 256;
  //read number of channels from BMP header
  if(bmp[28] != 24 && bmp[28] != 32) return 2; //only 24-bit and 32-bit BMPs are supported.
  unsigned numChannels = bmp[28] / 8;

  //The amount of scanline bytes is width of image times channels, with extra bytes added if needed
  //to make it a multiple of 4 bytes.
  unsigned scanlineBytes = w * numChannels;
  if(scanlineBytes % 4 != 0) scanlineBytes = (scanlineBytes / 4) * 4 + 4;

  unsigned dataSize = scanlineBytes * h;
  if(bmp.size() < dataSize + pixeloffset) return 3; //BMP file too small to contain all pixels

  image.resize(w * h * 4);

  /*
  There are 3 differences between BMP and the raw image buffer for LodePNG:
  -it's upside down
  -it's in BGR instead of RGB format (or BRGA instead of RGBA)
  -each scanline has padding bytes to make it a multiple of 4 if needed
  The 2D for loop below does all these 3 conversions at once.
  */
  for(unsigned y = 0; y < h; y++)
  for(unsigned x = 0; x < w; x++) {
    //pixel start byte position in the BMP
    unsigned bmpos = pixeloffset + (h - y - 1) * scanlineBytes + numChannels * x;
    //pixel start byte position in the new raw image
    unsigned newpos = 4 * y * w + 4 * x;
    if(numChannels == 3) {
      image[newpos + 0] = bmp[bmpos + 2]; //R
      image[newpos + 1] = bmp[bmpos + 1]; //G
      image[newpos + 2] = bmp[bmpos + 0]; //B
      image[newpos + 3] = 255;            //A
    } else {
      image[newpos + 0] = bmp[bmpos + 2]; //R
      image[newpos + 1] = bmp[bmpos + 1]; //G
      image[newpos + 2] = bmp[bmpos + 0]; //B
      image[newpos + 3] = bmp[bmpos + 3]; //A
    }
  }
  return 0;
}

int f (int x1, int y1, int x2, int y2, int x, int y) {
	int t = ((y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - y1 * x2));
	return t;
}

int main() {
	srand(static_cast<unsigned int>(time(0)));
	bitmap_image image1(800,800);

   // set background to orange
   image1.set_all_channels(255, 255, 150);

   image_drawer draw(image1);

   /*draw.pen_width(3);
   draw.pen_color(255, 0, 0);
   draw.circle(image1.width() / 2, image1.height() / 2, 50);

   draw.pen_width(1);
   draw.pen_color(0, 0, 255);
   draw.rectangle(50, 50, 150, 150);

   image1.save_image("output.bmp");*/
   /*for (int i = 390; i < 410; ++i) {
   	for (int j = 390; j < 410; ++j) {
   		image1.set_pixel(i, j, 0, 0, 0);
	}
   }*/
	for (int i = 0; i < 800; i += 100) {
		for (int j = 0; j < 800; j += 100) {
			int cmp = rand() % 8;
			if (cmp == 1) {
				int r = rand() % 30 + 10;
				for (int ii = i + 50 - r; ii < i + 50 + r; ++ii) {
					for (int jj = j + 50 - r; jj < j + 50 + r; ++jj) {
						if ((ii - i - 50) * (ii - i - 50) + (jj - j - 50) * (jj - j - 50) < r * r) {
							image1.set_pixel(ii, jj, 0, 0, 0);
						}
					}
				}
			} else {
				if (cmp == 2) {
					int x = rand() % 30 + 10 + i;
					int y = rand() % 30 + 10 + j;
					int ox = i + 50;
					int oy = j + 50;
					int x2 = ox + (ox - x);
					int y2 = oy + (oy - y);
					int x1 = ox - (oy - y);
					int y1 = oy + (ox - x);
					int x3 = ox + (oy - y);
					int y3 = oy - (ox - x);
					image1.set_pixel(x, y, 0, 0, 0);
					image1.set_pixel(x1, y1, 0, 0, 0);
					image1.set_pixel(x2, y2, 0, 0, 0);
					image1.set_pixel(x3, y3, 0, 0, 0);
					for (int ii = i; ii < i + 100; ++ii) {
						for (int jj = j; jj < j + 100; ++jj) {
							if (f(x1, y1, x2, y2, ii, jj) <= 0 && f(x2, y2, x3, y3, ii, jj) <= 0 && f(x, y, x3, y3, ii, jj) >= 0 && f(x1, y1, x, y, ii, jj) >= 0) {
								image1.set_pixel(ii, jj, 0, 0, 0);
							}
						}
					}
				} else {
					if (cmp == 3) {
						int x1 = i + 10 + rand() % 30;
						int y1 = j + 10 + rand() % 30;
						int x2 = i + 90 - rand() % 30;
						int y2 = j + 10 + rand() % 30;
						int x3 = i + 10 + rand() % 80;
						int y3 = j + 90 - rand() % 30;
						for (int ii = i; ii < i + 100; ++ii) {
							for (int jj = j; jj < j + 100; ++jj) {
								if (f(x1, y1, x2, y2, ii, jj) >= 0 && f(x3, y3, x2, y2, ii, jj) <= 0 && f(x1, y1, x3, y3, ii, jj) <= 0) {
									image1.set_pixel(ii, jj, 0, 0, 0);
								}
							}
						}	
					}
				}
			}
		}
	}
   image1.save_image("output.bmp");
  const char *argv1 = "output.bmp";
  const char* argv2 = "map.png";
  int argc = 3;

  if(argc < 3) {
    std::cout << "Please provide input BMP and output PNG file names" << std::endl;
    return 0;
  }

  std::vector<unsigned char> bmp;
  lodepng::load_file(bmp, argv1);
  std::vector<unsigned char> image;
  unsigned w, h;
  unsigned error = decodeBMP(image, w, h, bmp);

  if(error) {
    std::cout << "BMP decoding error " << error << std::endl;
    return 0;
  }

  std::vector<unsigned char> png;
  error = lodepng::encode(png, image, w, h);

  if(error) {
    std::cout << "PNG encoding error " << error << ": " << lodepng_error_text(error) << std::endl;
    return 0;
  }

  lodepng::save_file(png, argv2);
}
