#include "libpngFileReader.h"

#include <string>
#include <cstdio>
#include <cstdlib>
#include <png.h>


#include "utils.h"

using namespace corecvs;

    RGB24Buffer *PNGLoader::loadRGB(string name)
    {
        RGB24Buffer *toReturn;
        png_byte ** row_pointers;
        FILE *fp = fopen(name.c_str(), "rb");
        const int number = 4;
        uint8_t header[number];

        if (!fp)
            return NULL;

        if (fread(header, 1, number, fp) != number)
            return NULL;

        bool is_png = !png_sig_cmp(header, 0, number);
        if (!is_png)
            return NULL;

        png_structp png_ptr = png_create_read_struct
               (PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);

        if (!png_ptr)
            return NULL;

        png_infop info_ptr = png_create_info_struct(png_ptr);

        if (!info_ptr) {
            png_destroy_read_struct(&png_ptr,
                nullptr, nullptr);
            return NULL;
        }

        if (setjmp(png_jmpbuf(png_ptr))) {
            png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
            fclose(fp);
            return NULL;
        }

        //png_info_init(info_ptr);
        //png_read_init(png_ptr);

        png_init_io(png_ptr, fp);
        png_set_sig_bytes(png_ptr, number);
        png_read_info(png_ptr, info_ptr);

        png_uint_32 width, height;
        int color_type, bit_depth;

        png_get_IHDR(png_ptr, info_ptr, &width, &height,
              &bit_depth, &color_type, nullptr,
              nullptr, nullptr);

        if (color_type & PNG_COLOR_MASK_ALPHA)
              png_set_strip_alpha(png_ptr);

        png_read_update_info(png_ptr, info_ptr);


        row_pointers = (png_byte **) png_malloc(png_ptr, height * sizeof (png_byte *));
        for (int y = 0; y < height; y++)
            row_pointers[y] = (png_byte *) png_malloc (png_ptr, sizeof (uint8_t) * width * 3);

        png_read_image(png_ptr, row_pointers);
        png_read_end(png_ptr, nullptr);
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

        toReturn = new RGB24Buffer(height, width);

        for (uint32_t y = 0; y < height; y++) {
            png_byte *row = row_pointers[y];
            for (uint32_t x = 0; x < width; x++) {
                RGBColor *dest = &toReturn->element(y,x);
                uint32_t r = *row++, g = *row++, b = *row++;
                *dest = RGBColor(r,g,b);
            }
            png_free (png_ptr, row_pointers[y]);
        }
        png_free (png_ptr, row_pointers);

        return toReturn;
    }

    bool PNGLoader::save(string name, RGB24Buffer *buffer)
    {
        FILE * fp;
        png_byte ** row_pointers;
        png_structp png_ptr = nullptr;
        png_infop info_ptr = nullptr;
        size_t x, y;
        int status = -1;
        int pixel_size = 3;
        int depth = 8;
        png_uint_32 width = buffer->w, height = buffer->h;

        fp = fopen (name.c_str(), "wb");
        if (! fp) {
            goto fopen_failed;
        }

        png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
        if (png_ptr == nullptr) {
            goto png_create_write_struct_failed;
        }

        info_ptr = png_create_info_struct (png_ptr);
        if (info_ptr == nullptr) {
            goto png_create_info_struct_failed;
        }

        /* Set up error handling. */

        if (setjmp (png_jmpbuf (png_ptr))) {
            goto png_failure;
        }

        /* Set image attributes. */


        png_set_IHDR (png_ptr,
                      info_ptr,
                      width,
                      height,
                      depth,
                      PNG_COLOR_TYPE_RGB,
                      PNG_INTERLACE_NONE,
                      PNG_COMPRESSION_TYPE_DEFAULT,
                      PNG_FILTER_TYPE_DEFAULT);

        /* Initialize rows of PNG. */

        row_pointers = (png_byte **) png_malloc (png_ptr, height * sizeof (png_byte *));
        for (y = 0; y < height; ++y) {
            png_byte *row =
                (png_byte *) png_malloc (png_ptr, sizeof (uint8_t) * width * pixel_size);
            row_pointers[y] = row;
            for (x = 0; x < width; ++x) {
                RGBColor pixel = buffer->element(y, x);
                *row++ = pixel.r();
                *row++ = pixel.g();
                *row++ = pixel.b();
            }
        }

        /* Write the image data to "fp". */

        png_init_io (png_ptr, fp);
        png_set_rows (png_ptr, info_ptr, row_pointers);
        png_write_png (png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

        status = 0;

        for (y = 0; y < height; y++) {
            png_free (png_ptr, row_pointers[y]);
        }
        png_free (png_ptr, row_pointers);

        png_failure:
            png_create_info_struct_failed:
            png_destroy_write_struct (&png_ptr, &info_ptr);
        png_create_write_struct_failed:
            fclose (fp);
        fopen_failed:
            return status;
    }
