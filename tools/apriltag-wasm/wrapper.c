// Thin WebAssembly interface to the AprilRobotics apriltag detector (tag36h11), the library under the FTC SDK AprilTagProcessor.
#include <stdlib.h>
#include <string.h>
#include <emscripten/emscripten.h>
#include "apriltag.h"
#include "tag36h11.h"

static apriltag_detector_t *td = NULL;
static apriltag_family_t *tf = NULL;
static uint8_t *pixels = NULL;
static int pixel_capacity = 0;
// Per detection: id, hamming, decision_margin, centre x, centre y, then corners p0..p3 (x, y).
#define RESULT_STRIDE 13
static double *results = NULL;
static int result_capacity = 0;

EMSCRIPTEN_KEEPALIVE int at_init(void) {
    if (td) return 0;
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, 2);
    td->nthreads = 1;
    return 0;
}

EMSCRIPTEN_KEEPALIVE void at_config(float decimate, float sigma, int refine_edges, double sharpening) {
    td->quad_decimate = decimate;
    td->quad_sigma = sigma;
    td->refine_edges = refine_edges != 0;
    td->decode_sharpening = sharpening;
}

EMSCRIPTEN_KEEPALIVE uint8_t *at_buffer(int width, int height) {
    int need = width * height;
    if (need > pixel_capacity) {
        free(pixels);
        pixels = malloc(need);
        pixel_capacity = need;
    }
    return pixels;
}

EMSCRIPTEN_KEEPALIVE int at_detect(int width, int height) {
    image_u8_t im = { .width = width, .height = height, .stride = width, .buf = pixels };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    int n = zarray_size(detections);
    if (n * RESULT_STRIDE > result_capacity) {
        free(results);
        result_capacity = n * RESULT_STRIDE;
        results = malloc(sizeof(double) * result_capacity);
    }
    for (int i = 0; i < n; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        double *r = results + i * RESULT_STRIDE;
        r[0] = det->id;
        r[1] = det->hamming;
        r[2] = det->decision_margin;
        r[3] = det->c[0];
        r[4] = det->c[1];
        for (int k = 0; k < 4; k++) {
            r[5 + 2 * k] = det->p[k][0];
            r[6 + 2 * k] = det->p[k][1];
        }
    }
    apriltag_detections_destroy(detections);
    return n;
}

EMSCRIPTEN_KEEPALIVE double *at_results(void) { return results; }

// Writes the 10×10 tag image (white ring, black border, data bits) for one ID; returns its width.
EMSCRIPTEN_KEEPALIVE int at_tag_image(int id, uint8_t *out) {
    if (id < 0 || (uint32_t)id >= tf->ncodes) return 0;
    image_u8_t *im = apriltag_to_image(tf, id);
    for (int y = 0; y < im->height; y++) memcpy(out + y * im->width, im->buf + y * im->stride, im->width);
    int w = im->width;
    image_u8_destroy(im);
    return w;
}

EMSCRIPTEN_KEEPALIVE int at_tag_count(void) { return tf->ncodes; }
