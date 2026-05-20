#ifndef LOCALIZATION_RTK_COMPAT_H
#define LOCALIZATION_RTK_COMPAT_H

typedef struct _rtk_fig_t rtk_fig_t;
typedef struct _rtk_canvas_t rtk_canvas_t;

static inline void rtk_fig_rectangle(rtk_fig_t *, double, double, double,
                                     double, double, int) {}
static inline void rtk_fig_text(rtk_fig_t *, double, double, double,
                                const char *) {}

#endif
