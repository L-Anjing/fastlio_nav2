#ifndef FAST_LIO_RELOC_H
#define FAST_LIO_RELOC_H

// Minimal relocalization input retained for a future localization backend.
struct RelocState
{
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
    double qx_ = 0.0;
    double qy_ = 0.0;
    double qz_ = 0.0;
    double qw_ = 1.0;
    double timestamp_ = 0.0;

    RelocState() = default;
    RelocState(double x, double y, double z,
               double qx, double qy, double qz, double qw, double timestamp)
        : x_(x), y_(y), z_(z), qx_(qx), qy_(qy), qz_(qz), qw_(qw),
          timestamp_(timestamp)
    {
    }
};

#endif
