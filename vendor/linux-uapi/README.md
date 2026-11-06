# Linux V4L2 UAPI headers

`linux/videodev2.h`, `linux/v4l2-controls.h` and `linux/v4l2-common.h`, unmodified, from Linux 7.0's
sanitized UAPI headers (Ubuntu's `linux-libc-dev` 7.0.0-42.42). Each is licensed
`(GPL-2.0+ WITH Linux-syscall-note) OR BSD-3-Clause`, as its SPDX line says.

v4l2r generates its V4L2 bindings from `<linux/videodev2.h>` at build time, and refers to structures
that older kernels' headers lack (AV1 and stateless HEVC controls, for example; Ubuntu 22.04's are
Linux 5.15's). `.cargo/config.toml` points v4l2r here (`V4L2R_VIDEODEV2_H_PATH`), so every build,
native or cross, reads these and not the build machine's. The V4L2 ABI is stable, so newer headers
are fine with older kernels. The rest of `<linux/...>` still comes from the system or sysroot.

To update, copy the same three files from a newer kernel's `make headers_install` (or a distribution's
installed headers) and update the version above.
