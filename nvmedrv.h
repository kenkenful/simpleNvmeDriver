
#ifndef _NVME_H
#define _NVME_H

// ioctls
#define NVME 'N'
#define IOCTL_ADMIN_CMD _IOW(NVME, 1, struct nvme_command*)
#define IOCTL_IO_CMD _IOW(NVME, 2, struct nvme_command*)

#endif /* _NVME_H */