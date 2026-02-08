#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <linux/types.h>

#include "nvmedrv.h"

#define IDENTIFY_DATA_SIZE 4096


struct nvme_identify {
	__u8	opcode;        /* 0x06 */
	__u8	flags;
	__u16	command_id;

	__le32	nsid;          /* Namespace ID (Controller のときは 0) */
	__le64	rsvd2[2];

	__le64	prp1;          /* データ転送先（DMA） */
	__le64	prp2;

	__le32	cns;           /* Controller or Namespace Structure */
	__le32	rsvd11[5];
};

int main(void)
{
    int fd;
    struct nvme_identify cmd;
    void *buf;
    int ret;

    fd = open("/dev/nvmet0", O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    buf = aligned_alloc(4096, IDENTIFY_DATA_SIZE);
    if (!buf) {
        perror("aligned_alloc");
        close(fd);
        return 1;
    }
    memset(buf, 0, IDENTIFY_DATA_SIZE);

    memset(&cmd, 0, sizeof(cmd));
    cmd.opcode = 0x6;
    cmd.cns    = 1;
    cmd.prp1 = (uintptr_t)buf;

    ret = ioctl(fd, IOCTL_ADMIN_CMD, &cmd);
    if (ret < 0) {
        perror("ioctl(IOCTL_ADMIN_CMD)");
        free(buf);
        close(fd);
        return 1;
    }

    printf("VID  : %x\n", *(__u16*)buf);
    //printf("SSVID: %04x\n", id->ssvid);
    //printf("SN   : %.20s\n", id->sn);
    //printf("MN   : %.40s\n", id->mn);
    //printf("FR   : %.8s\n", id->fr);

    free(buf);
    close(fd);
    return 0;
}
