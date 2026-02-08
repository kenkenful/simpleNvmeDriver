#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/nvme.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include "nvmedrv.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Simple NVMe Device Driver");

#define MAJOR_NUM 100
#define DEVICE_NAME "nvmet"
#define NVME_MINORS		(1U << MINORBITS)

static DEFINE_IDA(nvme_instance_ida);

static struct class* nvme_class;
static dev_t nvme_chr_devt;   //major番号を動的に割り当てるために必要

#define SQ_SIZE(depth)		(depth * sizeof(struct nvme_command))
#define CQ_SIZE(depth)		(depth * sizeof(struct nvme_completion))
#define QUEUE_DEPTH (32)   // Admin Queue とIO Queueのdepthは32とする。
#define TIMEOUT (6*HZ)

// NVMeキューを管理する構造体
struct nvme_queue{
    struct nvme_command *sq_cmds;           // SQのカーネル仮想アドレス
    volatile struct nvme_completion *cqes;  // CQのカーネル仮想アドレス

	struct nvme_command __iomem *sq_cmds_io;
    struct nvme_dev *pnvme_dev;

    u32 __iomem *q_db;
    u16 q_depth;
	s16 cq_vector;
    u16 sq_head;
	u16 sq_tail;
	u16 cq_head;
    //u16 cq_tail;   //管理しなくてよさそう。

	u16 qid;
	u8 cq_phase;
	u8 cqe_seen;
    u32 *dbbuf_sq_db; 
	u32 *dbbuf_cq_db;
	u32 *dbbuf_sq_ei;
	u32 *dbbuf_cq_ei;

    spinlock_t q_lock;         // multi thread で同じQueueに同時にアクセスするときは、atomicに行う。

    dma_addr_t sq_dma_addr;    // SQの物理アドレス
	dma_addr_t cq_dma_addr;    // CQの物理アドレス　

    int command_id;

    wait_queue_head_t wq;
    int wait_condition;

};

// NVMeデバイスを管理する構造体
struct nvme_dev{
    struct pci_dev *pdev;
    struct cdev cdev;
    dev_t  devt;
    int instance;
    bool is_open;
    int io_qid;

    // for PCI pio/mmio
    unsigned long long mmio_base, mmio_flags, mmio_length;
    void __iomem *bar;

    struct nvme_queue *padminQ;
    struct nvme_queue *pioQ;

    struct dma_pool *cmd_pool;

    u32 db_stride;

};

static struct pci_device_id test_nvme_ids[] =
{
    { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
    { 0, },
};

MODULE_DEVICE_TABLE(pci, test_nvme_ids);

#if 0

static int submitCmd(struct nvme_command *c, struct nvme_queue *q){

    spin_lock(&q->q_lock);
    u16 tail = q->sq_tail;

    c->common.command_id = q->command_id++;

    struct nvme_dev *p = q->pnvme_dev;
    q -> wait_condition = 0;

    memcpy(&q->sq_cmds[tail], c, sizeof(struct nvme_command));
    if (++tail == q->q_depth){
        tail = 0;
    }
    writel(tail, p-> bar + NVME_REG_DBS + 8 * q->qid); 
    q->sq_tail = tail;

    spin_unlock(&q->q_lock);

    int left = wait_event_interruptible_timeout(q->wq, q->wait_condition == 1, msecs_to_jiffies(5000)); 

    if (left > 0) return 0;
    if (left == 0) return -ETIMEDOUT;
    return left; 
}
#else 

static inline void nvme_write_sq_db(struct nvme_dev *d, u16 qid, u16 tail)
{
    u32 stride = d->db_stride; /* 4 << DSTRD */
    /* SQ doorbell index = 2*qid */
    writel(tail, d->bar + NVME_REG_DBS + stride * (2 * qid));
}

static int submitCmd(struct nvme_command *c, struct nvme_queue *q)
{
    struct nvme_dev *dev = q->pnvme_dev;
    u16 tail;
    u16 mycid;
    long left;

    /* 並列を許さない前提でも、最低限の順序保証 */
    WRITE_ONCE(q->wait_condition, 0);

    spin_lock(&q->q_lock);

    tail = q->sq_tail;

    /* CID を発行（16bitで回るので u16 推奨） */
    mycid = (u16)q->command_id++;
    c->common.command_id = cpu_to_le16(mycid);

    /* SQE をメモリに書く */
    memcpy(&q->sq_cmds[tail], c, sizeof(*c));

    /* デバイスが読む前に SQE 書き込みを完了させる */
    wmb();

    /* tail 更新 */
    if (++tail == q->q_depth)
        tail = 0;
    q->sq_tail = tail;

    /* ドアベルを叩く（DSTRD対応） */
    nvme_write_sq_db(dev, q->qid, tail);

    spin_unlock(&q->q_lock);

    /*
     * 完了待ち：
     * いまの実装は「何か完了したら起きる」方式。
     * 本当は CID で待つべきだが、まずは現状維持で安全化。
     */
    left = wait_event_interruptible_timeout(
        q->wq,
        READ_ONCE(q->wait_condition) == 1,
        msecs_to_jiffies(5000)
    );

    if (left > 0)
        return 0;
    if (left == 0)
        return -ETIMEDOUT;
    return (int)left; /* -ERESTARTSYS 等 */
}



#endif



static int nvmet_pci_open(struct inode* inode, struct file* filp){
    struct nvme_dev *pnvme_dev = container_of(inode->i_cdev, struct nvme_dev, cdev);
    filp->private_data = pnvme_dev;
    pnvme_dev->is_open = 1;
    return 0;
}

static int nvmet_pci_close(struct inode* inode, struct file* filp){
    struct nvme_dev *pnvme_dev = filp->private_data;
    pnvme_dev->is_open = 0;
    return 0;
}

static long nvme_ioctl(struct file *filp, unsigned int ioctlnum, unsigned long ioctlparam){
    int rc = 0;
    struct nvme_command cmd = {0};
    struct nvme_dev *pnvme_dev = filp->private_data;
    
    void* virt_addr;
    dma_addr_t dma_addr;

    void __user *user_buf;
    
    switch(ioctlnum){
        case IOCTL_IO_CMD:
            // ユーザー空間からカーネル空間にコマンドをコピー
            if((rc = copy_from_user(&cmd, (void  __user*)ioctlparam, sizeof(struct nvme_command)))){
                return rc;
            }

            virt_addr = dma_pool_alloc(pnvme_dev->cmd_pool, GFP_ATOMIC, &dma_addr);

            // Writeのとき　ユーザー空間からカーネル空間にデーターバッファをコピー
            if((rc = copy_from_user(virt_addr, (void  __user*)cmd.rw.dptr.prp1, 4096))){
                return rc;
            }
            
            cmd.rw.dptr.prp1 = cpu_to_le64(dma_addr); 
            submitCmd(&cmd, pnvme_dev->pioQ);

            dma_pool_free(pnvme_dev->cmd_pool, virt_addr, dma_addr);

            break;
        
        case IOCTL_ADMIN_CMD:
            
            // ユーザー空間からカーネル空間にコマンドをコピー
            if((rc = copy_from_user(&cmd, (void  __user*)ioctlparam, sizeof(struct nvme_command)))){
                return rc;
            }

            user_buf = (void __user *)(uintptr_t)le64_to_cpu(cmd.common.dptr.prp1);

            virt_addr = dma_pool_alloc(pnvme_dev->cmd_pool, GFP_ATOMIC, &dma_addr);

            if (!virt_addr)
                return -ENOMEM;

            // Writeのとき　ユーザー空間からカーネル空間にデーターバッファをコピー
            //if((rc = copy_from_user(virt_addr, (void  __user*)cmd.rw.dptr.prp1, 4096))){
            //    return rc;
            //}
            
            cmd.rw.dptr.prp1 = cpu_to_le64(dma_addr);

            rc = submitCmd(&cmd, pnvme_dev->padminQ);

            if(rc){
                dma_pool_free(pnvme_dev->cmd_pool, virt_addr, dma_addr);
                return rc;
            }

            if ((rc = copy_to_user(user_buf, virt_addr, 4096))) {
                dma_pool_free(pnvme_dev->cmd_pool, virt_addr, dma_addr);
                return rc;
            }

            dma_pool_free(pnvme_dev->cmd_pool, virt_addr, dma_addr);

            break;

        default:
            break;
    }

    return 0;
}

static const struct file_operations nvme_chr_fops = {
    .owner = THIS_MODULE,
    .open = nvmet_pci_open,
    .release = nvmet_pci_close,
    .unlocked_ioctl = nvme_ioctl,
};

//割り込みハンドラ
#if 0
static irqreturn_t nvme_irq(int irq, void *data)
{
    struct nvme_queue *nvmeq = data;
    struct nvme_completion cqe;
    u16 head, phase;
    head = nvmeq -> cq_head;     
    phase = nvmeq -> cq_phase;   

    rmb();

    while(1){
        cqe = nvmeq->cqes[head];  
     
        if((le16_to_cpu(cqe.status) & 1) != phase){
            break;  
        }

        pr_info("Command id: 0x%x,  Status: 0x%x, phase tag %d\n", le16_to_cpu(cqe.command_id), (le16_to_cpu(cqe.status) & 0xfe) >> 1, le16_to_cpu(cqe.status) & 1);    

        nvmeq->sq_head = le16_to_cpu(cqe.sq_head);
        if(++head == nvmeq -> q_depth){
            head = 0;          //Completion Queueを一周したので、headを先頭に戻す。
            phase = !phase;    //Completion Queueを一周したので、phase tagを反転する。
        }
   }

    nvmeq->wait_condition = 1;
    wake_up_interruptible(&nvmeq->wq);    

    if (head == nvmeq->cq_head && phase == nvmeq->cq_phase)
		return IRQ_NONE;

	writel(head, nvmeq -> pnvme_dev-> bar + NVME_REG_DBS + pnvme_dev -> db_stride * (2 * nvmeq -> qid + 1) );  // Admin CQのドアベルレジスタは、0x1004にある。
	nvmeq->cq_head = head;
	nvmeq->cq_phase = phase;

    wmb();

    return  IRQ_HANDLED;
}
#else 

static inline void nvme_write_cq_db(struct nvme_dev *d, u16 qid, u16 head)
{
    u32 stride = d->db_stride;                  // 4 << DSTRD
    writel(head, d->bar + NVME_REG_DBS + stride * (2 * qid + 1));
}

static irqreturn_t nvme_irq(int irq, void *data)
{
    struct nvme_queue *nvmeq = data;
    u16 head  = nvmeq->cq_head;
    u16 phase = nvmeq->cq_phase;

    dma_rmb();

    while (1) {
        struct nvme_completion cqe = nvmeq->cqes[head];

        if ((le16_to_cpu(cqe.status) & 1) != phase)
            break;

        nvmeq->sq_head = le16_to_cpu(cqe.sq_head);

        if (++head == nvmeq->q_depth) {
            head = 0;
            phase = !phase;
        }
    }

    if (head == nvmeq->cq_head && phase == nvmeq->cq_phase)
        return IRQ_NONE;

    /* CQ head をデバイスへ通知 */
    nvme_write_cq_db(nvmeq->pnvme_dev, nvmeq->qid, head);

    nvmeq->cq_head  = head;
    nvmeq->cq_phase = phase;

    WRITE_ONCE(nvmeq->wait_condition, 1);
    smp_wmb();
    wake_up_interruptible(&nvmeq->wq);

    return IRQ_HANDLED;
}

#endif

static int nvmet_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id){
    u32 ctrl_config = 0;
    
    u32 aqa;
    u32 csts;
    unsigned long timeout;
    int ret = -ENOMEM;

    struct nvme_dev *pnvme_dev;
    struct nvme_queue *padminQ;       // Admin Queue管理構造体へのポインタ  
    struct nvme_queue *pioQ;          // IO Queue管理構造体へのポインタを一つだけ作成する。

    // Allocate device structure
    pnvme_dev = kmalloc(sizeof(struct nvme_dev), GFP_KERNEL);
    if(!pnvme_dev){
        return -ENOMEM;
    }

    pnvme_dev->io_qid = 0;

    ret = ida_simple_get(&nvme_instance_ida, 0, 0, GFP_KERNEL);
    if (ret < 0){
        kfree(pnvme_dev);
        return ret;
    }
		
    pnvme_dev -> instance = ret;

    pnvme_dev -> devt = MKDEV(MAJOR(nvme_chr_devt), pnvme_dev -> instance);

    cdev_init(&pnvme_dev->cdev, &nvme_chr_fops);
    pnvme_dev->cdev.owner = THIS_MODULE;

    // キャラクタデバイス作成
    if ((ret = cdev_add(&pnvme_dev->cdev, pnvme_dev -> devt , NVME_MINORS)) != 0) {
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return ret;
    }

    struct device *dev = device_create(nvme_class, NULL, MKDEV(MAJOR(nvme_chr_devt), pnvme_dev -> instance), NULL, "nvmet%d", pnvme_dev -> instance);
    
    if (IS_ERR(dev)) {
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return PTR_ERR(dev);   
    }

    ret = pci_enable_device_mem(pdev); 
    if (ret){
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return ret;
    }
		
    ret = pci_request_mem_regions(pdev, DEVICE_NAME);

    if(ret) {
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

        return ret;
    }

    // デバイスからpdevを参照できるようにする
    pnvme_dev->pdev = pci_dev_get(pdev);

    // pdevからデバイスを参照できるようにする
	pci_set_drvdata(pdev, pnvme_dev);


    ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (ret) {
        ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (ret){
            pci_release_mem_regions(pdev);
            pci_disable_device(pdev);
            device_destroy(nvme_class, pnvme_dev -> devt );
            cdev_del(&pnvme_dev->cdev); 
            ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
            kfree(pnvme_dev);
            return ret;
        }
    }

    pci_set_master(pdev);

    // Admin Queue管理構造体の作成
    padminQ = kmalloc(sizeof(struct nvme_queue), GFP_KERNEL);
    if(!padminQ){
        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }

    // デバイスからQueueを参照できるようにする
    pnvme_dev->padminQ = padminQ;
    
    // Queueからデバイスを参照できるようにする
    padminQ->pnvme_dev = pnvme_dev;

    //Admin Queue管理構造体の変数初期化
    spin_lock_init(&padminQ->q_lock);
    padminQ->sq_tail = 0;
    padminQ->cq_head = 0;
	padminQ->cq_phase = 1;    //デフォルトのphase tagは１。
    padminQ->q_depth = QUEUE_DEPTH;
	padminQ->qid = 0;
	padminQ->cq_vector = padminQ->qid;
    padminQ -> command_id = 0;

    //memset((void *)padminQ->cqes, 0, CQ_SIZE(QUEUE_DEPTH)); 

    //  IO Queue管理構造体の作成
    pioQ = kmalloc(sizeof(struct nvme_queue), GFP_KERNEL);
    if(!pioQ){
        kfree(padminQ);
        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }
    
    // デバイスからQueueを参照できるようにする
    pnvme_dev->pioQ = pioQ;

    // Queueからデバイスを参照できるようにする
    pioQ->pnvme_dev = pnvme_dev;

    //IO Queue管理構造体の変数初期化
    spin_lock_init(&pioQ->q_lock);
    pioQ -> sq_tail = 0;
    pioQ -> cq_head = 0;
    pioQ -> cq_phase = 1;    //デフォルトのphase tagは１です。
    pioQ -> q_depth = QUEUE_DEPTH;
    pioQ -> qid = 1;
    pioQ -> cq_vector = pioQ->qid;
    pioQ -> command_id = 0;

    // Admin CQ用のDMAメモリを確保
    padminQ->cqes = dma_alloc_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), &padminQ->cq_dma_addr, GFP_KERNEL);
    if(padminQ->cqes == NULL){
        kfree(pioQ);
        kfree(padminQ);
        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }

    memset((void *)padminQ->cqes, 0, CQ_SIZE(QUEUE_DEPTH)); 

    // Admin SQ用のDMAメモリを確保
    padminQ->sq_cmds = dma_alloc_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), &padminQ->sq_dma_addr, GFP_KERNEL);
    if(padminQ->sq_cmds == NULL){

        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }

    memset((void *)padminQ->sq_cmds , 0, SQ_SIZE(QUEUE_DEPTH));


// IO CQ用のDMAメモリを作成
    pioQ->cqes = dma_alloc_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), &pioQ->cq_dma_addr, GFP_KERNEL);
    if(pioQ->cqes == NULL){

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }

    memset((void *)pioQ->cqes, 0, CQ_SIZE(QUEUE_DEPTH)); 


    // IO SQ用のDMAメモリを作成
    pioQ->sq_cmds = dma_alloc_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), &pioQ->sq_dma_addr, GFP_KERNEL);
    if(pioQ->sq_cmds == NULL){
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;
    }

    memset((void *)pioQ->sq_cmds , 0, SQ_SIZE(QUEUE_DEPTH));


    // BAR0の物理アドレスを長さを取得
    pnvme_dev->mmio_base = pci_resource_start(pdev, 0);
    pnvme_dev->mmio_length = pci_resource_len(pdev, 0);
    pnvme_dev->mmio_flags = pci_resource_flags(pdev, 0);
    dev_info(&pdev->dev, "mmio_base: 0x%llx, mmio_length: 0x%llx, mmio_flags: 0x%llx\n",pnvme_dev->mmio_base, pnvme_dev->mmio_length, pnvme_dev->mmio_flags);

    // BAR0をカーネル仮想アドレスにマッピングする。
    pnvme_dev->bar = ioremap(pnvme_dev->mmio_base, pnvme_dev->mmio_length);

    if(!pnvme_dev->bar){
        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return -ENOMEM;

    }

    // Admin CQとIO CQ用にMSI-X 割り込みベクターは(1 + 1)個確保する。
    int nr_io_queues = pci_alloc_irq_vectors(pdev, 1, 2, PCI_IRQ_MSIX);

    if (nr_io_queues < 0) {
        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

        return nr_io_queues;
    }

    pr_info( "nr io queues: %d\n", nr_io_queues);

    ret = request_irq(
        pci_irq_vector(pdev, 0), 
        nvme_irq, 0, "adminQ", padminQ);

    if (ret){
        pci_free_irq_vectors(pdev);

        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);
        return ret;
    }   

    ret = request_irq(
        pci_irq_vector(pdev, 1),
        nvme_irq, 0, "ioQ", pioQ);

    if (ret){

        free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

        pci_free_irq_vectors(pdev);

        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

        return ret;
    }   

    u64 cap = lo_hi_readq(pnvme_dev->bar + NVME_REG_CAP);
    pnvme_dev->db_stride = 4U << NVME_CAP_STRIDE(cap); 
    //pr_info("Doorbell Stride :%d\n", NVME_CAP_STRIDE(cap));

    // disable device
    ctrl_config = 0;
    ctrl_config &= ~NVME_CC_SHN_MASK;
	ctrl_config &= ~NVME_CC_ENABLE;
    writel(ctrl_config, pnvme_dev-> bar + NVME_REG_CC);

    timeout = ((NVME_CAP_TIMEOUT(cap) + 1) * HZ / 2) + jiffies;

    //Controller not ready になるまで待つ 
    while((readl(pnvme_dev->bar + NVME_REG_CSTS) & NVME_CSTS_RDY) != 0){
        dev_info(&pdev->dev, "now ready. waiting...\n");
        msleep(100);
        if (time_after(jiffies, timeout)) {

            free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
            free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

            pci_free_irq_vectors(pdev);

            iounmap(pnvme_dev->bar);

            dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
            dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

            dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
            dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

            kfree(pnvme_dev ->pioQ);
            kfree(pnvme_dev ->padminQ);

            pci_release_mem_regions(pdev);
            pci_disable_device(pdev);
            device_destroy(nvme_class, pnvme_dev -> devt );
            cdev_del(&pnvme_dev->cdev); 
            ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
            kfree(pnvme_dev);
            return -ENODEV;

		}
    }

    csts = readl(pnvme_dev->bar + NVME_REG_CSTS);
    dev_info(&pdev->dev, "csts rdy : %d\n", csts & NVME_CSTS_RDY);

    //  コントローラのレジスタを設定する。
    aqa = padminQ->q_depth - 1;
    aqa |= aqa << 16;
    writel(aqa, pnvme_dev-> bar + NVME_REG_AQA); // Admin Queue Attributesを設定
    lo_hi_writeq(padminQ-> sq_dma_addr, pnvme_dev->bar + NVME_REG_ASQ); // Admin Submission Queue Base Address を設定
	lo_hi_writeq(padminQ-> cq_dma_addr, pnvme_dev->bar + NVME_REG_ACQ); // Admin Completion Queue Base Address を設定

    //unsigned dev_page_min = NVME_CAP_MPSMIN(cap) + 12;
    //pr_info( "Minimum device page size %u\n", 1 << dev_page_min);

    
    unsigned page_shift = 12;
    ctrl_config = NVME_CC_CSS_NVM;
	ctrl_config |= (page_shift - 12) << NVME_CC_MPS_SHIFT;
	ctrl_config |= NVME_CC_AMS_RR | NVME_CC_SHN_NONE;
	ctrl_config |= NVME_CC_IOSQES | NVME_CC_IOCQES;
    //ctrl_config &= ~NVME_CC_SHN_MASK;
	ctrl_config |= NVME_CC_ENABLE;

    writel(ctrl_config, pnvme_dev-> bar + NVME_REG_CC); // Controller Capabilitiesに書く
    timeout = ((NVME_CAP_TIMEOUT(cap) + 1) * HZ / 2) + jiffies;

    //ready になるまで待つ 
    while((readl(pnvme_dev->bar + NVME_REG_CSTS) & NVME_CSTS_RDY) != 1){
        pr_info("Polling until ready\n");
        msleep(100);
        if (time_after(jiffies, timeout)) {

            free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
            free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

            pci_free_irq_vectors(pdev);

            iounmap(pnvme_dev->bar);

            dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
            dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

            dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
            dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

            kfree(pnvme_dev ->pioQ);
            kfree(pnvme_dev ->padminQ);

            pci_release_mem_regions(pdev);
            pci_disable_device(pdev);
            device_destroy(nvme_class, pnvme_dev -> devt );
            cdev_del(&pnvme_dev->cdev); 
            ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
            kfree(pnvme_dev);
            return -ENODEV;

		}
    }

    dev_info(&pdev->dev, "csts rdy : %d\n", readl(pnvme_dev->bar + NVME_REG_CSTS) & NVME_CSTS_RDY);


    // wait queueの初期化
    init_waitqueue_head(&padminQ->wq);
    init_waitqueue_head(&pioQ->wq);

  	struct nvme_command cmd = {0};
    
    // Create IO CQコマンドを作成する。
    int flags = NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED;
    memset(&cmd, 0, sizeof(cmd));
    cmd.create_cq.opcode = nvme_admin_create_cq;
    cmd.create_cq.prp1 = cpu_to_le64(pioQ->cq_dma_addr);
    cmd.create_cq.cqid = cpu_to_le16(pioQ->qid);
    cmd.create_cq.qsize = cpu_to_le16(pioQ->q_depth - 1);
    cmd.create_cq.cq_flags = cpu_to_le16(flags);
    cmd.create_cq.irq_vector = cpu_to_le16(pioQ->cq_vector);
    
    // Create IO CQコマンドを発行する。
    ret = submitCmd(&cmd, padminQ);

    if(ret){
		dev_err(&pdev->dev, "Command submit failed\n");

        free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
        free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

        pci_free_irq_vectors(pdev);

        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

    }
    
    // Create IO SQコマンドを作成する。
    flags = NVME_QUEUE_PHYS_CONTIG;
    memset(&cmd, 0, sizeof(cmd));
    cmd.create_sq.opcode = nvme_admin_create_sq;
    cmd.create_sq.prp1 = cpu_to_le64(pioQ->sq_dma_addr);
    cmd.create_sq.sqid = cpu_to_le16(pioQ->qid);
    cmd.create_sq.qsize = cpu_to_le16(pioQ->q_depth - 1);
    cmd.create_sq.sq_flags = cpu_to_le16(flags);
    cmd.create_sq.cqid = cpu_to_le16(pioQ->qid);
    
    // Create IO SQコマンドを発行する。
    ret = submitCmd(&cmd, padminQ);

    if(ret){

		dev_err(&pdev->dev, "Command submit failed\n");

        //Delete CQコマンドを発行する
        memset(&cmd, 0, sizeof(cmd));
        cmd.delete_queue.opcode = nvme_admin_delete_cq;
	    cmd.delete_queue.qid = cpu_to_le16(pnvme_dev->pioQ->qid);
        submitCmd(&cmd, pnvme_dev->padminQ);

        free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
        free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

        pci_free_irq_vectors(pdev);

        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

    }

    msleep(100);

    pnvme_dev ->cmd_pool = dma_pool_create(
		"cmd_pool",
		&pdev->dev,
		4096,
		4096,   
		0     
	);


    if (!pnvme_dev->cmd_pool) {
		dev_err(&pdev->dev, "dma_pool_create failed\n");

        //Delete SQコマンドを発行する
        memset(&cmd, 0, sizeof(cmd));
        cmd.delete_queue.opcode = nvme_admin_delete_sq;
	    cmd.delete_queue.qid = cpu_to_le16(pnvme_dev->pioQ->qid);
        submitCmd(&cmd, pnvme_dev->padminQ);

        //Delete CQコマンドを発行する
        memset(&cmd, 0, sizeof(cmd));
        cmd.delete_queue.opcode = nvme_admin_delete_cq;
	    cmd.delete_queue.qid = cpu_to_le16(pnvme_dev->pioQ->qid);
        submitCmd(&cmd, pnvme_dev->padminQ);

        free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
        free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

        pci_free_irq_vectors(pdev);

        iounmap(pnvme_dev->bar);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

        dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
        dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

        kfree(pnvme_dev ->pioQ);
        kfree(pnvme_dev ->padminQ);

        pci_release_mem_regions(pdev);
        pci_disable_device(pdev);
        device_destroy(nvme_class, pnvme_dev -> devt );
        cdev_del(&pnvme_dev->cdev); 
        ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
        kfree(pnvme_dev);

		return -ENOMEM;
	}
    
    return 0;

}

static void nvmet_pci_remove(struct pci_dev* pdev){
    pr_info("Driver is removed.\n");
    struct nvme_command cmd = {0};
    struct nvme_dev *pnvme_dev = pci_get_drvdata(pdev);

    if (pnvme_dev->cmd_pool) dma_pool_destroy(pnvme_dev->cmd_pool);

    //Delete SQコマンドを発行する
    memset(&cmd, 0, sizeof(cmd));
    cmd.delete_queue.opcode = nvme_admin_delete_sq;
	cmd.delete_queue.qid = cpu_to_le16(pnvme_dev->pioQ->qid);
    submitCmd(&cmd, pnvme_dev->padminQ);

    //Delete CQコマンドを発行する
    memset(&cmd, 0, sizeof(cmd));
    cmd.delete_queue.opcode = nvme_admin_delete_cq;
	cmd.delete_queue.qid = cpu_to_le16(pnvme_dev->pioQ->qid);
    submitCmd(&cmd, pnvme_dev->padminQ);


    free_irq(pci_irq_vector(pdev, 1), pnvme_dev ->pioQ);
    free_irq(pci_irq_vector(pdev, 0), pnvme_dev ->padminQ);

    pci_free_irq_vectors(pdev);

    iounmap(pnvme_dev->bar);

    dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->sq_cmds, pnvme_dev ->pioQ->sq_dma_addr);
    dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->pioQ->cqes, pnvme_dev ->pioQ->cq_dma_addr);

    dma_free_coherent(&pdev->dev, SQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->sq_cmds, pnvme_dev ->padminQ->sq_dma_addr);
    dma_free_coherent(&pdev->dev, CQ_SIZE(QUEUE_DEPTH), (void *)pnvme_dev ->padminQ->cqes, pnvme_dev ->padminQ->cq_dma_addr);

    kfree(pnvme_dev ->pioQ);
    kfree(pnvme_dev ->padminQ);

    pci_release_mem_regions(pdev);
    pci_disable_device(pdev);
    device_destroy(nvme_class, pnvme_dev -> devt );
    cdev_del(&pnvme_dev->cdev); 
    ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );
    kfree(pnvme_dev);

}

static void nvme_shutdown(struct pci_dev *pdev)
{
    unsigned long timeout;
    struct nvme_dev *pnvme_dev = pci_get_drvdata(pdev);
    u32 cc = (readl(pnvme_dev->bar + NVME_REG_CC) & ~NVME_CC_SHN_MASK) | NVME_CC_SHN_NORMAL;
    writel(cc, pnvme_dev->bar + NVME_REG_CC);
    timeout = 2 * HZ + jiffies;
    while ((readl(pnvme_dev->bar + NVME_REG_CSTS) & NVME_CSTS_SHST_MASK) != NVME_CSTS_SHST_CMPLT) {
		msleep(100);

		if (time_after(jiffies, timeout)) {
			dev_err(&pnvme_dev->pdev->dev, "Device shutdown incomplete; abort shutdown\n");
			//			return -ENODEV;
			return;
		}
	}
}

static struct pci_driver simple_nvme_driver = {
    .name = DEVICE_NAME,
    .id_table = test_nvme_ids,
    .probe = nvmet_pci_probe,
    .remove = nvmet_pci_remove,
    .shutdown = nvme_shutdown,
};

/* インストール時に実行 */
static int nvmet_init(void) {
    int ret;

    ida_init(&nvme_instance_ida);

	ret = alloc_chrdev_region(&nvme_chr_devt, 0, NVME_MINORS, DEVICE_NAME);
	if (ret != 0){
        pr_err("%d: %s()   %d",__LINE__, __FUNCTION__, ret ); 
        return ret;
    }

	nvme_class = class_create(DEVICE_NAME);
	if (IS_ERR(nvme_class)) {
		ret = PTR_ERR(nvme_class);
        unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
		return ret;
	}

    ret = pci_register_driver(&simple_nvme_driver); //ドライバー名、table, コールバック関数(probe, remove)を登録
    if(ret != 0){
        pr_err("%d: %s()   %d",__LINE__, __FUNCTION__, ret );     
        return ret;
    }

    return ret; 
}

/* アンインストール時に実行 */
static void nvmet_exit(void) {

    //del_timer(&timerl);

    pci_unregister_driver(&simple_nvme_driver);  //ドライバー名、table, コールバック関数(probe, remove)を削除  
    class_destroy(nvme_class);
    unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
    ida_destroy(&nvme_instance_ida);
}

module_init(nvmet_init);
module_exit(nvmet_exit);