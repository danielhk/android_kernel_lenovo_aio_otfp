#ifndef __H_MTK_DISP_MGR__
#define __H_MTK_DISP_MGR__

long mtk_disp_mgr_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
extern unsigned int is_hwc_enabled;
extern int is_DAL_Enabled(void);
char *disp_session_mode_spy(unsigned int session_id);
#endif
