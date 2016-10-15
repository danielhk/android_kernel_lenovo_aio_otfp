cmd_drivers/misc/mediatek/masp/asfv2/core/sec_osal.o := aarch64-linux-android-gcc -Wp,-MD,drivers/misc/mediatek/masp/asfv2/core/.sec_osal.o.d  -nostdinc -isystem /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/android-build/0/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/bin/../lib/gcc/aarch64-linux-android/4.9.x-google/include -I/home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include -Iarch/arm64/include/generated  -Iinclude -I/home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi -Iarch/arm64/include/generated/uapi -I/home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi -Iinclude/generated/uapi -include /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/linux/kconfig.h -D__KERNEL__ -mlittle-endian -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Werror=format -Werror=int-to-pointer-cast -Werror=pointer-to-int-cast -O2 -mgeneral-regs-only -fno-pic -Werror=frame-larger-than=1 -Wframe-larger-than=1400 -fno-stack-protector -Wno-unused-but-set-variable -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-var-tracking-assignments -g -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -I/home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/drivers/misc/mediatek/mach/mt6752/include -Idrivers/misc/mediatek/masp/asfv2/asf_inc -Idrivers/misc/mediatek/masp/asfv2/asf_export_inc -Idrivers/misc/mediatek/masp/mt6752/module -Idrivers/misc/mediatek/masp/asfv2/asf_inc -Idrivers/misc/mediatek/masp/asfv2/asf_export_inc -Idrivers/misc/mediatek/masp/module    -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(sec_osal)"  -D"KBUILD_MODNAME=KBUILD_STR(sec)" -c -o drivers/misc/mediatek/masp/asfv2/core/sec_osal.o drivers/misc/mediatek/masp/asfv2/core/sec_osal.c

source_drivers/misc/mediatek/masp/asfv2/core/sec_osal.o := drivers/misc/mediatek/masp/asfv2/core/sec_osal.c

deps_drivers/misc/mediatek/masp/asfv2/core/sec_osal.o := \
  drivers/misc/mediatek/masp/asfv2/asf_export_inc/sec_osal.h \
  include/linux/string.h \
    $(wildcard include/config/binary/printf.h) \
  include/linux/compiler.h \
    $(wildcard include/config/sparse/rcu/pointer.h) \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
    $(wildcard include/config/kprobes.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
    $(wildcard include/config/arch/use/builtin/bswap.h) \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/arch/dma/addr/t/64bit.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  include/uapi/linux/types.h \
  arch/arm64/include/generated/asm/types.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/types.h \
  include/asm-generic/int-ll64.h \
  include/uapi/asm-generic/int-ll64.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/uapi/asm-generic/bitsperlong.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/posix_types.h \
  include/linux/stddef.h \
  include/uapi/linux/stddef.h \
  arch/arm64/include/generated/asm/posix_types.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/posix_types.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/android-build/0/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9/lib/gcc/aarch64-linux-android/4.9.x-google/include/stdarg.h \
  include/uapi/linux/string.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/string.h \
  include/linux/bug.h \
    $(wildcard include/config/generic/bug.h) \
  arch/arm64/include/generated/asm/bug.h \
  include/asm-generic/bug.h \
    $(wildcard include/config/bug.h) \
    $(wildcard include/config/generic/bug/relative/pointers.h) \
    $(wildcard include/config/debug/bugverbose.h) \
    $(wildcard include/config/smp.h) \
  include/linux/kernel.h \
    $(wildcard include/config/preempt/voluntary.h) \
    $(wildcard include/config/debug/atomic/sleep.h) \
    $(wildcard include/config/prove/locking.h) \
    $(wildcard include/config/ring/buffer.h) \
    $(wildcard include/config/tracing.h) \
    $(wildcard include/config/ftrace/mcount/record.h) \
  include/linux/linkage.h \
  include/linux/stringify.h \
  include/linux/export.h \
    $(wildcard include/config/have/underscore/symbol/prefix.h) \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/modversions.h) \
    $(wildcard include/config/unused/symbols.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/linkage.h \
  include/linux/bitops.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/bitops.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/barrier.h \
  include/asm-generic/bitops/builtin-__ffs.h \
  include/asm-generic/bitops/builtin-ffs.h \
  include/asm-generic/bitops/builtin-__fls.h \
  include/asm-generic/bitops/builtin-fls.h \
  include/asm-generic/bitops/ffz.h \
  include/asm-generic/bitops/fls64.h \
  include/asm-generic/bitops/find.h \
    $(wildcard include/config/generic/find/first/bit.h) \
  include/asm-generic/bitops/sched.h \
  include/asm-generic/bitops/hweight.h \
  include/asm-generic/bitops/arch_hweight.h \
  include/asm-generic/bitops/const_hweight.h \
  include/asm-generic/bitops/lock.h \
  include/asm-generic/bitops/non-atomic.h \
  include/asm-generic/bitops/le.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/byteorder.h \
  include/linux/byteorder/little_endian.h \
  include/uapi/linux/byteorder/little_endian.h \
  include/linux/swab.h \
  include/uapi/linux/swab.h \
  arch/arm64/include/generated/asm/swab.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/swab.h \
  include/linux/byteorder/generic.h \
  include/linux/log2.h \
    $(wildcard include/config/arch/has/ilog2/u32.h) \
    $(wildcard include/config/arch/has/ilog2/u64.h) \
  include/linux/typecheck.h \
  include/linux/printk.h \
    $(wildcard include/config/mt/printk/uart/console.h) \
    $(wildcard include/config/mt/eng/build.h) \
    $(wildcard include/config/log/too/much/warning.h) \
    $(wildcard include/config/early/printk.h) \
    $(wildcard include/config/printk.h) \
    $(wildcard include/config/dynamic/debug.h) \
  include/linux/init.h \
    $(wildcard include/config/broken/rodata.h) \
  include/linux/kern_levels.h \
  include/linux/dynamic_debug.h \
  include/uapi/linux/kernel.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/sysinfo.h \
  include/linux/spinlock.h \
    $(wildcard include/config/debug/spinlock.h) \
    $(wildcard include/config/generic/lockbreak.h) \
    $(wildcard include/config/preempt.h) \
    $(wildcard include/config/debug/lock/alloc.h) \
  include/linux/preempt.h \
    $(wildcard include/config/debug/preempt.h) \
    $(wildcard include/config/preempt/tracer.h) \
    $(wildcard include/config/context/tracking.h) \
    $(wildcard include/config/preempt/count.h) \
    $(wildcard include/config/preempt/notifiers.h) \
  include/linux/thread_info.h \
    $(wildcard include/config/compat.h) \
    $(wildcard include/config/debug/stack/usage.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/thread_info.h \
    $(wildcard include/config/arm64/64k/pages.h) \
    $(wildcard include/config/mt/rt/sched.h) \
  include/linux/list.h \
    $(wildcard include/config/debug/list.h) \
  include/linux/poison.h \
    $(wildcard include/config/illegal/pointer/value.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/const.h \
  include/linux/irqflags.h \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/preempt/monitor.h) \
    $(wildcard include/config/irqsoff/tracer.h) \
    $(wildcard include/config/trace/irqflags/support.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/irqflags.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/ptrace.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/ptrace.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/hwcap.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/hwcap.h \
  include/linux/bottom_half.h \
  include/linux/spinlock_types.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/spinlock_types.h \
  include/linux/lockdep.h \
    $(wildcard include/config/lockdep.h) \
    $(wildcard include/config/lock/stat.h) \
    $(wildcard include/config/prove/rcu.h) \
  include/linux/rwlock_types.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/spinlock.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/processor.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/fpsimd.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/hw_breakpoint.h \
    $(wildcard include/config/have/hw/breakpoint.h) \
  include/linux/rwlock.h \
  include/linux/spinlock_api_smp.h \
    $(wildcard include/config/inline/spin/lock.h) \
    $(wildcard include/config/inline/spin/lock/bh.h) \
    $(wildcard include/config/inline/spin/lock/irq.h) \
    $(wildcard include/config/inline/spin/lock/irqsave.h) \
    $(wildcard include/config/inline/spin/trylock.h) \
    $(wildcard include/config/inline/spin/trylock/bh.h) \
    $(wildcard include/config/uninline/spin/unlock.h) \
    $(wildcard include/config/inline/spin/unlock/bh.h) \
    $(wildcard include/config/inline/spin/unlock/irq.h) \
    $(wildcard include/config/inline/spin/unlock/irqrestore.h) \
  include/linux/rwlock_api_smp.h \
    $(wildcard include/config/inline/read/lock.h) \
    $(wildcard include/config/inline/write/lock.h) \
    $(wildcard include/config/inline/read/lock/bh.h) \
    $(wildcard include/config/inline/write/lock/bh.h) \
    $(wildcard include/config/inline/read/lock/irq.h) \
    $(wildcard include/config/inline/write/lock/irq.h) \
    $(wildcard include/config/inline/read/lock/irqsave.h) \
    $(wildcard include/config/inline/write/lock/irqsave.h) \
    $(wildcard include/config/inline/read/trylock.h) \
    $(wildcard include/config/inline/write/trylock.h) \
    $(wildcard include/config/inline/read/unlock.h) \
    $(wildcard include/config/inline/write/unlock.h) \
    $(wildcard include/config/inline/read/unlock/bh.h) \
    $(wildcard include/config/inline/write/unlock/bh.h) \
    $(wildcard include/config/inline/read/unlock/irq.h) \
    $(wildcard include/config/inline/write/unlock/irq.h) \
    $(wildcard include/config/inline/read/unlock/irqrestore.h) \
    $(wildcard include/config/inline/write/unlock/irqrestore.h) \
  include/linux/atomic.h \
    $(wildcard include/config/arch/has/atomic/or.h) \
    $(wildcard include/config/generic/atomic64.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/atomic.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/cmpxchg.h \
  include/asm-generic/atomic-long.h \
  include/linux/semaphore.h \
  include/linux/delay.h \
  arch/arm64/include/generated/asm/delay.h \
  include/asm-generic/delay.h \
  include/linux/errno.h \
  include/uapi/linux/errno.h \
  arch/arm64/include/generated/asm/errno.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/errno.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/errno-base.h \
  include/linux/mutex.h \
    $(wildcard include/config/debug/mutexes.h) \
    $(wildcard include/config/mutex/spin/on/owner.h) \
    $(wildcard include/config/mt/debug/mutexes.h) \
    $(wildcard include/config/have/arch/mutex/cpu/relax.h) \
  include/linux/mtd/mtd.h \
    $(wildcard include/config/mtk/mtd/nand.h) \
    $(wildcard include/config/mtd/debug.h) \
    $(wildcard include/config/mtd/debug/verbose.h) \
    $(wildcard include/config/mtk/nss/cachev/max/cnt.h) \
  include/linux/uio.h \
  include/uapi/linux/uio.h \
  include/linux/notifier.h \
  include/linux/rwsem.h \
    $(wildcard include/config/rwsem/generic/spinlock.h) \
  include/linux/rwsem-spinlock.h \
  include/linux/srcu.h \
  include/linux/rcupdate.h \
    $(wildcard include/config/rcu/torture/test.h) \
    $(wildcard include/config/tree/rcu.h) \
    $(wildcard include/config/tree/preempt/rcu.h) \
    $(wildcard include/config/rcu/trace.h) \
    $(wildcard include/config/preempt/rcu.h) \
    $(wildcard include/config/rcu/user/qs.h) \
    $(wildcard include/config/tiny/rcu.h) \
    $(wildcard include/config/tiny/preempt/rcu.h) \
    $(wildcard include/config/debug/objects/rcu/head.h) \
    $(wildcard include/config/hotplug/cpu.h) \
    $(wildcard include/config/rcu/nocb/cpu.h) \
  include/linux/cache.h \
    $(wildcard include/config/arch/has/cache/line/size.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/cache.h \
  include/linux/threads.h \
    $(wildcard include/config/nr/cpus.h) \
    $(wildcard include/config/base/small.h) \
  include/linux/cpumask.h \
    $(wildcard include/config/cpumask/offstack.h) \
    $(wildcard include/config/debug/per/cpu/maps.h) \
    $(wildcard include/config/disable/obsolete/cpumask/functions.h) \
  include/linux/bitmap.h \
  include/linux/seqlock.h \
  include/linux/completion.h \
  include/linux/wait.h \
  arch/arm64/include/generated/asm/current.h \
  include/asm-generic/current.h \
  include/uapi/linux/wait.h \
  include/linux/debugobjects.h \
    $(wildcard include/config/debug/objects.h) \
    $(wildcard include/config/debug/objects/free.h) \
  include/linux/rcutree.h \
  include/linux/workqueue.h \
    $(wildcard include/config/debug/objects/work.h) \
    $(wildcard include/config/freezer.h) \
    $(wildcard include/config/sysfs.h) \
  include/linux/timer.h \
    $(wildcard include/config/timer/stats.h) \
    $(wildcard include/config/debug/objects/timers.h) \
  include/linux/ktime.h \
    $(wildcard include/config/ktime/scalar.h) \
  include/linux/time.h \
    $(wildcard include/config/arch/uses/gettimeoffset.h) \
  include/linux/math64.h \
  arch/arm64/include/generated/asm/div64.h \
  include/asm-generic/div64.h \
  include/uapi/linux/time.h \
  include/linux/jiffies.h \
  include/linux/timex.h \
  include/uapi/linux/timex.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/param.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/param.h \
  include/asm-generic/param.h \
    $(wildcard include/config/hz.h) \
  include/uapi/asm-generic/param.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/timex.h \
  include/asm-generic/timex.h \
  include/linux/device.h \
    $(wildcard include/config/debug/devres.h) \
    $(wildcard include/config/acpi.h) \
    $(wildcard include/config/pinctrl.h) \
    $(wildcard include/config/numa.h) \
    $(wildcard include/config/cma.h) \
    $(wildcard include/config/pm/sleep.h) \
    $(wildcard include/config/devtmpfs.h) \
    $(wildcard include/config/sysfs/deprecated.h) \
  include/linux/ioport.h \
    $(wildcard include/config/memory/hotremove.h) \
  include/linux/kobject.h \
  include/linux/sysfs.h \
  include/linux/kobject_ns.h \
  include/linux/kref.h \
  include/linux/klist.h \
  include/linux/pinctrl/devinfo.h \
    $(wildcard include/config/pm.h) \
  include/linux/pinctrl/consumer.h \
  include/linux/err.h \
  include/linux/seq_file.h \
    $(wildcard include/config/user/ns.h) \
  include/linux/nodemask.h \
    $(wildcard include/config/highmem.h) \
    $(wildcard include/config/movable/node.h) \
  include/linux/numa.h \
    $(wildcard include/config/nodes/shift.h) \
  include/linux/pinctrl/pinctrl-state.h \
  include/linux/pm.h \
    $(wildcard include/config/vt/console/sleep.h) \
    $(wildcard include/config/pm/runtime.h) \
    $(wildcard include/config/pm/clk.h) \
    $(wildcard include/config/pm/generic/domains.h) \
  include/linux/ratelimit.h \
  include/linux/uidgid.h \
    $(wildcard include/config/uidgid/strict/type/checks.h) \
  include/linux/highuid.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/device.h \
  include/linux/pm_wakeup.h \
  include/mtd/mtd-abi.h \
  include/linux/fs.h \
    $(wildcard include/config/fs/posix/acl.h) \
    $(wildcard include/config/security.h) \
    $(wildcard include/config/quota.h) \
    $(wildcard include/config/fsnotify.h) \
    $(wildcard include/config/ima.h) \
    $(wildcard include/config/epoll.h) \
    $(wildcard include/config/debug/writecount.h) \
    $(wildcard include/config/file/locking.h) \
    $(wildcard include/config/auditsyscall.h) \
    $(wildcard include/config/block.h) \
    $(wildcard include/config/fs/xip.h) \
    $(wildcard include/config/migration.h) \
  include/linux/kdev_t.h \
  include/uapi/linux/kdev_t.h \
  include/linux/dcache.h \
  include/linux/rculist.h \
  include/linux/rculist_bl.h \
  include/linux/list_bl.h \
  include/linux/bit_spinlock.h \
  include/linux/path.h \
  include/linux/stat.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/stat.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/stat.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/stat.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/compat.h \
  include/linux/sched.h \
    $(wildcard include/config/sched/debug.h) \
    $(wildcard include/config/no/hz/common.h) \
    $(wildcard include/config/lockup/detector.h) \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/core/dump/default/elf/headers.h) \
    $(wildcard include/config/sched/autogroup.h) \
    $(wildcard include/config/virt/cpu/accounting/native.h) \
    $(wildcard include/config/bsd/process/acct.h) \
    $(wildcard include/config/taskstats.h) \
    $(wildcard include/config/audit.h) \
    $(wildcard include/config/cgroups.h) \
    $(wildcard include/config/inotify/user.h) \
    $(wildcard include/config/fanotify.h) \
    $(wildcard include/config/posix/mqueue.h) \
    $(wildcard include/config/keys.h) \
    $(wildcard include/config/perf/events.h) \
    $(wildcard include/config/schedstats.h) \
    $(wildcard include/config/task/delay/acct.h) \
    $(wildcard include/config/hmp/pack/small/task.h) \
    $(wildcard include/config/mtk/sched/cmp/tgs.h) \
    $(wildcard include/config/mtk/sched/cmp/pack/small/task.h) \
    $(wildcard include/config/mt/load/balance/profiler.h) \
    $(wildcard include/config/sched/hmp/prio/filter.h) \
    $(wildcard include/config/sched/hmp.h) \
    $(wildcard include/config/sched/hmp/enhancement.h) \
    $(wildcard include/config/hmp/tracer.h) \
    $(wildcard include/config/mtprof/cputime.h) \
    $(wildcard include/config/fair/group/sched.h) \
    $(wildcard include/config/mt/rt/throttle/mon.h) \
    $(wildcard include/config/rt/group/sched.h) \
    $(wildcard include/config/mt/sched/trace.h) \
    $(wildcard include/config/mt/sched/debug.h) \
    $(wildcard include/config/cgroup/sched.h) \
    $(wildcard include/config/blk/dev/io/trace.h) \
    $(wildcard include/config/rcu/boost.h) \
    $(wildcard include/config/compat/brk.h) \
    $(wildcard include/config/cc/stackprotector.h) \
    $(wildcard include/config/virt/cpu/accounting/gen.h) \
    $(wildcard include/config/zram.h) \
    $(wildcard include/config/sysvipc.h) \
    $(wildcard include/config/detect/hung/task.h) \
    $(wildcard include/config/rt/mutexes.h) \
    $(wildcard include/config/task/xacct.h) \
    $(wildcard include/config/cpusets.h) \
    $(wildcard include/config/futex.h) \
    $(wildcard include/config/numa/balancing.h) \
    $(wildcard include/config/fault/injection.h) \
    $(wildcard include/config/latencytop.h) \
    $(wildcard include/config/function/graph/tracer.h) \
    $(wildcard include/config/memcg.h) \
    $(wildcard include/config/uprobes.h) \
    $(wildcard include/config/bcache.h) \
    $(wildcard include/config/have/unstable/sched/clock.h) \
    $(wildcard include/config/irq/time/accounting.h) \
    $(wildcard include/config/no/hz/full.h) \
    $(wildcard include/config/proc/fs.h) \
    $(wildcard include/config/stack/growsup.h) \
    $(wildcard include/config/mm/owner.h) \
    $(wildcard include/config/mtk/sched/rqavg/us.h) \
    $(wildcard include/config/mtk/sched/rqavg/ks.h) \
  include/uapi/linux/sched.h \
  include/linux/capability.h \
  include/uapi/linux/capability.h \
  include/linux/rbtree.h \
  include/linux/mm_types.h \
    $(wildcard include/config/split/ptlock/cpus.h) \
    $(wildcard include/config/have/cmpxchg/double.h) \
    $(wildcard include/config/have/aligned/struct/page.h) \
    $(wildcard include/config/want/page/debug/flags.h) \
    $(wildcard include/config/kmemcheck.h) \
    $(wildcard include/config/aio.h) \
    $(wildcard include/config/mmu/notifier.h) \
    $(wildcard include/config/transparent/hugepage.h) \
    $(wildcard include/config/compaction.h) \
  include/linux/auxvec.h \
  include/uapi/linux/auxvec.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/auxvec.h \
  include/linux/page-debug-flags.h \
    $(wildcard include/config/page/poisoning.h) \
    $(wildcard include/config/page/guard.h) \
    $(wildcard include/config/page/debug/something/else.h) \
  include/linux/uprobes.h \
    $(wildcard include/config/arch/supports/uprobes.h) \
  include/linux/page-flags-layout.h \
    $(wildcard include/config/sparsemem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
  include/generated/bounds.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/sparsemem.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/page.h \
    $(wildcard include/config/have/arch/pfn/valid.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/pgtable-3level-types.h \
  include/asm-generic/pgtable-nopud.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/memory.h \
  arch/arm64/include/generated/asm/sizes.h \
  include/asm-generic/sizes.h \
  include/linux/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/discontigmem.h) \
  include/asm-generic/getorder.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/mmu.h \
  arch/arm64/include/generated/asm/cputime.h \
  include/asm-generic/cputime.h \
    $(wildcard include/config/virt/cpu/accounting.h) \
  include/asm-generic/cputime_jiffies.h \
  include/linux/smp.h \
    $(wildcard include/config/use/generic/smp/helpers.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/smp.h \
  include/linux/sem.h \
  include/uapi/linux/sem.h \
  include/linux/ipc.h \
  include/uapi/linux/ipc.h \
  arch/arm64/include/generated/asm/ipcbuf.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/ipcbuf.h \
  arch/arm64/include/generated/asm/sembuf.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/sembuf.h \
  include/linux/signal.h \
    $(wildcard include/config/old/sigaction.h) \
  include/uapi/linux/signal.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/signal.h \
  include/asm-generic/signal.h \
  include/uapi/asm-generic/signal.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/signal-defs.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/sigcontext.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/sigcontext.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/siginfo.h \
  include/asm-generic/siginfo.h \
  include/uapi/asm-generic/siginfo.h \
  include/linux/pid.h \
  include/linux/percpu.h \
    $(wildcard include/config/need/per/cpu/embed/first/chunk.h) \
    $(wildcard include/config/need/per/cpu/page/first/chunk.h) \
    $(wildcard include/config/have/setup/per/cpu/area.h) \
  include/linux/pfn.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/percpu.h \
  include/asm-generic/percpu.h \
  include/linux/percpu-defs.h \
    $(wildcard include/config/debug/force/weak/per/cpu.h) \
  include/linux/topology.h \
    $(wildcard include/config/sched/smt.h) \
    $(wildcard include/config/sched/mc.h) \
    $(wildcard include/config/mt/load/balance/enhancement.h) \
    $(wildcard include/config/sched/book.h) \
    $(wildcard include/config/use/percpu/numa/node/id.h) \
    $(wildcard include/config/have/memoryless/nodes.h) \
  include/linux/mmzone.h \
    $(wildcard include/config/force/max/zoneorder.h) \
    $(wildcard include/config/mtkpasr.h) \
    $(wildcard include/config/memory/isolation.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/zone/dma32.h) \
    $(wildcard include/config/memory/hotplug.h) \
    $(wildcard include/config/have/memblock/node/map.h) \
    $(wildcard include/config/flat/node/mem/map.h) \
    $(wildcard include/config/no/bootmem.h) \
    $(wildcard include/config/have/memory/present.h) \
    $(wildcard include/config/need/node/memmap/size.h) \
    $(wildcard include/config/need/multiple/nodes.h) \
    $(wildcard include/config/have/arch/early/pfn/to/nid.h) \
    $(wildcard include/config/sparsemem/extreme.h) \
    $(wildcard include/config/nodes/span/other/nodes.h) \
    $(wildcard include/config/holes/in/zone.h) \
    $(wildcard include/config/arch/has/holes/memorymodel.h) \
  include/linux/pageblock-flags.h \
    $(wildcard include/config/hugetlb/page.h) \
    $(wildcard include/config/hugetlb/page/size/variable.h) \
  include/linux/memory_hotplug.h \
    $(wildcard include/config/have/arch/nodedata/extension.h) \
    $(wildcard include/config/have/bootmem/info/node.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/topology.h \
    $(wildcard include/config/arm/cpu/topology.h) \
    $(wildcard include/config/arch/scale/invariant/cpu/capacity.h) \
    $(wildcard include/config/disable/cpu/sched/domain/balance.h) \
    $(wildcard include/config/mtk/sched/cmp.h) \
  include/asm-generic/topology.h \
  include/linux/proportions.h \
  include/linux/percpu_counter.h \
  include/linux/seccomp.h \
    $(wildcard include/config/seccomp.h) \
    $(wildcard include/config/seccomp/filter.h) \
  include/uapi/linux/seccomp.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/seccomp.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/unistd.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/unistd.h \
  include/asm-generic/unistd.h \
  include/uapi/asm-generic/unistd.h \
  include/asm-generic/seccomp.h \
  include/linux/rtmutex.h \
    $(wildcard include/config/debug/rt/mutexes.h) \
  include/linux/plist.h \
    $(wildcard include/config/debug/pi/list.h) \
  include/linux/resource.h \
  include/uapi/linux/resource.h \
  arch/arm64/include/generated/asm/resource.h \
  include/asm-generic/resource.h \
  include/uapi/asm-generic/resource.h \
  include/linux/hrtimer.h \
    $(wildcard include/config/high/res/timers.h) \
    $(wildcard include/config/timerfd.h) \
  include/linux/timerqueue.h \
  include/linux/task_io_accounting.h \
    $(wildcard include/config/task/io/accounting.h) \
  include/linux/latencytop.h \
  include/linux/cred.h \
    $(wildcard include/config/debug/credentials.h) \
  include/linux/key.h \
    $(wildcard include/config/sysctl.h) \
  include/linux/sysctl.h \
  include/uapi/linux/sysctl.h \
  include/linux/selinux.h \
    $(wildcard include/config/security/selinux.h) \
  include/linux/llist.h \
    $(wildcard include/config/arch/have/nmi/safe/cmpxchg.h) \
  include/linux/gfp.h \
  include/linux/mmdebug.h \
    $(wildcard include/config/debug/vm.h) \
    $(wildcard include/config/debug/virtual.h) \
  include/linux/rtpm_prio.h \
    $(wildcard include/config/mt/rt/monitor.h) \
  include/linux/ptrace.h \
  include/linux/pid_namespace.h \
    $(wildcard include/config/pid/ns.h) \
  include/linux/mm.h \
    $(wildcard include/config/x86.h) \
    $(wildcard include/config/ppc.h) \
    $(wildcard include/config/parisc.h) \
    $(wildcard include/config/metag.h) \
    $(wildcard include/config/ia64.h) \
    $(wildcard include/config/ksm.h) \
    $(wildcard include/config/debug/vm/rb.h) \
    $(wildcard include/config/arch/uses/numa/prot/none.h) \
    $(wildcard include/config/debug/pagealloc.h) \
    $(wildcard include/config/hibernation.h) \
    $(wildcard include/config/hugetlbfs.h) \
  include/linux/debug_locks.h \
    $(wildcard include/config/debug/locking/api/selftests.h) \
  include/linux/range.h \
  include/linux/shrinker.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/pgtable.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/proc-fns.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/pgtable-hwdef.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/pgtable-3level-hwdef.h \
  include/asm-generic/pgtable.h \
  include/linux/page-flags.h \
    $(wildcard include/config/pageflags/extended.h) \
    $(wildcard include/config/arch/uses/pg/uncached.h) \
    $(wildcard include/config/memory/failure.h) \
    $(wildcard include/config/swap.h) \
  include/linux/huge_mm.h \
  include/linux/vmstat.h \
    $(wildcard include/config/vm/event/counters.h) \
  include/linux/vm_event_item.h \
  include/linux/nsproxy.h \
  include/uapi/linux/ptrace.h \
  include/uapi/linux/stat.h \
  include/linux/radix-tree.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/fiemap.h \
  include/linux/migrate_mode.h \
  include/linux/percpu-rwsem.h \
  include/linux/blk_types.h \
    $(wildcard include/config/blk/cgroup.h) \
    $(wildcard include/config/blk/dev/integrity.h) \
  include/uapi/linux/fs.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/limits.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/ioctl.h \
  arch/arm64/include/generated/asm/ioctl.h \
  include/asm-generic/ioctl.h \
  include/uapi/asm-generic/ioctl.h \
  include/linux/quota.h \
    $(wildcard include/config/quota/netlink/interface.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/dqblk_xfs.h \
  include/linux/dqblk_v1.h \
  include/linux/dqblk_v2.h \
  include/linux/dqblk_qtree.h \
  include/linux/projid.h \
  include/uapi/linux/quota.h \
  include/linux/nfs_fs_i.h \
  include/linux/fcntl.h \
  include/uapi/linux/fcntl.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/uapi/asm/fcntl.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/asm-generic/fcntl.h \
  include/linux/mtd/partitions.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/uaccess.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/compiler.h \
  include/linux/slab.h \
    $(wildcard include/config/slab/debug.h) \
    $(wildcard include/config/failslab.h) \
    $(wildcard include/config/slob.h) \
    $(wildcard include/config/slab.h) \
    $(wildcard include/config/slub.h) \
    $(wildcard include/config/debug/slab.h) \
  include/linux/slub_def.h \
    $(wildcard include/config/slub/stats.h) \
    $(wildcard include/config/memcg/kmem.h) \
    $(wildcard include/config/slub/debug.h) \
  include/linux/kmemleak.h \
    $(wildcard include/config/debug/kmemleak.h) \
  include/generated/uapi/linux/version.h \
  include/linux/module.h \
    $(wildcard include/config/module/sig.h) \
    $(wildcard include/config/kallsyms.h) \
    $(wildcard include/config/tracepoints.h) \
    $(wildcard include/config/event/tracing.h) \
    $(wildcard include/config/module/unload.h) \
    $(wildcard include/config/constructors.h) \
    $(wildcard include/config/debug/set/module/ronx.h) \
  include/linux/kmod.h \
  include/linux/elf.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/elf.h \
  arch/arm64/include/generated/asm/user.h \
  include/asm-generic/user.h \
  include/uapi/linux/elf.h \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/include/uapi/linux/elf-em.h \
  include/linux/moduleparam.h \
    $(wildcard include/config/alpha.h) \
    $(wildcard include/config/ppc64.h) \
  include/linux/tracepoint.h \
  include/linux/static_key.h \
  include/linux/jump_label.h \
    $(wildcard include/config/jump/label.h) \
  /home2/shenying2/OpenSource_Build_Try/K50a40_M_upgrade/kernel-build/1/kernel-3.10/arch/arm64/include/asm/module.h \
  include/asm-generic/module.h \
    $(wildcard include/config/have/mod/arch/specific.h) \
    $(wildcard include/config/modules/use/elf/rel.h) \
    $(wildcard include/config/modules/use/elf/rela.h) \
  include/linux/vmalloc.h \

drivers/misc/mediatek/masp/asfv2/core/sec_osal.o: $(deps_drivers/misc/mediatek/masp/asfv2/core/sec_osal.o)

$(deps_drivers/misc/mediatek/masp/asfv2/core/sec_osal.o):
