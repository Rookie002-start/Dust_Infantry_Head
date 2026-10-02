# HPM5361 崩溃现场抓取（配 boards/hpm/hpm5361icb/openocd-debug.cfg 使用）
#
# 用法（先起调试服务器，再跑本脚本）：
#   ./tools/hpm-openocd-server.sh &                     # 起 openocd（gdb 口 3333）
#   riscv64-zephyr-elf-gdb build/zephyr/zephyr.elf -x tools/hpm-fatal-capture.gdb
#
# 脚本做三件事：
#   1) 复位板子让它从干净状态开始跑（monitor reset init）
#   2) 在 Zephyr 的 fatal 入口下断点
#   3) 命中后打印：调用栈、reason、异常帧里的 mepc/mstatus、以及全部寄存器
#      （openocd-debug.cfg 里用 `riscv expose_csrs` 暴露了 mcause/mepc/mtval，
#        它们会出现在 info all-registers 的输出里，形如 csr0x342 之类的名字）

set confirm off
set pagination off
set width 0
set print pretty off

target remote 127.0.0.1:3333

# 干净起点：复位 + 使能时钟 + 停在复位向量
monitor reset init

printf "\n==== 开始运行，等待 fatal ====\n"
printf "复位瞬间 thread::booster::a = %u\n", 'thread::booster::a'

break z_fatal_error
break k_sys_fatal_error_handler
break arch_system_halt

# 命中任意一个 fatal 断点，就把现场打全再退出
commands 1
  silent
  printf "\n===== FATAL 命中（z_fatal_error）=====\n"
  bt 10
  printf "reason(a0)=%#x  esf(a1)=%#x\n", $a0, $a1
  printf "esf->mepc=%#x  esf->mstatus=%#x\n", ((struct arch_esf *)$a1)->mepc, ((struct arch_esf *)$a1)->mstatus
  printf "\n---- 全部寄存器（含 expose_csrs 暴露的 CSR，找 mcause/mepc/mtval）----\n"
  info all-registers
  printf "\n---- 当前线程/内核状态 ----\n"
  p 'thread::booster::a'
  quit
end

commands 2
  silent
  printf "\n===== FATAL 命中（k_sys_fatal_error_handler）=====\n"
  bt 10
  printf "reason(a0)=%#x  esf(a1)=%#x\n", $a0, $a1
  info all-registers
  quit
end

commands 3
  silent
  printf "\n===== 命中 arch_system_halt（fatal 死循环入口）=====\n"
  bt 10
  info all-registers
  quit
end

continue
