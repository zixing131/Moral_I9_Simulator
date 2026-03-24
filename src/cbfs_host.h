#pragma once

/*
 * 将固件 vmio.c 中 drive 2(/NAND/)、4(/CARD/) 的 ven_file_* 访问重定向到宿主机目录，
 * 以便在 NAND FTL 未完全镜像时仍能加载 .system/MB_MSTAR_WQVGA 等 CBFS 资源。
 * 地址来自 IDA：cbfs_vm_file_*（8533n_7835 / vmio.c）。
 */
int moral_cbfs_host_dispatch(unsigned int guest_pc);
