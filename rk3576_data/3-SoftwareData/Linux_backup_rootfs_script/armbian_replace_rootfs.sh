
#!/bin/bash

# 配置参数
SOURCE_IMG="${1:-ext4.img}"    # 参数1：源镜像路径（默认ext4.img）
TARGET_IMG="${2:-armbian.img}" # 参数2：目标镜像路径（默认armbian.img）
LOOP_DEV="/dev/loop777"          # 固定使用loop避免冲突
MOUNT_DIRS=("./origin" "./copy") # 统一挂载点
FINAL_SIZE=0

# 安全校验函数
validate_images() {
    [ -f "$SOURCE_IMG" ] || { echo "[ERROR] 源镜像不存在: $SOURCE_IMG"; exit 1; }
    [ -f "$TARGET_IMG" ] || { echo "[ERROR] 目标镜像不存在: $TARGET_IMG"; exit 1; }
    
    local src_size=$(du -b "$SOURCE_IMG" | awk '{print $1}')
    local dst_size=$(du -b "$TARGET_IMG" | awk '{print $1}')
    local required_size=$((src_size * 102 / 100)) # 增加10%余量
    
    # echo "src_size = $src_size "
    # echo "dst_size = $dst_size "

    if (( src_size > dst_size )); then
        echo "[INFO] 需要扩容: 源镜像${src_size}字节 > 目标镜像${dst_size}字节"
        echo "[CALC] 推荐扩容至: $((required_size/1024/1024))MB"
        FINAL_SIZE=$((required_size/1024/1024))
        return
    fi
    FINAL_SIZE=0
}

# 智能扩容函数
resize_image() {    
    echo "[RESIZE] 调整镜像大小至${FINAL_SIZE}MB..."
    truncate -s ${FINAL_SIZE}M "$TARGET_IMG"

    sudo losetup -P "$LOOP_DEV" "$TARGET_IMG"
    sudo sgdisk -e "$LOOP_DEV"
    sudo parted "$LOOP_DEV" --script \
            print free\
            resizepart 1 100% \
            print free

    sudo e2fsck -fp "$LOOP_DEV"p1
    sudo resize2fs "$LOOP_DEV"p1
}

# 主流程
main() {
    echo "==== 镜像替换工具 v2.1 ===="
    mkdir -p "${MOUNT_DIRS[@]}"
    
    validate_images

    echo " $FINAL_SIZE "
    if [ $FINAL_SIZE = 0 ]; then
        echo "[PASS] 无需扩容"
    else
        resize_image $required_size
    fi
 
    # 挂载并同步
    sudo mount "$LOOP_DEV"p1 "${MOUNT_DIRS[0]}"
    sudo mount "$SOURCE_IMG" "${MOUNT_DIRS[1]}"
    sudo rsync -aAXv --delete "${MOUNT_DIRS[1]}/" "${MOUNT_DIRS[0]}/"
    
    # 清理
    sync
    sudo umount "${MOUNT_DIRS[@]}"
    sudo losetup -d "$LOOP_DEV"
    sudo rm -rf "${MOUNT_DIRS[@]}"

    echo "[SUCCESS] 镜像替换完成! 输出文件: $TARGET_IMG"
}

main "$@"
