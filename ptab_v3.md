# ptab设计

首先我们需要确定的是，ptab的抽象层次应该是什么样的。
在idf的设计中，分区表的抽象其实很薄，每个字段甚至和对应的二进制值是对应的，并且idf不需要去描述不同平台上的硬件差异，可以假定所有的平台都共用同一套描述。
在过去的ptab设计中，实际上是承担了不少功能：
- 描述memory layout，对应真实的物理上的存放位置
- 作为链接脚本中的字段
- 暗示了非xip场景下资源存放位置和实际执行的地址
- 分割同一个镜像中的bin文件
- 实际生成对应的分区表二进制文件，给brom以及其他应用程序识别
我们需要考虑到，对于不同系列的芯片来说，有不一样的存储器分布，同一个系列的芯片，内置合封的资源以及存储器类型也可能不同
我们可以假设有另外一张表里面记录了不同芯片的存储器类型和地址分布。
ptab v3 只考虑 **YAML**，不支持 JSON。

同时对字段有约束：
- **name 字段**只能由小写字母 + `_` + 数字组成，且**数字不能作为开头**。
- 需要生成 `_START_ADDR/_SIZE/_OFFSET` 宏时，将 `name` 转为大写直接生成（不再使用 tags）。

```yaml
chip: SF32LB525

partitions:
  - name: ptab
    type: ptab
    subtype: primary
    region: mpi2
    offset: 0
    size: xxxkb
    
  - name: bootloader
    type: bootloader
    subtype: primary
    region: mpi2
    offset: 0xyyyyyyyy
    size: xxxkb
    exec_offset: 0xyyyyyyyy
    core: HCPU
    
  - name: main
    type: app
    subtype: factory
    region: mpi2
    offset: 0xyyyyyyyy
    size: xxxkb
    exec_region: mpi1
    exec_offset: 0xyyyyyyyy
    core: HCPU
    
  - name: font
    type: app
    subtype: int_res
    region: mpi2
    offset: 0xyyyyyyyy
    size: xxxkb
    
  - name: image
    type: app
    subtype: int_res
    region: mpi2
    offset: 0xyyyyyyyy
    size: xxxkb
    
  - name: fs
    type: data
    subtype: nvds
    region: mpi2
    offset: 0xyyyyyyyy
    size: xxxkb
    
  - name: accelerate
    type: code
    subtype: acc
    region: mpi1
    offset: 0xyyyyyyyy
    size: xxxkb
```

对于需要在 PSRAM 中加速的代码，分为两种情况讨论。一种是整个代码都需要做映射，这样我们可以加上 exec_region 和 exec_offset 属性。exec_region 可以不填，在内存分配表中默认指定。另一种则是运行中进行搬运，这种我们将单独设置一个 `type=code + subtype=acc` 的分区，工具根据该分区生成 `PSRAM_ACC_BEGIN/PSRAM_ACC_SIZE` 之类的变量（不再依赖 tags）。
对于需要进行对链接出来的main.bin进行分割的情况，我们的type还是定为app，但是subtype将使用int_res。这样最终我们可以进行分割。
