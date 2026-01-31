// ============================================================================
// 文件：i3c_params.v
// 功能：MIPI I3C相关参数和常量定义
// 描述：包含命名常量的参数定义，包括参数使用的位域定义
// ============================================================================

  //
  // 实例化参数及其可能的值/含义
  // -- 详见微架构规范
  //

  // 允许的参数如下：
  // - ENA_ID48B: 选择ID定义方式。注意BCR/DCR单独设置
  // - ID_48B, ID_BCR, ID_DCR: 根据需要的常量
  // - ID_AS_REGS: 控制哪些是软件控制的寄存器 vs 网络或常量
  // - ENA_SADDR: 选择是否使用I2C静态地址以及如何提供
  // - SADDR_P: 需要的常量
  // - ENA_CCC_HANDLING: 选择除内置ENTDAA、SETDASA、RSTDAA、SETNEWDA外由模块处理的CCC
  // - MAX_DS_WR, MAX_DS_RD, MX_DS_RDTURN: 常量
  // - ENA_IBI_MR_HJ: 启用中断、主设备权限/点对点和热连接的事件生成，以及总线可用匹配的时序规则
  // - SEL_BUS_IF: 与总线和内存映射寄存器的使用相关
  // - PIN_MODEL: 控制引脚处理方式
  // - FIFO_TYPE: 控制FIFO使用，内部或外部
  // - EXT_FIFO: 控制外部FIFO类型
  // - RSTACT: 与从设备复位相关
  //

    // ENA_ID48B参数值
  `define ID48B_CONST         3'd1      // 全常量
  `define ID48B_CONST_INST    3'd2      // 除实例外全常量
  `define ID48B_CONST_PARTNO  3'd3      // VID常量，Partno非常量
  `define ID48B_CONST_NONE    3'd4      // 全MMR/网络
    // ID_AS_REGS位定义
  `define IDREGS_INST_b       0         // 位[0]=IDINST是寄存器
  `define IDREGS_RAND_b       1         // 位[1]=IDRAND是寄存器且使用
  `define IDREGS_DCR_b        2         // 位[2]=IDDCR是寄存器且使用
  `define IDREGS_BCR_b        3         // 位[3]=IDBCR是寄存器且使用
  `define IDREGS_VID_b        4         // 位[4]=IDVID（供应商ID）是寄存器且使用
  `define IDREGS_DAWR_b       5         // 位[5]=DA可写（覆盖）
  `define IDREGS_SLVENA_b     6         // 位[6]=SLVENA默认为1
  `define IDREGS_HDRCMD_b     7         // 位[7]=允许HDRCMD寄存器
  `define IDREGS_VGPIO_b      8         // 位[8]=创建VGPIO寄存器控制VGPIO
  `define IDREGS_CCCMSK_b     9         // 位[9]=创建CCCMASK寄存器用于屏蔽CCC
  `define IDREGS_MSK_EW_b     10        // 位[10]=创建ERRWARNMASK寄存器用于屏蔽ERRWARN
    // ENA_SADDR参数值
  `define SADDR_NONE          2'd0      // 无静态地址
  `define SADDR_CONST         2'd1      // 静态地址为常量
  `define SADDR_NET           2'd2      // 静态地址为网络信号
  `define SADDR_CONFIG        2'd3      // 静态地址为配置字段
    // ENA_CCC_HANDLING位定义
  `define ENCCC_BASIC_b       0         // 位[0]=处理基本CCC（如AS、EN、状态等）
  `define ENCCC_MAXES_b       1         // 位[1]=处理最大CCC（Rd、Wr、DS）
  `define ENCCC_STATINF_b     2         // 位[2]=GETSTATUS信息作为寄存器字段
  `define ENCCC_STATVEND_b    3         // 位[3]=GETSTATUS供应商字段作为寄存器字段
  `define ENCCC_AASA          4         // 位[4]=SETAASA
  `define ENCCC_V11MIN        5         // 位[5]=最低v1.1支持（GETCAP等）
    // ENA_IBI_MR_HJ位定义
  `define EV_IBI_b            0         // 位[0]=IBI
  `define EV_IBI_DAT_b        1         // 位[1]=带数据的IBI
  `define EV_MR_b             2         // 位[2]=MR或点对点
  `define EV_HJ_b             3         // 位[3]=HJ
  `define EV_BAMATCH_b        4         // 位[4]=使用BAMATCH寄存器
  `define EV_EXTFIFO_b        5         // 位[5]=IBI扩展数据FIFO（独立）
    // ERROR_HANDLING位定义
  `define ERR_RDABT_b         0         // 位[0]=I3C SDR和DDR的读取中止
  `define ERR_WR_RO_b         1         // 位[1]=1表示写入只读寄存器时产生PSLVERR
    // 时间控制使能位
  `define TC_SYNC_b           0         // 位[0]=同步时间控制
  `define TC_MODE0_b          1         // 位[1]=异步模式0时间控制
  `define TC_MODE1_b          2         // 位[2]=异步模式1时间控制
  `define TC_SYNC_INT_b       4         // 位[4]=内部支持SYNC（相对于外部）
  `define TC_FREQ_REG         5         // 位[5]=使用寄存器作为GETXTIME频率/精度
    // 时间控制SYNC值（从命令映射）
  `define SYNC_NONE           3'd0      // 无最后命令
  `define SYNC_ST             3'd1      // ST命令
  `define SYNC_DT             3'd2      // 延迟时间 - 带字节
  `define SYNC_TPH            3'd3      // TPH第1个字节
  `define SYNC_TPH2           3'd4      // TPH第2个字节（如有）
  `define SYNC_TU             3'd5      // RR - 带字节
  `define SYNC_ODR            3'd6      // ODR - 带字节
    // HDR使能
  `define HDR_DDR_b           0         // 位[0]=支持DDR
  `define HDR_BT_b            2:1       // 位[2:1]=1为BT1，2为BT1,2，3为BT1,2,4
    // SEL_BUS_IF位定义
  `define SBIF_APB_b          0         // 位[0]=启用APB
  `define SBIF_REGS_b         1         // 位[1]=通用寄存器 vs 网络信号
  `define SBIF_IRQ_b          2         // 位[2]=中断寄存器
  `define SBIF_DMA_b          3         // 位[3]=DMA支持
  `define SBIF_HALF_b         4         // 位[4]=WDATAH和RDATAH
    // DMA_TYPE参数值（当SBIF_DMA_b选择时）
  `define DMA_ACK1BEF         2'd0      // 读取前ACK，等待读取
  `define DMA_TRIG            2'd1      // 请求为触发：未使用
  `define DMA_ACK1AFT         2'd2      // 同时/后ACK，基于当前状态进行
  `define DMA_4PHASE          2'd3      // 4相握手
    // PIN_MODEL参数值
  `define PINM_COMBO          0         // 基本组合逻辑使用
  `define PINM_REG            1         // 基本寄存器使用
  `define PINM_EXT_REG        2         // 扩展寄存器使用，寄存器靠近焊盘
    // FIFO_TYPE位定义
  `define FIFO_INT_b          0         // 位[0]=内部（因此[1]=0）
  `define FIFO_EXT_b          1         // 位[1]=外部（因此[0]=0）
  `define FIFO_TBHOLD_b       2         // 位[2]=使用到总线保持缓冲
  `define FIFO_FBHOLD_b       3         // 位[3]=使用从总线保持缓冲
    // EXT_FIFO参数值
  `define EXT_FIFO_NORMAL     2'd1      // 正常可用/空闲跟踪
  `define EXT_FIFO_REQ        2'd2      // 请求模型
    // RSTACT位定义
  `define RSTA_ENA_b          0         // 如果为1，启用RSTACT和从设备复位
  `define RSTA_MMR_b          1         // 如果为1，时间来自MMR vs 参数
  `define RSTA_CUS_b          2         // 如果为1，自定义RSTACT
  `define RSTA_TIM_PER_b      11:4      // 复位外设所需时间
  `define RSTA_TIM_SYS_b      19:12     // 复位系统所需时间
  `define RSTA_VAL_CUS_b      25:20     // 自定义值（与0x40或运算）
    // ENA_MAPPED位定义
    // 注意：映射用于I2C以及额外（虚拟）DA和SA
    //       一些I2C功能是单独参数，但仅在MAP启用时
  `define MAP_ENA_b           0         // 允许多个DA/SA
  `define MAP_I2C_SA10_b      1         // 10位地址
  `define MAP_I2C_SLVRST_b    2         // I2C从设备复位
  `define MAP_I2C_HS_b        3         // 高速 - 仅检测器
  `define MAP_VGPIO_b         4         // 特殊VGPIO机制
    // MAP_DA_AUTO位定义
    // 映射自动化使用硬件构建DA
  `define MAPDA_DASA_b        0         // 1为自动SETDASA
  `define MAPDA_AASA_b        1         // 1为自动SETAASA（在映射上）
  `define MAPDA_DAA_b         2         // 1为自动ENTDAA（见下文）
  `define MAPDA_DAA_MMR_b     6         // 1为DAA PID/DCR是MMR
  `define MAPDA_DAA_DCR_b     7         // 1为DCR交换
  `define MAPDA_DAA_PID_lb    8         // 5位用于PID[n:0]或0（如果无）
      // 如果不是MMR，计算MAP_DA_DAA每个元素的宽度
  `define MAP_DACONST_w(mda)  ((mda[MAPDA_DAA_DCR_b]?8:0)+\
                               (mda[MAPDA_PID_lb+:5]?(mda[MAPDA_PID_lb+:5]+1):0)
  
  //
  // CCC相关值
  //

   // CCC检测标志的参数
  `define CF_POSS         0             // 可能为CCC，因为7E/W已见
  `define CF_ONLY_b       4:0           // 仅测试POSS的范围
  `define CF_ONLY_POSS    5'd1          // 仅POSS的值
  `define CF_BCAST        1             // 广播命令中
  `define CF_DIRECT       2             // 直接命令中
  `define CF_DAA_M        3             // DAA模式中（7E/R将激活每个）
  `define CF_HDR_M        4             // ENTHDR模式中（退出模式将退出）
  `define CF_GRP_b        7:5           // 检测到的CCC索引
  `define CFG_NEWDA       3'd1          // SETNEWDA直接命令中
  `define CFG_RSTDAA_D    3'd2          // RSTDAA直接命令
  `define CFG_RSTDAA_US   3'd3          // RSTDAA匹配我们或是广播
  `define CFG_HDRDDR      3'd4          // DDR是我们唯一拆分的
          // 5到7用于DASA
  `define CFG_DASA        3'd5          // SETDASA直接命令中
  `define CFG_DASA_SA     3'd6          // 匹配我们的SA
  `define CFG_DASA_P2P    3'd7          // 匹配01（点对点）

  // CCC命令码
  `define CCC_ENEC        8'h00         // 广播启用事件
  `define CCC_ENEC_D      8'h80         // 直接启用事件
  `define CCC_DISEC       8'h01         // 广播禁用事件
  `define CCC_DISEC_D     8'h81         // 直接禁用事件
  `define CCC_ENTAS0      8'h02         // 广播进入正常活动状态
  `define CCC_ENTAS0_D    8'h82         // 直接进入正常活动状态
  `define CCC_ENTAS1      8'h03         // 广播进入低活动状态
  `define CCC_ENTAS1_D    8'h83         // 直接进入低活动状态
  `define CCC_ENTAS2      8'h04         // 广播进入更低活动状态
  `define CCC_ENTAS2_D    8'h84         // 直接进入更低活动状态
  `define CCC_ENTAS3      8'h05         // 广播进入最低活动状态
  `define CCC_ENTAS3_D    8'h85         // 直接进入最低活动状态
  `define CCC_RSTDAA      8'h06         // 广播复位DAA
  `define CCC_RSTDAA_D    8'h86         // 直接复位DAA
  `define CCC_ENTDAA      8'h07         // 广播进入DAA模式
  `define CCC_SETDASA_D   8'h87         // 直接从SA或特殊地址分配DA
  `define CCC_SETNEWDA_D  8'h88         // 直接设置新动态地址
  `define CCC_SETMXWL     8'h09         // 设置最大写入长度
  `define CCC_SETMXWL_D   8'h89         // 设置最大写入长度
  `define CCC_SETMXRL     8'h0A         // 设置最大读取长度
  `define CCC_SETMXRL_D   8'h8A         // 设置最大读取长度
  `define CCC_GETMXWL_D   8'h8B         // 获取最大写入长度
  `define CCC_GETMXRL_D   8'h8C         // 获取最大读取长度
  `define CCC_GETPID_D    8'h8D         // 获取临时ID
  `define CCC_GETBCR_D    8'h8E         // 获取BCR
  `define CCC_GETDCR_D    8'h8F         // 获取DCR
  `define CCC_GETSTATUS_D 8'h90         // 获取状态字节对
  `define CCC_GETMXDS_D   8'h94         // 获取最大数据速度
  `define CCC_GETHDRCAP_D 8'h95         // 获取HDR能力
  `define CCC_ENTHDR_MSK  (8'h20>>3)    // 任何ENTHDR的掩码（低3位表示哪个）
  `define CCC_SETXTIME    8'h28         // 广播设置时间控制
  `define CCC_SETXTIME_D  8'h98         // 直接设置时间控制
  `define CCC_GETXTIME_D  8'h99         // 获取时间控制信息
  // 1.0.1和1.1版本的CCC
  `define CCC_RSTACT      8'h2A         // 为所有设置从设备复位行为
  `define CCC_RSTACT_D    8'h9A         // 为从设备设置或获取从设备复位
  `define CCC_SETAASA     8'h29         // 仅广播SA->DA

  //
  // 中断状态（状态寄存器）
  //
  `define IS_START        8             // 检测到起始位
  `define IS_MATCHED      9             // 匹配DA或SA
  `define IS_STOP         10            // 检测到停止位
  `define IS_RXPEND       11            // RX（从总线）或RX FIFO触发或从总线DMA完成
  `define IS_TXPEND       12            // TX（到总线）或TX FIFO触发或到总线DMA完成
  `define IS_DACHG        13            // 动态地址添加/丢失
  `define IS_CCC          14            // CCC未处理（不同于REQ CCC）
  `define IS_ERRWARN      15            // 检测到一个或多个错误
  `define IS_DDRMATCH     16            // DDR命令匹配我们
  `define IS_CHANDLED     17            // CCC由模块处理
  `define IS_EVENT        18            // 事件已发送
  `define IS_SLVRST       19            // 从设备复位块

  //
  // 从设备复位动作请求值（采取什么动作）
  //
  `define RACT_DEF        3'd0          // 默认外设，除非升级
  `define RACT_FULL       3'd1          // 复位芯片
  `define RACT_NONE       3'd2          // 此次不执行任何操作
  `define RACT_CUST       3'd4          // 自定义（如果启用）

  //
  // DFT专用观察器 - 可选
  //
  `ifdef ENA_OBSERVER
   `define Observe(oreg, clk, mon) reg oreg /* cadence preserve sequence*/; \
        always @ (posedge clk) oreg <= mon; /* clk是scan_clk或单源时钟 */
  `else
   `define Observe(oreg, clk, mon) /* 未使用 */
  `endif