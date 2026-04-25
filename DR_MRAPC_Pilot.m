classdef DR_MRAPC_Pilot < handle
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明

    %% ================================================================
    %                          属  性  定  义
    %  ================================================================
    properties%(Access = private)

        % ────────────── 枚举与选择 ──────────────
        mode_names = { ...                          % 控制模式名称 - [cell][char]
                      'PID', ...
                      'MPC', ...
                      'MRAPC', ...
                      'DR-MPC', ...
                      'DR-MRAPC'}
        traj_names = { ...                          % 轨迹类型名称 - [cell][char]
                      'fixed_point', ...
                      'figure_eight', ...
                      'spiral', ...
                      'square'}
        mode                                        % 控制模式 - [value]
                                                    %   0: PID
                                                    %   1: MPC
                                                    %   2: MRAPC
                                                    %   3: DR-MPC
                                                    %   4: DR-MRAPC
        traj                                        % 轨迹选择 - [value]
                                                    %   0: 定点
                                                    %   1: 8字型
                                                    %   2: 螺旋
                                                    %   3: 方形

        % ────────────── Base 基本信息 ──────────────
        Base                                        % 基本信息 - [struct]
            % .Params       -   [struct]: 基本参数
            %   .Ts         -   [value]:  采样时间/控制周期(s)
            %   .T          -   [value]:  控制时长(s)
            %   .umax       -   [array]:  最大控制量 (3)
            %   .umin       -   [array]:  最小控制量 (3)
            % .System       -   [struct]: 系统信息
            %   .Ac         -   [matrix]: 连续标称系统矩阵 (6x6)
            %   .Bc         -   [matrix]: 连续标称输入矩阵 (6x3)
            %   .Ad         -   [matrix]: 离散标称系统矩阵 (6x6)
            %   .Bd         -   [matrix]: 离散标称输入矩阵 (6x3)

        % ────────────── PID 控制器 ──────────────
        PID                                         % PID控制器 - [struct]
            % .Params       -   [struct]: PID参数
            %   .Kp         -   [array]:  比例系数 (3)
            %   .Ki         -   [array]:  积分系数 (3)
            %   .Kd         -   [array]:  微分系数 (3)
            %   .r0         -   [value]:  跟踪微分器跟踪速率
            %   .h          -   [value]:  跟踪微分器滤波倍数
            % .Utils        -   [struct]: PID辅助工具
            %   .TD_x       -   [class]:  x方向跟踪微分器
            %   .TD_y       -   [class]:  y方向跟踪微分器
            %   .TD_z       -   [class]:  z方向跟踪微分器
            %   .SUM        -   [value]:  积分累计值

        % ────────────── MPC 控制器 ──────────────
        MPC                                         % MPC控制器 - [struct]
            % .Params       -   [struct]: MPC参数
            %   .Q          -   [matrix]: 状态权重矩阵 (6x6)
            %   .R          -   [matrix]: 控制权重矩阵 (3x3)
            %   .horizen    -   [value]:  预测时域/步数
            %   .xmax       -   [array]:  最大状态量 (6)
            %   .xmin       -   [array]:  最小状态量 (6)
            % .Utils        -   [struct]: MPC辅助工具
            %   .prob       -   [struct]: MPC优化问题

        % ────────────── MRA 模型参考自适应 ──────────────
        MRA                                         % 模型参考自适应机制 - [struct]
            % .Params       -   [struct]: MRA参数
            %   .Q_lyap     -   [matrix]: Lyapunov正定矩阵 (6x6)
            %   .Gamma      -   [matrix]: Kx自适应率矩阵 (6x6)
            %   .Lambda     -   [matrix]: Ka自适应率矩阵 (3x3)
            %   .poles      -   [array]:  配置极点位置 (6)
            %   .Ka_max     -   [value]:  Ka限幅
            %   .Kx_max     -   [value]:  Kx限幅
            %   .epsilon_Ka -   [value]:  Ka保护阈值
            %   .epsilon_Kx -   [value]:  Kx保护阈值
            % .Utils        -   [struct]: MRA工具
            %   .cnst_fun_Ka-   [handle]: Ka的约束函数
            %   .cnst_fun_Kx-   [handle]: Kx的约束函数
            %   .H          -   [matrix]: 反馈镇定矩阵 (3x6)
            %   .P          -   [matrix]: Lyapunov解 (6x6)
            %   .LAW        -   [class]: 自适应律

        % ────────────── DR 抗扰补偿 ──────────────
        DR                                          % 抗扰补偿机制 - [struct]
            % .Params       -   [struct]: DRC参数
            %   .c1         -   [struct]: 反步增益
            %     .x        -   [value]:  x方向增益
            %     .y        -   [value]:  y方向增益
            %     .z        -   [value]:  z方向增益
            %   .c2         -   [struct]: 反步增益
            %     .x        -   [value]:  x方向增益
            %     .y        -   [value]:  y方向增益
            %     .z        -   [value]:  z方向增益
            %   .L          -   [matrix]: 观测器增益 (6x6)
            % .Utils        -   [struct]: DR工具
            %   .DOB        -   [class]:  扰动观测器
            %   .DRCx       -   [class]:  x通道抗扰补偿
            %   .DRCy       -   [class]:  y通道抗扰补偿
            %   .DRCz       -   [class]:  z通道抗扰补偿

        % ────────────── Reference 参考给定 ──────────────
        Reference                                   % 参考给定 - [struct]
            % .TrajGen      -   [class]:  轨迹生成器
            % .Params       -   [struct]: 轨迹参数
            %   .xc         -   [value]:  x中心坐标(m)
            %   .yc         -   [value]:  y中心坐标(m)
            %   .T          -   [value]:  轨迹周期(s)
            %   .dir        -   [bool]:   轨迹朝向/运动方向
            %     8字  - 0:朝x方向  1:朝y方向
            %     其他 - 0:逆时针   1:顺时针
            %   .Fixed      -   [struct]: 定点轨迹的特有参数
            %     .zc       -   [value]:  定点高度(m)
            %   .Eight      -   [struct]: 8字型的特有参数
            %     .zc       -   [value]:  形状高度(m)
            %     .rx       -   [value]:  方向x上的半径(m)
            %     .ry       -   [value]:  方向y上的半径(m)
            %   .Spiral     -   [struct]: 螺旋型的特有参数
            %     .z0       -   [value]:  起点高度(m)
            %     .r        -   [value]:  螺旋半径(m)
            %     .v        -   [value]:  上升速度(m/s)
            %   .Square     -   [struct]: 方形的特有参数
            %     .zc       -   [value]:  形状高度(m)
            %     .L        -   [value]:  方形边长(m)
            % .xi_r         -   [array]:  参考状态量 (6x(N+horizen))
            % .u_r          -   [array]:  参考控制量 (3x(N+horizen))
            
    end
    %% 公共属性
    properties(Access = public)
    end
    
    %% ================================================================
    %                          公  共  方  法
    %  ================================================================
    methods

        %% ═══════════════ 1. 初始化 ═══════════════
        function obj = DR_MRAPC_Pilot(Ts,T)
            %UNTITLED 构造此类的实例
            %   此处显示详细说明
            fprintf("[Info] [DR-MRAPC_Pilot]\t Customized Pilot initializing...\n")
            addpath(fullfile(fileparts(mfilename('fullpath')), 'LinearMPC_Simplified_Core'))
            addpath(fullfile(fileparts(mfilename('fullpath')), 'Utils'))

            %% 1.1.参数初始化
            % 基本信息初始化
            obj.mode = 0;
            obj.traj = 1;
            obj.Base.Params.Ts = Ts;
            obj.Base.Params.T = T;
            obj.Base.Params.umax = [5.0;5.0;5.0];
            obj.Base.Params.umin = -[5.0;5.0;5.0];
            I = [0 1;0 0];  E = [0;1];      % 双积分形式
            Ac = blkdiag(I,I,I);    Bc = blkdiag(E,E,E);
            obj.Base.System.Ac = Ac;
            obj.Base.System.Bc = Bc;
            % PID控制器参数初始化   -   [x方向;y方向;z方向]
            obj.PID.Params.Kp = [0.5;0.5;0.5];
            obj.PID.Params.Ki = [0.0;0.0;0.0];
            obj.PID.Params.Kd = [1.0;1.0;1.0];
            obj.PID.Params.r0 = 50;
            obj.PID.Params.h = 5;
            % MPC控制器参数初始化
            obj.MPC.Params.Q = diag([10,1,10,1,10,1]);
            obj.MPC.Params.R = diag([1,1,1]);
            obj.MPC.Params.horizen = 20;
            obj.MPC.Params.xmax = [3.0;2.0;1.5;2.0;inf;2.0];
            obj.MPC.Params.xmin = -[3.0;2.0;1.5;2.0;0.1;2.0];
            % MRA模型参考自适应参数初始化
            obj.MRA.Params.Q_lyap = 1*eye(6);
            obj.MRA.Params.Gamma = 1*diag([1,1,1,1,1,1]);
            obj.MRA.Params.Lambda = 1*diag([1,1,1]);
            obj.MRA.Params.poles = [-2;-3;-2;-3;-2;-3];
            obj.MRA.Params.Ka_max = 100;
            obj.MRA.Params.Kx_max = 100;
            obj.MRA.Params.epsilon_Ka = 0.5;
            obj.MRA.Params.epsilon_Kx = 0.5;
            % DR抗扰补偿参数初始化
            obj.DR.Params.c1.x = 1;     obj.DR.Params.c2.x = 1;
            obj.DR.Params.c1.y = 1;     obj.DR.Params.c2.y = 1;
            obj.DR.Params.c1.z = 1;     obj.DR.Params.c2.z = 1;
            obj.DR.Params.L = -diag([10,10,10,10,10,10]);
            %% 1.2.工具初始化
            obj.build_PID;
            obj.build_MPC_prob;
            obj.build_MRA_H;
            obj.build_DR_DRC;
            obj.build_DR_DOB;
            %% 1.3.轨迹初始化
            obj.Reference.TrajGen = TrajGen(0:Ts:T+obj.MPC.Params.horizen*Ts);   % 多给定horizon*Ts
            % ---------- 默认参数 ----------
            obj.Reference.Params.xc = 0;
            obj.Reference.Params.yc = 0;
            obj.Reference.Params.T = 10;
            obj.Reference.Params.dir = 0;
                % ----- 定点参数 -----
                obj.Reference.Params.Fixed.zc = 1.5;
                % ----- 8字形参数 -----
                obj.Reference.Params.Eight.zc = 1.5;
                obj.Reference.Params.Eight.rx = 2.0;
                obj.Reference.Params.Eight.ry = 1.5;
                % ----- 螺旋形参数 -----
                obj.Reference.Params.Spiral.z0 = 0.5;
                obj.Reference.Params.Spiral.r = 1.2;
                obj.Reference.Params.Spiral.v = 0.05;
                % ----- 方形参数 -----
                obj.Reference.Params.Square.zc = 1.5;
                obj.Reference.Params.Square.L = 2.5;
            % ------------------------------
            obj.build_traj;  % 生成轨迹

            %% 初始化完成
            fprintf("[Info] [DR-MRAPC_Pilot]\t Customized Pilot [%s] for " + ...
                    "[%s] trajectory has been created!\n", ...
                    obj.mode_names{obj.mode+1}, obj.traj_names{obj.traj+1});

        end
        
        %% ═══════════════ 2. 设置参数 ═══════════════
        % ────────────── 2.1 基本 ──────────────
        function set_mode(obj,mode)
            %SET_MODE 设置控制模式
            %   mode  -  0:PID  1:MPC  2:MRAPC  3:DR-MPC  4:DR-MRAPC
            obj.mode = mode;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Control mode set to [%s]\n", obj.mode_names{mode+1});
        end
        function set_traj(obj,traj)
            %SET_TRAJ 设置轨迹类型
            %   traj  -  0:定点  1:8字型  2:螺旋  3:方形
            obj.traj = traj;
            obj.build_traj;
        end
        function set_bound_control(obj,umax,umin)
            %SET_BOUND_CONTROL 设置控制量的范围
            obj.Base.Params.umax = umax;
            obj.Base.Params.umin = umin;
            obj.build_MPC_prob;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Control bounds updated, MPC problem rebuilt\n");
        end

        % ────────────── 2.2 PID ──────────────
        function set_PID_K(obj,Kp,Ki,Kd)
            %SET_PID_K 设置PID控制器系数
            obj.PID.Params.Kp = Kp;
            obj.PID.Params.Ki = Ki;
            obj.PID.Params.Kd = Kd;
            fprintf("[Info] [DR-MRAPC_Pilot]\t PID gains updated: Kp=[%.2f,%.2f,%.2f], Ki=[%.2f,%.2f,%.2f], Kd=[%.2f,%.2f,%.2f]\n", ...
                    Kp(1),Kp(2),Kp(3), Ki(1),Ki(2),Ki(3), Kd(1),Kd(2),Kd(3));
        end
        function set_PID_TD(obj,r0,h)
            %SET_PID_TD 设置PID跟踪微分器的参数
            %   r0  -   跟踪速率
            %   h   -   滤波倍数
            obj.PID.Params.r0 = r0;
            obj.PID.Params.h = h;
            obj.build_PID;
            fprintf("[Info] [DR-MRAPC_Pilot]\t PID TD params updated: r0=%.1f, h=%.1f\n", r0, h);
        end

        % ────────────── 2.3 MPC ──────────────
        function set_MPC(obj,Q,R,horizen)
            %SET_MPC 设置MPC控制器参数
            obj.MPC.Params.Q = Q;
            obj.MPC.Params.R = R;
            obj.MPC.Params.horizen = horizen;
            obj.build_MPC_prob;
            fprintf("[Info] [DR-MRAPC_Pilot]\t MPC params updated (horizen=%d), problem rebuilt\n", horizen);
        end
        function set_bound_state(obj,xmax,xmin)
            %SET_BOUND_STATE 设置状态量的范围
            obj.MPC.Params.xmax = xmax;
            obj.MPC.Params.xmin = xmin;
            obj.build_MPC_prob;
            fprintf("[Info] [DR-MRAPC_Pilot]\t State bounds updated, MPC problem rebuilt\n");
        end

        % ────────────── 2.4 MRA ──────────────
        function set_MRA_adaptive(obj,Q_lyap,Gamma,Lambda)
            %SET_MRA_ADAPTIVE 设置MRA自适应率参数
            obj.MRA.Params.Q_lyap = Q_lyap;
            obj.MRA.Params.Gamma = Gamma;
            obj.MRA.Params.Lambda = Lambda;
            obj.build_MRA_LAW;
            fprintf("[Info] [DR-MRAPC_Pilot]\t MRA adaptive params updated, LAW rebuilt\n");
        end
        function set_MRA_poles(obj,poles)
            %SET_MRA_POLES 设置MRA极点配置
            obj.MRA.Params.poles = poles;
            obj.build_MRA_H;
            fprintf("[Info] [DR-MRAPC_Pilot]\t MRA poles updated, H & LAW rebuilt\n");
        end
        function set_MRA_bound(obj,Ka_max,Kx_max,epsilon_Ka,epsilon_Kx)
            %SET_MRA_BOUND 设置MRA限幅与保护阈值
            obj.MRA.Params.Ka_max = Ka_max;
            obj.MRA.Params.Kx_max = Kx_max;
            obj.MRA.Params.epsilon_Ka = epsilon_Ka;
            obj.MRA.Params.epsilon_Kx = epsilon_Kx;
            obj.build_MRA_LAW;
            fprintf("[Info] [DR-MRAPC_Pilot]\t MRA bounds updated (Ka_max=%.1f, Kx_max=%.1f), LAW rebuilt\n", Ka_max, Kx_max);
        end
        % ────────────── 2.5 DR ──────────────
        function set_DR_backstep(obj,c1_x,c2_x,c1_y,c2_y,c1_z,c2_z)
            %SET_DR_BACKSTEP 设置DR反步增益
            obj.DR.Params.c1.x = c1_x;   obj.DR.Params.c2.x = c2_x;
            obj.DR.Params.c1.y = c1_y;   obj.DR.Params.c2.y = c2_y;
            obj.DR.Params.c1.z = c1_z;   obj.DR.Params.c2.z = c2_z;
            obj.build_DR_DRC;
            fprintf("[Info] [DR-MRAPC_Pilot]\t DR backstep gains updated, DRC rebuilt\n");
        end
        function set_DR_observer(obj,L)
            %SET_DR_OBSERVER 设置DR扰动观测器增益
            obj.DR.Params.L = L;
            obj.build_DR_DOB;
            fprintf("[Info] [DR-MRAPC_Pilot]\t DR observer gain updated, DOB rebuilt\n");
        end
        % ────────────── 2.6 轨迹 ──────────────
        function set_traj_period(obj,T)
            %SET_TRAJ_PERIOD 设置轨迹周期
            obj.Reference.Params.T = T;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Trajectory period updated: T=%.1fs\n", T);
        end
        function set_traj_direction(obj,dir)
            %SET_TRAJ_DIRECTION 设置轨迹朝向/运动方向
            obj.Reference.Params.dir = dir;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Trajectory direction updated: dir=%d\n", dir);
        end
        function set_traj_fixed(obj,xc,yc,zc)
            %SET_TRAJ_FIXED 设置定点轨迹
            obj.Reference.Params.xc = xc;
            obj.Reference.Params.yc = yc;
            obj.Reference.Params.Fixed.zc = zc;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Fixed-point trajectory updated: (%.2f, %.2f, %.2f)\n", xc, yc, zc);
        end
        function set_traj_eight(obj,xc,yc,zc,rx,ry)
            %SET_TRAJ_EIGHT 设置8字形轨迹
            obj.Reference.Params.xc = xc;
            obj.Reference.Params.yc = yc;
            obj.Reference.Params.Eight.zc = zc;
            obj.Reference.Params.Eight.rx = rx;
            obj.Reference.Params.Eight.ry = ry;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Figure-8 trajectory updated: center=(%.2f,%.2f), zc=%.2f, rx=%.2f, ry=%.2f\n", xc, yc, zc, rx, ry);
        end
        function set_traj_spiral(obj,xc,yc,z0,v,r)
            %SET_TRAJ_SPIRAL 设置螺旋轨迹
            obj.Reference.Params.xc = xc;
            obj.Reference.Params.yc = yc;
            obj.Reference.Params.Spiral.z0 = z0;
            obj.Reference.Params.Spiral.v = v;
            obj.Reference.Params.Spiral.r = r;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Spiral trajectory updated: center=(%.2f,%.2f), z0=%.2f, v=%.3f, r=%.2f\n", xc, yc, z0, v, r);
        end
        function set_traj_square(obj,xc,yc,zc,L)
            %SET_TRAJ_SQUARE 设置方形轨迹
            obj.Reference.Params.xc = xc;
            obj.Reference.Params.yc = yc;
            obj.Reference.Params.Square.L = L;
            obj.Reference.Params.Square.zc = zc;
            obj.build_traj;
            fprintf("[Info] [DR-MRAPC_Pilot]\t Square trajectory updated: center=(%.2f,%.2f), zc=%.2f, L=%.2f\n", xc, yc, zc, L);
        end

        %% ═══════════════ 3. 参数总览 ═══════════════
        function display_params(obj)
            %DISPLAY_PARAMS 打印所有参数的总览
            sep  = '═══════════════════════════════════════════════════════════';
            sep2 = '───────────────────────────────────────────────────────────';
            fprintf('\n%s\n', sep);
            fprintf('              DR-MRAPC Pilot 参数总览\n');
            fprintf('%s\n\n', sep);

            % ── 基本信息 ──
            fprintf('  ■ Base 基本信息\n');
            fprintf('    %-16s = %s\n', 'mode',  obj.mode_names{obj.mode+1});
            fprintf('    %-16s = %s\n', 'traj',  obj.traj_names{obj.traj+1});
            fprintf('    %-16s = %.4f\n', 'Ts',   obj.Base.Params.Ts);
            fprintf('    %-16s = %.2f\n', 'T',    obj.Base.Params.T);
            fprintf('    %-16s = [%.1f, %.1f, %.1f]\n', 'umax', obj.Base.Params.umax(1), obj.Base.Params.umax(2), obj.Base.Params.umax(3));
            fprintf('    %-16s = [%.1f, %.1f, %.1f]\n', 'umin', obj.Base.Params.umin(1), obj.Base.Params.umin(2), obj.Base.Params.umin(3));
            fprintf('%s\n', sep2);

            % ── PID ──
            fprintf('  ■ PID 控制器\n');
            fprintf('    %-16s = [%.2f, %.2f, %.2f]\n', 'Kp', obj.PID.Params.Kp(1), obj.PID.Params.Kp(2), obj.PID.Params.Kp(3));
            fprintf('    %-16s = [%.2f, %.2f, %.2f]\n', 'Ki', obj.PID.Params.Ki(1), obj.PID.Params.Ki(2), obj.PID.Params.Ki(3));
            fprintf('    %-16s = [%.2f, %.2f, %.2f]\n', 'Kd', obj.PID.Params.Kd(1), obj.PID.Params.Kd(2), obj.PID.Params.Kd(3));
            fprintf('    %-16s = %.1f\n', 'TD.r0', obj.PID.Params.r0);
            fprintf('    %-16s = %.1f\n', 'TD.h',  obj.PID.Params.h);
            fprintf('%s\n', sep2);

            % ── MPC ──
            fprintf('  ■ MPC 控制器\n');
            fprintf('    %-16s = %d\n', 'horizen', obj.MPC.Params.horizen);
            fprintf('    %-16s = diag([%s])\n', 'Q', num2str(diag(obj.MPC.Params.Q)'));
            fprintf('    %-16s = diag([%s])\n', 'R', num2str(diag(obj.MPC.Params.R)'));
            fprintf('    %-16s = [%s]\n', 'xmax', num2str(obj.MPC.Params.xmax'));
            fprintf('    %-16s = [%s]\n', 'xmin', num2str(obj.MPC.Params.xmin'));
            fprintf('%s\n', sep2);

            % ── MRA ──
            fprintf('  ■ MRA 模型参考自适应\n');
            fprintf('    %-16s = [%s]\n', 'poles', num2str(obj.MRA.Params.poles'));
            fprintf('    %-16s = %.1f\n', 'Ka_max', obj.MRA.Params.Ka_max);
            fprintf('    %-16s = %.1f\n', 'Kx_max', obj.MRA.Params.Kx_max);
            fprintf('    %-16s = %.2f\n', 'epsilon_Ka', obj.MRA.Params.epsilon_Ka);
            fprintf('    %-16s = %.2f\n', 'epsilon_Kx', obj.MRA.Params.epsilon_Kx);
            fprintf('    %-16s = diag([%s])\n', 'Q_lyap', num2str(diag(obj.MRA.Params.Q_lyap)'));
            fprintf('    %-16s = diag([%s])\n', 'Gamma',  num2str(diag(obj.MRA.Params.Gamma)'));
            fprintf('    %-16s = diag([%s])\n', 'Lambda', num2str(diag(obj.MRA.Params.Lambda)'));
            fprintf('%s\n', sep2);

            % ── DR ──
            fprintf('  ■ DR 抗扰补偿\n');
            fprintf('    %-16s = {x:%.1f, y:%.1f, z:%.1f}\n', 'c1', obj.DR.Params.c1.x, obj.DR.Params.c1.y, obj.DR.Params.c1.z);
            fprintf('    %-16s = {x:%.1f, y:%.1f, z:%.1f}\n', 'c2', obj.DR.Params.c2.x, obj.DR.Params.c2.y, obj.DR.Params.c2.z);
            fprintf('    %-16s = diag([%s])\n', 'L', num2str(diag(obj.DR.Params.L)'));
            fprintf('%s\n', sep2);

            % ── 轨迹 ──
            fprintf('  ■ Reference 轨迹参数 [%s]\n', obj.traj_names{obj.traj+1});
            fprintf('    %-16s = %.2f\n', 'xc', obj.Reference.Params.xc);
            fprintf('    %-16s = %.2f\n', 'yc', obj.Reference.Params.yc);
            fprintf('    %-16s = %.2f\n', 'T',  obj.Reference.Params.T);
            fprintf('    %-16s = %d\n',   'dir',obj.Reference.Params.dir);
            switch obj.traj
                case 0
                    fprintf('    %-16s = %.2f\n', 'Fixed.zc', obj.Reference.Params.Fixed.zc);
                case 1
                    fprintf('    %-16s = %.2f\n', 'Eight.zc', obj.Reference.Params.Eight.zc);
                    fprintf('    %-16s = %.2f\n', 'Eight.rx', obj.Reference.Params.Eight.rx);
                    fprintf('    %-16s = %.2f\n', 'Eight.ry', obj.Reference.Params.Eight.ry);
                case 2
                    fprintf('    %-16s = %.2f\n', 'Spiral.z0', obj.Reference.Params.Spiral.z0);
                    fprintf('    %-16s = %.2f\n', 'Spiral.r',  obj.Reference.Params.Spiral.r);
                    fprintf('    %-16s = %.3f\n', 'Spiral.v',  obj.Reference.Params.Spiral.v);
                case 3
                    fprintf('    %-16s = %.2f\n', 'Square.zc', obj.Reference.Params.Square.zc);
                    fprintf('    %-16s = %.2f\n', 'Square.L',  obj.Reference.Params.Square.L);
            end
            fprintf('%s\n\n', sep);
        end

        %% ═══════════════ 4. 轨迹预览 ═══════════════
        function preview_traj(obj)
            %PREVIEW_TRAJ 预览所有三维轨迹（定点除外）
            xc  = obj.Reference.Params.xc;
            yc  = obj.Reference.Params.yc;
            T   = obj.Reference.Params.T;
            dir = obj.Reference.Params.dir;

            % 生成三条轨迹数据
            % 8字形
            zc8 = obj.Reference.Params.Eight.zc;
            rx  = obj.Reference.Params.Eight.rx;
            ry  = obj.Reference.Params.Eight.ry;
            [x8,y8,z8,~,~,~,~,~,~] = obj.Reference.TrajGen.EightTraj(xc,yc,zc8,rx,ry,T,dir);
            % 螺旋形
            z0  = obj.Reference.Params.Spiral.z0;
            r   = obj.Reference.Params.Spiral.r;
            v   = obj.Reference.Params.Spiral.v;
            [xsp,ysp,zsp,~,~,~,~,~,~] = obj.Reference.TrajGen.SpiralTraj(xc,yc,z0,v,T,r,dir);
            % 方形
            zcS = obj.Reference.Params.Square.zc;
            L   = obj.Reference.Params.Square.L;
            [xsq,ysq,zsq,~,~,~,~,~,~] = obj.Reference.TrajGen.SquareTraj(xc,yc,zcS,L,T,dir);

            figure('Name','Trajectory Preview','NumberTitle','off');
            % ── 8字形 ──
            subplot(1,3,1);
            plot3(x8,y8,z8,'-b','LineWidth',1.5); hold on;
            plot3(x8(1),y8(1),z8(1),'ob','MarkerFaceColor','b');
            xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
            title('Figure-8'); grid on; axis equal; view(3);
            % ── 螺旋形 ──
            subplot(1,3,2);
            plot3(xsp,ysp,zsp,'-r','LineWidth',1.5); hold on;
            plot3(xsp(1),ysp(1),zsp(1),'or','MarkerFaceColor','r');
            xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
            title('Spiral'); grid on; axis equal; view(3);
            % ── 方形 ──
            subplot(1,3,3);
            plot3(xsq,ysq,zsq,'-g','LineWidth',1.5); hold on;
            plot3(xsq(1),ysq(1),zsq(1),'og','MarkerFaceColor','g');
            xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
            title('Square'); grid on; axis equal; view(3);

            fprintf("[Info] [DR-MRAPC_Pilot]\t Trajectory preview displayed\n");
        end
    
        %% ═══════════════ 5. 控制接口 ═══════════════
        function u = control(obj,xi_mea,k)
            %CONTROL 实时控制接口
            % xi_mea    -   实时反馈信息
            % k         -   当前时间序号
            xi_e = obj.Reference.xi_r(:,k) - xi_mea;    % 转换为误差调节问题
        if obj.mode == 0
        % --------------- PID ---------------
            obj.PID.Utils.TD_x.update(xi_e(1));
            obj.PID.Utils.TD_y.update(xi_e(3));
            obj.PID.Utils.TD_z.update(xi_e(5));
            u = obj.PID.Params.Kp.*xi_e([1,3,5]) + ...
                    obj.PID.Params.Ki.*obj.PID.Utils.SUM * obj.Base.Params.Ts + ...
                    obj.PID.Params.Kd.*[obj.PID.Utils.TD_x.output;obj.PID.Utils.TD_y.output;obj.PID.Utils.TD_z.output];
            u = clip(u,obj.Base.Params.umin,obj.Base.Params.umax);
            obj.PID.Utils.SUM = obj.PID.Utils.SUM + xi_e([1,3,5]);
        else
        end
        end
    end
    %% ================================================================
    %                          私  有  方  法
    %  ================================================================
    methods(Access = private)
        function build_PID(obj)
            %BUILD_PID 重建PID
            Ts = obj.Base.Params.Ts;
            obj.PID.Utils.TD_x = TrckDiff(Ts,obj.PID.Params.r0,obj.PID.Params.h*Ts);
            obj.PID.Utils.TD_y = TrckDiff(Ts,obj.PID.Params.r0,obj.PID.Params.h*Ts);
            obj.PID.Utils.TD_z = TrckDiff(Ts,obj.PID.Params.r0,obj.PID.Params.h*Ts);
            obj.PID.Utils.SUM = [0;0;0];
        end
        function build_MPC_prob(obj)
            %BUILD_MPC_PROB 重建MPC优化问题
            obj.MPC.Utils.prob = MPCSim_init(obj.Base.System.Ac, obj.Base.System.Bc, ...
                        obj.MPC.Params.Q, obj.MPC.Params.R, obj.MPC.Params.horizen, ...
                        obj.Base.Params.Ts, obj.MPC.Params.xmax, obj.MPC.Params.xmin, ...
                        obj.Base.Params.umax, obj.Base.Params.umin);
            obj.Base.System.Ad = obj.MPC.Utils.prob.Ad;
            obj.Base.System.Bd = obj.MPC.Utils.prob.Bd;
        end
        function build_DR_DRC(obj)
            %BUILD_DR_DRC 重建DR抗扰补偿器
            Ts = obj.Base.Params.Ts;
            obj.DR.Utils.DRCx = DRC_2nd(Ts,obj.DR.Params.c1.x,obj.DR.Params.c2.x,1);
            obj.DR.Utils.DRCy = DRC_2nd(Ts,obj.DR.Params.c1.y,obj.DR.Params.c2.y,1);
            obj.DR.Utils.DRCz = DRC_2nd(Ts,obj.DR.Params.c1.z,obj.DR.Params.c2.z,1);
        end
        function build_DR_DOB(obj)
            %BUILD_DR_DOB 重建DR扰动观测器
            Ts = obj.Base.Params.Ts;
            obj.DR.Utils.DOB = LinearDOB(Ts,obj.Base.System.Ac,obj.Base.System.Bc,obj.DR.Params.L,[],[]);
        end
        function build_MRA_H(obj)
            %BUILD_MRA_H 重建MRA反馈镇定矩阵与Lyapunov解
            obj.MRA.Utils.H = place(obj.Base.System.Ac,obj.Base.System.Bc,obj.MRA.Params.poles);
            obj.MRA.Utils.P = lyap((obj.Base.System.Ac-obj.Base.System.Bc*obj.MRA.Utils.H)', obj.MRA.Params.Q_lyap);
            obj.build_MRA_LAW;
        end
        function build_MRA_LAW(obj)
            %BUILD_MRA_LAW 重建MRA约束函数与自适应律
            obj.MRA.Utils.cnst_fun_Ka = @(Ka) (Ka(1)^2+Ka(2)^2+Ka(3)^2+Ka(4)^2+Ka(5)^2+Ka(6)^2+Ka(7)^2+Ka(8)^2+Ka(9)^2 ...
                                               - obj.MRA.Params.Ka_max^2)/(2*obj.MRA.Params.Ka_max*obj.MRA.Params.epsilon_Ka + obj.MRA.Params.epsilon_Ka^2);
            obj.MRA.Utils.cnst_fun_Kx = @(Kx) (Kx(1)^2+Kx(2)^2+Kx(3)^2+Kx(4)^2+Kx(5)^2+Kx(6)^2+Kx(7)^2+Kx(8)^2+Kx(9)^2 + ...
                                               Kx(10)^2+Kx(11)^2+Kx(12)^2+Kx(13)^2+Kx(14)^2+Kx(15)^2+Kx(16)^2+Kx(17)^2+Kx(18)^2 ...
                                               - obj.MRA.Params.Kx_max^2)/(2*obj.MRA.Params.Kx_max*obj.MRA.Params.epsilon_Kx + obj.MRA.Params.epsilon_Kx^2);
            obj.MRA.Utils.LAW = MRAC_Law(obj.Base.Params.Ts,obj.MRA.Params.Gamma,obj.MRA.Params.Lambda,...
                                          obj.Base.System.Bc,obj.MRA.Utils.P,obj.MRA.Utils.cnst_fun_Ka,obj.MRA.Utils.cnst_fun_Kx);
        end
        function build_traj(obj)
            %BUILD_TRAJ 生成参考轨迹
            xc = obj.Reference.Params.xc;
            yc = obj.Reference.Params.yc;
            T = obj.Reference.Params.T;
            dir = obj.Reference.Params.dir;
            switch obj.traj
                case 0  % 定点
                    zc = obj.Reference.Params.Fixed.zc;
                    [x,y,z,vx,vy,vz,ax,ay,az] = obj.Reference.TrajGen.FixedPoint(xc,yc,zc);
                    fprintf("[Info] [DR-MRAPC_Pilot]\t [Fixed Point] trajectory is selected...\n");
                case 1  % 8字形
                    zc = obj.Reference.Params.Eight.zc;
                    rx = obj.Reference.Params.Eight.rx;
                    ry = obj.Reference.Params.Eight.ry;
                    [x,y,z,vx,vy,vz,ax,ay,az] = obj.Reference.TrajGen.EightTraj(xc,yc,zc,rx,ry,T,dir);
                    fprintf("[Info] [DR-MRAPC_Pilot]\t [Figure-8] trajectory is selected...\n");
                case 2  % 螺旋形
                    z0 = obj.Reference.Params.Spiral.z0;
                    r = obj.Reference.Params.Spiral.r;
                    v = obj.Reference.Params.Spiral.v;
                    [x,y,z,vx,vy,vz,ax,ay,az] = obj.Reference.TrajGen.SpiralTraj(xc,yc,z0,v,T,r,dir);
                    fprintf("[Info] [DR-MRAPC_Pilot]\t [Spiral] trajectory is selected...\n");
                case 3  % 方形
                    zc = obj.Reference.Params.Square.zc;
                    L = obj.Reference.Params.Square.L;
                    [x,y,z,vx,vy,vz,ax,ay,az] = obj.Reference.TrajGen.SquareTraj(xc,yc,zc,L,T,dir);
                    fprintf("[Info] [DR-MRAPC_Pilot]\t [Square] trajectory is selected...\n");
            end
            obj.Reference.xi_r = [x;vx;y;vy;z;vz];
            obj.Reference.u_r = [ax;ay;az];
        end
    end
end