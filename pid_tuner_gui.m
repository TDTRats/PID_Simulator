function pid_tuner_gui()
% Author: liuskywalkerjskd

fig = uifigure('Name', 'PID控制器仿真', 'Position', [100 100 1100 700]);
mainGrid = uigridlayout(fig, [2, 1]);
mainGrid.RowHeight = {'1x', 40};
contentGrid = uigridlayout(mainGrid, [1, 2]);
contentGrid.Layout.Row = 1; contentGrid.Layout.Column = 1;
contentGrid.ColumnWidth = {'2x', '1x'};

ax = uiaxes(contentGrid);
ax.Layout.Row = 1; ax.Layout.Column = 1;
title(ax, '系统响应曲线'); xlabel(ax, '时间 (s)'); ylabel(ax, '幅值');
grid(ax, 'on'); hold(ax, 'on');

controlPanel = uigridlayout(contentGrid);
controlPanel.Layout.Row = 1; controlPanel.Layout.Column = 2;
controlPanel.RowHeight = {'fit', 'fit', 'fit', 20, 'fit', 'fit', 20, 'fit', 'fit', 'fit', 'fit', 'fit', 20, 'fit', 'fit', 'fit'};
controlPanel.ColumnWidth = {'fit', '1x', 'fit'};
controlPanel.Padding = [10 10 10 20];

handles = struct(); 
handles.detailsFig = []; 

% 定义默认参数
defaultParams.Kp = 50; defaultParams.Ki = 20; defaultParams.Kd = 5;
defaultParams.J = 10; defaultParams.b = 2;
defaultParams.intSepThreshold = 0.2;
defaultParams.intLimit = 20;
defaultParams.noiseAmplitude = 0.01; % 噪声幅值
defaultParams.dFilterFc = 20;      % 微分滤波截止频率 (Hz)

% 创建UI组件
createSliderRow('比例 (Kp)', 'Kp', 1, [0, 500], defaultParams.Kp);
createSliderRow('积分 (Ki)', 'Ki', 2, [0, 200], defaultParams.Ki);
createSliderRow('微分 (Kd)', 'Kd', 3, [0, 100], defaultParams.Kd);
addSeparator(4);
createSliderRow('系统惯量 (J)', 'J', 5, [1, 50], defaultParams.J);
createSliderRow('系统阻尼 (b)', 'b', 6, [0.1, 50], defaultParams.b);
addSeparator(7);
handles.derivOnMeasCheck = createCheckbox('微分先行', 8, true);
handles.intSepCheck = createCheckbox('启用积分分离', 9, false);
createSliderRow('分离阈值', 'intSepThreshold', 10, [0.01, 2], defaultParams.intSepThreshold);
handles.intLimitCheck = createCheckbox('启用积分限幅', 11, false);
createSliderRow('积分限幅', 'intLimit', 12, [1, 100], defaultParams.intLimit);

addSeparator(13);
createSliderRow('反馈噪声幅值', 'noiseAmplitude', 14, [0, 0.5], defaultParams.noiseAmplitude);
handles.dFilterCheck = createCheckbox('启用微分低通滤波', 15, true);
createSliderRow('滤波截止频率 (Hz)', 'dFilterFc', 16, [1, 200], defaultParams.dFilterFc);

buttonPanel = uipanel(mainGrid, 'BorderType', 'none');
buttonPanel.Layout.Row = 2; buttonPanel.Layout.Column = 1;
y_pos = 8; button_height = 25; reset_button_width = 100;
reset_button_x = 1100 - reset_button_width - 15;
resetButton = uibutton(buttonPanel, 'Text', '复位参数', 'Position', [reset_button_x, y_pos, reset_button_width, button_height], 'ButtonPushedFcn', @resetSimulation);
resetButton.BackgroundColor = [0.85, 0.33, 0.1]; resetButton.FontColor = [1, 1, 1];
start_button_width = 100;
start_button_x = reset_button_x - start_button_width - 10;
startButton = uibutton(buttonPanel, 'Text', '开始仿真', 'Position', [start_button_x, y_pos, start_button_width, button_height], 'ButtonPushedFcn', @runSimulation);
startButton.BackgroundColor = [0.47, 0.67, 0.19]; startButton.FontColor = [1, 1, 1];
label_width = 100; label_x = 15;
label_signal_type = uilabel(buttonPanel, 'Text', '输入信号类型:', 'Position', [label_x, y_pos, label_width, button_height], 'HorizontalAlignment', 'right');
dropdown_width = 150; dropdown_x = label_x + label_width;
handles.inputTypeDropdown = uidropdown(buttonPanel, 'Items', {'阶跃响应', '正弦跟踪'}, 'Value', '阶跃响应', 'Position', [dropdown_x, y_pos, dropdown_width, button_height]);

    function createSliderRow(labelText, paramName, row, limits, defaultValue)
        label = uilabel(controlPanel, 'Text', labelText, 'HorizontalAlignment', 'right');
        label.Layout.Row = row; label.Layout.Column = 1;
        slider = uislider(controlPanel, 'Limits', limits, 'Value', defaultValue);
        slider.Layout.Row = row; slider.Layout.Column = 2;
        valueLabel = uilabel(controlPanel, 'Text', sprintf('%.2f', defaultValue), 'FontWeight', 'bold');
        valueLabel.Layout.Row = row; valueLabel.Layout.Column = 3;
        slider.ValueChangedFcn = @(src, ~) updateLabel(valueLabel, src.Value);
        handles.([paramName 'Slider']) = slider;
        handles.([paramName 'ValueLabel']) = valueLabel;
    end
    function chk = createCheckbox(labelText, row, defaultValue)
        chk = uicheckbox(controlPanel, 'Text', labelText, 'Value', defaultValue);
        chk.Layout.Row = row; chk.Layout.Column = [1, 3];
    end
    function addSeparator(row)
        sep = uilabel(controlPanel, 'Text', '', 'BackgroundColor', [0.8 0.8 0.8]);
        sep.Layout.Row = row; sep.Layout.Column = [1, 3];
    end
    function updateLabel(labelHandle, value)
        labelHandle.Text = sprintf('%.2f', value);
    end

% 复位回调
    function resetSimulation(~, ~)
        params = {'Kp', 'Ki', 'Kd', 'J', 'b', 'intSepThreshold', 'intLimit', 'noiseAmplitude', 'dFilterFc'};
        for i = 1:length(params)
            p = params{i};
            handles.([p 'Slider']).Value = defaultParams.(p);
            updateLabel(handles.([p 'ValueLabel']), defaultParams.(p));
        end
        handles.derivOnMeasCheck.Value = true;
        handles.intSepCheck.Value = false;
        handles.intLimitCheck.Value = false;
        handles.dFilterCheck.Value = true;
        handles.inputTypeDropdown.Value = '阶跃响应';
        cla(ax); title(ax, '系统响应曲线'); legend(ax, 'off');
        
        if isfield(handles, 'detailsFig') && isvalid(handles.detailsFig)
            close(handles.detailsFig);
            handles.detailsFig = [];
        end
    end

% 仿真回调
    function runSimulation(~, ~)
        Kp = handles.KpSlider.Value; Ki = handles.KiSlider.Value; Kd = handles.KdSlider.Value;
        J = handles.JSlider.Value; b = handles.bSlider.Value;
        useDerivOnMeas = handles.derivOnMeasCheck.Value;
        useIntSep = handles.intSepCheck.Value;
        intSepThreshold = handles.intSepThresholdSlider.Value;
        useIntLimit = handles.intLimitCheck.Value;
        intLimit = handles.intLimitSlider.Value;
        noiseAmplitude = handles.noiseAmplitudeSlider.Value;
        useDFilter = handles.dFilterCheck.Value;
        dFilterFc = handles.dFilterFcSlider.Value;
        
        T_final = 15; dt = 0.01;
        t = (0:dt:T_final)';
        N = length(t);
        inputType = handles.inputTypeDropdown.Value;
        if strcmp(inputType, '阶跃响应')
            setpoint_signal = ones(N, 1);
            titleText = '系统阶跃响应';
        else
            setpoint_signal = 0.5 + 0.5*sin(1.5 * t); 
            titleText = '系统正弦跟踪响应';
        end

        % 模型与滤波器离散化 
        s = tf('s');
        plant = 1 / (J*s^2 + b*s);
        d_plant = c2d(plant, dt, 'zoh');
        [A, B, C, D] = ssdata(d_plant);
        plant_states = zeros(size(A,1), 1);
        
        % 计算低通滤波器系数 alpha
        tau = 1 / (2 * pi * dFilterFc);
        alpha = dt / (tau + dt);
        filtered_derivative_term = 0; % 滤波器状态初始化
        
        % --- 初始化变量 ---
        y = zeros(N, 1); u = zeros(N, 1);
        e = zeros(N, 1);
        integral_term = 0;
        prev_e = 0;
        prev_y_measured = 0; % 使用测量值作为历史值
        p_term_vec = zeros(N, 1);
        i_term_vec = zeros(N, 1);
        d_term_vec = zeros(N, 1);
        
        % 时域仿真
        for k = 1:N
            y_true = y(k);
            y_measured = y_true + noiseAmplitude * randn();
            
            e(k) = setpoint_signal(k) - y_measured;
            
            error_for_integral = e(k);
            if useIntSep && (abs(e(k)) > intSepThreshold)
                error_for_integral = 0;
            end
            integral_term = integral_term + Ki * error_for_integral * dt;
            if useIntLimit
                integral_term = max(-intLimit, min(intLimit, integral_term));
            end
            
            if useDerivOnMeas
                raw_derivative_term = Kd * (prev_y_measured - y_measured) / dt;
            else
                raw_derivative_term = Kd * (e(k) - prev_e) / dt;
            end

            if useDFilter
                derivative_term = alpha * raw_derivative_term + (1 - alpha) * filtered_derivative_term;
                filtered_derivative_term = derivative_term; % 更新滤波器状态
            else
                derivative_term = raw_derivative_term;
            end
            
            proportional_term = Kp * e(k);
            
            p_term_vec(k) = proportional_term;
            i_term_vec(k) = integral_term;
            d_term_vec(k) = derivative_term;
            
            u(k) = p_term_vec(k) + i_term_vec(k) + d_term_vec(k);
            
            plant_states = A * plant_states + B * u(k);
            y_out = C * plant_states + D * u(k);
            
            prev_e = e(k);
            prev_y_measured = y_measured; % 保存的是带噪声的测量值
            if k < N
                y(k+1) = y_out; % 更新的是理想的系统输出
            end
        end
        
        % 绘图 
        cla(ax);
        plot(ax, t, y, 'b-', 'LineWidth', 1.5, 'DisplayName', '实际值 (理想)');
        plot(ax, t, setpoint_signal, 'r--', 'LineWidth', 1.5, 'DisplayName', '设定值');
        ylim(ax, 'auto');
        currentYlim = ylim(ax);
        ylim(ax, [min(-0.2, currentYlim(1)), max(2.2, currentYlim(2))]);
        legend(ax, 'Location', 'southeast');
        title(ax, titleText);
        grid(ax, 'on');
        
        if isempty(handles.detailsFig) || ~isvalid(handles.detailsFig)
             handles.detailsFig = figure('Name', 'PID控制器各部分作用', 'Position', [1220 100 500 700], 'NumberTitle', 'off', 'MenuBar', 'none');
        end
        figure(handles.detailsFig);
        clf;
        ax_p = subplot(3, 1, 1);
        plot(ax_p, t, p_term_vec, 'r', 'LineWidth', 1.5);
        title(ax_p, '比例 (P) 部分作用'); ylabel(ax_p, '控制量幅值'); grid(ax_p, 'on');
        ax_i = subplot(3, 1, 2);
        plot(ax_i, t, i_term_vec, 'g', 'LineWidth', 1.5);
        title(ax_i, '积分 (I) 部分作用'); ylabel(ax_i, '控制量幅值'); grid(ax_i, 'on');
        ax_d = subplot(3, 1, 3);
        plot(ax_d, t, d_term_vec, 'b', 'LineWidth', 1.5);
        title(ax_d, '微分 (D) 部分作用'); xlabel(ax_d, '时间 (s)'); ylabel(ax_d, '控制量幅值'); grid(ax_d, 'on');
    end
end