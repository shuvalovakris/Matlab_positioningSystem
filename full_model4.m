
clear;
simple_time = 0.005;   %с, Дискретный шаг времени
max_time = 24;         %с, Максимальное время движения робота по траектории

beacon1_pos = [-2 -2]; %Положение первого маяка [X Y], левый нижний угол
beacon2_pos = [-2 4];  %Положение второго маяка [X Y], левый верхний угол
beacon3_pos = [3 4]; %Положение третьего маяка [X Y], правый верхний угол
robot_pos = [-1 1 0]; %Начальное положение робота [X Y fi], fi - угол поворота относительно оси X, в радианах

%предыдущая позиция робота относительно каждого маяка
old_dist1 = sqrt((beacon1_pos(1)-robot_pos(1))^2 + (beacon1_pos(2)-robot_pos(2))^2);
old_dist2 = sqrt((beacon2_pos(1)-robot_pos(1))^2 + (beacon2_pos(2)-robot_pos(2))^2);
old_dist3 = sqrt((beacon3_pos(1)-robot_pos(1))^2 + (beacon3_pos(2)-robot_pos(2))^2);
old_time = 0;
old_pos1 = old_dist1; dist1_t = old_dist1;
old_pos2 = old_dist2; dist2_t = old_dist2;
old_pos3 = old_dist3; dist3_t = old_dist3;
[Xo_t, Yo_t] = coordinats_calc(dist1_t, dist2_t, dist3_t, beacon1_pos, beacon2_pos, beacon3_pos);
old_speed1 = 0; speed1_t = 0;
old_speed2 = 0; speed2_t = 0;
old_speed3 = 0; speed3_t = 0;

for time = 0:simple_time:max_time
    i = int32(time/simple_time+1);
    %Генерируем траекторию движения робота
    [X(i), Y(i), X0, Y0] = traj(time, simple_time);
    %Получаем скорость движения робота по координатам и угловому положению
    [x_s, y_s, fi_s] = Robot_model(X(i), Y(i), X0, Y0, robot_pos(1), robot_pos(2), robot_pos(3), simple_time); 
    %Рассчитываем реальное положение робота в пространстве
    robot_pos(1) = robot_pos(1) + x_s*simple_time;
    robot_pos(2) = robot_pos(2) + y_s*simple_time;
    robot_pos(3) = robot_pos(3) + fi_s*simple_time;
    v(i) = sqrt(x_s^2 + y_s^2);
    %Вычисляем реальное расстояние от маяков до робота
    real_dist1 = sqrt((beacon1_pos(1)-robot_pos(1))^2 + (beacon1_pos(2)-robot_pos(2))^2);
    real_dist2 = sqrt((beacon2_pos(1)-robot_pos(1))^2 + (beacon2_pos(2)-robot_pos(2))^2);
    real_dist3 = sqrt((beacon3_pos(1)-robot_pos(1))^2 + (beacon3_pos(2)-robot_pos(2))^2);   
    %Вычисляем реальную скорость движения робота относительно маяков
    real_speed1 = (real_dist1 - old_dist1)/simple_time;
    real_speed2 = (real_dist2 - old_dist2)/simple_time;
    real_speed3 = (real_dist3 - old_dist3)/simple_time;
    %Запоминаем старое реальное расстояние от маяков до робота для вычисления скорости
    old_dist1 = real_dist1;
    old_dist2 = real_dist2;
    old_dist3 = real_dist3;
    
    %Подаём реальные значения расстояния и скорости на модель маяковой системы
    %И получаем расстояния и скорости посчитанные маяковой системой
    
    %По расстояниям от маяковой системы вычисляем координаты робота
    if time - old_time >= 0.1
        [dist1_t, dist2_t, dist3_t, speed1_t, speed2_t, speed3_t] = dsss(old_pos1, old_pos2, old_pos3, old_speed1, old_speed2, old_speed3);
        [Xo_t, Yo_t] = coordinats_calc(dist1_t, dist2_t, dist3_t, beacon1_pos, beacon2_pos, beacon3_pos);
        old_pos1 = real_dist1;
        old_pos2 = real_dist2;
        old_pos3 = real_dist3;
        old_speed1 = real_speed1;
        old_speed2 = real_speed2;
        old_speed3 = real_speed3;
        old_time = time;
    end
    dist1(i) = dist1_t;
    dist2(i) = dist2_t;
    dist3(i) = dist3_t;
    speed1(i) = speed1_t;
    speed2(i) = speed2_t;
    speed3(i) = speed3_t;
    Xo(i) = Xo_t;
    Yo(i) = Yo_t;
    %для графиков
    xout(i) = robot_pos(1);
    yout(i) = robot_pos(2);
    
    test1(i) = real_dist1;
    test2(i) = real_dist2;
    test3(i) = real_dist3;
    test4(i) = real_speed1;
    test5(i) = real_speed2;
    test6(i) = real_speed3;
    diff_error(i) = sqrt((xout(i) - Xo(i))^2 + (yout(i) - Yo(i))^2);
end

t = 0:simple_time:max_time;
%Рисуем графики
set(0,'DefaultAxesFontSize',14,'DefaultAxesFontName','Times New Roman');
% figure('Color','w');
% plot(t, v, 'r--');

figure('Color','w');
plot(xout, yout, 'r--');
hold on;
plot(Xo, Yo, 'k-o', 'MarkerSize', 2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor','k'); 
plot(beacon1_pos(1), beacon1_pos(2),'-s', 'MarkerSize', 10,'MarkerEdgeColor', 'k', 'MarkerFaceColor','m');
plot(beacon2_pos(1), beacon2_pos(2),'-s', 'MarkerSize', 10,'MarkerEdgeColor', 'k', 'MarkerFaceColor','c');
plot(beacon3_pos(1), beacon3_pos(2),'-s', 'MarkerSize', 10,'MarkerEdgeColor', 'k', 'MarkerFaceColor','b');
hold off;
legend('траектория', 'координаты');
ylabel('Y, м');
xlabel('X, м');

figure('Color','w');
plot(t, dist1,'m-', t, dist2,'c-', t, dist3,'b-', t, test1,'m--', t, test2, 'c--', t, test3, 'b--');
ylabel('L, м');
xlabel('t, c');
figure('Color','w');
plot(t, speed1,'m-', t, speed2,'c-', t, speed3,'b-', t, test4,'m--', t, test5, 'c--', t, test6, 'b--');
ylabel('v, м/с');
xlabel('t, c');

%diff_error = sqrt((xout - Xo).^2 + (yout - Yo).^2);
%length_error
%result_error = sum(diff_error)/size(diff_error);
figure('Color','w');
plot(t, diff_error);
ylabel('ошибка, м');
xlabel('t, c');



%--------------------Функции------------------------
%Трилатирация, вычисление координат
%На входе 3 расстояния до объекта и координаты трёх маяков
%На выходе координаты объекта
function [Xo, Yo] = coordinats_calc(S1, S2, S3, point1, point2, point3)
    XB1=point1(1);
    YB1=point1(2);
    ZB1=0;
    XB2=point2(1);
    YB2=point2(2);
    ZB2=0;
    XB3=point3(1);
    YB3=point3(2);
    ZB3=0;    
	ex1 = (XB2-XB1)/sqrt((XB2-XB1)^2 + (YB2-YB1)^2 + (ZB2-ZB1)^2);
	ex2 = (YB2-YB1)/sqrt((XB2-XB1)^2 + (YB2-YB1)^2 + (ZB2-ZB1)^2);
	ex3 = (ZB2-ZB1)/sqrt((XB2-XB1)^2 + (YB2-YB1)^2 + (ZB2-ZB1)^2);    
	i = ex1*(XB3-XB1) + ex2*(YB3-YB1) + ex3*(ZB3-ZB1);    
	ey1 = (XB3-XB1-i*ex1)/sqrt((XB3-XB1-i*ex1)^2 + (YB3-YB1-i*ex2)^2 + (ZB3-ZB1-i*ex3)^2);
	ey2 = (YB3-YB1-i*ex2)/sqrt((XB3-XB1-i*ex1)^2 + (YB3-YB1-i*ex2)^2 + (ZB3-ZB1-i*ex3)^2);
	ey3 = (ZB3-ZB1-i*ex3)/sqrt((XB3-XB1-i*ex1)^2 + (YB3-YB1-i*ex2)^2 + (ZB3-ZB1-i*ex3)^2);    
	ex =[ex1 ex2 ex3];
	ey =[ey1 ey2 ey3];
	%ez = [ex ey]; 
	d = sqrt((XB2-XB1)^2 + (YB2-YB1)^2 + (ZB2-ZB1)^2);
	j = ey1*(XB3-XB1) + ey2*(YB3-YB1) + ey3*(ZB3-ZB1);
	x0 = (S1^2 - S2^2 + d^2)/(2*d);
	y0 = (S1^2 - S3^2 + i^2 + j^2)/(2*j) - ((i/j)*x0);
	z0 = sqrt(S1^2 - x0^2 - y0^2);

	Xo = XB1 + x0*ex(1) + y0*ex(2) + z0*ex(3);
	Yo = YB1 + x0*ey(1) + y0*ey(2) + z0*ey(3);
	%Zo = ZB1 + x0*ez(1) + y0*ez(2) + z0*ez(3);
end

%математическая Модель робота
%На входе требуемое положение, прошлое требуемое положение, текущее положение и дискретный шаг
%На выходе скорость движения робота по координатам x и y и скорость вращения робота
function [x_s, y_s, fi_s] = Robot_model(X, Y, X0, Y0, X_current, Y_current, fi_current, simple_time)
    BaseWidth = 0.7; %м Ширина робота
    whRad = 0.3; %м Диаметр колеса
    P_gain = 20;
    
    fi = atan2(Y-Y_current, X-X_current);
    v = sqrt(((X-X0)/simple_time)^2+((Y-Y0)/simple_time)^2);
    diff_fi = fi - fi_current;
    if diff_fi > pi  
        diff_fi = fi - fi_current - 2*pi; 
    end
    if diff_fi < -pi 
        diff_fi = fi - fi_current + 2*pi; 
    end
    Vmax=v-0.1;
    w1=Vmax*cos(diff_fi)/whRad - (P_gain*diff_fi*BaseWidth/2 + Vmax*cos(diff_fi)*sin(diff_fi));
    w2=Vmax*cos(diff_fi)/whRad + (P_gain*diff_fi*BaseWidth/2 + Vmax*cos(diff_fi)*sin(diff_fi));
    x_s = cos(fi_current)*(w1+w2)/2*whRad;
    y_s = sin(fi_current)*(w1+w2)/2*whRad;
    fi_s = (w2-w1)*whRad/BaseWidth;
end

%Генератор траектории синусоиды от времени для робота
function [x, y, x0, y0] = traj(t, simple_time)

    x = t/8-1;
    x0 = (t-simple_time)/8-1;
    y = 2*sin(t/2)+1;
    y0 = 2*sin((t-simple_time)/2)+1;
end

%Модель маяковой системы
function [dist1, dist2, dist3, speed1, speed2, speed3] = dsss(dist1_in, dist2_in, dist3_in, speed1_in, speed2_in, speed3_in)
vS = 343; %м/с, скорость звука в воздухе
f_carrier = 40000;
fs = 160000;
tr_bw = 6000;  % полоса пропускания трандьюсера
t_chip = 1/f_carrier * 80;     % 80 периодов несущей на один чип
seq_len = 32;

% м/с, целевая скорость до маяков (+/- vT_max)
vD1 = speed1_in; 
vD2 = speed2_in; 
vD3 = speed3_in;

vT_max = 1.5; % м/с, максимальная скорость объекта
max_dist = 50.0; % максимальная дистанция от маяка до объекта

% скорость объекта относительно маяков
beacon_1_delay = dist1_in/vS; 
beacon_2_delay = dist2_in/vS;
beacon_3_delay = dist3_in/vS; 


% затухание сигнала относительно маяков
% beacon_1_gain = 2*10^(-3.3e-5*f_carrier*dist1_in/20);
% beacon_2_gain = 2*10^(-3.3e-5*f_carrier*dist2_in/20);
% beacon_3_gain = 2*10^(-3.3e-5*f_carrier*dist3_in/20);
beacon_1_gain = 1;
beacon_2_gain = 1;
beacon_3_gain = 1;
% фазы сигналов маяков
beacon_1_phase = 0*pi/4;
beacon_2_phase = 1*pi/4;
beacon_3_phase = 2*pi/4;

tof = max_dist / vS; % с, время пролета
sys_time = 0:1/fs:tof; % с, системное время
frame_smps = size(sys_time, 2);%int32(tof*fs);

% псевдослучайные последовательности маяков
ref_seq1 = [1 0 0 1 1 1 0 0 0 1 0 1 0 0 1 0 0 0 1 1 1 1 0 1 1 0 1 1 0 0 1 0 ];
ref_seq2 = [1 0 0 0 0 1 0 1 1 0 0 0 1 0 1 0 0 0 0 0 0 0 0 1 0 0 1 0 0 1 1 1 ];
ref_seq3 = [1 0 0 0 1 1 0 1 0 0 1 1 1 1 0 1 1 1 1 0 1 0 1 0 1 0 1 0 1 0 1 1 ];

beacons_ref_len = t_chip * seq_len * fs; %длина эталонного сигнала

% расчет эталонных сигналов маяков
beacon_1_ref = interp1(t_chip : t_chip*fs : t_chip*seq_len*fs, ref_seq1-0.5, 0 : beacons_ref_len, 'previous'); beacon_1_ref(isnan(beacon_1_ref))=0;
beacon_2_ref = interp1(t_chip : t_chip*fs : t_chip*seq_len*fs, ref_seq2-0.5, 0 : beacons_ref_len, 'previous'); beacon_2_ref(isnan(beacon_2_ref))=0;
beacon_3_ref = interp1(t_chip : t_chip*fs : t_chip*seq_len*fs, ref_seq3-0.5, 0 : beacons_ref_len, 'previous'); beacon_3_ref(isnan(beacon_3_ref))=0;
% нормализация
beacon_1_ref = beacon_1_ref / sqrt( sum( beacon_1_ref.^2 ) );
beacon_2_ref = beacon_2_ref / sqrt( sum( beacon_2_ref.^2 ) );
beacon_3_ref = beacon_3_ref / sqrt( sum( beacon_3_ref.^2 ) );

beacon_1_sig = sin(2*pi*f_carrier*sys_time+beacon_1_phase) .* [zeros(1,int32(beacon_1_delay*fs)) beacon_1_ref zeros(1, int32(frame_smps - beacons_ref_len - beacon_1_delay*fs-1))];
beacon_2_sig = sin(2*pi*f_carrier*sys_time+beacon_2_phase) .* [zeros(1,int32(beacon_2_delay*fs)) beacon_2_ref zeros(1, int32(frame_smps - beacons_ref_len - beacon_2_delay*fs-1))];
beacon_3_sig = sin(2*pi*f_carrier*sys_time+beacon_3_phase) .* [zeros(1,int32(beacon_3_delay*fs)) beacon_3_ref zeros(1, int32(frame_smps - beacons_ref_len - beacon_3_delay*fs-1))];

beacon_1_sig = apply_doppler(beacon_1_sig, vD1);
beacon_2_sig = apply_doppler(beacon_2_sig, vD2);
beacon_3_sig = apply_doppler(beacon_3_sig, vD3);

rx_sig = beacon_1_gain * beacon_1_sig + beacon_2_gain * beacon_2_sig + beacon_3_gain * beacon_3_sig;%

% имитация полосы пропускания передатчика
[b, a] = butter(4, [f_carrier-tr_bw/2 f_carrier+tr_bw/2]/(fs/2));
rx_sig = filter(b, a, rx_sig);

%квадратурный приемник
rx_lo_q = sin(2*pi*f_carrier*sys_time);
rx_lo_i = -cos(2*pi*f_carrier*sys_time);

iwin_sz = 8; a = 1; b = (1/iwin_sz)*ones(1, iwin_sz);
rx_q = filter(b, a, rx_lo_q .* rx_sig);
rx_i = filter(b, a, rx_lo_i .* rx_sig);

%корреляция
rx_fft = fft(complex(rx_i, rx_q));
beacon_1_fft = conj(fft([beacon_1_ref zeros(1,frame_smps-beacons_ref_len-1)]));
beacon_2_fft = conj(fft([beacon_2_ref zeros(1,frame_smps-beacons_ref_len-1)]));
beacon_3_fft = conj(fft([beacon_3_ref zeros(1,frame_smps-beacons_ref_len-1)]));

beacon_1_xcorr_max = [];
beacon_2_xcorr_max = [];
beacon_3_xcorr_max = [];

% поиск наиболее подходящего доплеровского сдвига
max_doppler_shift = vT_max / (vS / f_carrier);
fft_resolution = fs/double(frame_smps);
fft_shift = int32(max_doppler_shift / fft_resolution);
fft_shift_idx = -fft_shift : fft_shift;
for n = fft_shift_idx
    beacon_1_fft_sh = vec_shift(beacon_1_fft, n);
    beacon_2_fft_sh = vec_shift(beacon_2_fft, n);
    beacon_3_fft_sh = vec_shift(beacon_3_fft, n);
    
    beacon_1_xcorr = abs(ifft(rx_fft .* beacon_1_fft_sh));
    beacon_2_xcorr = abs(ifft(rx_fft .* beacon_2_fft_sh));
    beacon_3_xcorr = abs(ifft(rx_fft .* beacon_3_fft_sh));
    
    beacon_1_xcorr_max = peak_search(beacon_1_xcorr_max, beacon_1_xcorr);
    beacon_2_xcorr_max = peak_search(beacon_2_xcorr_max, beacon_2_xcorr);
    beacon_3_xcorr_max = peak_search(beacon_3_xcorr_max, beacon_3_xcorr);

    %pause (0.01);
end

%расчет доплеровских скоростей и дистанции до каждого маяка
[pks, idx] = max(beacon_1_xcorr_max(:,1)');
doppler_speed_beacon_1 = double(fft_shift_idx(idx))*fft_resolution*(vS/f_carrier);
distance_beacon_1 = beacon_1_xcorr_max(idx,2)/fs*vS;

[pks, idx] = max(beacon_2_xcorr_max(:,1)');
doppler_speed_beacon_2 = double(fft_shift_idx(idx))*fft_resolution*(vS/f_carrier);
distance_beacon_2 = beacon_2_xcorr_max(idx,2)/fs*vS;

[pks, idx] = max(beacon_3_xcorr_max(:,1)');
doppler_speed_beacon_3 = double(fft_shift_idx(idx))*fft_resolution*(vS/f_carrier);
distance_beacon_3 = beacon_3_xcorr_max(idx,2)/fs*vS;

dist1 = distance_beacon_1;
dist2 = distance_beacon_2;
dist3 = distance_beacon_3;
speed1 = doppler_speed_beacon_1;
speed2 = doppler_speed_beacon_2;
speed3 = doppler_speed_beacon_3;
end

function [vec_out] =  vec_shift(vec_in, n)
    vec_out = circshift(vec_in, n);
    if n>0
        vec_out(1:n) = 0;
    else
        vec_out(end+n+1:end) = 0;
    end
end

% функция нахождения максимума
function  [maxi] = peak_search(maxi, vec)
    [vmax, vidx] = max(vec);
    maxi = [maxi; [vmax, vidx]];
end

function [out_sig] = apply_doppler(inp_sig, vD)
    vS = 343;
    tx_len = size(inp_sig, 2);
    out_sig = interp1(1:tx_len, inp_sig, linspace(1, tx_len, tx_len * vS / (vS + vD)));
    if size(inp_sig, 2) >= size(out_sig, 2)
        out_sig = padarray(out_sig', tx_len-numel(out_sig), 0, 'post')';
    else
        out_sig = out_sig(1:tx_len);
    end
end
