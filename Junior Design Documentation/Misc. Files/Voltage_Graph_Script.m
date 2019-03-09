 %%Time specifications:
   Fs = 8000;                   % samples per second
   dt = 1/Fs;                   % seconds per sample
   StopTime = (1/59);             % seconds
   t = (0:dt:StopTime-dt)';     % seconds
   %%Sine wave:
   Fc = 60;                     % hertz
   x = sin(2*pi*Fc*t);
   rms_calc = 120*sqrt(2);
   half = 1/240;
   neg_half = (3*half);
   % Plot the signal versus time:
   figure;
   plot(t,rms_calc*x, 'k');
   hold on
   
   line([half half], [0 rms_calc], 'Color','red','LineStyle','--');
   plot(half,rms_calc, 'ro');
   
   line([half/4 half/4], [0 rms_calc*sin(2*pi*Fc*half/4)], 'Color','red','LineStyle','--');
   plot(half/4,rms_calc*sin(2*pi*Fc*half/4), 'ro');
   
   line([2*half/4 2*half/4], [0 rms_calc*sin(2*pi*Fc*half/2)], 'Color','red','LineStyle','--');
   plot(2*half/4,rms_calc*sin(2*pi*Fc*half/2), 'ro');
   
   line([3*half/4 3*half/4], [0 rms_calc*sin(3*2*pi*Fc*half/4)], 'Color','red','LineStyle','--');
   plot(3*half/4,rms_calc*sin(2*3*pi*Fc*half/4), 'ro');
   
   line([3*half/2 3*half/2], [0 rms_calc*sin(2*pi*Fc*3*half/2)], 'Color','red','LineStyle','--');
   plot(3*half/2,rms_calc*sin(3*2*pi*Fc*half/2), 'ro');
   
   line([5*half/4 5*half/4], [0 rms_calc*sin(2*pi*Fc*5*half/4)], 'Color','red','LineStyle','--');
   plot(5*half/4,rms_calc*sin(5*2*pi*Fc*half/4), 'ro');
   
   line([7*half/4 7*half/4], [0 rms_calc*sin(2*pi*Fc*7*half/4)], 'Color','red','LineStyle','--');
   plot(7*half/4,rms_calc*sin(7*2*pi*Fc*half/4), 'ro');
   
   line([8*half/4 7*half/4], [0 rms_calc*sin(2*pi*Fc*8*half/4)], 'Color','red','LineStyle','--');
   plot(8*half/4,rms_calc*sin(8*2*pi*Fc*half/4), 'ro');
   
   line([10*half/4 10*half/4], [0 rms_calc*sin(2*pi*Fc*10*half/4)], 'Color','red','LineStyle','--');
   plot(10*half/4,rms_calc*sin(10*2*pi*Fc*half/4), 'ro');
   
   line([11*half/4 11*half/4], [0 rms_calc*sin(2*pi*Fc*11*half/4)], 'Color','red','LineStyle','--');
   plot(11*half/4,rms_calc*sin(11*2*pi*Fc*half/4), 'ro');
   
   line([13*half/4 13*half/4], [0 rms_calc*sin(2*pi*Fc*13*half/4)], 'Color','red','LineStyle','--');
   plot(13*half/4,rms_calc*sin(13*2*pi*Fc*half/4), 'ro');
   
   line([14*half/4 14*half/4], [0 rms_calc*sin(2*pi*Fc*14*half/4)], 'Color','red','LineStyle','--');
   plot(14*half/4,rms_calc*sin(14*2*pi*Fc*half/4), 'ro');
   
   line([neg_half neg_half], [0 -rms_calc], 'Color','red','LineStyle','--');
   plot(neg_half,-rms_calc, 'ro');
   
   line([3*neg_half/4 3*neg_half/4], [0 rms_calc*sin(3*2*pi*Fc*neg_half/4)], 'Color','red','LineStyle','--');
   plot(3*neg_half/4,rms_calc*sin(2*3*pi*Fc*neg_half/4), 'ro');
   
   line([5*neg_half/4 5*neg_half/4], [0 rms_calc*sin(2*pi*Fc*5*neg_half/4)], 'Color','red','LineStyle','--');
   plot(5*neg_half/4,rms_calc*sin(5*2*pi*Fc*neg_half/4), 'ro');
   
   line([0 .04], [0 0], 'Color','black');
   xlabel('Time (seconds)');
   
   line([0 half*4], [120 120], 'Color','blue','LineStyle','--');
   
   h = zeros(3, 1);
   h(2) = plot(NaN,NaN,'or');   
   h(3) = plot(NaN,NaN,'--b');
   h(1) = plot(NaN,NaN,'-k');
   legend(h, 'Voltage','Sampling Point','Root Mean Square');
   
   ylabel('Voltage (volts)');
   xlim([0 1/60]);
   title('Voltage vs. Time');
   zoom xon;
   